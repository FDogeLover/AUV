"""Full warehouse-inventory mission integration.

This module connects the existing flight controller to the warehouse-specific
planner, gimbal, QR, laser, result-store, and state-trace components.  The
route-only entry point intentionally does not import this controller.
"""

import os
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from Lcode.ground_link import GroundMessageType
from Lcode.inventory_planner import MissionWaypoint, WaypointKind
from Lcode.inventory_state import InventoryState, InventoryStateMachine
from Lcode.inventory_store import InventoryConflict, InventoryStore
from Lcode.laser_pointer import LaserPointer
from Lcode.Logger import logger
from Lcode.qr_vision import QRConsensus, QRDecoder
from Lcode.sensor_gimbal import SensorGimbal
from Mission_GPT import mission as FlightMission

try:
    import cv2
except ImportError:
    cv2 = None


@dataclass(frozen=True)
class InventoryMissionConfig:
    # The deployed UVC camera enumerates as video0 on the flight board.
    # Keep DRONE_CAMERA_DEVICE as an override for alternate capture nodes.
    camera_device: str = "/dev/video0"
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 15
    qr_mapping_file: str = "qr_mapping.txt"
    scan_timeout_s: float = 8.0
    scan_poll_s: float = 0.03
    laser_aim_x_ratio: float = 0.5
    laser_aim_y_ratio: float = 0.5

    def __post_init__(self):
        if self.camera_width < 1 or self.camera_height < 1 or self.camera_fps < 1:
            raise ValueError("摄像头参数必须为正数")
        if self.scan_timeout_s < 1.0:
            raise ValueError("扫码超时不能小于 1 秒")
        if not 0.0 <= self.laser_aim_x_ratio <= 1.0:
            raise ValueError("激光瞄准点 X 比例必须在 [0,1] 内")
        if not 0.0 <= self.laser_aim_y_ratio <= 1.0:
            raise ValueError("激光瞄准点 Y 比例必须在 [0,1] 内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        return cls(
            camera_device=env.get("DRONE_CAMERA_DEVICE", "/dev/video0"),
            camera_width=int(env.get("DRONE_CAMERA_WIDTH", "1280")),
            camera_height=int(env.get("DRONE_CAMERA_HEIGHT", "720")),
            camera_fps=int(env.get("DRONE_CAMERA_FPS", "15")),
            qr_mapping_file=env.get("DRONE_QR_MAPPING_FILE", "qr_mapping.txt"),
            scan_timeout_s=float(env.get("DRONE_QR_SCAN_TIMEOUT_S", "8.0")),
            scan_poll_s=float(env.get("DRONE_QR_SCAN_POLL_S", "0.03")),
            laser_aim_x_ratio=float(env.get("DRONE_LASER_AIM_X_RATIO", "0.5")),
            laser_aim_y_ratio=float(env.get("DRONE_LASER_AIM_Y_RATIO", "0.5")),
        )


class CameraSource:
    """Small, injectable OpenCV camera adapter used by the scan state."""

    def __init__(self, config=None, capture_factory=None):
        self.config = config or InventoryMissionConfig.from_env()
        self._capture_factory = capture_factory
        self._capture = None
        self.started = False

    @property
    def device(self):
        value = self.config.camera_device
        return int(value) if str(value).isdigit() else value

    def start(self) -> bool:
        if cv2 is None and self._capture_factory is None:
            logger.error("二维码摄像头需要 OpenCV")
            return False
        try:
            factory = self._capture_factory or cv2.VideoCapture
            self._capture = factory(self.device)
            if hasattr(self._capture, "isOpened") and not self._capture.isOpened():
                logger.error(f"二维码摄像头打开失败: {self.config.camera_device}")
                self.close()
                return False
            if hasattr(self._capture, "set") and cv2 is not None:
                # The UVC camera opens as 320x240 YUYV by default. Select
                # MJPEG before requesting the scan resolution; otherwise
                # OpenCV may silently keep the low-resolution mode.
                self._capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*"MJPG"),
                )
                self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
                self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)
                self._capture.set(cv2.CAP_PROP_FPS, self.config.camera_fps)
            self.started = True
            return True
        except Exception as exc:
            logger.error(f"二维码摄像头初始化失败: {exc}")
            self.close()
            return False

    def read(self):
        if not self.started or self._capture is None:
            return None
        try:
            ok, frame = self._capture.read()
            return frame if ok else None
        except Exception as exc:
            logger.error(f"二维码摄像头读取失败: {exc}")
            return None

    def laser_aim_point(self, frame):
        height, width = frame.shape[:2]
        return (
            width * self.config.laser_aim_x_ratio,
            height * self.config.laser_aim_y_ratio,
        )

    def close(self):
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
        self._capture = None
        self.started = False


class InventoryMissionCoordinator:
    """Procedural actions executed when the flight driver reaches a waypoint."""

    def __init__(
        self,
        route,
        state_machine: InventoryStateMachine,
        gimbal: SensorGimbal,
        laser: LaserPointer,
        camera: CameraSource,
        decoder: QRDecoder,
        consensus: QRConsensus,
        store: InventoryStore,
        ground_link=None,
        config=None,
        clock=time.monotonic,
        sleep_fn=time.sleep,
    ):
        self.route = list(route)
        self.state_machine = state_machine
        self.gimbal = gimbal
        self.laser = laser
        self.camera = camera
        self.decoder = decoder
        self.consensus = consensus
        self.store = store
        self.ground_link = ground_link
        self.config = config or InventoryMissionConfig.from_env()
        self._clock = clock
        self._sleep = sleep_fn
        self.driver = None
        self.last_detection = None

    def attach_driver(self, driver):
        self.driver = driver

    def _go(self, state, reason, **fields):
        state = InventoryState(state)
        if self.state_machine.state != state:
            self.state_machine.transition(state, reason, **fields)

    def on_takeoff_complete(self):
        self._go(InventoryState.TRANSIT, "takeoff_complete")

    def on_waypoint_arrived(self, index, position, reason):
        waypoint = self.route[index]
        self.state_machine.sample(
            waypoint_index=index,
            waypoint_kind=waypoint.kind.value,
            slot_label=waypoint.slot_label,
            arrival_reason=reason,
            position=list(position),
        )

        if waypoint.kind in {WaypointKind.TAKEOFF, WaypointKind.TRANSIT}:
            self._go(InventoryState.TRANSIT, "transit_arrival", waypoint_index=index)
            return True

        if waypoint.kind == WaypointKind.SET_GIMBAL:
            self._go(
                InventoryState.SET_GIMBAL,
                "set_face",
                face=waypoint.face.value if waypoint.face else None,
            )
            try:
                gimbal_ok = waypoint.face is not None and self.gimbal.set_face(waypoint.face)
            except Exception as exc:
                return self._abort(
                    "gimbal_set_exception", waypoint_index=index, error=str(exc)
                )
            if not gimbal_ok:
                return self._abort("gimbal_set_failed", waypoint_index=index)
            self._go(InventoryState.APPROACH_SLOT, "gimbal_set")
            return True

        if waypoint.kind == WaypointKind.INSPECT:
            return self._inspect_slot(index, waypoint, position)

        if waypoint.kind == WaypointKind.LAND_APPROACH:
            self._go(InventoryState.RETURN, "landing_approach")
            return True

        if waypoint.kind == WaypointKind.LAND:
            self._go(InventoryState.LAND, "landing_point")
            return True

        return self._abort("unknown_waypoint_kind", waypoint_index=index)

    def _inspect_slot(self, index, waypoint: MissionWaypoint, position):
        self._go(
            InventoryState.APPROACH_SLOT,
            "inspect_arrival",
            slot_label=waypoint.slot_label,
        )
        self._go(InventoryState.VISUAL_ALIGN, "hold_position_for_scan")
        self._go(InventoryState.VERIFY_QR, "visual_alignment_ready")
        self.consensus.reset()
        deadline = self._clock() + self.config.scan_timeout_s
        accepted = None

        while self._clock() < deadline:
            try:
                self._hold_position(waypoint.point.z)
                frame = self.camera.read()
            except Exception as exc:
                return self._abort(
                    "camera_read_exception", waypoint_index=index, error=str(exc)
                )
            if frame is not None:
                try:
                    detection = self.decoder.detect(frame)
                    accepted = self.consensus.update(
                        detection, self.camera.laser_aim_point(frame)
                    )
                except Exception as exc:
                    return self._abort(
                        "vision_exception", waypoint_index=index, error=str(exc)
                    )
                self.state_machine.sample(
                    waypoint_index=index,
                    slot_label=waypoint.slot_label,
                    detected_number=(detection.number if detection else None),
                    accepted_number=(accepted.number if accepted else None),
                )
                if accepted is not None:
                    break
            self._sleep(self.config.scan_poll_s)

        if accepted is None:
            return self._abort("qr_timeout", waypoint_index=index, slot_label=waypoint.slot_label)

        try:
            self.store.check_available(
                accepted.number,
                waypoint.slot_label,
            )
        except InventoryConflict:
            # A stable but already-used number is treated as a visual false
            # positive. Keep the drone hovering and retry this same slot.
            logger.warning(f"货位 {waypoint.slot_label} 识别到重复编号 {accepted.number}，重试")
            self.consensus.reset()
            return self._abort(
                "qr_duplicate", waypoint_index=index, slot_label=waypoint.slot_label
            )

        self._go(InventoryState.ILLUMINATE, "qr_verified", cargo_id=accepted.number)
        try:
            if not self.laser.pulse_async():
                return self._abort("laser_pulse_failed", waypoint_index=index)
            if not self.laser.wait(timeout=self.laser.config.duration_s + 0.5):
                return self._abort("laser_pulse_timeout", waypoint_index=index)
        except Exception as exc:
            return self._abort(
                "laser_pulse_exception", waypoint_index=index, error=str(exc)
            )

        result = self.store.add(
            accepted.number,
            waypoint.slot_label,
            self.consensus.config.required_count / self.consensus.config.window_size,
        )

        self._go(
            InventoryState.REPORT,
            "laser_complete",
            cargo_id=result.cargo_id,
            slot_label=result.slot_label,
        )
        self._publish_result(result)

        next_kind = self.route[index + 1].kind if index + 1 < len(self.route) else None
        if next_kind == WaypointKind.SET_GIMBAL:
            self._go(InventoryState.SET_GIMBAL, "next_face")
        elif next_kind in {WaypointKind.LAND_APPROACH, WaypointKind.LAND}:
            self._go(InventoryState.RETURN, "inventory_complete")
        else:
            self._go(InventoryState.TRANSIT, "next_slot")
        return True

    def _hold_position(self, z_m):
        if self.driver is not None:
            self.driver.hold_position(z_m)

    def _publish_result(self, result):
        if self.ground_link is None:
            return
        try:
            self.ground_link.publish(
                GroundMessageType.INVENTORY_RESULT,
                {
                    "cargo_id": result.cargo_id,
                    "slot_label": result.slot_label,
                    "confidence": result.confidence,
                    "timestamp": result.timestamp,
                },
            )
        except Exception as exc:
            self.state_machine.trace.fault("ground_result_publish_failed", error=str(exc))

    def _abort(self, code, **fields):
        logger.error(f"盘点状态机故障: {code} {fields}")
        self.state_machine.fault(code, **fields)
        if self.driver is not None:
            self.driver.abort_to_land()
        if self.state_machine.state == InventoryState.RETURN:
            self._go(InventoryState.LAND, "fault_land", fault_code=code)
        return False


def default_inventory_config(base_dir=None):
    config = InventoryMissionConfig.from_env()
    if base_dir is None:
        return config
    mapping = Path(config.qr_mapping_file)
    if not mapping.is_absolute():
        mapping = Path(base_dir) / mapping
    return InventoryMissionConfig(
        camera_device=config.camera_device,
        camera_width=config.camera_width,
        camera_height=config.camera_height,
        camera_fps=config.camera_fps,
        qr_mapping_file=str(mapping),
        scan_timeout_s=config.scan_timeout_s,
        scan_poll_s=config.scan_poll_s,
        laser_aim_x_ratio=config.laser_aim_x_ratio,
        laser_aim_y_ratio=config.laser_aim_y_ratio,
    )


class InventoryFlightMission(FlightMission):
    """Flight driver subclass that invokes the inventory coordinator at stops."""

    def __init__(self, re_fc, se_fc, realsense, serial_fc, route, coordinator):
        super().__init__(
            re_fc,
            se_fc,
            realsense,
            serial_fc_ref=serial_fc,
            route_file=None,
        )
        self.inventory_route = list(route)
        self.targets = [waypoint.point.as_list() for waypoint in self.inventory_route]
        self.coordinator = coordinator
        self.coordinator.attach_driver(self)

    def start(self):
        if self.coordinator.state_machine.state == InventoryState.PREFLIGHT:
            self.coordinator._go(InventoryState.WARNING_5S, "preflight_passed")
        super().start()
        if self.task_running:
            self.coordinator._go(InventoryState.TAKEOFF, "flight_task_started")
        elif self.coordinator.state_machine.state == InventoryState.WARNING_5S:
            # The base driver can refuse to start after a T265 or warning-light
            # failure. Do not leave the inventory state machine looking ready.
            self.coordinator._abort("flight_start_failed")

    def takeoff(self):
        previous_state = self.state
        super().takeoff()
        if previous_state == "TAKEOFF" and self.state == "NAVIGATE":
            self.coordinator.on_takeoff_complete()

    def _advance_waypoint(self, reason, pos, target, arrival_distance):
        index = self.target_index
        if index >= len(self.inventory_route):
            return super()._advance_waypoint(reason, pos, target, arrival_distance)
        if not self.coordinator.on_waypoint_arrived(index, pos, reason):
            return
        super()._advance_waypoint(reason, pos, target, arrival_distance)

    def hold_position(self, z_m):
        yaw_cmd = self._heading_status.command_dps
        self.set_speed(0, 0, yaw_cmd, int(float(z_m) * 100))

    def abort_to_land(self):
        self.set_speed(0, 0, 0, int(self._ramp_z_cm))
        self.state = "LAND"

    def land(self):
        previous_state = self.state
        super().land()
        if (
            previous_state == "LAND"
            and self.state == "END"
            and self.coordinator.state_machine.state == InventoryState.LAND
        ):
            self.coordinator._go(InventoryState.END, "land_confirmed")
