"""Full warehouse-inventory mission integration.

This module connects the existing flight controller to the warehouse-specific
planner, gimbal, QR, laser, result-store, and state-trace components.  The
route-only entry point intentionally does not import this controller.
"""

import math
import os
import threading
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
from Lcode.vision_servo import VisionServoConfig, VisionServoResult, servo_command
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
    # This board's UVC driver uses V4L2 values: 1=manual, 3=aperture priority.
    # Default to automatic exposure because the camera can otherwise be left at
    # exposure_time_absolute=50, which is too dark for the QR modules in flight.
    camera_auto_exposure: Optional[float] = 3.0
    camera_exposure: Optional[float] = None
    camera_gain: Optional[float] = None
    camera_auto_focus: Optional[float] = None
    camera_focus: Optional[float] = None
    camera_zoom: Optional[float] = None
    camera_warmup_frames: int = 10
    qr_mapping_file: str = "qr_mapping.txt"
    scan_timeout_s: float = 8.0
    scan_poll_s: float = 0.03
    laser_aim_x_ratio: float = 0.5
    laser_aim_y_ratio: float = 0.5
    vision_servo: VisionServoConfig = None

    def __post_init__(self):
        if self.vision_servo is None:
            object.__setattr__(self, "vision_servo", VisionServoConfig.from_env())
        if self.camera_width < 1 or self.camera_height < 1 or self.camera_fps < 1:
            raise ValueError("摄像头参数必须为正数")
        if self.camera_warmup_frames < 0:
            raise ValueError("camera_warmup_frames must not be negative")
        if self.scan_timeout_s < 1.0:
            raise ValueError("扫码超时不能小于 1 秒")
        if not 0.0 <= self.laser_aim_x_ratio <= 1.0:
            raise ValueError("激光瞄准点 X 比例必须在 [0,1] 内")
        if not 0.0 <= self.laser_aim_y_ratio <= 1.0:
            raise ValueError("激光瞄准点 Y 比例必须在 [0,1] 内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ

        def optional_float(name):
            raw = env.get(name, "").strip()
            return None if not raw else float(raw)

        return cls(
            camera_device=env.get("DRONE_CAMERA_DEVICE", "/dev/video0"),
            camera_width=int(env.get("DRONE_CAMERA_WIDTH", "1280")),
            camera_height=int(env.get("DRONE_CAMERA_HEIGHT", "720")),
            camera_fps=int(env.get("DRONE_CAMERA_FPS", "15")),
            camera_auto_exposure=float(env.get("DRONE_CAMERA_AUTO_EXPOSURE", "3")),
            camera_exposure=optional_float("DRONE_CAMERA_EXPOSURE"),
            camera_gain=optional_float("DRONE_CAMERA_GAIN"),
            camera_auto_focus=optional_float("DRONE_CAMERA_AUTOFOCUS"),
            camera_focus=optional_float("DRONE_CAMERA_FOCUS"),
            camera_zoom=optional_float("DRONE_CAMERA_ZOOM"),
            camera_warmup_frames=int(env.get("DRONE_CAMERA_WARMUP_FRAMES", "10")),
            qr_mapping_file=env.get("DRONE_QR_MAPPING_FILE", "qr_mapping.txt"),
            scan_timeout_s=float(env.get("DRONE_QR_SCAN_TIMEOUT_S", "8.0")),
            scan_poll_s=float(env.get("DRONE_QR_SCAN_POLL_S", "0.03")),
            laser_aim_x_ratio=float(env.get("DRONE_LASER_AIM_X_RATIO", "0.5")),
            laser_aim_y_ratio=float(env.get("DRONE_LASER_AIM_Y_RATIO", "0.5")),
            vision_servo=VisionServoConfig.from_env(env),
        )


class CameraSource:
    """Small, injectable OpenCV camera adapter used by the scan state."""

    def __init__(self, config=None, capture_factory=None):
        self.config = config or InventoryMissionConfig.from_env()
        self._capture_factory = capture_factory
        self._capture = None
        self._capture_thread = None
        self._capture_stop = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_sequence = 0
        self._frame_timestamp = None
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
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if self.config.camera_auto_exposure is not None:
                    self._capture.set(
                        cv2.CAP_PROP_AUTO_EXPOSURE,
                        self.config.camera_auto_exposure,
                    )
                if self.config.camera_exposure is not None:
                    self._capture.set(cv2.CAP_PROP_EXPOSURE, self.config.camera_exposure)
                if self.config.camera_gain is not None:
                    self._capture.set(cv2.CAP_PROP_GAIN, self.config.camera_gain)
                if (
                    self.config.camera_auto_focus is not None
                    and hasattr(cv2, "CAP_PROP_AUTOFOCUS")
                ):
                    self._capture.set(
                        cv2.CAP_PROP_AUTOFOCUS,
                        self.config.camera_auto_focus,
                    )
                if (
                    self.config.camera_focus is not None
                    and hasattr(cv2, "CAP_PROP_FOCUS")
                ):
                    self._capture.set(cv2.CAP_PROP_FOCUS, self.config.camera_focus)
                if self.config.camera_zoom is not None and hasattr(cv2, "CAP_PROP_ZOOM"):
                    # The deployed UVC camera exposes zoom_absolute 0..3.
                    # Keep this opt-in because some alternate cameras reject
                    # CAP_PROP_ZOOM entirely.
                    self._capture.set(cv2.CAP_PROP_ZOOM, self.config.camera_zoom)
                # Give the UVC driver/ISP time to settle before QR processing.
                for _ in range(self.config.camera_warmup_frames):
                    ok, frame = self._capture.read()
                    if ok and frame is not None:
                        self._store_frame(frame)
            self._capture_stop.clear()
            self.started = True
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                name="inventory-camera-capture",
                daemon=True,
            )
            self._capture_thread.start()
            return True
        except Exception as exc:
            logger.error(f"二维码摄像头初始化失败: {exc}")
            self.close()
            return False

    def _capture_loop(self):
        while not self._capture_stop.is_set() and self._capture is not None:
            try:
                ok, frame = self._capture.read()
            except Exception as exc:
                logger.error(f"二维码摄像头后台取帧失败: {exc}")
                time.sleep(0.01)
                continue
            if ok and frame is not None:
                self._store_frame(frame)
            else:
                time.sleep(0.01)

    def _store_frame(self, frame):
        with self._frame_lock:
            self._frame_sequence += 1
            self._latest_frame = frame
            self._frame_timestamp = time.monotonic()

    def read_with_sequence(self):
        """Return the latest frame and its capture sequence/timestamp.

        The capture thread runs independently of QR decoding.  Consumers can
        use the sequence to avoid decoding the same latest frame repeatedly.
        """
        if not self.started:
            return None, None, None
        with self._frame_lock:
            if self._latest_frame is None:
                return None, None, None
            return (
                self._frame_sequence,
                self._latest_frame.copy(),
                self._frame_timestamp,
            )

    def read(self):
        _, frame, _ = self.read_with_sequence()
        return frame

    def laser_aim_point(self, frame):
        height, width = frame.shape[:2]
        return (
            width * self.config.laser_aim_x_ratio,
            height * self.config.laser_aim_y_ratio,
        )

    def close(self):
        self._capture_stop.set()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        self._capture_thread = None
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
        self._capture = None
        self.started = False
        with self._frame_lock:
            self._latest_frame = None
            self._frame_sequence = 0
            self._frame_timestamp = None


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
        vision_debug=None,
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
        self.vision_debug = vision_debug
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
        scan_z_m = waypoint.point.z
        if self.config.vision_servo.enabled:
            self._go(InventoryState.VISUAL_SERVO, "geometry_alignment_start")
            servo_result = self._run_visual_servo(index, waypoint, position)
            if not servo_result.success:
                return self._abort(
                    "vision_servo_" + (servo_result.reason or "failed"),
                    waypoint_index=index,
                    slot_label=waypoint.slot_label,
                    servo_frames=servo_result.frames,
                )
            scan_z_m = servo_result.z_target_m
            self.state_machine.sample(
                waypoint_index=index,
                slot_label=waypoint.slot_label,
                visual_servo_frames=servo_result.frames,
                visual_servo_stable_frames=servo_result.stable_frames,
                visual_servo_z_target_m=round(scan_z_m, 3),
                visual_servo_error_px=[
                    round(servo_result.error_x_px, 1)
                    if servo_result.error_x_px is not None else None,
                    round(servo_result.error_y_px, 1)
                    if servo_result.error_y_px is not None else None,
                ],
            )
        self._go(InventoryState.VERIFY_QR, "visual_alignment_ready")
        self.consensus.reset()
        deadline = self._clock() + self.config.scan_timeout_s
        accepted = None
        last_frame = None
        last_frame_sequence = None
        processed_frame_count = 0

        while self._clock() < deadline:
            try:
                self._hold_position(scan_z_m)
                if hasattr(self.camera, "read_with_sequence"):
                    frame_sequence, frame, frame_timestamp = self.camera.read_with_sequence()
                else:
                    # Keep injected/test cameras and older camera adapters
                    # compatible; they have no sequence, so every read is
                    # treated as a new sample.
                    frame_sequence, frame_timestamp = None, None
                    frame = self.camera.read()
            except Exception as exc:
                return self._abort(
                    "camera_read_exception", waypoint_index=index, error=str(exc)
                )
            if frame is not None:
                if (
                    frame_sequence is not None
                    and frame_sequence == last_frame_sequence
                ):
                    self._sleep(self.config.scan_poll_s)
                    continue
                last_frame_sequence = frame_sequence
                last_frame = frame
                try:
                    laser_aim_px = self.camera.laser_aim_point(frame)
                    try:
                        detection = self.decoder.detect(
                            frame,
                            target_point=laser_aim_px,
                        )
                    except TypeError:
                        # Keep older/test decoder adapters compatible with the
                        # new target-aware API.
                        detection = self.decoder.detect(frame)
                    accepted = self.consensus.update(detection, laser_aim_px)
                except Exception as exc:
                    return self._abort(
                        "vision_exception", waypoint_index=index, error=str(exc)
                    )
                processed_frame_count += 1
                debug_metadata = {
                    "state": InventoryState.VERIFY_QR.value,
                    "capture": "scan_frame",
                    "waypoint_index": index,
                    "slot_label": waypoint.slot_label,
                    "position": list(position),
                    "laser_aim_px": list(laser_aim_px),
                    "detected_number": detection.number if detection else None,
                    "accepted_number": accepted.number if accepted else None,
                    "frame_sequence": frame_sequence,
                    "capture_timestamp": frame_timestamp,
                    "processed_frame_count": processed_frame_count,
                    "frame_shape": list(frame.shape),
                    "timestamp": time.time(),
                }
                if self.vision_debug is not None:
                    self.vision_debug.capture_scan(frame, debug_metadata)
                self.state_machine.sample(
                    waypoint_index=index,
                    slot_label=waypoint.slot_label,
                    position=list(position),
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

    def _run_visual_servo(self, index, waypoint, position):
        """Center QR geometry with bounded X velocity and Z setpoint changes."""
        config = self.config.vision_servo
        started = self._clock()
        deadline = started + config.timeout_s
        last_seen = started
        stable = 0
        frames = 0
        base_z_m = float(waypoint.point.z)
        z_target_m = base_z_m
        last_error = (None, None)
        last_frame = None
        last_center = None
        face_sign = config.x_direction(waypoint.face)
        start_x = float(position[0]) if position else None
        last_debug_capture_t = started
        _debug_interval = 0.5  # 周期性存图间隔（秒）

        while self._clock() < deadline:
            frames += 1
            try:
                frame = self.camera.read()
            except Exception:
                self._send_servo_command(0.0, z_target_m)
                return VisionServoResult(
                    False, z_target_m, frames, stable, reason="camera_read"
                )
            if frame is None:
                if frames >= config.min_lost_frames and self._clock() - last_seen > config.lost_timeout_s:
                    self._capture_servo_failure(last_frame, index, waypoint, "target_lost")
                    return VisionServoResult(
                        False, z_target_m, frames, stable, reason="target_lost"
                    )
                self._send_servo_command(0.0, z_target_m)
                self._sleep(self.config.scan_poll_s)
                continue

            try:
                try:
                    geometry = self.decoder.detect_geometry(frame, decode_content=False)
                except TypeError:
                    # Keep test doubles and older decoders that only accept
                    # the original one-argument API usable.
                    geometry = self.decoder.detect_geometry(frame)
                except AttributeError:
                    # Keep test doubles and older injected decoders usable;
                    # the real QRDecoder always provides detect_geometry().
                    geometry = self.decoder.detect(frame)
                aim = self.camera.laser_aim_point(frame)
            except Exception:
                self._send_servo_command(0.0, z_target_m)
                return VisionServoResult(
                    False, z_target_m, frames, stable, reason="vision_exception"
                )
            last_frame = frame
            if frames == 1 and self.vision_debug is not None:
                self.vision_debug.capture_scan(
                    frame,
                    {
                        "state": InventoryState.VISUAL_SERVO.value,
                        "capture": "servo_first",
                        "waypoint_index": index,
                        "slot_label": waypoint.slot_label,
                        "position": list(position),
                        "timestamp": time.time(),
                    },
                )
            if geometry is None:
                if frames >= config.min_lost_frames and self._clock() - last_seen > config.lost_timeout_s:
                    self._capture_servo_failure(last_frame, index, waypoint, "target_lost")
                    return VisionServoResult(
                        False, z_target_m, frames, stable, reason="target_lost"
                    )
                stable = 0
                self._send_servo_command(0.0, z_target_m)
            else:
                last_seen = self._clock()
                center = geometry.center
                error_x = center[0] - aim[0]
                error_y = center[1] - aim[1]
                if last_center is not None:
                    center_jump = math.hypot(
                        center[0] - last_center[0], center[1] - last_center[1]
                    )
                    if center_jump > config.max_center_jump_px:
                        # A second QR code or a rail artefact appeared. Hold
                        # position and wait for the previously tracked target;
                        # never steer toward an unbounded candidate jump.
                        stable = 0
                        self._send_servo_command(0.0, z_target_m)
                        self._sleep(self.config.scan_poll_s)
                        continue
                last_center = center
                last_error = (error_x, error_y)
                _cur_pos = self._current_position()

                centered = (
                    abs(error_x) <= config.center_tolerance_px
                    and abs(error_y) <= config.center_tolerance_px
                )
                if centered:
                    stable += 1
                else:
                    stable = 0
                x_cmd, z_target_m = servo_command(
                    config,
                    error_x,
                    error_y,
                    base_z_m,
                    z_target_m,
                    face_sign,
                )
                # 复用已读取的 _cur_pos，避免再次串口读取（Fix 中风险1）
                if start_x is not None and _cur_pos is not None:
                    if abs(_cur_pos[0] - start_x) > config.max_lateral_adjust_m:
                        return VisionServoResult(
                            False, z_target_m, frames, stable,
                            error_x, error_y, "lateral_limit"
                        )
                self._send_servo_command(0.0 if centered else x_cmd, z_target_m)

                # 每隔 _debug_interval 秒存一张伺服过程帧，移到指令下发后避免 I/O 阻塞（Fix 中风险2）
                _now_t = self._clock()
                if self.vision_debug is not None and (_now_t - last_debug_capture_t) >= _debug_interval:
                    last_debug_capture_t = _now_t
                    self.vision_debug.capture_scan(
                        frame,
                        {
                            "state": InventoryState.VISUAL_SERVO.value,
                            "capture": "servo_periodic",
                            "waypoint_index": index,
                            "slot_label": waypoint.slot_label,
                            "position": list(_cur_pos) if _cur_pos is not None else None,
                            "error_px": [round(error_x, 1), round(error_y, 1)],
                            "geometry_number": geometry.number,
                            "stable": stable,
                            "frame": frames,
                            "timestamp": time.time(),
                        },
                    )
                self.state_machine.sample(
                    waypoint_index=index,
                    slot_label=waypoint.slot_label,
                    position=list(_cur_pos) if _cur_pos is not None else None,
                    visual_servo_error_px=[round(error_x, 1), round(error_y, 1)],
                    visual_servo_centered=centered,
                    visual_servo_target_number=geometry.number,
                )
                if stable >= config.stable_frames:
                    if self.vision_debug is not None and last_frame is not None:
                        self.vision_debug.capture_scan(
                            last_frame,
                            {
                                "state": InventoryState.VISUAL_SERVO.value,
                                "capture": "servo_centered",
                                "waypoint_index": index,
                                "slot_label": waypoint.slot_label,
                                "error_px": [error_x, error_y],
                                "timestamp": time.time(),
                            },
                        )
                    self._send_servo_command(0.0, z_target_m)
                    return VisionServoResult(
                        True, z_target_m, frames, stable, error_x, error_y, "centered"
                    )
            self._sleep(self.config.scan_poll_s)

        return VisionServoResult(
            False, z_target_m, frames, stable,
            last_error[0], last_error[1], "timeout"
        )

    def _capture_servo_failure(self, frame, index, waypoint, reason):
        if self.vision_debug is None or frame is None:
            return
        self.vision_debug.capture_scan(
            frame,
            {
                "state": InventoryState.VISUAL_SERVO.value,
                "capture": "servo_" + reason,
                "waypoint_index": index,
                "slot_label": waypoint.slot_label,
                "timestamp": time.time(),
            },
        )

    def _current_position(self):
        realsense = getattr(self.driver, "realsense", None)
        if realsense is None:
            return None
        try:
            return tuple(float(value) for value in realsense.get_position())
        except Exception:
            return None

    def _send_servo_command(self, x_cmd, z_target_m):
        if self.driver is None:
            return
        yaw_cmd = getattr(getattr(self.driver, "_heading_status", None), "command_dps", 0)
        self.driver.set_speed(
            int(round(x_cmd)),
            0,
            int(round(yaw_cmd)),
            int(round(z_target_m * 100)),
        )

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
        camera_auto_exposure=config.camera_auto_exposure,
        camera_exposure=config.camera_exposure,
        camera_gain=config.camera_gain,
        camera_auto_focus=config.camera_auto_focus,
        camera_focus=config.camera_focus,
        camera_zoom=config.camera_zoom,
        camera_warmup_frames=config.camera_warmup_frames,
        qr_mapping_file=str(mapping),
        scan_timeout_s=config.scan_timeout_s,
        scan_poll_s=config.scan_poll_s,
        laser_aim_x_ratio=config.laser_aim_x_ratio,
        laser_aim_y_ratio=config.laser_aim_y_ratio,
        vision_servo=config.vision_servo,
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
