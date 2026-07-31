"""任务二核心与 basic 飞控安全状态机的飞行适配类。

C 点之前复用任务一的路径跟随+视觉对中+WorldDeckHeightController；
C 点之后切换到 PlatformTracker（T265 坐标系伴飞估计）+
FormationController（水平速度）+ DynamicLandingController（垂直速度）。
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from Lcode.Logger import logger  # noqa: E402
from Mission_GPT import laser_height_valid, mission  # noqa: E402

from .control.formation_controller import (  # noqa: E402
    FormationConfig,
    FormationController,
)
from .dynamic_landing import (  # noqa: E402
    DynamicLandingController,
    LandingConfig,
    LandingInput,
    LandingState,
)
from .payload_actuator import PayloadActuator  # noqa: E402
from .task1_runtime import (  # noqa: E402
    HeightReference,
    Task1T265SafetyMonitor,
    WorldDeckHeightController,
    observation_to_gate_sample,
)
from .task2_mission import (  # noqa: E402
    B_PRE,
    C,
    H,
    Task2Config,
    Task2Input,
    Task2MissionDirector,
    Task2Phase,
)
from .task2_runtime import (  # noqa: E402
    LaserContactConfig,
    LaserContactDetector,
    OffsetCalibrator,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402
from .vision.platform_observation import FeatureFlag  # noqa: E402
from .vision.platform_tracker import (  # noqa: E402
    PlatformEstimate,
    PlatformTracker,
    TrackerConfig,
)


class Task2FlightMission(mission):
    """飞控循环内唯一写入 XY 速度的任务二适配器。"""

    def __init__(
        self,
        re_fc,
        se_fc,
        realsense_obj,
        serial_fc_ref,
        *,
        vision_reader: CyberCamReader,
        payload_actuator: PayloadActuator,
        task_config: Task2Config | None = None,
        tracker_config: TrackerConfig | None = None,
        formation_config: FormationConfig | None = None,
        landing_config: LandingConfig | None = None,
        vision_config: dict | None = None,
        car_speed_provider: Callable[[], float | None] | None = None,
        car_position_provider: Callable[[], tuple[float, float] | None] | None = None,
        car_velocity_provider: Callable[[], tuple[float, float] | None] | None = None,
        height_source: str = "t265",
        preflight_warning_completed: bool = False,
        preflight_warning_started_at: float | None = None,
    ) -> None:
        super().__init__(
            re_fc,
            se_fc,
            realsense_obj,
            serial_fc_ref,
            interactive_preflight=False,
            preflight_warning_completed=preflight_warning_completed,
            preflight_warning_started_at=preflight_warning_started_at,
        )
        self.director = Task2MissionDirector(task_config)
        self.vision_reader = vision_reader
        self.payload_actuator = payload_actuator
        self.height_controller = WorldDeckHeightController()
        self.t265_safety = Task1T265SafetyMonitor()
        self.platform_tracker = PlatformTracker(tracker_config)
        self.formation_controller = FormationController(formation_config)
        self.dynamic_landing = DynamicLandingController(landing_config)
        self.offset_calibrator = OffsetCalibrator()
        self.laser_contact = LaserContactDetector(
            LaserContactConfig(
                contact_height_m=self.dynamic_landing.config.contact_height_m
            )
        )
        self.car_speed_provider = car_speed_provider
        self.car_position_provider = car_position_provider
        self.car_velocity_provider = car_velocity_provider
        if height_source not in ("t265", "laser"):
            raise ValueError("height_source must be 't265' or 'laser'")
        self.height_source = height_source
        self.vision_config = {
            "max_age_s": 0.15,
            "min_quality": self.director.config.vision_min_quality,
            "image_center_px": (320.0, 240.0),
            "focal_px": (570.0, 570.0),
            **(vision_config or {}),
        }
        self._last_task2_phase = self.director.phase
        self._last_runtime_log = 0.0
        self._last_nav_time: float | None = None
        self._tracker_active_prev = False
        # 供 telemetry 提取
        self.last_landing_state = None
        self.last_world_position = (0.0, 0.0, 0.0)
        # 上一帧的降落反馈（给下一帧的 director 用）
        self._landing_gate_passed = False
        self._touchdown_confirmed = False
        self._deck_ride_complete = False
        self._landing_aborted = False

    def navigate(self, _laser_position, yaw):
        now = time.monotonic()
        try:
            world_position = tuple(
                float(v) for v in self.realsense.get_position()
            )
            velocity = tuple(
                float(v) for v in self.realsense.get_velocity()
            )
            confidence = int(self.realsense.get_tracking_confidence())
        except Exception as exc:
            logger.error(f"任务二读取T265失败，本周期悬停: {exc}")
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return

        laser_height = self._laser_height()
        if confidence == 0:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            logger.warning("任务二T265置信度为0，冻结状态机并原地悬停")
            return

        phase_before_tick = self.director.phase
        hold_anchor = self._hold_anchor_for_phase(phase_before_tick)
        ground_reference_expected = phase_before_tick in (
            Task2Phase.WAIT_START,
            Task2Phase.TAKEOFF,
            Task2Phase.HOLD_3S,
            Task2Phase.INTERCEPT_B_PRE,
            Task2Phase.ACQUIRE_TARGET,
            Task2Phase.SYNC_TARGET_AT_C,
            Task2Phase.ACTIVATE_TRACKER,
            Task2Phase.LAND_H,
        )
        safety_fault = self.t265_safety.update(
            timestamp=now,
            world_position_xyz_m=world_position,
            laser_height_m=laser_height,
            ground_reference_expected=ground_reference_expected,
            hold_anchor_xy_m=hold_anchor,
        )
        if safety_fault is not None:
            if laser_height is not None:
                self._ramp_z_cm = laser_height * 100.0
            self.set_speed(0, 0, 0, int(round(self._ramp_z_cm)))
            logger.error(
                f"任务二T265安全保护触发: {safety_fault}；转HOVER_WAIT"
            )
            self.state = "HOVER_WAIT"
            return

        observation = self.vision_reader.latest(
            now, float(self.vision_config["max_age_s"])
        )
        relative_height = (
            laser_height
            if laser_height is not None
            else max(0.01, world_position[2])
        )
        gate = observation_to_gate_sample(
            observation,
            now=now,
            relative_height_m=relative_height,
            max_age_s=float(self.vision_config["max_age_s"]),
            min_quality=int(self.vision_config["min_quality"]),
            image_center_px=tuple(self.vision_config["image_center_px"]),
            focal_px=tuple(self.vision_config["focal_px"]),
        )

        car_speed = (
            None if self.car_speed_provider is None else self.car_speed_provider()
        )
        car_position = (
            None
            if self.car_position_provider is None
            else self.car_position_provider()
        )
        car_velocity = (
            None
            if self.car_velocity_provider is None
            else self.car_velocity_provider()
        )

        # offset 标定：C 点前持续收集样本
        offset_ready = self.offset_calibrator.ready
        offset = self.offset_calibrator.offset() if offset_ready else (0.0, 0.0)
        if phase_before_tick in (
            Task2Phase.TAKEOFF,
            Task2Phase.HOLD_3S,
            Task2Phase.INTERCEPT_B_PRE,
            Task2Phase.ACQUIRE_TARGET,
            Task2Phase.FOLLOW_B_C,
        ):
            if car_position is not None and confidence >= 2:
                self.offset_calibrator.record_sample(
                    car_position,
                    (world_position[0], world_position[1]),
                )
                offset_ready = self.offset_calibrator.ready
                offset = (
                    self.offset_calibrator.offset() if offset_ready else (0.0, 0.0)
                )

        contact_evidence = self.laser_contact.update(laser_height)

        self.last_world_position = world_position

        control_height = (
            laser_height
            if self.height_source == "laser" and laser_height is not None
            else world_position[2]
        )
        command = self.director.tick(
            Task2Input(
                now=now,
                position_xyz_m=(
                    world_position[0],
                    world_position[1],
                    control_height,
                ),
                velocity_xy_m_s=(velocity[0], velocity[1]),
                t265_confidence=confidence,
                car_start=True,
                car_speed_m_s=car_speed,
                vision_seq=gate.seq,
                vision_found=gate.found,
                vision_quality=gate.quality,
                vision_ambiguous=gate.ambiguous,
                vision_error_xy_m=gate.error_xy_m,
                deck_relative_height_m=laser_height,
                car_position_xy_m=car_position,
                car_velocity_xy_m_s=car_velocity,
                offset_ready=offset_ready,
                landing_gate_passed=self._landing_gate_passed,
                touchdown_confirmed=self._touchdown_confirmed,
                deck_ride_complete=self._deck_ride_complete,
                landing_aborted=self._landing_aborted,
            )
        )

        vx = command.vx_m_s
        vy = command.vy_m_s
        landing_vz = 0.0
        estimate: PlatformEstimate | None = None

        if command.tracker_active:
            if not self._tracker_active_prev:
                self.platform_tracker.reset()
                self.formation_controller.reset(
                    timestamp=now,
                    velocity=(velocity[0], velocity[1]),
                )
                self.dynamic_landing.reset()
                self._landing_gate_passed = False
                self._touchdown_confirmed = False
                self._deck_ride_complete = False
                self._landing_aborted = False
                logger.info(
                    "任务二激活 PlatformTracker + FormationController + "
                    "DynamicLandingController"
                )

            # 小车坐标 + offset 变换到无人机 T265 系
            if car_position is not None and offset_ready:
                car_in_uav = (
                    car_position[0] - offset[0],
                    car_position[1] - offset[1],
                )
                estimate = self.platform_tracker.update(
                    car_in_uav[0], car_in_uav[1], now
                )
            else:
                estimate = self.platform_tracker.predict(now)

            if command.landing_active and estimate is not None:
                landing_cmd = self._run_landing(
                    estimate,
                    world_position,
                    velocity,
                    laser_height,
                    gate,
                    observation,
                    contact_evidence,
                    safety_fault,
                    car_velocity,
                )
                if landing_cmd is not None:
                    self._landing_gate_passed = (
                        landing_cmd.state != LandingState.LANDING_GATE
                    )
                    self._touchdown_confirmed = landing_cmd.touchdown_confirmed
                    self._deck_ride_complete = (
                        landing_cmd.state == LandingState.RETAKEOFF_GATE
                    )
                    self._landing_aborted = (
                        landing_cmd.state == LandingState.CONTROLLED_ABORT
                    )
                    if command.phase == Task2Phase.DYNAMIC_LANDING:
                        landing_vz = landing_cmd.vertical_speed_m_s
                        formation_cmd = self.formation_controller.command(
                            estimate,
                            (world_position[0], world_position[1]),
                            (velocity[0], velocity[1]),
                            now,
                        )
                        if formation_cmd.valid:
                            vx = formation_cmd.vx_m_s
                            vy = formation_cmd.vy_m_s
        else:
            if self._tracker_active_prev:
                self._landing_gate_passed = False
                self._touchdown_confirmed = False
                self._deck_ride_complete = False
                self._landing_aborted = False

        self._tracker_active_prev = command.tracker_active

        self.last_landing_state = (
            self.dynamic_landing.state
            if command.landing_active
            else None
        )

        # 高度控制
        if (
            command.phase == Task2Phase.DYNAMIC_LANDING
            and command.landing_active
        ):
            if self._last_nav_time is not None:
                dt = min(max(now - self._last_nav_time, 0.001), 0.05)
            else:
                dt = 0.02
            # vertical_speed_m_s 正为爬升、负为下降
            self._ramp_z_cm -= landing_vz * 100.0 * dt
            self._ramp_z_cm = max(5.0, min(self._ramp_z_cm, 200.0))
            height = HeightReference(
                self._ramp_z_cm / 100.0, True, "landing_vz", "ok"
            )
        elif self.height_source == "laser":
            target_laser_height = (
                command.target_world_height_m
                if command.target_deck_height_m is None
                else command.target_deck_height_m
            )
            if laser_height is None:
                height = HeightReference(
                    self._ramp_z_cm / 100.0,
                    False,
                    "laser_height",
                    "laser_height_unavailable",
                )
            else:
                self._step_ramp_z(target_laser_height * 100.0)
                height = HeightReference(
                    self._ramp_z_cm / 100.0, True, "laser_height", "ok"
                )
        else:
            height = self.height_controller.command(
                timestamp=now,
                current_world_height_m=world_position[2],
                current_laser_height_m=laser_height,
                target_world_height_m=command.target_world_height_m,
                target_deck_height_m=command.target_deck_height_m,
            )
            if height.valid:
                self._ramp_z_cm = height.laser_setpoint_m * 100.0

        self._last_nav_time = now

        self._heading_status = self._update_heading_hold(yaw, confidence)
        yaw_cmd = self._heading_status.command_dps
        vx_cms = int(round(vx * 100.0))
        vy_cms = int(round(vy * 100.0))
        self.set_speed(vx_cms, vy_cms, yaw_cmd, int(round(self._ramp_z_cm)))

        if command.phase != self._last_task2_phase:
            logger.info(
                f"任务二状态: {self._last_task2_phase.value} -> "
                f"{command.phase.value} ({command.reason})"
            )
            self._last_task2_phase = command.phase

        self._log_runtime(
            now,
            command,
            world_position,
            velocity,
            confidence,
            laser_height,
            height,
            gate,
            car_position,
            car_velocity,
            offset_ready,
            offset,
            estimate,
            landing_vz,
        )

        if (
            command.phase == Task2Phase.LAND_H
            and command.land_requested
            and math.hypot(world_position[0], world_position[1])
            <= self.director.config.point_arrival_radius_m
        ):
            logger.info("任务二到达H点0.15m末航点，转入basic两级降落")
            self.state = "DESCEND"

    @staticmethod
    def _hold_anchor_for_phase(phase: Task2Phase):
        if phase in (Task2Phase.WAIT_START, Task2Phase.TAKEOFF, Task2Phase.HOLD_3S):
            return H
        if phase == Task2Phase.ACQUIRE_TARGET:
            return B_PRE
        if phase in (Task2Phase.SYNC_TARGET_AT_C, Task2Phase.ACTIVATE_TRACKER):
            return C
        return None

    def _run_landing(
        self,
        estimate: PlatformEstimate,
        world_position,
        velocity,
        laser_height,
        gate,
        observation,
        contact_evidence: bool,
        safety_fault,
        car_velocity,
    ):
        """组装 LandingInput 并调 DynamicLandingController.tick()。"""
        relative_height = (
            laser_height
            if laser_height is not None
            else max(0.01, world_position[2])
        )
        relative_vel = (
            estimate.vx_m_s - velocity[0],
            estimate.vy_m_s - velocity[1],
        )
        position_error = (
            estimate.x_m - world_position[0],
            estimate.y_m - world_position[1],
        )
        visual_too_close = False
        if observation is not None:
            flags = FeatureFlag(observation.flags)
            visual_too_close = bool(flags & FeatureFlag.TOO_CLOSE)
        visual_usable = gate.found and not gate.ambiguous
        car_motion_fresh = car_velocity is not None
        # roll/pitch: 后续接入飞控遥测
        roll_deg = 0.0
        pitch_deg = 0.0
        landing_input = LandingInput(
            relative_height_m=relative_height,
            vertical_speed_m_s=float(velocity[2]),
            relative_velocity_xy_m_s=relative_vel,
            position_error_xy_m=position_error,
            estimate_uncertainty_m=estimate.uncertainty_m,
            visual_usable=visual_usable,
            visual_too_close=visual_too_close,
            car_motion_fresh=car_motion_fresh,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            contact_evidence=contact_evidence,
            t265_healthy=safety_fault is None,
        )
        return self.dynamic_landing.tick(landing_input)

    def _laser_height(self) -> float | None:
        if self.serial_fc_ref is None:
            return None
        value = float(self.serial_fc_ref._last_laser_height_cm)
        return value if laser_height_valid(value) else None

    def _log_runtime(
        self,
        now,
        command,
        world_position,
        velocity,
        confidence,
        laser_height,
        height,
        gate,
        car_position,
        car_velocity,
        offset_ready,
        offset,
        estimate,
        landing_vz,
    ) -> None:
        if self._log_file is None or now - self._last_runtime_log < 0.10:
            return
        self._last_runtime_log = now
        try:
            with self._log_lock:
                self._log_file.write(
                    json.dumps(
                        {
                            "event": "task2_runtime",
                            "t": round(time.time(), 3),
                            "phase": command.phase.value,
                            "reason": command.reason,
                            "world_pos": [round(v, 4) for v in world_position],
                            "t265_velocity": [round(v, 4) for v in velocity[:2]],
                            "t265_confidence": confidence,
                            "laser_height_m": laser_height,
                            "laser_setpoint_m": round(
                                height.laser_setpoint_m, 4
                            ),
                            "height_mode": height.mode,
                            "target_xy_m": [
                                round(command.target_xy_m[0], 4),
                                round(command.target_xy_m[1], 4),
                            ],
                            "command_xy_m_s": [
                                round(command.vx_m_s, 4),
                                round(command.vy_m_s, 4),
                            ],
                            "vision_seq": gate.seq,
                            "vision_found": gate.found,
                            "vision_quality": gate.quality,
                            "vision_error_xy_m": gate.error_xy_m,
                            "vision_reason": gate.reason,
                            "car_position_xy_m": (
                                [round(car_position[0], 4), round(car_position[1], 4)]
                                if car_position
                                else None
                            ),
                            "car_velocity_xy_m_s": (
                                [round(car_velocity[0], 4), round(car_velocity[1], 4)]
                                if car_velocity
                                else None
                            ),
                            "offset_ready": offset_ready,
                            "offset_xy_m": [
                                round(offset[0], 4),
                                round(offset[1], 4),
                            ],
                            "tracker_initialized": self.platform_tracker.initialized,
                            "platform_estimate": (
                                {
                                    "x_m": round(estimate.x_m, 4),
                                    "y_m": round(estimate.y_m, 4),
                                    "vx_m_s": round(estimate.vx_m_s, 4),
                                    "vy_m_s": round(estimate.vy_m_s, 4),
                                    "uncertainty_m": round(
                                        estimate.uncertainty_m, 4
                                    ),
                                    "predicted": estimate.predicted,
                                }
                                if estimate
                                else None
                            ),
                            "landing_state": (
                                self.dynamic_landing.state.value
                                if command.landing_active
                                else None
                            ),
                            "landing_vz_m_s": round(landing_vz, 4),
                            "landing_gate_passed": self._landing_gate_passed,
                            "touchdown_confirmed": self._touchdown_confirmed,
                            "deck_ride_complete": self._deck_ride_complete,
                            "landing_aborted": self._landing_aborted,
                            "tracker_active": command.tracker_active,
                            "landing_active": command.landing_active,
                            "mission_success": command.mission_success,
                            "horizontal_control_source": (
                                "formation_controller"
                                if command.phase
                                == Task2Phase.DYNAMIC_LANDING
                                and command.tracker_active
                                else "task2_director"
                            ),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                self._log_file.flush()
        except Exception:
            pass
