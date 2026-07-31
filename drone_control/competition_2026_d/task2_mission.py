"""D题任务二的纯任务状态机。

本模块不直接访问串口、GPIO或飞控，也不持有 PlatformTracker、
FormationController 或 DynamicLandingController。飞行适配层每个控制周期
提供 T265、激光、视觉、小车坐标和动态降落子状态机的反馈，再执行返回的
水平速度、高度目标与起飞/降落请求。

任务二与任务一的区别：C 点之前复用任务一的路径跟随+视觉对中逻辑；
在 SYNC_TARGET_AT_C 完成后不进入投放，而是激活 T265 坐标系下的精确
伴飞与动态降落。
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

from .control.task1_path_controller import (
    PathCommand,
    PolylinePath,
    Task1PathFollower,
    Task1PathFollowerConfig,
)
from .payload_actuator import ActuatorState


# 场地航点（与任务一一致）
H = (0.0, 0.0)
B_PRE = (0.375, 1.75)
B = (0.375, 2.25)
BC_1 = (0.595, 2.78)
BC_2 = (1.125, 3.00)
BC_3 = (1.655, 2.78)
C = (1.875, 2.25)
D = (1.875, 0.75)


class Task2Phase(enum.Enum):
    WAIT_START = "wait_start"
    TAKEOFF = "takeoff"
    HOLD_3S = "hold_3s"
    INTERCEPT_B_PRE = "intercept_b_pre"
    ACQUIRE_TARGET = "acquire_target"
    FOLLOW_B_C = "follow_b_c"
    SYNC_TARGET_AT_C = "sync_target_at_c"
    ACTIVATE_TRACKER = "activate_tracker"
    DYNAMIC_LANDING = "dynamic_landing"
    RETAKEOFF = "retakeoff"
    CLIMB_150CM = "climb_150cm"
    RETURN_H = "return_h"
    LAND_H = "land_h"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Task2Config:
    # 路径与巡航参数（与任务一一致）
    cruise_height_m: float = 1.50
    follow_height_m: float = 1.00
    final_height_m: float = 0.15
    hold_duration_s: float = 3.0
    intercept_speed_m_s: float = 0.20
    car_speed_m_s: float = 0.13
    curve_speed_m_s: float | None = None
    car_speed_scale: float = 1.0
    return_speed_m_s: float = 0.35
    point_kp: float = 0.90
    point_arrival_radius_m: float = 0.14
    max_point_speed_m_s: float = 0.40
    hold_position_max_speed_m_s: float = 0.15
    hold_stable_speed_m_s: float = 0.12
    path_lookahead_m: float = 0.20
    path_cross_track_kp: float = 0.80
    path_max_correction_m_s: float = 0.08
    path_max_speed_m_s: float = 0.20
    path_max_accel_m_s2: float = 0.30
    acquire_timeout_s: float = 4.0
    vision_min_quality: int = 55
    vision_confirm_frames: int = 2
    drop_max_error_m: float = 0.12
    t265_min_confidence: int = 2
    path_only_b_pre_descent: bool = False
    c_sync_vision_enabled: bool = True
    # 任务二特有参数
    retakeoff_height_m: float = 1.50
    abort_climb_height_m: float = 1.50
    activate_tracker_timeout_s: float = 3.0
    require_car_position_at_c: bool = True


@dataclass(frozen=True)
class Task2Input:
    now: float
    position_xyz_m: tuple[float, float, float]
    velocity_xy_m_s: tuple[float, float] = (0.0, 0.0)
    t265_confidence: int = 3
    car_start: bool = False
    car_speed_m_s: float | None = None
    vision_seq: int | None = None
    vision_found: bool = False
    vision_quality: int = 0
    vision_ambiguous: bool = False
    vision_error_xy_m: tuple[float, float] | None = None
    deck_relative_height_m: float | None = None
    payload_state: ActuatorState = ActuatorState.LOCKED
    landed: bool = False
    # 任务二特有
    car_position_xy_m: tuple[float, float] | None = None
    car_velocity_xy_m_s: tuple[float, float] | None = None
    offset_ready: bool = False
    # 动态降落子状态机反馈（由飞行适配层提取）
    landing_gate_passed: bool = False
    touchdown_confirmed: bool = False
    deck_ride_complete: bool = False
    landing_aborted: bool = False


@dataclass(frozen=True)
class Task2Command:
    phase: Task2Phase
    target_xy_m: tuple[float, float]
    target_world_height_m: float
    target_deck_height_m: float | None
    vx_m_s: float
    vy_m_s: float
    takeoff_requested: bool
    land_requested: bool
    tracker_active: bool
    landing_active: bool
    fixed_heading: bool
    mission_success: bool
    reason: str


class Task2MissionDirector:
    """任务二决策源。

    C 点之前复用任务一的路径跟随与视觉对中逻辑；C 点之后不投放，而是
    激活 T265 坐标系下的精确伴飞与动态降落。本类不直接持有控制模块，
    通过 tracker_active/landing_active 标志通知飞行适配层切换控制源。
    """

    def __init__(self, config: Task2Config | None = None) -> None:
        self.config = config or Task2Config()
        path_cfg = Task1PathFollowerConfig(
            lookahead_m=self.config.path_lookahead_m,
            cross_track_kp=self.config.path_cross_track_kp,
            max_correction_m_s=self.config.path_max_correction_m_s,
            max_speed_m_s=self.config.path_max_speed_m_s,
            max_accel_m_s2=self.config.path_max_accel_m_s2,
            completion_radius_m=self.config.point_arrival_radius_m,
        )
        self.bc_follower = Task1PathFollower(
            PolylinePath((B_PRE, B, BC_1, BC_2, BC_3, C)), path_cfg
        )
        self.phase = Task2Phase.WAIT_START
        self._phase_since = 0.0
        self._stable_since: float | None = None
        self._last_vision_seq: int | None = None
        self._vision_confirm_count = 0
        self.target_acquired = False
        self.mission_success = False
        self.reason = "waiting_for_car_start"
        self._climb_anchor = H

    def tick(self, data: Task2Input) -> Task2Command:
        cfg = self.config
        x, y, z_world = data.position_xyz_m
        position_xy = (x, y)
        raw_car_speed = (
            cfg.car_speed_m_s
            if data.car_speed_m_s is None
            else max(0.0, float(data.car_speed_m_s))
        )
        car_speed = raw_car_speed * max(0.0, cfg.car_speed_scale)
        curve_speed = (
            car_speed
            if cfg.curve_speed_m_s is None
            else max(0.0, cfg.curve_speed_m_s) * max(0.0, cfg.car_speed_scale)
        )
        target_xy = position_xy
        target_world_height = cfg.cruise_height_m
        target_deck_height: float | None = None
        vx = vy = 0.0
        takeoff_requested = False
        land_requested = False
        tracker_active = False
        landing_active = False

        if self.phase == Task2Phase.WAIT_START:
            target_xy = H
            if data.car_start and data.t265_confidence >= cfg.t265_min_confidence:
                self._transition(Task2Phase.TAKEOFF, data.now, "car_start_accepted")

        elif self.phase == Task2Phase.TAKEOFF:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            takeoff_requested = True
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(
                    Task2Phase.HOLD_3S, data.now, "takeoff_height_reached"
                )
                self._stable_since = None

        elif self.phase == Task2Phase.HOLD_3S:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            hold_elapsed = data.now - self._phase_since
            height_safe = abs(z_world - cfg.cruise_height_m) <= 0.15
            if hold_elapsed >= cfg.hold_duration_s and height_safe:
                self._transition(
                    Task2Phase.INTERCEPT_B_PRE, data.now, "hover_3s_complete"
                )

        elif self.phase == Task2Phase.INTERCEPT_B_PRE:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.intercept_speed_m_s
            )
            if self._near(position_xy, B_PRE) and math.hypot(
                *data.velocity_xy_m_s
            ) <= cfg.hold_stable_speed_m_s:
                self._transition(
                    Task2Phase.ACQUIRE_TARGET, data.now, "b_pre_reached"
                )

        elif self.phase == Task2Phase.ACQUIRE_TARGET:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.hold_position_max_speed_m_s
            )
            if cfg.path_only_b_pre_descent:
                target_world_height = cfg.follow_height_m
                if abs(z_world - cfg.follow_height_m) <= 0.10:
                    self.target_acquired = True
                    self.bc_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task2Phase.FOLLOW_B_C,
                        data.now,
                        "path_test_b_pre_descent_complete",
                    )
            elif self._confirm_vision(data, require_centered=True):
                self.target_acquired = True
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task2Phase.FOLLOW_B_C, data.now, "target_acquired_at_b_pre"
                )
            elif data.now - self._phase_since >= cfg.acquire_timeout_s:
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task2Phase.FOLLOW_B_C,
                    data.now,
                    "b_pre_timeout_path_fallback",
                )

        elif self.phase == Task2Phase.FOLLOW_B_C:
            if not self.target_acquired and self._confirm_vision(
                data, require_centered=False
            ):
                self.target_acquired = True
                self.reason = "target_acquired_on_b_c"
            target_world_height = (
                cfg.follow_height_m if self.target_acquired else cfg.cruise_height_m
            )
            path = self.bc_follower.command(
                position_xy,
                nominal_speed_m_s=curve_speed,
                timestamp=data.now,
            )
            target_xy, vx, vy = self._from_path(path)
            if path.completed:
                if cfg.c_sync_vision_enabled:
                    self._transition(
                        Task2Phase.SYNC_TARGET_AT_C,
                        data.now,
                        "c_reached_waiting_for_centered_target",
                    )
                else:
                    self._transition(
                        Task2Phase.SYNC_TARGET_AT_C, data.now, "c_reached"
                    )

        elif self.phase == Task2Phase.SYNC_TARGET_AT_C:
            target_xy = C
            target_world_height = cfg.follow_height_m
            vx, vy = self._point_velocity(
                position_xy, C, cfg.hold_position_max_speed_m_s
            )
            ready = True
            if cfg.c_sync_vision_enabled and not self._confirm_vision(
                data, require_centered=True
            ):
                ready = False
            if cfg.require_car_position_at_c and not (
                data.offset_ready and data.car_position_xy_m is not None
            ):
                ready = False
            if ready:
                self._transition(
                    Task2Phase.ACTIVATE_TRACKER, data.now, "c_sync_ready"
                )

        elif self.phase == Task2Phase.ACTIVATE_TRACKER:
            # 激活 PlatformTracker 和 DynamicLandingController；水平控制仍由
            # 点位控制保持悬停在 C 点（vz 不使用），待 LANDING_GATE 通过后
            # 切换到 FormationController
            tracker_active = True
            landing_active = True
            target_xy = C
            target_world_height = cfg.follow_height_m
            vx, vy = self._point_velocity(
                position_xy, C, cfg.hold_position_max_speed_m_s
            )
            if data.landing_aborted:
                self._begin_climb(data.now, position_xy, "landing_aborted_at_gate")
            elif data.landing_gate_passed:
                self._transition(
                    Task2Phase.DYNAMIC_LANDING, data.now, "landing_gate_passed"
                )
            elif data.now - self._phase_since >= cfg.activate_tracker_timeout_s:
                self._begin_climb(data.now, position_xy, "activate_tracker_timeout")

        elif self.phase == Task2Phase.DYNAMIC_LANDING:
            # 委托给 DynamicLandingController，水平控制交给 FormationController
            tracker_active = True
            landing_active = True
            target_xy = position_xy
            target_world_height = cfg.follow_height_m
            if data.landing_aborted:
                self._begin_climb(data.now, position_xy, "landing_aborted")
            elif data.deck_ride_complete:
                self.mission_success = True
                self._transition(
                    Task2Phase.RETAKEOFF, data.now, "deck_ride_complete"
                )

        elif self.phase == Task2Phase.RETAKEOFF:
            tracker_active = False
            landing_active = False
            target_xy = position_xy
            target_world_height = cfg.retakeoff_height_m
            if z_world >= cfg.retakeoff_height_m - 0.10:
                self._transition(
                    Task2Phase.RETURN_H, data.now, "retakeoff_height_reached"
                )

        elif self.phase == Task2Phase.CLIMB_150CM:
            target_xy = self._climb_anchor
            vx, vy = self._point_velocity(
                position_xy,
                self._climb_anchor,
                cfg.hold_position_max_speed_m_s,
            )
            if z_world >= cfg.abort_climb_height_m - 0.10:
                self._transition(Task2Phase.RETURN_H, data.now, self.reason)

        elif self.phase == Task2Phase.RETURN_H:
            target_xy = H
            vx, vy = self._point_velocity(position_xy, H, cfg.return_speed_m_s)
            if self._near(position_xy, H):
                self._transition(Task2Phase.LAND_H, data.now, "h_reached")

        elif self.phase == Task2Phase.LAND_H:
            target_xy = H
            target_world_height = cfg.final_height_m
            vx, vy = self._point_velocity(position_xy, H, 0.10)
            land_requested = z_world <= cfg.final_height_m + 0.05
            if data.landed:
                self._transition(Task2Phase.COMPLETE, data.now, "landed_at_h")

        elif self.phase == Task2Phase.COMPLETE:
            target_xy = H
            target_world_height = cfg.final_height_m

        return Task2Command(
            phase=self.phase,
            target_xy_m=target_xy,
            target_world_height_m=target_world_height,
            target_deck_height_m=target_deck_height,
            vx_m_s=vx,
            vy_m_s=vy,
            takeoff_requested=takeoff_requested,
            land_requested=land_requested,
            tracker_active=tracker_active,
            landing_active=landing_active,
            fixed_heading=True,
            mission_success=self.mission_success,
            reason=self.reason,
        )

    def _confirm_vision(
        self, data: Task2Input, *, require_centered: bool
    ) -> bool:
        valid = (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= self.config.vision_min_quality
        )
        if require_centered:
            valid = (
                valid
                and data.vision_error_xy_m is not None
                and math.hypot(*data.vision_error_xy_m)
                <= self.config.drop_max_error_m
            )
        if not valid:
            self._vision_confirm_count = 0
            self._last_vision_seq = None
            return False
        if data.vision_seq != self._last_vision_seq:
            self._last_vision_seq = data.vision_seq
            self._vision_confirm_count += 1
        return self._vision_confirm_count >= self.config.vision_confirm_frames

    def _point_velocity(
        self,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        speed_limit: float,
    ) -> tuple[float, float]:
        dx = target_xy[0] - position_xy[0]
        dy = target_xy[1] - position_xy[1]
        vx = self.config.point_kp * dx
        vy = self.config.point_kp * dy
        return _limit_norm(
            vx,
            vy,
            min(self.config.max_point_speed_m_s, max(0.0, speed_limit)),
        )

    def _near(
        self, position_xy: tuple[float, float], target_xy: tuple[float, float]
    ) -> bool:
        return math.hypot(
            position_xy[0] - target_xy[0],
            position_xy[1] - target_xy[1],
        ) <= self.config.point_arrival_radius_m

    @staticmethod
    def _from_path(
        path: PathCommand,
    ) -> tuple[tuple[float, float], float, float]:
        return path.target_xy_m, path.vx_m_s, path.vy_m_s

    def _begin_climb(
        self,
        now: float,
        position_xy: tuple[float, float],
        reason: str,
    ) -> None:
        self._climb_anchor = position_xy
        self._transition(Task2Phase.CLIMB_150CM, now, reason)

    def _transition(
        self, phase: Task2Phase, now: float, reason: str
    ) -> None:
        self.phase = phase
        self._phase_since = float(now)
        self.reason = reason
        self._stable_since = None
        self._vision_confirm_count = 0
        self._last_vision_seq = None


def _limit_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= limit or norm <= 1e-9:
        return x, y
    scale = limit / norm
    return x * scale, y * scale
