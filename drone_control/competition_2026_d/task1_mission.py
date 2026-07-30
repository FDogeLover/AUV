"""D题任务一的纯任务状态机。

本模块不直接访问串口、GPIO或飞控。飞行适配层每个控制周期提供 T265、激光、
视觉和舵机状态，再执行返回的唯一水平速度、高度目标与一次性投放请求。
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


H = (0.0, 0.0)
B_PRE = (0.375, 1.75)
B = (0.375, 2.25)
BC_1 = (0.595, 2.78)
BC_2 = (1.125, 3.00)
BC_3 = (1.655, 2.78)
C = (1.875, 2.25)
D = (1.875, 0.75)


class Task1Phase(enum.Enum):
    WAIT_START = "wait_start"
    TAKEOFF = "takeoff"
    HOLD_3S = "hold_3s"
    INTERCEPT_B_PRE = "intercept_b_pre"
    ACQUIRE_TARGET = "acquire_target"
    FOLLOW_B_C = "follow_b_c"
    DROP_WINDOW_C_D = "drop_window_c_d"
    DROP_DESCENT = "drop_descent"
    RELEASING = "releasing"
    CLIMB = "climb"
    RETURN_H = "return_h"
    LAND_H = "land_h"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Task1Config:
    cruise_height_m: float = 1.50
    follow_height_m: float = 1.00
    drop_height_m: float = 0.65
    final_height_m: float = 0.15
    hold_duration_s: float = 3.0
    intercept_speed_m_s: float = 0.38
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
    drop_rel_height_tolerance_m: float = 0.05
    drop_descent_speed_m_s: float = 0.09
    drop_time_margin_s: float = 0.80
    release_timeout_s: float = 1.0
    t265_min_confidence: int = 2
    path_only_b_pre_descent: bool = False
    payload_drop_enabled: bool = True


@dataclass(frozen=True)
class Task1Input:
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


@dataclass(frozen=True)
class Task1Command:
    phase: Task1Phase
    target_xy_m: tuple[float, float]
    target_world_height_m: float
    target_deck_height_m: float | None
    vx_m_s: float
    vy_m_s: float
    takeoff_requested: bool
    release_requested: bool
    land_requested: bool
    fixed_heading: bool
    target_acquired: bool
    drop_committed: bool
    drop_released: bool
    mission_success: bool
    reason: str


class Task1MissionDirector:
    """任务一唯一决策源；视觉只改变阶段和投放门禁，不生成水平速度。"""

    def __init__(self, config: Task1Config | None = None) -> None:
        self.config = config or Task1Config()
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
        self.cd_follower = Task1PathFollower(PolylinePath((C, D)), path_cfg)
        self.phase = Task1Phase.WAIT_START
        self._phase_since = 0.0
        self._stable_since: float | None = None
        self._last_vision_seq: int | None = None
        self._vision_confirm_count = 0
        self.target_acquired = False
        self.drop_committed = False
        self.drop_released = False
        self.mission_success = False
        self.reason = "waiting_for_car_start"
        self._climb_anchor = H

    def tick(self, data: Task1Input) -> Task1Command:
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
            else max(0.0, cfg.curve_speed_m_s)
            * max(0.0, cfg.car_speed_scale)
        )
        target_xy = position_xy
        target_world_height = cfg.cruise_height_m
        target_deck_height = None
        vx = vy = 0.0
        takeoff_requested = False
        release_requested = False
        land_requested = False

        if self.phase == Task1Phase.WAIT_START:
            target_xy = H
            if data.car_start and data.t265_confidence >= cfg.t265_min_confidence:
                self._transition(Task1Phase.TAKEOFF, data.now, "car_start_accepted")

        elif self.phase == Task1Phase.TAKEOFF:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            takeoff_requested = True
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(Task1Phase.HOLD_3S, data.now, "takeoff_height_reached")
                self._stable_since = None

        elif self.phase == Task1Phase.HOLD_3S:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            hold_elapsed = data.now - self._phase_since
            height_safe = abs(z_world - cfg.cruise_height_m) <= 0.15
            if hold_elapsed >= cfg.hold_duration_s and height_safe:
                self._transition(
                    Task1Phase.INTERCEPT_B_PRE, data.now, "hover_3s_complete"
                )

        elif self.phase == Task1Phase.INTERCEPT_B_PRE:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.intercept_speed_m_s
            )
            if self._near(position_xy, B_PRE):
                self._transition(
                    Task1Phase.ACQUIRE_TARGET, data.now, "b_pre_reached"
                )

        elif self.phase == Task1Phase.ACQUIRE_TARGET:
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
                        Task1Phase.FOLLOW_B_C,
                        data.now,
                        "path_test_b_pre_descent_complete",
                    )
            elif self._confirm_vision(data, require_centered=False):
                self.target_acquired = True
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task1Phase.FOLLOW_B_C, data.now, "target_acquired_at_b_pre"
                )
            elif data.now - self._phase_since >= cfg.acquire_timeout_s:
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task1Phase.FOLLOW_B_C,
                    data.now,
                    "b_pre_timeout_path_fallback",
                )

        elif self.phase == Task1Phase.FOLLOW_B_C:
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
                self.cd_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task1Phase.DROP_WINDOW_C_D, data.now, "c_reached"
                )

        elif self.phase == Task1Phase.DROP_WINDOW_C_D:
            target_world_height = cfg.follow_height_m
            path = self.cd_follower.command(
                position_xy, nominal_speed_m_s=car_speed, timestamp=data.now
            )
            target_xy, vx, vy = self._from_path(path)
            descent_time = max(
                0.0, cfg.follow_height_m - cfg.drop_height_m
            ) / max(cfg.drop_descent_speed_m_s, 1e-3)
            available_time = path.remaining_m / max(car_speed, 1e-3)
            gate_has_time = available_time >= descent_time + cfg.drop_time_margin_s
            if (
                cfg.payload_drop_enabled
                and self.target_acquired
                and gate_has_time
                and self._confirm_vision(data, require_centered=True)
            ):
                self.drop_committed = True
                self._transition(
                    Task1Phase.DROP_DESCENT, data.now, "drop_gate_latched"
                )
            elif path.completed:
                self._begin_return(
                    data.now, position_xy, "d_reached_without_drop_gate"
                )

        elif self.phase == Task1Phase.DROP_DESCENT:
            target_world_height = cfg.follow_height_m
            target_deck_height = cfg.drop_height_m
            path = self.cd_follower.command(
                position_xy, nominal_speed_m_s=car_speed, timestamp=data.now
            )
            target_xy, vx, vy = self._from_path(path)
            relative_height = data.deck_relative_height_m
            if (
                relative_height is not None
                and abs(relative_height - cfg.drop_height_m)
                <= cfg.drop_rel_height_tolerance_m
            ):
                release_requested = True
                self._transition(
                    Task1Phase.RELEASING, data.now, "drop_height_reached"
                )
            elif path.completed:
                self._begin_return(
                    data.now, position_xy, "d_reached_before_drop_height"
                )

        elif self.phase == Task1Phase.RELEASING:
            target_world_height = cfg.follow_height_m
            target_deck_height = cfg.drop_height_m
            release_requested = True
            path = self.cd_follower.command(
                position_xy, nominal_speed_m_s=car_speed, timestamp=data.now
            )
            target_xy, vx, vy = self._from_path(path)
            if data.payload_state == ActuatorState.RELEASED:
                self.drop_released = True
                self.mission_success = True
                self._begin_return(data.now, position_xy, "payload_released")
            elif data.payload_state == ActuatorState.UNCERTAIN:
                self._begin_return(
                    data.now, position_xy, "payload_release_uncertain"
                )
            elif data.now - self._phase_since >= cfg.release_timeout_s:
                self._begin_return(
                    data.now, position_xy, "payload_release_timeout"
                )

        elif self.phase == Task1Phase.CLIMB:
            target_xy = self._climb_anchor
            vx, vy = self._point_velocity(
                position_xy,
                self._climb_anchor,
                cfg.hold_position_max_speed_m_s,
            )
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(Task1Phase.RETURN_H, data.now, self.reason)

        elif self.phase == Task1Phase.RETURN_H:
            target_xy = H
            vx, vy = self._point_velocity(position_xy, H, cfg.return_speed_m_s)
            if self._near(position_xy, H):
                self._transition(Task1Phase.LAND_H, data.now, "h_reached")

        elif self.phase == Task1Phase.LAND_H:
            target_xy = H
            target_world_height = cfg.final_height_m
            vx, vy = self._point_velocity(position_xy, H, 0.10)
            land_requested = z_world <= cfg.final_height_m + 0.05
            if data.landed:
                self._transition(Task1Phase.COMPLETE, data.now, "landed_at_h")

        elif self.phase == Task1Phase.COMPLETE:
            target_xy = H
            target_world_height = cfg.final_height_m

        return Task1Command(
            phase=self.phase,
            target_xy_m=target_xy,
            target_world_height_m=target_world_height,
            target_deck_height_m=target_deck_height,
            vx_m_s=vx,
            vy_m_s=vy,
            takeoff_requested=takeoff_requested,
            release_requested=release_requested,
            land_requested=land_requested,
            fixed_heading=True,
            target_acquired=self.target_acquired,
            drop_committed=self.drop_committed,
            drop_released=self.drop_released,
            mission_success=self.mission_success,
            reason=self.reason,
        )

    def _confirm_vision(
        self, data: Task1Input, *, require_centered: bool
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

    def _begin_return(
        self,
        now: float,
        position_xy: tuple[float, float],
        reason: str,
    ) -> None:
        self._climb_anchor = position_xy
        self._transition(Task1Phase.CLIMB, now, reason)

    def _transition(
        self, phase: Task1Phase, now: float, reason: str
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
