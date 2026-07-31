"""任务二状态机 Task2MissionDirector 的单元测试。

只测试纯状态机逻辑，不依赖硬件模块。
"""

import math

import pytest

from drone_control.competition_2026_d import task2_start
from drone_control.competition_2026_d.payload_actuator import ActuatorState
from drone_control.competition_2026_d.task2_mission import (
    B_PRE,
    C,
    D,
    H,
    Task2Config,
    Task2Input,
    Task2MissionDirector,
    Task2Phase,
)
from drone_control.competition_2026_d.task2_start import (
    NullVisionReader,
    build_open_loop_cd_test_config,
    build_stationary_platform_retakeoff_test_config,
    build_vision_landing_test_config,
    validate_task2_start_modes,
    wait_for_task2_start,
)


def mission_input(now, position, **kwargs):
    return Task2Input(now=now, position_xyz_m=position, **kwargs)


# ---- C 点前：复用任务一逻辑 ----


def test_wait_start_transitions_to_takeoff_on_car_start():
    director = Task2MissionDirector()
    cmd = director.tick(
        mission_input(0.0, (0, 0, 0), car_start=True, t265_confidence=3)
    )
    assert director.phase == Task2Phase.TAKEOFF


def test_wait_start_stays_without_car_start():
    director = Task2MissionDirector()
    director.tick(mission_input(0.0, (0, 0, 0), car_start=False))
    assert director.phase == Task2Phase.WAIT_START


def test_takeoff_transitions_to_hold_at_cruise_height():
    director = Task2MissionDirector()
    director._transition(Task2Phase.TAKEOFF, 0.0, "test")
    director.tick(mission_input(0.1, (0, 0, 1.44)))
    assert director.phase == Task2Phase.HOLD_3S


def test_hold_3s_waits_for_duration_and_height():
    director = Task2MissionDirector()
    director._transition(Task2Phase.HOLD_3S, 0.0, "test")
    director.tick(mission_input(2.9, (0, 0, 1.5)))
    assert director.phase == Task2Phase.HOLD_3S
    director.tick(mission_input(3.01, (0, 0, 1.5)))
    assert director.phase == Task2Phase.INTERCEPT_B_PRE


def test_intercept_b_pre_to_acquire_target():
    director = Task2MissionDirector()
    director._transition(Task2Phase.INTERCEPT_B_PRE, 0.0, "test")
    director.tick(mission_input(0.1, (B_PRE[0], B_PRE[1], 1.0), velocity_xy_m_s=(0.05, 0.0)))
    assert director.phase == Task2Phase.ACQUIRE_TARGET


# ---- C 点切换 ----


def test_sync_at_c_requires_offset_and_car_position():
    cfg = Task2Config(c_sync_vision_enabled=False, require_car_position_at_c=True)
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.SYNC_TARGET_AT_C, 0.0, "test")
    # 无 offset 和 car_position，不转
    director.tick(mission_input(0.1, (C[0], C[1], 1.0)))
    assert director.phase == Task2Phase.SYNC_TARGET_AT_C
    # 有 offset 和 car_position，转
    director.tick(
        mission_input(
            0.2, (C[0], C[1], 1.0),
            offset_ready=True, car_position_xy_m=(1.0, 1.0),
        )
    )
    assert director.phase == Task2Phase.ACTIVATE_TRACKER


def test_sync_at_c_skips_offset_check_when_disabled():
    cfg = Task2Config(c_sync_vision_enabled=False, require_car_position_at_c=False)
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.SYNC_TARGET_AT_C, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.0)))
    assert director.phase == Task2Phase.ACTIVATE_TRACKER


# ---- ACTIVATE_TRACKER ----


def test_activate_tracker_sets_tracker_and_landing_active():
    director = Task2MissionDirector()
    director._transition(Task2Phase.ACTIVATE_TRACKER, 0.0, "test")
    cmd = director.tick(mission_input(0.1, (C[0], C[1], 1.0)))
    assert cmd.tracker_active
    assert cmd.landing_active


def test_activate_tracker_to_dynamic_landing_on_gate_passed():
    director = Task2MissionDirector()
    director._transition(Task2Phase.ACTIVATE_TRACKER, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.0), landing_gate_passed=True))
    assert director.phase == Task2Phase.DYNAMIC_LANDING


def test_activate_tracker_timeout_to_climb():
    cfg = Task2Config(activate_tracker_timeout_s=1.0)
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.ACTIVATE_TRACKER, 0.0, "test")
    director.tick(mission_input(0.5, (C[0], C[1], 1.0)))
    assert director.phase == Task2Phase.ACTIVATE_TRACKER
    director.tick(mission_input(1.1, (C[0], C[1], 1.0)))
    assert director.phase == Task2Phase.CLIMB_150CM


def test_activate_tracker_abort_to_climb():
    director = Task2MissionDirector()
    director._transition(Task2Phase.ACTIVATE_TRACKER, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.0), landing_aborted=True))
    assert director.phase == Task2Phase.CLIMB_150CM


# ---- DYNAMIC_LANDING ----


def test_dynamic_landing_sets_tracker_and_landing_active():
    director = Task2MissionDirector()
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")
    cmd = director.tick(mission_input(0.1, (C[0], C[1], 1.0)))
    assert cmd.tracker_active
    assert cmd.landing_active


def test_dynamic_landing_to_retakeoff_on_deck_ride_complete():
    director = Task2MissionDirector()
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")
    cmd = director.tick(mission_input(0.1, (C[0], C[1], 1.0), deck_ride_complete=True))
    assert director.phase == Task2Phase.RETAKEOFF
    assert cmd.mission_success
    assert cmd.keep_armed
    assert not cmd.takeoff_requested
    assert not cmd.land_requested


def test_dynamic_landing_abort_to_climb():
    director = Task2MissionDirector()
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.0), landing_aborted=True))
    assert director.phase == Task2Phase.CLIMB_150CM


# ---- RETAKEOFF / CLIMB / RETURN / LAND ----


def test_retakeoff_clears_tracker_and_landing():
    director = Task2MissionDirector()
    director._transition(Task2Phase.RETAKEOFF, 0.0, "test")
    cmd = director.tick(mission_input(0.1, (C[0], C[1], 1.4)))
    assert not cmd.tracker_active
    assert not cmd.landing_active
    assert cmd.keep_armed
    assert not cmd.takeoff_requested
    assert not cmd.land_requested


def test_retakeoff_to_return_at_height():
    director = Task2MissionDirector()
    director._transition(Task2Phase.RETAKEOFF, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.39)))
    assert director.phase == Task2Phase.RETAKEOFF
    director.tick(mission_input(0.2, (C[0], C[1], 1.45)))
    assert director.phase == Task2Phase.RETURN_H


def test_climb_150cm_to_return_at_height():
    director = Task2MissionDirector()
    director._transition(Task2Phase.CLIMB_150CM, 0.0, "test")
    director.tick(mission_input(0.1, (C[0], C[1], 1.39)))
    assert director.phase == Task2Phase.CLIMB_150CM
    director.tick(mission_input(0.2, (C[0], C[1], 1.45)))
    assert director.phase == Task2Phase.RETURN_H


def test_return_h_to_land_h():
    director = Task2MissionDirector()
    director._transition(Task2Phase.RETURN_H, 0.0, "test")
    director.tick(mission_input(0.1, (0.05, 0.05, 1.5)))
    assert director.phase == Task2Phase.LAND_H


def test_land_h_to_complete():
    director = Task2MissionDirector()
    director._transition(Task2Phase.LAND_H, 0.0, "test")
    cmd = director.tick(mission_input(0.1, (0, 0, 0.1), landed=True))
    assert director.phase == Task2Phase.COMPLETE
    assert cmd.land_requested


def test_deck_touchdown_waits_without_requesting_lock_or_second_takeoff():
    director = Task2MissionDirector()
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")

    cmd = director.tick(
        mission_input(
            0.1,
            (C[0], C[1], 0.10),
            touchdown_confirmed=True,
            deck_ride_complete=False,
        )
    )

    assert director.phase == Task2Phase.DYNAMIC_LANDING
    assert cmd.keep_armed
    assert not cmd.takeoff_requested
    assert not cmd.land_requested


def test_only_initial_takeoff_requests_takeoff_and_only_final_h_requests_land():
    director = Task2MissionDirector()
    director._transition(Task2Phase.TAKEOFF, 0.0, "test")
    takeoff = director.tick(mission_input(0.1, (*H, 0.20)))
    assert takeoff.takeoff_requested
    assert not takeoff.land_requested
    assert takeoff.keep_armed

    director._transition(Task2Phase.LAND_H, 1.0, "test")
    final_land = director.tick(mission_input(1.1, (*H, 0.15)))
    assert not final_land.takeoff_requested
    assert final_land.land_requested
    assert final_land.keep_armed


# ---- 任务二0.5m安全联调 ----


def test_open_loop_cd_test_config_is_isolated_and_bounded():
    cfg = build_open_loop_cd_test_config(Task2Config(), cd_speed_m_s=0.08)
    assert cfg.safe_open_loop_cd_test
    assert cfg.cruise_height_m == 1.20
    assert cfg.follow_height_m == 1.20
    assert cfg.safe_hover_height_m == 0.50
    assert cfg.safe_c_offset_x_m == -0.15
    assert cfg.safe_c_offset_y_m == 0.25
    assert cfg.safe_descent_rate_m_s == 0.12
    assert cfg.safe_follow_cutoff_s == 25.0
    assert cfg.car_speed_m_s == 0.08
    assert cfg.hold_duration_s == 0.0
    assert cfg.vision_confirm_frames == 5
    assert cfg.drop_max_error_m == 0.15
    assert not cfg.require_car_position_at_c


def test_open_loop_cd_test_flies_directly_to_c_after_takeoff():
    config = build_open_loop_cd_test_config(Task2Config())
    assert config.car_speed_m_s == 0.06
    director = Task2MissionDirector(config)
    safe_c = (C[0] - 0.15, C[1] + 0.25)
    assert director.sync_c == safe_c
    director._transition(Task2Phase.HOLD_3S, 0.0, "test")
    command = director.tick(mission_input(0.1, (*H, 1.20)))
    assert command.phase == Task2Phase.TRANSIT_C

    moving = director.tick(mission_input(0.2, (*H, 1.20)))
    assert moving.target_xy_m == safe_c
    assert moving.phase == Task2Phase.TRANSIT_C

    director.tick(
        mission_input(
            1.0,
            (*safe_c, 1.20),
            velocity_xy_m_s=(0.0, 0.0),
        )
    )
    assert director.phase == Task2Phase.SYNC_TARGET_AT_C


def test_vision_landing_test_flies_directly_to_c_then_activates_tracker():
    cfg = build_vision_landing_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    assert cfg.vision_landing_test
    assert cfg.land_only_after_touchdown
    assert not cfg.require_car_position_at_c
    assert cfg.hold_position_max_speed_m_s == 0.22
    assert cfg.hold_velocity_kd == 0.45
    assert cfg.landing_xy_speed_high_m_s == 0.12
    assert cfg.landing_xy_speed_mid_m_s == 0.09
    assert cfg.landing_xy_speed_low_m_s == 0.06

    director._transition(Task2Phase.HOLD_3S, 0.0, "test")
    transit = director.tick(mission_input(0.1, (*H, 1.20)))
    assert transit.phase == Task2Phase.TRANSIT_C

    director._transition(Task2Phase.SYNC_TARGET_AT_C, 1.0, "test")
    for seq in range(1, 6):
        ready = director.tick(
            mission_input(
                1.0 + seq * 0.1,
                (*C, 1.20),
                vision_seq=seq,
                vision_found=True,
                vision_quality=80,
                vision_error_xy_m=(0.02, -0.01),
            )
        )
    assert ready.phase == Task2Phase.ACTIVATE_TRACKER


def test_vision_landing_mode_is_mutually_exclusive_with_other_test_modes():
    with pytest.raises(ValueError, match="只能选择一个"):
        validate_task2_start_modes(
            open_loop_cd_test=True,
            vision_landing_test=True,
            stationary_retakeoff_test=False,
            local_button_start=False,
        )


def test_open_loop_cd_test_requires_five_centered_frames_at_c():
    director = Task2MissionDirector(
        build_open_loop_cd_test_config(Task2Config())
    )
    safe_c = director.sync_c
    director._transition(Task2Phase.SYNC_TARGET_AT_C, 0.0, "test")

    for seq in range(1, 5):
        command = director.tick(
            mission_input(
                seq * 0.1,
                (*safe_c, 1.20),
                vision_seq=seq,
                vision_found=True,
                vision_quality=80,
                vision_error_xy_m=(0.05, -0.02),
            )
        )
        assert command.phase == Task2Phase.SYNC_TARGET_AT_C

    command = director.tick(
        mission_input(
            0.5,
            (*safe_c, 1.20),
            vision_seq=5,
            vision_found=True,
            vision_quality=80,
            vision_error_xy_m=(0.05, -0.02),
        )
    )
    assert command.phase == Task2Phase.OPEN_LOOP_C_D
    assert not command.tracker_active
    assert not command.landing_active
    assert not command.land_requested


def test_open_loop_cd_test_descends_by_progress_and_stops_at_half_meter():
    cfg = build_open_loop_cd_test_config(Task2Config(), cd_speed_m_s=0.03)
    director = Task2MissionDirector(cfg)
    safe_c = director.sync_c
    director._transition(Task2Phase.OPEN_LOOP_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)

    start = director.tick(
        mission_input(
            0.2,
            (*safe_c, 1.20),
            car_speed_m_s=0.20,
        )
    )
    assert math.isclose(start.target_world_height_m, 1.20, abs_tol=1e-9)
    assert start.vy_m_s < 0.0
    assert math.hypot(start.vx_m_s, start.vy_m_s) <= 0.03 + 1e-9

    middle = director.tick(
        mission_input(
            0.4,
            (
                (safe_c[0] + D[0]) / 2.0,
                (safe_c[1] + D[1]) / 2.0,
                0.90,
            ),
            car_speed_m_s=0.20,
        )
    )
    expected_middle_height = max(
        0.50,
        1.20
        - 0.5
        * director.cd_follower.path.length_m
        * cfg.safe_descent_rate_m_s
        / cfg.car_speed_m_s,
    )
    assert math.isclose(
        middle.target_world_height_m,
        expected_middle_height,
        abs_tol=1e-9,
    )
    assert not middle.tracker_active
    assert not middle.landing_active
    assert not middle.land_requested

    waiting_for_height = director.tick(
        mission_input(
            0.6,
            (*D, 0.70),
            velocity_xy_m_s=(0.0, 0.0),
        )
    )
    assert waiting_for_height.phase == Task2Phase.OPEN_LOOP_C_D
    assert waiting_for_height.target_xy_m == D
    assert (waiting_for_height.vx_m_s, waiting_for_height.vy_m_s) == (
        0.0,
        0.0,
    )
    assert waiting_for_height.target_world_height_m == 0.50

    hovering = director.tick(
        mission_input(
            0.8,
            (*D, 0.55),
            velocity_xy_m_s=(0.0, 0.0),
        )
    )
    assert hovering.phase == Task2Phase.SAFE_HOVER_D
    assert not hovering.land_requested
    assert not hovering.tracker_active
    assert not hovering.landing_active

    held = director.tick(mission_input(1.0, (*D, 0.50)))
    assert held.phase == Task2Phase.SAFE_HOVER_D
    assert held.target_world_height_m == 0.50
    assert not held.mission_success


def test_open_loop_cd_test_cuts_off_before_car_turns_at_d():
    cfg = build_open_loop_cd_test_config(Task2Config(), cd_speed_m_s=0.03)
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.OPEN_LOOP_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)
    position = (
        (director.sync_c[0] + D[0]) / 2.0,
        (director.sync_c[1] + D[1]) / 2.0,
        0.50,
    )

    cutoff = director.tick(
        mission_input(
            25.01,
            position,
            velocity_xy_m_s=(0.03, 0.0),
        )
    )
    assert cutoff.phase == Task2Phase.SAFE_HOVER_D
    assert cutoff.target_xy_m == position[:2]
    assert (cutoff.vx_m_s, cutoff.vy_m_s) == (0.0, 0.0)
    assert cutoff.target_world_height_m == 0.50
    assert not cutoff.land_requested
    assert not cutoff.tracker_active
    assert not cutoff.landing_active


def test_open_loop_cd_test_stops_forward_motion_when_half_meter_reached():
    cfg = build_open_loop_cd_test_config(Task2Config(), cd_speed_m_s=0.04)
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.OPEN_LOOP_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)
    sample = director.cd_follower.path.sample(0.40)
    position = (*sample.point_xy_m, 0.55)

    command = director.tick(
        mission_input(
            10.0,
            position,
            velocity_xy_m_s=(0.04, 0.0),
        )
    )
    assert command.phase == Task2Phase.SAFE_HOVER_D
    assert command.target_xy_m == position[:2]
    assert (command.vx_m_s, command.vy_m_s) == (0.0, 0.0)
    assert command.target_world_height_m == 0.50
    assert not command.land_requested


# ---- 正式控制链静止平台降落/复升联调 ----


def test_stationary_retakeoff_config_is_low_altitude_and_isolated():
    base = Task2Config()
    cfg = build_stationary_platform_retakeoff_test_config(
        base,
        point_x_m=0.10,
        point_y_m=0.50,
        retakeoff_height_m=0.80,
    )

    assert cfg.stationary_retakeoff_test
    assert not cfg.safe_open_loop_cd_test
    assert cfg.cruise_height_m == 1.00
    assert cfg.follow_height_m == 1.00
    assert cfg.hold_duration_s == 2.0
    assert cfg.intercept_speed_m_s == 0.10
    assert cfg.hold_position_max_speed_m_s == 0.22
    assert cfg.hold_velocity_kd == 0.45
    assert cfg.retakeoff_height_m == 0.80
    assert cfg.stationary_test_point_x_m == 0.10
    assert cfg.stationary_test_point_y_m == 0.50
    assert not cfg.require_car_position_at_c
    assert not base.stationary_retakeoff_test


def test_stationary_no_vision_config_is_explicit_and_isolated():
    base = Task2Config()
    cfg = build_stationary_platform_retakeoff_test_config(
        base, skip_vision=True
    )

    assert cfg.stationary_retakeoff_test
    assert cfg.stationary_skip_vision
    assert not base.stationary_skip_vision


def test_stationary_land_only_config_is_explicit_and_isolated():
    base = Task2Config()
    cfg = build_stationary_platform_retakeoff_test_config(
        base, land_only=True
    )

    assert cfg.land_only_after_touchdown
    assert not base.land_only_after_touchdown


def test_null_vision_reader_is_safe_for_no_vision_flight_adapter():
    reader = NullVisionReader()
    assert reader.start()
    assert reader.latest(1.0, 0.15) is None
    assert reader.close() is None


def test_local_button_start_is_restricted_to_stationary_retakeoff_mode():
    with pytest.raises(ValueError, match="只能与"):
        validate_task2_start_modes(
            open_loop_cd_test=False,
            stationary_retakeoff_test=False,
            local_button_start=True,
        )
    validate_task2_start_modes(
        open_loop_cd_test=False,
        stationary_retakeoff_test=True,
        local_button_start=True,
    )
    with pytest.raises(ValueError, match="stationary-skip-vision"):
        validate_task2_start_modes(
            open_loop_cd_test=False,
            stationary_retakeoff_test=False,
            local_button_start=False,
            stationary_skip_vision=True,
        )
    with pytest.raises(ValueError, match="stationary-land-only"):
        validate_task2_start_modes(
            open_loop_cd_test=False,
            stationary_retakeoff_test=False,
            local_button_start=False,
            stationary_land_only=True,
        )


def test_local_button_start_success_skips_car_session(monkeypatch):
    monkeypatch.setattr(task2_start, "wait_for_start_button", lambda: True)

    accepted, session_id = wait_for_task2_start(
        local_button_start=True,
        start_gate=None,
        start_timeout=None,
    )

    assert accepted
    assert session_id is None


def test_local_button_failure_fails_closed(monkeypatch):
    monkeypatch.setattr(task2_start, "wait_for_start_button", lambda: False)

    accepted, session_id = wait_for_task2_start(
        local_button_start=True,
        start_gate=None,
        start_timeout=None,
    )

    assert not accepted
    assert session_id is None


def test_car_start_path_remains_unchanged():
    class Gate:
        def wait(self, timeout):
            assert timeout == 2.0
            return 1234

    accepted, session_id = wait_for_task2_start(
        local_button_start=False,
        start_gate=Gate(),
        start_timeout=2.0,
    )

    assert accepted
    assert session_id == 1234


def test_stationary_retakeoff_config_rejects_unsafe_point_or_height():
    with pytest.raises(ValueError, match="距H点"):
        build_stationary_platform_retakeoff_test_config(
            Task2Config(), point_x_m=0.0, point_y_m=0.10
        )
    with pytest.raises(ValueError, match="复升高度"):
        build_stationary_platform_retakeoff_test_config(
            Task2Config(), retakeoff_height_m=1.20
        )


def test_stationary_mode_uses_normal_takeoff_then_direct_test_point_transit():
    cfg = build_stationary_platform_retakeoff_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    assert director.sync_c == (0.0, 0.50)

    director._transition(Task2Phase.HOLD_3S, 0.0, "test")
    waiting = director.tick(mission_input(1.9, (*H, 1.00)))
    assert waiting.phase == Task2Phase.HOLD_3S

    transit = director.tick(mission_input(2.01, (*H, 1.00)))
    assert transit.phase == Task2Phase.TRANSIT_C
    moving = director.tick(mission_input(2.1, (*H, 1.00)))
    assert moving.target_xy_m == (0.0, 0.50)
    assert math.hypot(moving.vx_m_s, moving.vy_m_s) <= 0.10 + 1e-9


def test_stationary_mode_takeoff_hold_uses_velocity_damping():
    cfg = build_stationary_platform_retakeoff_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.TAKEOFF, 0.0, "test")

    command = director.tick(
        mission_input(
            0.1,
            (0.10, -0.20, 0.50),
            velocity_xy_m_s=(0.20, -0.10),
        )
    )

    p_only_x = cfg.point_kp * -0.10
    p_only_y = cfg.point_kp * 0.20
    assert command.vx_m_s < p_only_x
    assert (
        command.vx_m_s * 0.20 + command.vy_m_s * -0.10
        < p_only_x * 0.20 + p_only_y * -0.10
    )
    assert math.hypot(command.vx_m_s, command.vy_m_s) <= 0.22 + 1e-9


def test_stationary_tracker_activation_holds_test_point_not_formal_c():
    cfg = build_stationary_platform_retakeoff_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.ACTIVATE_TRACKER, 0.0, "test")

    command = director.tick(mission_input(0.1, (0.10, 0.40, 1.00)))

    assert command.target_xy_m == director.sync_c
    assert command.vx_m_s < 0.0
    assert command.vy_m_s > 0.0


def test_stationary_mode_visual_gate_does_not_require_car_t265_position():
    cfg = build_stationary_platform_retakeoff_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.SYNC_TARGET_AT_C, 0.0, "test")

    for seq in range(1, 6):
        command = director.tick(
            mission_input(
                seq * 0.1,
                (*director.sync_c, 1.00),
                vision_seq=seq,
                vision_found=True,
                vision_quality=80,
                vision_error_xy_m=(0.03, -0.02),
                car_position_xy_m=None,
                offset_ready=False,
            )
        )

    assert command.phase == Task2Phase.ACTIVATE_TRACKER
    activated = director.tick(
        mission_input(
            0.6,
            (*director.sync_c, 1.00),
            vision_seq=6,
            vision_found=True,
            vision_quality=80,
            vision_error_xy_m=(0.03, -0.02),
        )
    )
    assert activated.tracker_active
    assert activated.landing_active


def test_stationary_no_vision_mode_enters_landing_at_fixed_point():
    cfg = build_stationary_platform_retakeoff_test_config(
        Task2Config(), skip_vision=True
    )
    director = Task2MissionDirector(cfg)
    director._transition(Task2Phase.SYNC_TARGET_AT_C, 0.0, "test")

    ready = director.tick(
        mission_input(
            0.1,
            (*director.sync_c, 1.00),
            vision_found=False,
            vision_quality=0,
            vision_error_xy_m=None,
        )
    )
    assert ready.phase == Task2Phase.ACTIVATE_TRACKER
    assert ready.reason == "stationary_test_fixed_point_ready_no_vision"

    activated = director.tick(
        mission_input(
            0.2,
            (*director.sync_c, 1.00),
            vision_found=False,
        )
    )
    assert activated.tracker_active
    assert activated.landing_active


def test_stationary_mode_retakeoff_holds_touchdown_xy_then_hovers():
    cfg = build_stationary_platform_retakeoff_test_config(Task2Config())
    director = Task2MissionDirector(cfg)
    touchdown_xy = (0.03, 0.48)
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")

    retakeoff = director.tick(
        mission_input(
            5.1,
            (*touchdown_xy, 0.10),
            deck_ride_complete=True,
        )
    )
    assert retakeoff.phase == Task2Phase.RETAKEOFF
    assert director._retakeoff_anchor == touchdown_xy
    assert retakeoff.keep_armed
    assert not retakeoff.takeoff_requested
    assert not retakeoff.land_requested

    climbing = director.tick(
        mission_input(5.2, (touchdown_xy[0] + 0.05, touchdown_xy[1], 0.60))
    )
    assert climbing.phase == Task2Phase.RETAKEOFF
    assert climbing.target_xy_m == touchdown_xy
    assert climbing.vx_m_s < 0.0

    hovering = director.tick(mission_input(6.0, (*touchdown_xy, 0.71)))
    assert hovering.phase == Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF
    held = director.tick(mission_input(6.1, (*touchdown_xy, 0.80)))
    assert held.phase == Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF
    assert held.target_xy_m == touchdown_xy
    assert held.target_world_height_m == 0.80
    assert held.keep_armed
    assert not held.land_requested


def test_stationary_land_only_stops_after_deck_hold_without_retakeoff():
    cfg = build_stationary_platform_retakeoff_test_config(
        Task2Config(), land_only=True
    )
    director = Task2MissionDirector(cfg)
    touchdown_xy = (0.02, 0.49)
    director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")

    landed = director.tick(
        mission_input(
            5.1,
            (*touchdown_xy, 0.05),
            deck_ride_complete=True,
        )
    )
    assert landed.phase == Task2Phase.LANDED_ON_PLATFORM
    assert landed.mission_success
    assert not landed.takeoff_requested

    held = director.tick(mission_input(6.0, (*touchdown_xy, 0.05)))
    assert held.phase == Task2Phase.LANDED_ON_PLATFORM
    assert held.target_world_height_m == 0.05
    assert (held.vx_m_s, held.vy_m_s) == (0.0, 0.0)
    assert held.keep_armed
