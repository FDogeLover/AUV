import math

from drone_control.competition_2026_d.control.task1_path_controller import (
    PolylinePath,
    Task1PathFollower,
)
from drone_control.competition_2026_d.payload_actuator import ActuatorState
from drone_control.competition_2026_d.task1_mission import (
    B_PRE,
    C,
    D,
    Task1Input,
    Task1MissionDirector,
    Task1Phase,
)


def mission_input(now, position, **kwargs):
    return Task1Input(now=now, position_xyz_m=position, **kwargs)


def test_polyline_projection_does_not_regress_progress():
    path = PolylinePath(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)))
    follower = Task1PathFollower(path)
    follower.reset(progress_m=0.8, timestamp=0.0)
    command = follower.command(
        (0.2, 0.1), nominal_speed_m_s=0.13, timestamp=0.1
    )
    assert command.progress_m >= 0.77
    assert math.hypot(command.vx_m_s, command.vy_m_s) <= 0.20 + 1e-9


def test_path_reset_clamps_inherited_intercept_velocity():
    path = PolylinePath(((0.0, 0.0), (1.0, 0.0)))
    follower = Task1PathFollower(path)
    follower.reset(
        timestamp=0.0, velocity_xy_m_s=(0.38, 0.0)
    )
    command = follower.command(
        (0.0, 0.0), nominal_speed_m_s=0.13, timestamp=0.1
    )
    assert math.hypot(command.vx_m_s, command.vy_m_s) <= 0.20 + 1e-9


def test_hold_uses_fixed_three_seconds_despite_velocity_noise():
    director = Task1MissionDirector()
    director._transition(Task1Phase.HOLD_3S, 0.0, "test")
    director.tick(
        mission_input(0.0, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    director.tick(
        mission_input(2.9, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    assert director.phase == Task1Phase.HOLD_3S
    director.tick(
        mission_input(3.01, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    assert director.phase == Task1Phase.INTERCEPT_B_PRE


def test_hold_waits_for_safe_height_without_restarting_timer():
    director = Task1MissionDirector()
    director._transition(Task1Phase.HOLD_3S, 0.0, "test")
    director.tick(mission_input(3.1, (0.0, 0.0, 1.3)))
    assert director.phase == Task1Phase.HOLD_3S
    director.tick(mission_input(3.2, (0.0, 0.0, 1.4)))
    assert director.phase == Task1Phase.INTERCEPT_B_PRE


def test_takeoff_and_hold_use_t265_position_feedback():
    director = Task1MissionDirector()
    director._transition(Task1Phase.TAKEOFF, 0.0, "test")
    takeoff = director.tick(mission_input(0.1, (0.20, -0.10, 0.8)))
    assert takeoff.vx_m_s < 0.0
    assert takeoff.vy_m_s > 0.0
    assert math.hypot(takeoff.vx_m_s, takeoff.vy_m_s) <= 0.15 + 1e-9

    director._transition(Task1Phase.HOLD_3S, 1.0, "test")
    hold = director.tick(mission_input(1.1, (-0.12, 0.08, 1.5)))
    assert hold.vx_m_s > 0.0
    assert hold.vy_m_s < 0.0


def test_b_pre_timeout_continues_fixed_path_without_descending():
    director = Task1MissionDirector()
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")
    command = director.tick(mission_input(4.1, (*B_PRE, 1.5)))
    assert command.phase == Task1Phase.FOLLOW_B_C
    command = director.tick(mission_input(4.2, (*B_PRE, 1.5)))
    assert command.target_world_height_m == 1.5
    assert not command.target_acquired


def test_vision_does_not_write_horizontal_velocity_during_b_c_follow():
    without_vision = Task1MissionDirector()
    with_vision = Task1MissionDirector()
    for director in (without_vision, with_vision):
        director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
        director.bc_follower.reset(timestamp=0.0)
    with_vision.target_acquired = True

    plain = without_vision.tick(mission_input(0.1, (*B_PRE, 1.5)))
    detected = with_vision.tick(
        mission_input(
            0.1,
            (*B_PRE, 1.5),
            vision_seq=1,
            vision_found=True,
            vision_quality=100,
            vision_error_xy_m=(0.30, -0.20),
        )
    )
    assert (plain.vx_m_s, plain.vy_m_s) == (
        detected.vx_m_s,
        detected.vy_m_s,
    )
    assert plain.target_world_height_m == 1.5
    assert detected.target_world_height_m == 1.0


def test_drop_gate_latches_and_visual_is_ignored_after_descent_starts():
    director = Task1MissionDirector()
    director.target_acquired = True
    director._transition(Task1Phase.DROP_WINDOW_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)

    first = director.tick(
        mission_input(
            0.1,
            (*C, 1.0),
            vision_seq=10,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.05, 0.02),
        )
    )
    assert first.phase == Task1Phase.DROP_WINDOW_C_D
    second = director.tick(
        mission_input(
            0.2,
            (C[0] - 0.01, C[1], 1.0),
            vision_seq=11,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.05, 0.02),
        )
    )
    assert second.phase == Task1Phase.DROP_DESCENT
    assert second.drop_committed

    release = director.tick(
        mission_input(
            0.3,
            (C[0] - 0.02, C[1], 0.9),
            vision_found=False,
            deck_relative_height_m=0.65,
        )
    )
    assert release.phase == Task1Phase.RELEASING
    assert release.release_requested

    climb = director.tick(
        mission_input(
            0.4,
            (C[0] - 0.03, C[1], 0.9),
            vision_found=False,
            deck_relative_height_m=0.65,
            payload_state=ActuatorState.RELEASED,
        )
    )
    assert climb.phase == Task1Phase.CLIMB
    assert climb.drop_released
    assert climb.mission_success


def test_reaching_d_without_gate_skips_release_and_returns():
    director = Task1MissionDirector()
    director.target_acquired = True
    director._transition(Task1Phase.DROP_WINDOW_C_D, 0.0, "test")
    director.cd_follower.reset(
        progress_m=director.cd_follower.path.length_m, timestamp=0.0
    )
    command = director.tick(mission_input(0.1, (*D, 1.0)))
    assert command.phase == Task1Phase.CLIMB
    assert not command.release_requested
    assert not command.mission_success
    assert command.reason == "d_reached_without_drop_gate"
