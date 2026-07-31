"""任务二状态机 Task2MissionDirector 的单元测试。

只测试纯状态机逻辑，不依赖硬件模块。
"""

import math

from drone_control.competition_2026_d.payload_actuator import ActuatorState
from drone_control.competition_2026_d.task2_mission import (
    B_PRE,
    C,
    H,
    Task2Config,
    Task2Input,
    Task2MissionDirector,
    Task2Phase,
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
