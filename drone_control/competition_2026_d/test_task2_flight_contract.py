"""任务二正式飞行适配层的持续解锁契约测试（不访问硬件）。"""

from types import SimpleNamespace

from drone_control.competition_2026_d.task2_flight import Task2FlightMission
from drone_control.competition_2026_d.task2_mission import Task2Phase
from drone_control.competition_2026_d.task2_mission import (
    Task2Config,
    Task2MissionDirector,
)
from drone_control.competition_2026_d.vision.platform_tracker import (
    PlatformTracker,
)


def _flight(*, task_sta=1, next_task_sign=0, unlock_sta=1, dry_run=False):
    flight = Task2FlightMission.__new__(Task2FlightMission)
    flight.se_fc = [170, 2, task_sta, 500, 500, 100, 500, next_task_sign]
    flight.re_fc = [0, 0, 0, 0, 0, unlock_sta]
    flight._ramp_z_cm = 10.0
    flight._dry_run = dry_run
    flight.state = "NAVIGATE"
    flight.last_speed = None
    flight.set_speed = lambda *args: setattr(flight, "last_speed", args)
    return flight


def _command(phase, *, keep_armed=True):
    return SimpleNamespace(phase=phase, keep_armed=keep_armed)


def test_deck_ride_keeps_original_task_active_without_mutating_command_bits():
    flight = _flight()
    before = tuple(flight.se_fc)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.DYNAMIC_LANDING)
    )
    assert tuple(flight.se_fc) == before
    assert flight.state == "NAVIGATE"


def test_retakeoff_is_allowed_only_while_flight_controller_remains_unlocked():
    flight = _flight(unlock_sta=1)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.se_fc[2] == 1
    assert flight.se_fc[7] == 0
    assert flight.state == "NAVIGATE"


def test_retakeoff_refuses_second_unlock_when_flight_controller_is_locked():
    flight = _flight(unlock_sta=0)

    assert not flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.state == "HOVER_WAIT"
    assert flight.last_speed == (0, 0, 0, 10)
    assert flight.se_fc[2] == 1
    assert flight.se_fc[7] == 0


def test_unexpected_midmission_command_bit_change_stops_instead_of_rearming():
    flight = _flight(task_sta=0, next_task_sign=101, unlock_sta=0)

    assert not flight._verify_continuous_arm_contract(
        _command(Task2Phase.DYNAMIC_LANDING)
    )
    assert flight.state == "HOVER_WAIT"
    assert flight.last_speed == (0, 0, 0, 10)
    # 检查器只拒绝继续，不制造新的 task_sta 0->1 起飞边沿。
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 101


def test_contract_is_inactive_after_mission_completion():
    flight = _flight(task_sta=0, next_task_sign=101, unlock_sta=0)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.COMPLETE, keep_armed=False)
    )
    assert flight.state == "NAVIGATE"


def test_dry_run_keeps_task_zero_and_does_not_require_unlock_feedback():
    flight = _flight(task_sta=0, unlock_sta=0, dry_run=True)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.state == "NAVIGATE"
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 0


def test_stationary_platform_estimate_uses_vision_then_freezes_last_position():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            stationary_retakeoff_test=True,
            stationary_test_point_x_m=0.0,
            stationary_test_point_y_m=0.50,
        )
    )
    flight.platform_tracker = PlatformTracker()
    flight._stationary_platform_measurement = None

    visible = SimpleNamespace(
        found=True,
        error_xy_m=(0.05, -0.02),
        quality=90,
    )
    estimate = flight._update_stationary_platform_estimate(
        gate=visible,
        world_position=(0.10, 0.40, 1.0),
        now=1.0,
    )
    assert estimate is not None
    assert abs(estimate.x_m - 0.15) < 1e-9
    assert abs(estimate.y_m - 0.38) < 1e-9

    lost_too_close = SimpleNamespace(
        found=False,
        error_xy_m=None,
        quality=0,
    )
    frozen = flight._update_stationary_platform_estimate(
        gate=lost_too_close,
        world_position=(0.15, 0.38, 0.10),
        now=1.1,
    )
    assert frozen is not None
    assert abs(flight._stationary_platform_measurement[0] - 0.15) < 1e-9
    assert abs(flight._stationary_platform_measurement[1] - 0.38) < 1e-9


def test_stationary_no_vision_landing_uses_fixed_point_as_fresh_gate():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            stationary_retakeoff_test=True,
            stationary_skip_vision=True,
        )
    )

    class LandingProbe:
        def __init__(self):
            self.input = None

        def tick(self, landing_input):
            self.input = landing_input
            return SimpleNamespace()

    flight.dynamic_landing = LandingProbe()
    estimate = SimpleNamespace(
        x_m=0.0,
        y_m=0.50,
        vx_m_s=0.0,
        vy_m_s=0.0,
        uncertainty_m=0.01,
    )
    gate = SimpleNamespace(found=False, ambiguous=False)

    flight._run_landing(
        estimate,
        world_position=(0.0, 0.50, 1.0),
        velocity=(0.0, 0.0, 0.0),
        laser_height=1.0,
        gate=gate,
        observation=None,
        contact_evidence=False,
        safety_fault=None,
        car_velocity=None,
    )

    assert flight.dynamic_landing.input.visual_usable
    assert flight.dynamic_landing.input.car_motion_fresh


def test_landing_height_integrates_negative_speed_downward():
    flight = _flight()
    flight._ramp_z_cm = 100.0

    flight._step_landing_height(-0.20, 0.10)
    assert flight._ramp_z_cm == 98.0

    flight._step_landing_height(0.12, 0.10)
    assert flight._ramp_z_cm == 99.2


def test_landing_descent_speed_slews_but_stop_is_immediate():
    flight = _flight()
    flight.dynamic_landing = SimpleNamespace(
        config=SimpleNamespace(descend_slew_m_s2=0.40)
    )
    flight._landing_vz_applied_m_s = 0.0

    first = flight._smooth_landing_vz(-0.30, 0.05)
    assert abs(first - -0.02) < 1e-9
    flight._landing_vz_applied_m_s = first
    second = flight._smooth_landing_vz(-0.30, 0.05)
    assert abs(second - -0.04) < 1e-9

    # Contact/gate hold must stop descent without a lingering ramp.
    flight._landing_vz_applied_m_s = second
    assert flight._smooth_landing_vz(0.0, 0.05) == 0.0


def test_visual_platform_measurement_updates_once_per_camera_frame():
    flight = _flight()
    flight.platform_tracker = PlatformTracker()
    flight._last_platform_vision_seq = None
    flight._platform_measurement_source = "none"
    gate = SimpleNamespace(
        found=True,
        error_xy_m=(0.05, -0.03),
        seq=10,
        quality=90,
    )

    first = flight._update_visual_platform_estimate(
        gate=gate,
        world_position=(1.0, 2.0, 1.0),
        now=1.0,
    )
    assert first is not None
    assert abs(first.x_m - 1.05) < 1e-9
    assert abs(first.y_m - 1.97) < 1e-9
    assert flight._platform_measurement_source == "visual"

    repeated = flight._update_visual_platform_estimate(
        gate=gate,
        world_position=(1.0, 2.0, 1.0),
        now=1.05,
    )
    assert repeated is not None
    assert repeated.predicted
    assert flight._platform_measurement_source == "visual_prediction"


def test_visual_landing_xy_speed_limit_reduces_near_ground():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            landing_xy_speed_high_m_s=0.15,
            landing_xy_speed_mid_m_s=0.10,
            landing_xy_speed_low_m_s=0.07,
        )
    )
    flight.dynamic_landing = SimpleNamespace(
        config=SimpleNamespace(mid_height_m=0.80, low_height_m=0.35)
    )

    assert flight._landing_xy_speed_limit(1.0) == 0.15
    assert flight._landing_xy_speed_limit(0.50) == 0.10
    assert flight._landing_xy_speed_limit(0.20) == 0.07
    vx, vy = flight._limit_xy_speed(0.30, 0.40, 0.10)
    assert abs((vx * vx + vy * vy) ** 0.5 - 0.10) < 1e-9
