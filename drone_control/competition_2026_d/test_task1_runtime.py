from drone_control.competition_2026_d.task1_runtime import (
    Task1T265SafetyMonitor,
    WorldDeckHeightController,
    observation_to_gate_sample,
)
from drone_control.competition_2026_d.vision.platform_observation import (
    FeatureFlag,
    PlatformObservation,
)


def observation(**kwargs):
    values = dict(
        stream_id=1,
        seq=10,
        capture_ms=100,
        found=True,
        cx=300,
        cy=220,
        outer_px=100,
        inner_px=50,
        angle_cdeg=0,
        quality=90,
        flags=int(FeatureFlag.APRILTAG_VALID),
        received_monotonic=1.0,
    )
    values.update(kwargs)
    return PlatformObservation(**values)


def test_observation_is_converted_to_gate_error_but_not_velocity():
    sample = observation_to_gate_sample(
        observation(),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert sample.found
    assert sample.error_xy_m == (0.04, -0.04)


def test_observation_without_height_can_confirm_identity_not_center_error():
    sample = observation_to_gate_sample(
        observation(),
        now=1.05,
        relative_height_m=None,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert sample.found
    assert sample.error_xy_m is None
    assert sample.reason == "height_unavailable"


def test_ambiguous_observation_is_rejected():
    sample = observation_to_gate_sample(
        observation(flags=int(FeatureFlag.AMBIGUOUS)),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert not sample.found
    assert sample.ambiguous


def test_platform_entering_laser_view_does_not_command_climb():
    controller = WorldDeckHeightController()
    first = controller.command(
        timestamp=0.0,
        current_world_height_m=1.0,
        current_laser_height_m=1.0,
        target_world_height_m=1.0,
        target_deck_height_m=None,
    )
    assert first.laser_setpoint_m == 1.0
    over_deck = controller.command(
        timestamp=0.1,
        current_world_height_m=1.0,
        current_laser_height_m=0.72,
        target_world_height_m=1.0,
        target_deck_height_m=None,
    )
    assert abs(over_deck.laser_setpoint_m - 0.72) < 1e-9


def test_first_height_sample_holds_current_laser_reference():
    controller = WorldDeckHeightController()
    command = controller.command(
        timestamp=0.0,
        current_world_height_m=0.15,
        current_laser_height_m=0.15,
        target_world_height_m=1.5,
        target_deck_height_m=None,
    )
    assert command.laser_setpoint_m == 0.15
    assert command.mode == "initial_hold"


def test_drop_height_uses_slew_limited_deck_relative_reference():
    controller = WorldDeckHeightController()
    controller.reset(timestamp=0.0, laser_height_m=1.0)
    command = controller.command(
        timestamp=1.0,
        current_world_height_m=1.0,
        current_laser_height_m=1.0,
        target_world_height_m=1.0,
        target_deck_height_m=0.65,
    )
    # dt is capped at 0.2 s: 0.09 m/s * 0.2 s = 0.018 m.
    assert abs(command.laser_setpoint_m - 0.982) < 1e-9
    assert command.mode == "deck_relative"


def test_t265_safety_detects_position_jump_even_with_valid_confidence():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=(0.0, 0.0),
    ) is None
    reason = monitor.update(
        timestamp=0.1,
        world_position_xyz_m=(0.7, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    )
    assert reason.startswith("t265_position_jump")


def test_t265_safety_detects_hold_drift():
    monitor = Task1T265SafetyMonitor()
    reason = monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.41, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=(0.0, 0.0),
    )
    assert reason == "t265_hold_geofence_exceeded"


def test_t265_safety_detects_ground_height_disagreement():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    reason = None
    for timestamp in (1.0, 1.2, 1.4, 1.6):
        reason = monitor.update(
            timestamp=timestamp,
            world_position_xyz_m=(0.0, 0.0, -0.7),
            laser_height_m=0.09,
            ground_reference_expected=True,
            hold_anchor_xy_m=None,
        )
    assert reason.startswith("t265_laser_height_mismatch")


def test_t265_safety_ignores_single_laser_height_glitch():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.53),
        laser_height_m=1.63,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=0.1,
        world_position_xyz_m=(0.02, 0.0, 1.53),
        laser_height_m=1.33,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=0.2,
        world_position_xyz_m=(0.04, 0.0, 1.53),
        laser_height_m=1.63,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None


def test_platform_height_change_is_ignored_outside_ground_reference_phase():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.0),
        laser_height_m=0.95,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=1.0,
        world_position_xyz_m=(0.1, 0.0, 1.0),
        laser_height_m=0.55,
        ground_reference_expected=False,
        hold_anchor_xy_m=None,
    ) is None
