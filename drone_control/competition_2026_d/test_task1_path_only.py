from drone_control.competition_2026_d.task1_mission import Task1Config
from drone_control.competition_2026_d.task1_path_only_test import (
    build_path_test_config,
)


def test_path_only_keeps_constant_height_and_disables_vision_wait():
    config = build_path_test_config(
        Task1Config(),
        height_m=1.5,
        path_speed_m_s=0.13,
        intercept_speed_m_s=0.30,
        return_speed_m_s=0.25,
    )
    assert config.cruise_height_m == 1.5
    assert config.follow_height_m == 1.5
    assert config.acquire_timeout_s == 0.0
    assert config.car_speed_m_s == 0.13
    assert config.car_speed_scale == 1.0
    assert config.intercept_speed_m_s == 0.30
    assert config.return_speed_m_s == 0.25
