"""PATROL/CIRCLING/TO_LANDING状态机单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_circle_state_machine.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, POLE_CIRCLE_N_POINTS, LANDING_POINT


def _make_mission(radar_obj=None, pole_total=1):
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)
    m.pole_total = pole_total
    return m


class TestPatrolTriggersCircling:
    def test_confirmed_new_pole_switches_to_circling(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "CIRCLING"
        assert m._circle_pole_center == pytest.approx((0.5, 0.3))
        assert len(m.targets) == POLE_CIRCLE_N_POINTS + 1
        assert m.target_index == 0

    def test_already_circled_pole_does_not_retrigger(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        m.circled_poles = [(0.5, 0.3)]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "PATROL"


class TestCirclingHoverExclusion:
    def test_own_circling_target_does_not_trigger_hover(self):
        m = _make_mission(radar_obj=object())
        m._circle_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])  # 就是环绕目标本身

        m.set_speed = lambda *a, **k: None
        m.navigate([0.45, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False

    def test_other_confirmed_pole_still_triggers_hover_during_circling(self):
        m = _make_mission(radar_obj=object())
        m._circle_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 另一根杆子，离(0,0)只有0.3m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is True


class TestCircleCompletion:
    def test_single_pole_mission_switches_to_to_landing_when_circle_done(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "CIRCLING"
        m._circle_pole_center = (0.5, 0.3)
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1  # 已飞完最后一个环绕航点

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "TO_LANDING"
        assert m.circled_poles == [(0.5, 0.3)]
        assert m.targets == [[LANDING_POINT[0], LANDING_POINT[1], m._cruise_z]]
        assert m.target_index == 0

    def test_multi_pole_mission_resumes_patrol_when_more_poles_remain(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        patrol_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2], [2.0, 0.0, 1.2]]
        m._patrol_saved_targets = patrol_targets
        m._patrol_saved_index = 1
        m.nav_mode = "CIRCLING"
        m._circle_pole_center = (0.5, 0.3)
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.circled_poles == [(0.5, 0.3)]
        assert m.targets == patrol_targets
        assert m.target_index == 1

    def test_to_landing_arrival_transitions_to_land_state(self):
        m = _make_mission(radar_obj=None)
        m.nav_mode = "TO_LANDING"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 1  # 已到达唯一的降落航点

        m.navigate([2.0, 0.0, 1.2], 0.0)

        assert m.state == "LAND"
