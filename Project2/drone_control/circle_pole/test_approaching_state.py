"""PATROL→APPROACHING触发滞回逻辑 + APPROACHING状态控制律单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_approaching_state.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import (
    mission, POLE_TRIGGER_CONFIRM_S, POLE_VISION_STALE_S,
    APPROACH_X_SPEED_FAR, APPROACH_X_SPEED_NEAR,
)
from Lcode.circle_planner import generate_circle_waypoints  # noqa: E402  (文件顶部已有sys.path.insert)


def _make_mission(radar_obj=None, pole_vision_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None,
                   radar_obj=radar_obj, pole_vision_obj=pole_vision_obj)


class _FakeVision:
    def __init__(self, dx_px=0.0, color="red", fresh=True):
        self._dx_px = dx_px
        self._color = color
        self._fresh = fresh
        self.locked_color = None

    def latest(self):
        t = time.time() if self._fresh else 0.0
        return {"dx_px": self._dx_px, "color": self._color, "t": t}

    def set_locked_color(self, color):
        self.locked_color = color


class TestPatrolTriggerRequiresBothRadarAndVision(object):
    def test_radar_only_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=None)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_radar_and_vision_together_do_not_trigger_before_confirm_window(self):
        """条件刚满足的第一帧不应该立刻触发，需要持续POLE_TRIGGER_CONFIRM_S。"""
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate == (0.5, 0.3, "red")

    def test_radar_and_vision_together_trigger_approaching_after_confirm_window(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一帧：只是记下候选，不触发
        assert m.nav_mode == "PATROL"

        m._trigger_candidate_since = time.time() - POLE_TRIGGER_CONFIRM_S - 0.01
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "APPROACHING"
        assert m._approach_pole_center == pytest.approx((0.5, 0.3))
        assert m._approach_color == "red"

    def test_already_circled_color_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m.circled_colors.add("red")
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None

    def test_stale_vision_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red", fresh=False))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_color_change_resets_confirm_window(self):
        """候选颜色中途变化(比如视觉误判抖动)要重新计时，不能沿用旧计时器——
        这也是颜色锁定防抖机制的一部分，见2026-07-14设计文档"颜色锁定"一节。"""
        vision = _FakeVision(color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        first_since = m._trigger_candidate_since

        vision._color = "green"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._trigger_candidate == (0.5, 0.3, "green")
        assert m._trigger_candidate_since > first_since


class TestApproachingControlLaw:
    def _start(self, pos, pole=(2.0, 0.0), color="red", dx_px=0.0):
        vision = _FakeVision(dx_px=dx_px, color=color)
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = pole
        m._approach_color = color
        m.nav_mode = "APPROACHING"
        m.set_speed = lambda *a, **k: None
        m.navigate(pos, 0.0)
        return m

    def test_far_distance_uses_fast_speed(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_FAR)

    def test_near_distance_uses_slow_speed(self):
        calls = []
        m = self._start(pos=[1.25, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([1.25, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_NEAR)

    def test_approach_speed_direction_toward_pole(self):
        """杆塔在飞机后方(x更小)时，接近速度应该是负的(往回飞)。"""
        calls = []
        m = self._start(pos=[3.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([3.0, 0.0, 1.2], 0.0)
        assert calls[-1] < 0

    def test_vision_dx_drives_y_speed_via_pid(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0), dx_px=500.0)
        m.set_speed = lambda x, y, yaw, z: calls.append(y)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] != 0

    def test_target_lost_beyond_timeout_retreats_to_patrol(self):
        stale_vision = _FakeVision(color="red", fresh=False)
        m = _make_mission(radar_obj=object(), pole_vision_obj=stale_vision)
        m._patrol_saved_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        m._patrol_saved_index = 1
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一次陈旧：只是开始计时，不立刻退回
        assert m.nav_mode == "APPROACHING"

        m._approach_lost_since = time.time() - POLE_VISION_STALE_S - 0.01
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.targets == [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        assert m.target_index == 1
        assert m._approach_pole_center is None
        assert stale_vision.locked_color is None

    def test_reaching_trigger_distance_switches_to_circling(self):
        # 距离0.6m，在触发半径内——_start()内部的第一次navigate()调用就已经
        # 触发切换，不需要(也不能)再调用第二次navigate()：切换后nav_mode
        # 不再是APPROACHING，会走非APPROACHING分支并访问雷达scan接口，
        # 跟本测试用的fake radar(object())不兼容。
        m = self._start(pos=[2.1, 0.0, 1.2], pole=(2.7, 0.0))
        assert m.nav_mode == "CIRCLING"


class TestStartCirclingFromApproach:
    def test_red_pole_circles_clockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        assert m.nav_mode == "CIRCLING"
        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="cw", z=1.2)
        assert m.targets == expected
        assert m.target_index == 0

    def test_green_pole_circles_counterclockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "green"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="ccw", z=1.2)
        assert m.targets == expected
