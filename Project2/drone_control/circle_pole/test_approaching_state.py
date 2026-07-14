"""PATROL→APPROACHING触发滞回逻辑 + APPROACHING状态控制律单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_approaching_state.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, POLE_TRIGGER_CONFIRM_S


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
