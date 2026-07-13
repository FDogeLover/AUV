"""PoleTracker接入Mission_GPT导航流程的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/circle_pole && python -m pytest test_mission_pole_integration.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, nearest_confirmed_pole_dist, POLE_DANGER_DIST_M, POLE_RESUME_DIST_M


# ════════════════════ nearest_confirmed_pole_dist ════════════════════

class TestNearestConfirmedPoleDist:
    def test_empty_list_returns_none(self):
        assert nearest_confirmed_pole_dist([], 0.0, 0.0) is None

    def test_single_pole_returns_its_distance(self):
        poles = [{"x": 0.3, "y": 0.0, "hits": 3}]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.3)

    def test_multiple_poles_returns_nearest(self):
        poles = [
            {"x": 5.0, "y": 0.0, "hits": 3},
            {"x": 0.4, "y": 0.3, "hits": 3},  # 距(0,0) = 0.5
        ]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.5)


# ════════════════════ mission 悬停/恢复行为 ════════════════════
#
# circle_pole特有说明(2026-07-13)：这份测试原样复制自basic_radar，但在circle_pole里，
# PATROL态第一次发现一个"未环绕过"的确认杆塔时，行为已经从"悬停"改成"立即开始环绕"
# (见test_circle_state_machine.py)。悬停逻辑在circle_pole里仍然存在，但只在"这个杆子
# 已经环绕过(在circled_poles里)、不是当前环绕目标"这种情况下才会触发——所以下面几个
# 测试统一改成先把杆子标记为已环绕(m.circled_poles = [...])，模拟这个仍然合法的悬停场景。

def _make_mission(radar_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)


class TestMissionPoleHover:
    def test_no_radar_means_no_pole_tracker_and_no_hover(self):
        m = _make_mission(radar_obj=None)
        assert m.pole_tracker is None
        m.navigate([0.0, 0.0, 1.0], 0.0)  # 不应该抛异常
        assert m._pole_hovering is False

    def test_navigate_hovers_when_already_circled_pole_confirmed_nearby(self):
        m = _make_mission(radar_obj=object())  # 哨兵对象，本测试跳过真实轮询，不会被调用
        m.circled_poles = [(0.3, 0.0)]  # 已环绕过，重新出现时走悬停而不是再次触发环绕
        m._last_pole_poll_time = time.time()  # 跳过本帧的雷达轮询(节流)，直接摆好历史数据
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 世界坐标(0.3,0)，离(0,0)只有0.3m

        sent = []
        m.set_speed = lambda x, y, yaw, z: sent.append((x, y, yaw, z))

        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is True
        assert sent == [(0, 0, 0, int(m._ramp_z_cm))]

    def test_navigate_does_not_hover_when_pole_far_away(self):
        m = _make_mission(radar_obj=object())
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(5.0, 5.0)])  # 远超 POLE_DANGER_DIST_M

        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is False

    def test_navigate_resumes_after_already_circled_pole_no_longer_confirmed(self):
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(0.3, 0.0)]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        m.pole_tracker.reset()
        m._last_pole_poll_time = time.time()
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is False

    def test_navigate_keeps_hovering_within_resume_hysteresis_band(self):
        """已经悬停时，杆子距离哪怕超过了触发阈值(POLE_DANGER_DIST_M)，只要还没到
        恢复阈值(POLE_RESUME_DIST_M)，应该继续悬停——避免距离刚好卡在触发阈值附近
        抖动时悬停状态跟着反复横跳(2026-07-08真机测试观察到的问题)。"""
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(0.3, 0.0)]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        # 飞机远离杆子，让实际距离变成触发/恢复阈值之间的中点：超过触发阈值但还没到
        # 恢复阈值，应该继续悬停。杆子世界坐标必须保持不变(仍是circled_poles里那根
        # 已环绕过的杆子)——如果像basic_radar原版那样改杆子的模拟坐标来代表"距离变化"，
        # 在circle_pole里会被_already_circled按世界坐标距离判断成"新杆子"，
        # 提前触发_start_circling并return，_pole_hovering根本不会被本次调用重新赋值，
        # 断言表面通过实为残留的上一次调用的值，不是真的测到了悬停滞回逻辑。
        # 用飞机自身位置远离来改变距离，才是真机场景下距离变化的真实成因。
        # (2026-07-10阈值从0.9/1.05回退到0.75/0.9，见CLAUDE.md问题20——用常量算中点
        # 而不是硬编码具体数字，避免下次再调阈值时这个测试又悄悄失效)
        mid_dist = (POLE_DANGER_DIST_M + POLE_RESUME_DIST_M) / 2
        m.pole_tracker.reset()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m._last_pole_poll_time = time.time()
        m.navigate([0.3 - mid_dist, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

    def test_navigate_resumes_once_pole_dist_exceeds_resume_threshold(self):
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(0.3, 0.0)]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        # 飞机远离杆子到实际距离1.1m：超过恢复阈值(0.9)，应该恢复导航。
        # 同上一个测试的说明：杆子世界坐标保持不变，靠飞机位置远离来改变距离。
        m.pole_tracker.reset()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m._last_pole_poll_time = time.time()
        m.navigate([0.3 - 1.1, 0.0, 1.0], 0.0)
        assert m._pole_hovering is False
