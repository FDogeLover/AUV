import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import (
    mission,
    APPROACH_CENTERED_DIST_M,
    HOVER_DROP_DURATION_S,
)


class _FakeRealsense:
    def __init__(self, pos=(0.0, 0.0, 1.8), yaw=0.0, velocity=(0.0, 0.0, 0.0), confidence=3):
        self._pos = pos
        self._yaw = yaw
        self._velocity = velocity
        self._confidence = confidence
        self._running = True

    def start(self):
        return True

    def autoset(self):
        pass

    def get_tracking_confidence(self):
        return self._confidence

    def get_position(self):
        return self._pos

    def get_orientation(self):
        return (0.0, 0.0, self._yaw)

    def get_velocity(self):
        return self._velocity

    def is_running(self):
        return self._running

    def stop(self):
        self._running = False


def _make_mission(tmp_path):
    re_fc = [0] * 14
    se_fc = [0] * 11
    router = tmp_path / "router.txt"
    router.write_text("0.0,0.0,1.8\n4.0,0.0,1.8\n0.0,0.0,0.0\n")
    old_cwd = os.getcwd()
    os.chdir(tmp_path)
    try:
        m = mission(re_fc, se_fc, realsense_obj=_FakeRealsense())
    finally:
        os.chdir(old_cwd)
    m.t265_ok = True
    m.nav_mode = "PATROL"
    return m


class TestFireTriggerLatch:
    def test_fire_triggered_defaults_false(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.fire_triggered is False

    def test_maybe_trigger_approach_switches_mode_once(self, tmp_path):
        m = _make_mission(tmp_path)
        m.saved_target_index_before_fire = None
        triggered = m.maybe_trigger_approach(detection=(50.0, 30.0))
        assert triggered is True
        assert m.nav_mode == "APPROACH"
        assert m.fire_triggered is True

    def test_second_detection_does_not_retrigger(self, tmp_path):
        m = _make_mission(tmp_path)
        m.maybe_trigger_approach(detection=(50.0, 30.0))
        m.nav_mode = "PATROL"  # 模拟已经处理完一次火情后回到PATROL
        triggered = m.maybe_trigger_approach(detection=(10.0, 10.0))
        assert triggered is False
        assert m.nav_mode == "PATROL"  # 不会被重新触发进APPROACH

    def test_no_detection_does_not_trigger(self, tmp_path):
        m = _make_mission(tmp_path)
        triggered = m.maybe_trigger_approach(detection=None)
        assert triggered is False
        assert m.fire_triggered is False


class TestApproachCentering:
    def test_pixel_offset_within_deadband_counts_as_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        # 像素偏移换算成的水平距离 < APPROACH_CENTERED_DIST_M 才算居中
        assert m.is_approach_centered(dx_px=2, dy_px=2) is True

    def test_large_pixel_offset_not_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.is_approach_centered(dx_px=800, dy_px=800) is False


class TestHoverDropDuration:
    def test_hover_drop_duration_is_independent_constant(self):
        """见设计文档：HOVER_DROP_DURATION_S是赛题写死的3秒，不能跟navigate()
        到达确认用的arrival_hold_s混用。"""
        assert HOVER_DROP_DURATION_S == 3.0


class TestResumeAfterHoverDrop:
    def test_resume_continues_from_saved_index_not_reset(self, tmp_path):
        m = _make_mission(tmp_path)
        m.target_index = 3
        m.maybe_trigger_approach(detection=(50.0, 30.0))  # 保存target_index=3
        m.finish_hover_drop_and_resume()
        assert m.nav_mode == "PATROL"
        assert m.target_index == 3  # 恢复到触发时保存的索引，不重置为0
