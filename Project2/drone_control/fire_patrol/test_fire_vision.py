import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest

from Lcode.fire_vision import detect_fire, SmoothedFireDetector, FireVision, MIN_FIRE_AREA_PX, MAX_FIRE_AREA_PX


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_circle_bgr(frame, color_bgr, cx, cy, radius=40):
    cv2_circle(frame, color_bgr, cx, cy, radius)
    return frame


def cv2_circle(frame, color_bgr, cx, cy, radius):
    import cv2
    cv2.circle(frame, (cx, cy), radius, color_bgr, -1)


class TestDetectFire:
    def test_no_target_returns_none(self):
        frame = _blank_frame()
        result = detect_fire(frame)
        assert result is None

    def test_red_circle_detected_with_centered_offset_near_zero(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px == pytest.approx(0.0, abs=5)
        assert dy_px == pytest.approx(0.0, abs=5)

    def test_offset_right_and_below_center_is_positive(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=1200, cy=700, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px > 0
        assert dy_px > 0

    def test_offset_left_and_above_center_is_negative(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=700, cy=300, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px < 0
        assert dy_px < 0

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=3)  # 面积远小于MIN_FIRE_AREA_PX
        assert detect_fire(frame) is None

    def test_area_above_max_threshold_ignored(self):
        """面积过大(比如整面墙反光)不当作火源，见设计文档审查发现的"误触发风险"应对。"""
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=500)  # 远超MAX_FIRE_AREA_PX
        assert detect_fire(frame) is None


class TestSmoothedFireDetector:
    def test_no_detection_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update(None) is None

    def test_single_frame_not_enough_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update((10.0, 5.0)) is None

    def test_window_full_returns_average(self):
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        result = det.update((30.0, 0.0))
        assert result == pytest.approx((20.0, 0.0))

    def test_single_none_frame_resets_window(self):
        """检测中途丢失一帧应该重新积累，不能用陈旧帧掺进平均——避免真实火源移出
        画面后仍用旧偏移量继续修正(见设计文档APPROACH阶段防抖要求)。"""
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        det.update(None)  # 丢失一帧，窗口应清空
        assert det.update((5.0, 0.0)) is None  # 只有1帧，还不够


class _FakeVisionOpenFails:
    def __init__(self, **kwargs):
        pass

    def open(self):
        raise RuntimeError("Timed out waiting for the first camera frame")


class _FakeVisionOneFrame:
    """只在第一次get_frame()返回一帧红色图像，之后一直返回None，供测试用。"""

    def __init__(self, frame, **kwargs):
        self._frame = frame
        self._served = False
        self.released = False

    def open(self):
        return self

    def get_frame(self):
        if not self._served:
            self._served = True
            return self._frame.copy()
        return None

    def release(self):
        self.released = True


class TestFireVisionStartFailure:
    def test_start_returns_false_when_camera_open_raises(self, monkeypatch):
        """VisionSystem.open()打不开摄像头时抛RuntimeError(不是像
        cv2.VideoCapture.isOpened()那样返回False)——start()要把这个异常转换成
        统一的"返回False"约定，不能让main.py被这个异常打断。"""
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", _FakeVisionOpenFails)
        fv = FireVision()
        assert fv.start() is False

    def test_latest_before_any_frame_is_all_none(self):
        fv = FireVision()
        latest = fv.latest()
        assert latest["dx_px"] is None
        assert latest["dy_px"] is None
        assert latest["t"] == 0.0


class TestFireVisionBackgroundLoop:
    def test_loop_publishes_smoothed_detection_from_captured_frames(self, monkeypatch):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1)  # window=1: 单帧即可出结果，不用等多帧
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)
        fv.stop()

        latest = fv.latest()
        assert latest["dx_px"] == pytest.approx(0.0, abs=5)
        assert latest["dy_px"] == pytest.approx(0.0, abs=5)
        assert latest["t"] > 0.0

    def test_stop_releases_camera(self, monkeypatch):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1)
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)
        fv.stop()

        assert fake_vision.released is True
