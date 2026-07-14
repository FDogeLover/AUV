"""pole_vision 颜色检测纯函数单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_pole_vision.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest

from Lcode.pole_vision import detect_target, azimuth_from_dx, CAMERA_FOCAL_PX


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_rect_bgr(frame, color_bgr, x_center, half_w=60, half_h=200):
    y_center = frame.shape[0] // 2
    frame[y_center - half_h:y_center + half_h,
          x_center - half_w:x_center + half_w] = color_bgr
    return frame


class TestDetectTarget:
    def test_no_target_returns_none_none(self):
        frame = _blank_frame()
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None

    def test_red_rectangle_detected_as_red(self):
        frame = _blank_frame()
        # OpenCV红色HSV在色相环两端(0附近和180附近)，用纯红BGR(0,0,255)覆盖两段其中一段
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px == pytest.approx(0.0, abs=5)

    def test_green_rectangle_detected_as_green(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "green"

    def test_dx_px_positive_when_target_right_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=1200)  # 画面中心960，目标在右侧
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px > 0

    def test_dx_px_negative_when_target_left_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=700)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px < 0

    def test_colors_filter_ignores_unlisted_color(self):
        """颜色锁定用：只传锁定的颜色列表，画面里出现另一色也应该忽略。"""
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)  # 只画绿色
        dx_px, color = detect_target(frame, colors=("red",))  # 只找红色
        assert dx_px is None
        assert color is None

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960, half_w=2, half_h=2)  # 4x4像素，远小于阈值
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None


class TestAzimuthFromDx:
    def test_zero_dx_is_zero_azimuth(self):
        assert azimuth_from_dx(0.0) == pytest.approx(0.0)

    def test_positive_dx_gives_positive_azimuth(self):
        assert azimuth_from_dx(500.0) > 0

    def test_negative_dx_gives_negative_azimuth(self):
        assert azimuth_from_dx(-500.0) < 0

    def test_matches_atan_formula_with_default_focal(self):
        dx = 300.0
        expected = math.atan(dx / CAMERA_FOCAL_PX)
        assert azimuth_from_dx(dx) == pytest.approx(expected)


import time

from Lcode.pole_vision import PoleVision


class _FakeCapOpenFails:
    def isOpened(self):
        return False


class _FakeCapOneFrame:
    """只在第一次read()返回一帧红色图像，之后一直返回失败，供测试用。"""

    def __init__(self, frame):
        self._frame = frame
        self._served = False
        self.released = False

    def isOpened(self):
        return True

    def read(self):
        if not self._served:
            self._served = True
            return True, self._frame
        return False, None

    def release(self):
        self.released = True


class TestPoleVisionStartFailure:
    def test_start_returns_false_when_camera_cannot_open(self, monkeypatch):
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture",
                             lambda *_a, **_k: _FakeCapOpenFails())
        pv = PoleVision()
        assert pv.start() is False

    def test_latest_before_any_frame_is_all_none(self):
        pv = PoleVision()
        latest = pv.latest()
        assert latest["dx_px"] is None
        assert latest["color"] is None
        assert latest["t"] == 0.0


class TestPoleVisionLockedColor:
    def test_set_locked_color_updates_state_and_is_readable(self):
        pv = PoleVision()
        assert pv._locked_color is None
        pv.set_locked_color("red")
        assert pv._locked_color == "red"
        pv.set_locked_color(None)
        assert pv._locked_color is None


class TestPoleVisionBackgroundLoop:
    def test_loop_publishes_detection_from_captured_frame(self, monkeypatch):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        # 后台线程读一帧需要一点时间，轮询等待而不是固定sleep
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        latest = pv.latest()
        assert latest["color"] == "red"
        assert latest["dx_px"] == pytest.approx(0.0, abs=5)
        assert latest["t"] > 0.0

    def test_stop_releases_camera_capture(self, monkeypatch):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        # release() 只会在后台线程下一次循环检测到 _running=False 后才调用，
        # 轮询等待而不是固定sleep或立即断言
        deadline = time.time() + 2.0
        while not fake_cap.released and time.time() < deadline:
            time.sleep(0.02)

        assert fake_cap.released is True
