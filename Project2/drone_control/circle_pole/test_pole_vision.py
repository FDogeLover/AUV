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
