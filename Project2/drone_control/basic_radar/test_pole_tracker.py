"""PoleTracker 世界系重构单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.Lradar import (
    radar_angle_to_body_xy,
    body_to_world_xy,
    world_to_body_angle_dist,
)


# ════════════════════ body_to_world_xy / world_to_body_angle_dist ════════════════════

class TestBodyWorldTransform:
    def test_zero_yaw_identity_shift(self):
        # yaw=0 时，世界系就是机体系平移了飞机当前位置
        bx, by = radar_angle_to_body_xy(0, 1000)  # 机体正前方1m
        wx, wy = body_to_world_xy(2.0, -1.0, 0.0, bx, by, yaw_sign=1)
        assert wx == pytest.approx(3.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_yaw_90deg_sign_positive(self):
        bx, by = radar_angle_to_body_xy(0, 1000)  # bx=1.0, by=0.0
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(1.0, abs=1e-6)

    def test_yaw_90deg_sign_negative(self):
        # yaw_sign 翻转应该翻转旋转方向，这是给以后真机标定用的开关
        bx, by = radar_angle_to_body_xy(0, 1000)
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=-1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_round_trip_recovers_original_angle_distance(self):
        # world_to_body_angle_dist 是 body_to_world_xy 的逆变换：
        # 任意机体系候选点转到世界系、再转回来，应该拿回原始角度/距离
        angle_deg, distance_mm = 30.0, 800.0
        x_m, y_m, yaw_rad, sign = 2.0, -1.5, math.radians(40), 1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)

    def test_round_trip_with_negative_sign(self):
        angle_deg, distance_mm = 200.0, 650.0
        x_m, y_m, yaw_rad, sign = -0.6, 0.07, math.radians(3.0), -1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)
