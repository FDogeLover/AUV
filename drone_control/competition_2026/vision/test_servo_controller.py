"""
test_servo_controller.py — VisualServoController 状态机单元测试（v3）

使用合成帧 + mock，不依赖真机。
运行：cd drone_control/competition_2026 && python -m pytest vision/test_servo_controller.py -v
"""

import time
import numpy as np
import pytest

from .servo_controller import ServoConfig, ServoTick, VisualServoController
from .square_detector import SquareDetector


# ─────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────

def _frame_with_square(cx=160, cy=120, half=40, w=320, h=240):
    """生成含黑色方块的 BGR 帧。"""
    f = np.full((h, w, 3), 200, dtype=np.uint8)
    f[cy - half:cy + half, cx - half:cx + half] = 0
    return f


def _blank():
    return np.full((240, 320, 3), 200, dtype=np.uint8)


def _fast_cfg(**kw):
    defaults = dict(
        search_timeout_s=0.3,
        centering_timeout_s=1.0,
        centering_consec_frames=3,
        centering_threshold_m=0.30,   # 放宽，合成图在1m高度误差约0.1m
        focal_length_px=400.0,
        alt_stop_m=0.30,
        max_correction_cm_s=30.0,
        kp=1.0,
    )
    defaults.update(kw)
    return ServoConfig(**defaults)


# ─────────────────────────────────────────────────────────────────
# 高度截止：低于 alt_stop_m → done
# ─────────────────────────────────────────────────────────────────

class TestAltitudeStop:
    def test_below_threshold_returns_done(self):
        ctrl = VisualServoController(config=_fast_cfg())
        tick = ctrl.tick(_blank(), altitude_m=0.20)
        assert tick.done
        assert tick.reason == "alt_below_stop"

    def test_above_threshold_not_done(self):
        ctrl = VisualServoController(config=_fast_cfg())
        tick = ctrl.tick(_blank(), altitude_m=1.0)
        assert not tick.done
        assert not tick.failed


# ─────────────────────────────────────────────────────────────────
# SEARCHING 超时
# ─────────────────────────────────────────────────────────────────

class TestSearchTimeout:
    def test_no_frame_times_out(self):
        ctrl = VisualServoController(config=_fast_cfg(search_timeout_s=0.1))
        tick = ServoTick()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            tick = ctrl.tick(None, altitude_m=1.0)
            if tick.failed:
                break
            time.sleep(0.02)
        assert tick.failed
        assert tick.reason == "search_timeout"

    def test_blank_frame_times_out(self):
        ctrl = VisualServoController(config=_fast_cfg(search_timeout_s=0.1))
        tick = ServoTick()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            tick = ctrl.tick(_blank(), altitude_m=1.0)
            if tick.failed:
                break
            time.sleep(0.02)
        assert tick.failed


# ─────────────────────────────────────────────────────────────────
# CENTERING 正常收敛
# ─────────────────────────────────────────────────────────────────

class TestCenteringConvergence:
    def test_centered_square_eventually_done(self):
        """方块在画面正中，连续 K 帧应返回 done=True。"""
        ctrl = VisualServoController(config=_fast_cfg(centering_consec_frames=3))
        frame = _frame_with_square(160, 120, 40)
        tick = ServoTick()
        for _ in range(50):
            tick = ctrl.tick(frame, altitude_m=1.0)
            if tick.done or tick.failed:
                break
        assert tick.done, f"Expected done=True, got failed={tick.failed} reason={tick.reason}"

    def test_off_center_returns_nonzero_velocity(self):
        """方块偏离中心，应返回非零速度修正。"""
        ctrl = VisualServoController(config=_fast_cfg(centering_consec_frames=100))
        frame = _frame_with_square(240, 120, 40)  # 偏右 80px
        # 先进入 CENTERING
        for _ in range(5):
            tick = ctrl.tick(frame, altitude_m=1.0)
        assert tick.state == "CENTERING"
        assert abs(tick.vy_cm_s) > 0.1, "Expected nonzero vy for off-center target"


# ─────────────────────────────────────────────────────────────────
# CENTERING 超时
# ─────────────────────────────────────────────────────────────────

class TestCenteringTimeout:
    def test_centering_timeout_returns_failed(self):
        """方块存在但无法收敛，centering 超时应返回 failed。"""
        # 极严格阈值：永远无法收敛
        ctrl = VisualServoController(
            config=_fast_cfg(centering_timeout_s=0.2, centering_threshold_m=0.0001)
        )
        frame = _frame_with_square(160, 120, 40)
        tick = ServoTick()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            tick = ctrl.tick(frame, altitude_m=1.0)
            if tick.failed or tick.done:
                break
            time.sleep(0.02)
        assert tick.failed
        assert tick.reason == "centering_timeout"


# ─────────────────────────────────────────────────────────────────
# reset() 重置
# ─────────────────────────────────────────────────────────────────

class TestReset:
    def test_reset_clears_state(self):
        """reset() 后应重新从 SEARCHING 开始。"""
        ctrl = VisualServoController(config=_fast_cfg())
        frame = _frame_with_square()
        # 推进到 CENTERING
        for _ in range(10):
            ctrl.tick(frame, altitude_m=1.0)
        # reset
        ctrl.reset()
        tick = ctrl.tick(None, altitude_m=1.0)
        assert tick.state == "SEARCHING"
        assert not tick.done
        assert not tick.failed


# ─────────────────────────────────────────────────────────────────
# 速度修正方向
# ─────────────────────────────────────────────────────────────────

class TestCorrectionDirection:
    def test_target_right_positive_vy(self):
        """目标在画面右侧 → vy > 0（飞机向右修正）。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        frame = _frame_with_square(cx=250, cy=120, half=30)  # 偏右
        for _ in range(5):
            tick = ctrl.tick(frame, altitude_m=1.0)
        if tick.state == "CENTERING":
            assert tick.vy_cm_s > 0, "Target right → vy should be positive"

    def test_target_left_negative_vy(self):
        """目标在画面左侧 → vy < 0（飞机向左修正）。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        frame = _frame_with_square(cx=70, cy=120, half=30)   # 偏左
        for _ in range(5):
            tick = ctrl.tick(frame, altitude_m=1.0)
        if tick.state == "CENTERING":
            assert tick.vy_cm_s < 0, "Target left → vy should be negative"
