"""
servo_controller.py — tick-based IBVS 视觉伺服控制器（v3）

设计原则：
  - 无副作用：tick() 不调用任何飞控接口，只返回速度修正量
  - 无阻塞：每次调用在毫秒内完成，适合在30ms 主循环中直接调用
  - 无线程：状态机完全内置，由调用方（Mission_GPT）驱动

调用方（Mission_GPT._visual_servo_tick）负责：
  - 每30ms 调用 tick(frame, altitude_m)
  - 读取 ServoTick.vx_cm_s / vy_cm_s 并写入 set_speed()
  - 收到 done=True / failed=True 时转换状态机至 LAND
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from .square_detector import DetectionResult, SquareDetector


# ═══════════════════════════════════════════════════════════════════════ #
# Config
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class ServoConfig:
    # 相机参数（需现场标定，未标定时用保守估计）
    focal_length_px: float = 400.0

    # 高度阈值
    alt_stop_m: float = 0.30        # 低于此高度停止修正，返回 done=True（交 Mission_GPT 盲降）

    # 收敛判据
    centering_threshold_m: float = 0.05    # 位置误差 < 5cm 视为对中
    centering_consec_frames: int = 5       # 连续满足的帧数

    # 速度限制
    max_correction_cm_s: float = 30.0     # 单轴最大修正速度（cm/s）

    # 比例增益（默认 1.0：误差1m → 速度100cm/s，再经 max_correction_cm_s 限幅）
    kp: float = 1.0

    # 超时
    search_timeout_s: float = 5.0
    centering_timeout_s: float = 15.0


# ═══════════════════════════════════════════════════════════════════════ #
# Output dataclass
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class ServoTick:
    vx_cm_s: float = 0.0    # 前后速度修正（cm/s），+ 为前进
    vy_cm_s: float = 0.0    # 左右速度修正（cm/s），+ 为右移
    state: str = "SEARCHING"
    done: bool = False       # 对中成功，可以降落
    failed: bool = False     # 超时 / 异常，建议直接转 LAND 坐标兜底
    reason: str = ""


# ═══════════════════════════════════════════════════════════════════════ #
# Controller
# ═══════════════════════════════════════════════════════════════════════ #

class _State(Enum):
    SEARCHING = auto()
    CENTERING = auto()


class VisualServoController:
    """
    tick-based IBVS 控制器。

    使用示例（Mission_GPT._visual_servo_tick 中）：

        tick = self._vs_ctrl.tick(frame, pos[2])
        if tick.done or tick.failed:
            logger.info("[VS] → LAND  reason=%s", tick.reason)
            self.state = "LAND"
        else:
            self.set_speed(tick.vx_cm_s, tick.vy_cm_s, 0, current_z)
    """

    def __init__(
        self,
        config: Optional[ServoConfig] = None,
        detector: Optional[SquareDetector] = None,
    ) -> None:
        self._cfg = config or ServoConfig()
        self._detector = detector or SquareDetector()
        self._state = _State.SEARCHING
        self._state_start = time.monotonic()
        self._consec = 0

    def reset(self) -> None:
        """重置状态机。每次从 NAVIGATE 进入 VISUAL_SERVO 状态前调用。"""
        self._state = _State.SEARCHING
        self._state_start = time.monotonic()
        self._consec = 0

    # ------------------------------------------------------------------ #
    def tick(
        self,
        frame: Optional[np.ndarray],
        altitude_m: float,
    ) -> ServoTick:
        """
        由 Mission_GPT.loop() 每 30ms 调用一次。

        Parameters
        ----------
        frame      : BGR 帧（np.ndarray），None 表示本轮相机无帧
        altitude_m : 当前高度（米），来自 Mission_GPT loop 的 pos[2]
                     （已含激光高度计覆盖逻辑）
        """
        cfg = self._cfg
        now = time.monotonic()
        elapsed = now - self._state_start

        # ── 低于盲降阈值：停止修正，通知降落 ────────────────────────── #
        if altitude_m < cfg.alt_stop_m:
            return ServoTick(
                done=True,
                state=self._state.name,
                reason="alt_below_stop",
            )

        # ── 检测 ─────────────────────────────────────────────────────── #
        det: Optional[DetectionResult] = None
        if frame is not None:
            det = self._detector.detect(frame)
        found = det is not None and det.found and not det.too_close

        # ── SEARCHING ────────────────────────────────────────────────── #
        if self._state == _State.SEARCHING:
            if elapsed > cfg.search_timeout_s:
                return ServoTick(
                    failed=True,
                    state="SEARCHING",
                    reason="search_timeout",
                )
            if found:
                self._state = _State.CENTERING
                self._state_start = now
                self._consec = 0
            return ServoTick(state="SEARCHING")

        # ── CENTERING ────────────────────────────────────────────────── #
        if elapsed > cfg.centering_timeout_s:
            return ServoTick(
                failed=True,
                state="CENTERING",
                reason="centering_timeout",
            )

        if not found:
            # 丢帧：重置连续计数，保持速度为0（悬停等待）
            self._consec = 0
            return ServoTick(state="CENTERING")

        # 计算位置误差（米）和速度修正（cm/s）
        vx, vy, err_m = self._compute(det, altitude_m, cfg)

        if err_m < cfg.centering_threshold_m:
            self._consec += 1
        else:
            self._consec = 0

        if self._consec >= cfg.centering_consec_frames:
            return ServoTick(
                done=True,
                state="CENTERING",
                reason="centered",
            )

        return ServoTick(vx_cm_s=vx, vy_cm_s=vy, state="CENTERING")

    # ------------------------------------------------------------------ #
    def _compute(
        self,
        det: DetectionResult,
        altitude_m: float,
        cfg: ServoConfig,
    ):
        """
        像素误差 → 位置误差（米）→ 速度修正（cm/s）。

        坐标映射（需真机验证符号方向，标准相机朝下情形）：
          图像 x 轴（左→右）对应飞机 y 轴（左→右）
          图像 y 轴（上→下）对应飞机 x 轴（后→前，注意方向）
        """
        err_x_px = det.cx_px - det.frame_w / 2.0   # 正值：目标在图像右侧
        err_y_px = det.cy_px - det.frame_h / 2.0   # 正值：目标在图像下方

        # 像素 → 米（针孔相机模型）
        scale = altitude_m / cfg.focal_length_px
        err_x_m = err_x_px * scale   # 飞机 y 轴偏差（米）
        err_y_m = err_y_px * scale   # 飞机 x 轴偏差（米）
        err_m = (err_x_m ** 2 + err_y_m ** 2) ** 0.5

        # 米 → cm/s（比例控制，限幅）
        clamp = cfg.max_correction_cm_s
        # 修正方向：目标在右 → 向右飞（vy > 0）
        vy = float(max(min(err_x_m * 100.0 * cfg.kp, clamp), -clamp))
        # 修正方向：目标在下（图像） → 向前飞（vx > 0，按飞机前方定义）
        vx = float(max(min(err_y_m * 100.0 * cfg.kp, clamp), -clamp))

        return vx, vy, err_m
