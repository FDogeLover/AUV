"""移动平台世界位置/速度的轻量alpha-beta跟踪器。"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlatformEstimate:
    x_m: float
    y_m: float
    vx_m_s: float
    vy_m_s: float
    timestamp: float
    uncertainty_m: float
    predicted: bool


@dataclass(frozen=True)
class TrackerConfig:
    alpha: float = 0.55
    beta: float = 0.12
    max_speed_m_s: float = 1.5
    max_innovation_m: float = 0.8
    max_predict_s: float = 0.5
    initial_uncertainty_m: float = 0.25
    uncertainty_growth_m_s: float = 0.35


class PlatformTracker:
    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self._state: PlatformEstimate | None = None
        self.rejected = 0

    @property
    def initialized(self) -> bool:
        return self._state is not None

    def reset(self) -> None:
        self._state = None
        self.rejected = 0

    def update(self, x_m: float, y_m: float, timestamp: float, quality: int = 100) -> PlatformEstimate | None:
        if not (math.isfinite(x_m) and math.isfinite(y_m) and math.isfinite(timestamp)):
            self.rejected += 1
            return None
        if self._state is None:
            self._state = PlatformEstimate(
                x_m, y_m, 0.0, 0.0, timestamp,
                self.config.initial_uncertainty_m, False,
            )
            return self._state
        if timestamp <= self._state.timestamp:
            self.rejected += 1
            return None
        dt = timestamp - self._state.timestamp
        pred_x = self._state.x_m + self._state.vx_m_s * dt
        pred_y = self._state.y_m + self._state.vy_m_s * dt
        innovation_x = x_m - pred_x
        innovation_y = y_m - pred_y
        innovation = math.hypot(innovation_x, innovation_y)
        allowed = self.config.max_innovation_m + self.config.max_speed_m_s * min(dt, 1.0)
        if innovation > allowed:
            self.rejected += 1
            return None
        quality_scale = max(0.15, min(1.0, quality / 100.0))
        alpha = self.config.alpha * quality_scale
        beta = self.config.beta * quality_scale
        x = pred_x + alpha * innovation_x
        y = pred_y + alpha * innovation_y
        vx = self._state.vx_m_s + beta * innovation_x / dt
        vy = self._state.vy_m_s + beta * innovation_y / dt
        speed = math.hypot(vx, vy)
        if speed > self.config.max_speed_m_s:
            scale = self.config.max_speed_m_s / speed
            vx *= scale
            vy *= scale
        uncertainty = max(0.015, (1.0 - alpha) * (self._state.uncertainty_m + 0.05 * dt))
        self._state = PlatformEstimate(x, y, vx, vy, timestamp, uncertainty, False)
        return self._state

    def predict(self, timestamp: float) -> PlatformEstimate | None:
        if self._state is None or timestamp < self._state.timestamp:
            return None
        dt = timestamp - self._state.timestamp
        if dt > self.config.max_predict_s:
            return None
        return PlatformEstimate(
            self._state.x_m + self._state.vx_m_s * dt,
            self._state.y_m + self._state.vy_m_s * dt,
            self._state.vx_m_s,
            self._state.vy_m_s,
            timestamp,
            self._state.uncertainty_m + self.config.uncertainty_growth_m_s * dt,
            dt > 0,
        )
