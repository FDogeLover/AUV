"""30/50 cm同心圆环与十字的多尺度检测器。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag

import cv2
import numpy as np


class FeatureFlag(IntFlag):
    OUTER_VALID = 1 << 0
    INNER_VALID = 1 << 1
    CROSS_VALID = 1 << 2
    PARTIAL = 1 << 3
    TOO_CLOSE = 1 << 4
    AMBIGUOUS = 1 << 5


@dataclass(frozen=True)
class PlatformDetection:
    found: bool
    cx: int = 0
    cy: int = 0
    outer_px: int = 0
    inner_px: int = 0
    angle_cdeg: int = 0
    quality: int = 0
    flags: int = 0


@dataclass(frozen=True)
class _Ellipse:
    cx: float
    cy: float
    major: float
    minor: float
    angle: float
    area: float

    @property
    def diameter(self) -> float:
        return 0.5 * (self.major + self.minor)


class PlatformDetector:
    """先找同心椭圆对；近地时退化到内圆/十字部分特征。"""

    def __init__(
        self,
        min_diameter_px: float = 12.0,
        min_axis_ratio: float = 0.55,
        diameter_ratio: tuple[float, float] = (1.35, 2.05),
        max_center_error_ratio: float = 0.12,
        too_close_ratio: float = 0.82,
        min_partial_diameter_px: float = 30.0,
        min_partial_cross_score: float = 0.60,
    ) -> None:
        self.min_diameter_px = min_diameter_px
        self.min_axis_ratio = min_axis_ratio
        self.diameter_ratio = diameter_ratio
        self.max_center_error_ratio = max_center_error_ratio
        self.too_close_ratio = too_close_ratio
        self.min_partial_diameter_px = min_partial_diameter_px
        self.min_partial_cross_score = min_partial_cross_score

    def detect(self, frame: np.ndarray) -> PlatformDetection:
        if frame is None or frame.size == 0:
            return PlatformDetection(False)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 31, 7,
        )
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        ellipses = self._ellipses(contours)
        pair = self._best_pair(ellipses)
        h, w = gray.shape[:2]
        if pair is not None:
            outer, inner, score = pair
            cx = int(round((outer.cx + inner.cx) * 0.5))
            cy = int(round((outer.cy + inner.cy) * 0.5))
            cross = self._cross_score(binary, cx, cy, max(4, int(inner.diameter * 0.28)))
            flags = FeatureFlag.OUTER_VALID | FeatureFlag.INNER_VALID
            if cross >= 0.42:
                flags |= FeatureFlag.CROSS_VALID
            if outer.diameter > self.too_close_ratio * min(w, h):
                flags |= FeatureFlag.TOO_CLOSE
            quality = int(round(max(0.0, min(1.0, 0.72 * score + 0.28 * cross)) * 100))
            return PlatformDetection(
                True, cx, cy, int(round(outer.diameter)), int(round(inner.diameter)),
                int(round(outer.angle * 100.0)), quality, int(flags),
            )

        # 外圆出画后，完整的内圆仍可能保留。要求圆度和中心十字共同成立，
        # 避免把赛道单条黑线当作近地目标。
        partial = self._best_partial(ellipses, binary, w, h)
        if partial is None:
            return PlatformDetection(False)
        ellipse, cross = partial
        flags = FeatureFlag.INNER_VALID | FeatureFlag.CROSS_VALID | FeatureFlag.PARTIAL
        if ellipse.diameter > self.too_close_ratio * min(w, h):
            flags |= FeatureFlag.TOO_CLOSE
        quality = int(round(min(0.78, 0.38 + 0.40 * cross) * 100))
        return PlatformDetection(
            True, int(round(ellipse.cx)), int(round(ellipse.cy)), 0,
            int(round(ellipse.diameter)), int(round(ellipse.angle * 100.0)),
            quality, int(flags),
        )

    def _ellipses(self, contours) -> list[_Ellipse]:
        result: list[_Ellipse] = []
        for contour in contours:
            if len(contour) < 20:
                continue
            area = abs(float(cv2.contourArea(contour)))
            if area < 30.0:
                continue
            (cx, cy), (a, b), angle = cv2.fitEllipse(contour)
            major, minor = max(a, b), min(a, b)
            if minor < self.min_diameter_px or minor / max(major, 1e-6) < self.min_axis_ratio:
                continue
            result.append(_Ellipse(cx, cy, major, minor, angle, area))
        return result

    def _best_pair(self, ellipses: list[_Ellipse]):
        best = None
        for outer in ellipses:
            for inner in ellipses:
                if outer.diameter <= inner.diameter:
                    continue
                ratio = outer.diameter / max(inner.diameter, 1e-6)
                if not self.diameter_ratio[0] <= ratio <= self.diameter_ratio[1]:
                    continue
                distance = ((outer.cx - inner.cx) ** 2 + (outer.cy - inner.cy) ** 2) ** 0.5
                center_norm = distance / max(inner.diameter, 1.0)
                if center_norm > self.max_center_error_ratio:
                    continue
                ratio_score = 1.0 - min(abs(ratio - 5.0 / 3.0) / 0.45, 1.0)
                center_score = 1.0 - center_norm / self.max_center_error_ratio
                score = 0.55 * ratio_score + 0.45 * center_score
                if best is None or score > best[2]:
                    best = (outer, inner, score)
        return best

    def _best_partial(self, ellipses, binary, width, height):
        candidates = []
        for ellipse in ellipses:
            if ellipse.diameter < self.min_partial_diameter_px:
                continue
            if ellipse.minor / max(ellipse.major, 1e-6) < 0.70:
                continue
            margin = 0.06 * min(width, height)
            if not (margin <= ellipse.cx <= width - margin and margin <= ellipse.cy <= height - margin):
                continue
            cross = self._cross_score(
                binary, int(round(ellipse.cx)), int(round(ellipse.cy)),
                max(4, int(ellipse.diameter * 0.28)),
            )
            if cross >= self.min_partial_cross_score:
                candidates.append((ellipse, cross))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0].diameter * item[1])

    @staticmethod
    def _cross_score(binary: np.ndarray, cx: int, cy: int, radius: int) -> float:
        h, w = binary.shape[:2]
        x0, x1 = max(0, cx - radius), min(w, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(h, cy + radius + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return 0.0
        thickness = max(1, radius // 10)
        horizontal = binary[max(y0, cy - thickness):min(y1, cy + thickness + 1), x0:x1]
        vertical = binary[y0:y1, max(x0, cx - thickness):min(x1, cx + thickness + 1)]
        if horizontal.size == 0 or vertical.size == 0:
            return 0.0
        return float(min(np.mean(horizontal > 0), np.mean(vertical > 0)))
