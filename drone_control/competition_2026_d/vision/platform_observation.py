from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag


class FeatureFlag(IntFlag):
    OUTER_VALID = 1 << 0
    INNER_VALID = 1 << 1
    CROSS_VALID = 1 << 2
    PARTIAL = 1 << 3
    TOO_CLOSE = 1 << 4
    AMBIGUOUS = 1 << 5


@dataclass(frozen=True)
class PlatformObservation:
    stream_id: int
    seq: int
    capture_ms: int
    found: bool
    cx: int
    cy: int
    outer_px: int
    inner_px: int
    angle_cdeg: int
    quality: int
    flags: int
    received_monotonic: float

    def age_s(self, now: float) -> float:
        return max(0.0, float(now) - self.received_monotonic)

    def usable(self, now: float, max_age_s: float, min_quality: int) -> bool:
        bad = FeatureFlag.TOO_CLOSE | FeatureFlag.AMBIGUOUS
        return (
            self.found
            and self.age_s(now) <= max_age_s
            and self.quality >= min_quality
            and not (FeatureFlag(self.flags) & bad)
        )
