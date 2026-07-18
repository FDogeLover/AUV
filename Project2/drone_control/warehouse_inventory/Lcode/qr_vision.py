"""二维码解码、多帧确认与可开关的真实飞行图片留档。"""

import json
import math
import os
import threading
import time
from collections import Counter, deque
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

from Lcode.Logger import logger

try:
    import cv2
except ImportError:
    cv2 = None


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class QRDetection:
    number: int
    content: str
    corners: Tuple[Point2D, Point2D, Point2D, Point2D]

    @property
    def center(self) -> Point2D:
        return (
            sum(point[0] for point in self.corners) / 4.0,
            sum(point[1] for point in self.corners) / 4.0,
        )


class QRMapping:
    def __init__(self, mapping_file):
        self.path = Path(mapping_file)
        self.number_to_content = {}
        self.content_to_number = {}
        self.load()

    def load(self):
        if not self.path.exists():
            raise FileNotFoundError(f"二维码映射不存在: {self.path}")
        for line_number, line in enumerate(
            self.path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                number_text, content = line.split("\t", 1)
                number = int(number_text)
            except ValueError as exc:
                raise ValueError(f"二维码映射第{line_number}行无效") from exc
            content = content.strip()
            if not 1 <= number <= 24 or not content:
                raise ValueError(f"二维码映射第{line_number}行超出范围")
            if number in self.number_to_content or content in self.content_to_number:
                raise ValueError("二维码映射包含重复编号或内容")
            self.number_to_content[number] = content
            self.content_to_number[content] = number
        if set(self.number_to_content) != set(range(1, 25)):
            raise ValueError("二维码映射必须完整包含1~24")


class QRDecoder:
    def __init__(self, mapping: QRMapping, detection_width=800, detector=None):
        if cv2 is None and detector is None:
            raise RuntimeError("二维码识别需要OpenCV")
        self.mapping = mapping
        self.detection_width = int(detection_width)
        self.detector = detector or cv2.QRCodeDetector()

    def detect(self, frame) -> Optional[QRDetection]:
        height, width = frame.shape[:2]
        scale = 1.0
        detection_frame = frame
        if self.detection_width > 0 and width > self.detection_width:
            scale = self.detection_width / width
            detection_frame = cv2.resize(
                frame,
                (self.detection_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        content, points, _ = self.detector.detectAndDecode(detection_frame)
        if not content or points is None:
            return None
        number = self.mapping.content_to_number.get(content.strip())
        if number is None:
            return None
        raw_points = points.reshape(-1, 2)
        if len(raw_points) != 4:
            return None
        corners = tuple((float(x / scale), float(y / scale)) for x, y in raw_points)
        return QRDetection(number, content.strip(), corners)


def _distance_to_line(point: Point2D, a: Point2D, b: Point2D) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return 0.0
    return abs(dy * point[0] - dx * point[1] + b[0] * a[1] - b[1] * a[0]) / length


def point_inside_qr(corners: Sequence[Point2D], point: Point2D, margin_px=0.0) -> bool:
    if len(corners) != 4:
        return False
    signs = []
    for index, a in enumerate(corners):
        b = corners[(index + 1) % len(corners)]
        cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])
        signs.append(cross)
        if _distance_to_line(point, a, b) < margin_px:
            return False
    return all(value >= 0 for value in signs) or all(value <= 0 for value in signs)


@dataclass(frozen=True)
class QRConsensusConfig:
    window_size: int = 5
    required_count: int = 3
    laser_margin_px: float = 12.0

    def __post_init__(self):
        if not 1 <= self.required_count <= self.window_size <= 30:
            raise ValueError("二维码多帧确认参数无效")
        if self.laser_margin_px < 0:
            raise ValueError("激光像素安全边距不能为负")


class QRConsensus:
    def __init__(self, config: QRConsensusConfig = None):
        self.config = config or QRConsensusConfig()
        self._window = deque(maxlen=self.config.window_size)

    def reset(self):
        self._window.clear()

    def update(self, detection: Optional[QRDetection], laser_aim_px: Point2D):
        valid = (
            detection
            if detection is not None
            and point_inside_qr(
                detection.corners, laser_aim_px, self.config.laser_margin_px
            )
            else None
        )
        self._window.append(valid)
        counts = Counter(item.number for item in self._window if item is not None)
        if not counts:
            return None
        number, count = counts.most_common(1)[0]
        if count < self.config.required_count:
            return None
        return next(item for item in reversed(self._window) if item and item.number == number)


@dataclass(frozen=True)
class VisionDebugConfig:
    enabled: bool = False
    transit_interval_s: float = 1.0
    transit_interval_m: float = 0.25

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        enabled = env.get("DRONE_VISION_DEBUG_CAPTURE", "0").strip().lower()
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_VISION_DEBUG_CAPTURE只能是0/1/false/true")
        return cls(
            enabled=enabled in {"1", "true"},
            transit_interval_s=float(env.get("DRONE_VISION_CAPTURE_INTERVAL_S", "1.0")),
            transit_interval_m=float(env.get("DRONE_VISION_CAPTURE_INTERVAL_M", "0.25")),
        )


class VisionDebugCapture:
    def __init__(self, directory, config: VisionDebugConfig = None, image_writer=None):
        self.directory = Path(directory)
        self.config = config or VisionDebugConfig.from_env()
        self._image_writer = image_writer or (cv2.imwrite if cv2 is not None else None)
        self._last_time = None
        self._last_position = None
        self._counter = 0
        self._lock = threading.Lock()

    def _save(self, frame, prefix, metadata) -> Optional[Path]:
        if not self.config.enabled or self._image_writer is None:
            return None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            with self._lock:
                self._counter += 1
                stem = f"{int(time.time() * 1000)}_{self._counter:05d}_{prefix}"
            image_path = self.directory / f"{stem}.jpg"
            if not self._image_writer(str(image_path), frame):
                raise RuntimeError("图像编码失败")
            metadata_path = self.directory / f"{stem}.json"
            metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            return image_path
        except Exception as exc:
            logger.error(f"视觉调试图片保存失败，飞行继续: {exc}")
            return None

    def capture_fixed(self, frame, metadata) -> Optional[Path]:
        return self._save(frame, "fixed", metadata)

    def maybe_capture_transit(self, frame, position_xy, metadata, now=None) -> Optional[Path]:
        if not self.config.enabled:
            return None
        now = time.monotonic() if now is None else float(now)
        position = (float(position_xy[0]), float(position_xy[1]))
        time_due = self._last_time is None or now - self._last_time >= self.config.transit_interval_s
        distance_due = (
            self._last_position is None
            or math.hypot(position[0] - self._last_position[0], position[1] - self._last_position[1])
            >= self.config.transit_interval_m
        )
        if not (time_due or distance_due):
            return None
        path = self._save(frame, "transit", metadata)
        if path is not None:
            self._last_time = now
            self._last_position = position
        return path
