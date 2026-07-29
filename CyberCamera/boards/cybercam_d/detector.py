"""30/50 cm同心圆环与十字的多尺度检测器。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag
import time

import cv2
import numpy as np


class FeatureFlag(IntFlag):
    OUTER_VALID = 1 << 0
    INNER_VALID = 1 << 1
    CROSS_VALID = 1 << 2
    PARTIAL = 1 << 3
    TOO_CLOSE = 1 << 4
    AMBIGUOUS = 1 << 5
    SURROGATE_SQUARE = 1 << 6
    APRILTAG_VALID = 1 << 7
    TEMPORAL_TRACKED = 1 << 8
    COLOR_SHAPE_TRACKED = 1 << 9


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
    debug_polygon: tuple[tuple[int, int], ...] = ()


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


class AprilTagDetector:
    """Detect one configured AprilTag using grayscale geometry only."""

    _DICTIONARIES = {
        "tag16h5": "DICT_APRILTAG_16H5",
        "tag25h9": "DICT_APRILTAG_25H9",
        "tag36h10": "DICT_APRILTAG_36H10",
        "tag36h11": "DICT_APRILTAG_36H11",
    }

    def __init__(
        self,
        family: str = "tag36h11",
        tag_id: int = 0,
        min_side_px: float = 18.0,
        detect_scale: float = 0.75,
        redetect_interval: int = 5,
        max_flow_age_s: float = 0.5,
        min_flow_points: int = 6,
    ) -> None:
        family = str(family).lower()
        if family not in self._DICTIONARIES:
            raise ValueError(f"unsupported AprilTag family: {family}")
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
            raise RuntimeError("OpenCV aruco/AprilTag support is unavailable")
        dictionary_name = self._DICTIONARIES[family]
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f"OpenCV lacks {dictionary_name}")
        self.family = family
        self.tag_id = int(tag_id)
        self.min_side_px = max(1.0, float(min_side_px))
        self.detect_scale = max(0.25, min(1.0, float(detect_scale)))
        self.redetect_interval = max(1, int(redetect_interval))
        self.max_flow_age_s = max(0.1, float(max_flow_age_s))
        self.min_flow_points = max(4, int(min_flow_points))
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
        parameters = cv2.aruco.DetectorParameters()
        # OpenCV's APRILTAG corner refiner drops the GC2093 live stream to
        # about 1.6 FPS on Cyber Camera. Decoding already supplies four
        # corners; Kalman filtering is performed on the RDK, so keep the
        # detector unrefined here to preserve the observation rate.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        # The 12 cm printed tag is about 56 px wide at 1.5 m. Use the full
        # dictionary correction capability and denser canonical cell sampling
        # to reduce intermittent bit errors after the half-scale search.
        parameters.errorCorrectionRate = 1.0
        parameters.perspectiveRemovePixelPerCell = 8
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self._frame_index = 0
        self._prev_gray: np.ndarray | None = None
        self._track_points: np.ndarray | None = None
        self._track_polygon: np.ndarray | None = None
        self._last_direct_time: float | None = None

    @staticmethod
    def _clip01(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _candidate(self, corners: np.ndarray, width: int, height: int):
        points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        contour = points.reshape(-1, 1, 2)
        if not cv2.isContourConvex(contour):
            return None
        if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width):
            return None
        if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
            return None
        sides = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
        mean_side = float(np.mean(sides))
        area = abs(float(cv2.contourArea(contour)))
        (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(contour)
        rect_area = float(rect_w * rect_h)
        if mean_side < self.min_side_px or area <= 0.0 or rect_area <= 0.0:
            return None
        border = float(np.min(np.column_stack((
            points[:, 0], width - 1.0 - points[:, 0],
            points[:, 1], height - 1.0 - points[:, 1],
        ))))
        size_score = self._clip01((mean_side - self.min_side_px) / 22.0)
        margin_score = self._clip01(border / max(0.35 * mean_side, 1.0))
        consistency_score = self._clip01(
            1.0 - (float(np.max(sides)) - float(np.min(sides)))
            / max(0.30 * mean_side, 1.0)
        )
        shape_score = self._clip01((area / rect_area - 0.55) / 0.35)
        quality = int(round(100.0 * min(
            size_score, margin_score, consistency_score, shape_score,
        )))
        center = np.mean(points, axis=0)
        top_edge = points[1] - points[0]
        angle_deg = float(np.degrees(np.arctan2(top_edge[1], top_edge[0])))
        polygon = tuple(
            (int(round(float(point[0]))), int(round(float(point[1]))))
            for point in points
        )
        return PlatformDetection(
            True,
            int(round(float(center[0]))),
            int(round(float(center[1]))),
            int(round(mean_side)),
            0,
            int(round(angle_deg * 100.0)),
            quality,
            int(FeatureFlag.APRILTAG_VALID),
            polygon,
        )

    def _direct_detect(self, gray: np.ndarray) -> PlatformDetection:
        if self.detect_scale < 1.0:
            detection_gray = cv2.resize(
                gray, None, fx=self.detect_scale, fy=self.detect_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            detection_gray = gray
        corners, ids, _ = self._detector.detectMarkers(detection_gray)
        if ids is None or len(ids) == 0:
            return PlatformDetection(False)
        valid = []
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            if self.detect_scale < 1.0:
                marker_corners = np.asarray(marker_corners) / self.detect_scale
            candidate = self._candidate(marker_corners, gray.shape[1], gray.shape[0])
            if candidate is not None:
                valid.append((int(marker_id), candidate))
        if not valid:
            return PlatformDetection(False)
        if len(valid) != 1 or valid[0][0] != self.tag_id:
            return PlatformDetection(False, flags=int(FeatureFlag.AMBIGUOUS))
        return valid[0][1]

    def _reset_track(self) -> None:
        self._prev_gray = None
        self._track_points = None
        self._track_polygon = None
        self._last_direct_time = None

    def _start_track(
        self, gray: np.ndarray, detection: PlatformDetection, now: float
    ) -> None:
        polygon = np.asarray(detection.debug_polygon, dtype=np.float32).reshape(4, 2)
        mask = np.zeros(gray.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(polygon).astype(np.int32), 255)
        features = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=40,
            qualityLevel=0.01,
            minDistance=4,
            mask=mask,
            blockSize=5,
        )
        corners = polygon.reshape(-1, 1, 2)
        if features is None:
            points = corners
        else:
            points = np.concatenate((features.astype(np.float32), corners), axis=0)
        self._prev_gray = gray.copy()
        self._track_points = points.astype(np.float32)
        self._track_polygon = polygon
        self._last_direct_time = float(now)

    def _flow_detect(self, gray: np.ndarray, now: float) -> PlatformDetection:
        if (
            self._prev_gray is None
            or self._track_points is None
            or self._track_polygon is None
            or self._last_direct_time is None
            or now - self._last_direct_time > self.max_flow_age_s
        ):
            return PlatformDetection(False)
        current, status_forward, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray,
            self._track_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
        )
        if current is None or status_forward is None:
            return PlatformDetection(False)
        backward, status_backward, _ = cv2.calcOpticalFlowPyrLK(
            gray,
            self._prev_gray,
            current,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
        )
        if backward is None or status_backward is None:
            return PlatformDetection(False)
        old = self._track_points.reshape(-1, 2)
        new = current.reshape(-1, 2)
        back = backward.reshape(-1, 2)
        h, w = gray.shape[:2]
        valid = (
            status_forward.reshape(-1).astype(bool)
            & status_backward.reshape(-1).astype(bool)
            & (np.linalg.norm(old - back, axis=1) <= 1.5)
            & (new[:, 0] >= 0.0)
            & (new[:, 0] < w)
            & (new[:, 1] >= 0.0)
            & (new[:, 1] < h)
        )
        old_good = old[valid]
        new_good = new[valid]
        if len(old_good) < self.min_flow_points:
            return PlatformDetection(False)
        affine, inliers = cv2.estimateAffinePartial2D(
            old_good,
            new_good,
            method=cv2.RANSAC,
            ransacReprojThreshold=2.5,
            maxIters=500,
            confidence=0.99,
            refineIters=5,
        )
        if affine is None or inliers is None:
            return PlatformDetection(False)
        inlier_mask = inliers.reshape(-1).astype(bool)
        inlier_count = int(np.count_nonzero(inlier_mask))
        if (
            inlier_count < self.min_flow_points
            or inlier_count / max(len(old_good), 1) < 0.55
        ):
            return PlatformDetection(False)
        scale = float(np.hypot(affine[0, 0], affine[0, 1]))
        translation = float(np.hypot(affine[0, 2], affine[1, 2]))
        if not 0.82 <= scale <= 1.22 or translation > 0.30 * min(w, h):
            return PlatformDetection(False)
        homogeneous = np.column_stack((self._track_polygon, np.ones(4)))
        polygon = homogeneous @ affine.T
        candidate = self._candidate(polygon, w, h)
        if candidate is None:
            return PlatformDetection(False)
        forward_backward = np.linalg.norm(old_good[inlier_mask] - back[valid][inlier_mask], axis=1)
        error_score = self._clip01(1.0 - float(np.median(forward_backward)) / 1.5)
        inlier_score = inlier_count / max(len(old_good), 1)
        flow_quality = int(round(55.0 + 35.0 * min(error_score, inlier_score)))
        quality = min(candidate.quality, flow_quality)
        self._prev_gray = gray.copy()
        self._track_points = new_good[inlier_mask].reshape(-1, 1, 2).astype(np.float32)
        self._track_polygon = polygon.astype(np.float32)
        return PlatformDetection(
            True,
            candidate.cx,
            candidate.cy,
            candidate.outer_px,
            candidate.inner_px,
            candidate.angle_cdeg,
            quality,
            int(FeatureFlag.APRILTAG_VALID | FeatureFlag.TEMPORAL_TRACKED),
            candidate.debug_polygon,
        )

    def detect(self, frame: np.ndarray) -> PlatformDetection:
        if frame is None or frame.size == 0 or frame.ndim not in (2, 3):
            self._reset_track()
            return PlatformDetection(False)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        now = time.monotonic()
        self._frame_index += 1
        direct_attempted = False
        should_redetect = (
            self._track_points is None
            or self._frame_index % self.redetect_interval == 0
        )
        if should_redetect:
            direct_attempted = True
            direct = self._direct_detect(gray)
            if direct.found:
                self._start_track(gray, direct, now)
                return direct
            if FeatureFlag(direct.flags) & FeatureFlag.AMBIGUOUS:
                self._reset_track()
                return direct
        tracked = self._flow_detect(gray, now)
        if tracked.found:
            return tracked
        if not direct_attempted:
            direct = self._direct_detect(gray)
            if direct.found:
                self._start_track(gray, direct, now)
                return direct
            if FeatureFlag(direct.flags) & FeatureFlag.AMBIGUOUS:
                self._reset_track()
                return direct
        self._reset_track()
        return PlatformDetection(False)


class BlueSquareDetector:
    """25 cm蓝色方形临时靶标；只用于静态控制测试。"""

    def __init__(
        self,
        hsv_lower: tuple[int, int, int] = (85, 75, 55),
        hsv_upper: tuple[int, int, int] = (135, 255, 255),
        min_area_ratio: float = 0.002,
        max_area_ratio: float = 0.70,
        min_aspect_ratio: float = 0.70,
        min_rectangularity: float = 0.72,
        min_solidity: float = 0.88,
        edge_margin_px: int = 3,
        partial_min_solidity: float = 0.84,
        partial_min_thickness_px: float = 12.0,
    ) -> None:
        self.hsv_lower = np.asarray(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.asarray(hsv_upper, dtype=np.uint8)
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.min_rectangularity = min_rectangularity
        self.min_solidity = min_solidity
        self.edge_margin_px = max(0, int(edge_margin_px))
        self.partial_min_solidity = partial_min_solidity
        self.partial_min_thickness_px = partial_min_thickness_px

    def detect(self, frame: np.ndarray) -> PlatformDetection:
        if frame is None or frame.size == 0 or frame.ndim != 3:
            return PlatformDetection(False)
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = float(w * h)
        candidates = []
        for contour in contours:
            area = abs(float(cv2.contourArea(contour)))
            area_ratio = area / max(frame_area, 1.0)
            if not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                continue
            hull_area = abs(float(cv2.contourArea(cv2.convexHull(contour))))
            solidity = area / max(hull_area, 1.0)
            x, y, bw, bh = cv2.boundingRect(contour)
            touches_edge = (
                x <= self.edge_margin_px
                or y <= self.edge_margin_px
                or x + bw >= w - self.edge_margin_px
                or y + bh >= h - self.edge_margin_px
            )
            if touches_edge:
                if (
                    solidity < self.partial_min_solidity
                    or min(bw, bh) < self.partial_min_thickness_px
                ):
                    continue
                moments = cv2.moments(contour)
                if abs(moments["m00"]) > 1e-6:
                    cx = moments["m10"] / moments["m00"]
                    cy = moments["m01"] / moments["m00"]
                else:
                    cx, cy = x + 0.5 * bw, y + 0.5 * bh
                area_score = min(1.0, area_ratio / 0.04)
                quality = 100.0 * (
                    0.52 * solidity + 0.28 * area_score
                    + 0.20 * min(1.0, min(bw, bh) / 60.0)
                )
                polygon = (
                    (int(x), int(y)), (int(x + bw), int(y)),
                    (int(x + bw), int(y + bh)), (int(x), int(y + bh)),
                )
                candidates.append((
                    "partial", quality, cx, cy, 0.0, 0.0, quality, polygon,
                ))
                continue
            if solidity < self.min_solidity:
                continue
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), angle = rect
            short, long = min(rw, rh), max(rw, rh)
            if short < 8.0 or long <= 0.0:
                continue
            aspect = short / long
            rectangularity = area / max(rw * rh, 1.0)
            if aspect < self.min_aspect_ratio or rectangularity < self.min_rectangularity:
                continue
            aspect_score = (aspect - self.min_aspect_ratio) / (1.0 - self.min_aspect_ratio)
            rect_score = (rectangularity - self.min_rectangularity) / (1.0 - self.min_rectangularity)
            area_score = min(1.0, area_ratio / 0.04)
            quality = 100.0 * (
                0.34 * aspect_score + 0.30 * rect_score
                + 0.20 * solidity + 0.16 * area_score
            )
            score = quality + min(20.0, area_ratio * 200.0)
            box = cv2.boxPoints(rect)
            polygon = tuple((int(round(px)), int(round(py))) for px, py in box)
            candidates.append((
                "full", score, cx, cy, 0.5 * (rw + rh), angle, quality, polygon,
            ))
        if not candidates:
            return PlatformDetection(False)
        # 飞行安全优先：两个相互独立的合格蓝色块不能默认选最大块。
        # found=False确保控制端进入LOST；flags保留歧义原因供日志/调试显示。
        if len(candidates) > 1:
            return PlatformDetection(
                False, quality=0,
                flags=int(FeatureFlag.AMBIGUOUS | FeatureFlag.SURROGATE_SQUARE),
            )
        kind, _, cx, cy, side, angle, quality, polygon = candidates[0]
        if kind == "partial":
            return PlatformDetection(
                True,
                int(round(cx)),
                int(round(cy)),
                0,
                0,
                0,
                int(round(max(0.0, min(100.0, quality)))),
                int(FeatureFlag.PARTIAL | FeatureFlag.SURROGATE_SQUARE),
                polygon,
            )
        return PlatformDetection(
            True,
            int(round(cx)),
            int(round(cy)),
            int(round(side)),
            0,
            int(round(angle * 100.0)),
            int(round(max(0.0, min(100.0, quality)))),
            int(FeatureFlag.SURROGATE_SQUARE),
            polygon,
        )


class AprilTagBlueFusionDetector:
    """Tag确认身份，25cm蓝色外框提供高频中心观测。"""

    def __init__(
        self,
        family: str = "tag36h11",
        tag_id: int = 0,
        min_side_px: float = 18.0,
        detect_scale: float = 0.75,
        redetect_interval: int = 5,
        max_flow_age_s: float = 0.5,
    ) -> None:
        self.tag_detector = AprilTagDetector(
            family,
            tag_id,
            min_side_px,
            detect_scale,
            redetect_interval,
            max_flow_age_s,
        )
        self.blue_detector = BlueSquareDetector()
        self._identity_locked = False
        self._last_center: tuple[int, int] | None = None

    def detect(self, frame: np.ndarray) -> PlatformDetection:
        tag = self.tag_detector.detect(frame)
        tag_flags = FeatureFlag(tag.flags)
        if tag_flags & FeatureFlag.AMBIGUOUS:
            self._identity_locked = False
            self._last_center = None
            return tag
        direct_tag = (
            tag.found
            and bool(tag_flags & FeatureFlag.APRILTAG_VALID)
            and not bool(tag_flags & FeatureFlag.TEMPORAL_TRACKED)
        )
        if direct_tag:
            self._identity_locked = True
            self._last_center = (tag.cx, tag.cy)

        blue = self.blue_detector.detect(frame)
        blue_flags = FeatureFlag(blue.flags)
        if not self._identity_locked:
            # 蓝色外框不能独立确认目标身份。
            return tag if tag.found else PlatformDetection(False, flags=tag.flags)

        blue_usable = (
            blue.found
            and bool(blue_flags & FeatureFlag.SURROGATE_SQUARE)
            and not bool(blue_flags & (FeatureFlag.PARTIAL | FeatureFlag.AMBIGUOUS))
            and blue.outer_px > 0
        )
        if blue_usable:
            reference = (tag.cx, tag.cy) if tag.found else self._last_center
            if reference is not None:
                jump = float(np.hypot(blue.cx - reference[0], blue.cy - reference[1]))
                if jump > max(80.0, 0.60 * blue.outer_px):
                    blue_usable = False
            if blue_usable and direct_tag:
                ratio = blue.outer_px / max(tag.outer_px, 1)
                center_error = float(np.hypot(blue.cx - tag.cx, blue.cy - tag.cy))
                if not 1.25 <= ratio <= 2.40 or center_error > 0.25 * blue.outer_px:
                    blue_usable = False

        if blue_usable:
            self._last_center = (blue.cx, blue.cy)
            return PlatformDetection(
                True,
                blue.cx,
                blue.cy,
                blue.outer_px,
                tag.outer_px if tag.found else 0,
                blue.angle_cdeg,
                max(55, min(100, blue.quality)),
                int(FeatureFlag.APRILTAG_VALID | FeatureFlag.COLOR_SHAPE_TRACKED),
                blue.debug_polygon,
            )
        if tag.found:
            self._last_center = (tag.cx, tag.cy)
            return tag
        if blue_flags & FeatureFlag.AMBIGUOUS:
            return PlatformDetection(False, flags=int(FeatureFlag.AMBIGUOUS))
        return PlatformDetection(False)


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
        source_confirm_frames: int = 3,
        max_center_jump_ratio: float = 0.25,
    ) -> None:
        self.min_diameter_px = min_diameter_px
        self.min_axis_ratio = min_axis_ratio
        self.diameter_ratio = diameter_ratio
        self.max_center_error_ratio = max_center_error_ratio
        self.too_close_ratio = too_close_ratio
        self.min_partial_diameter_px = min_partial_diameter_px
        self.min_partial_cross_score = min_partial_cross_score
        self.source_confirm_frames = max(1, int(source_confirm_frames))
        self.max_center_jump_ratio = max_center_jump_ratio
        self._last_source: str | None = None
        self._last_center: tuple[int, int] | None = None
        self._pending_source: str | None = None
        self._pending_count = 0
        self._miss_count = 0

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
        candidate = None
        source = None
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
            candidate = PlatformDetection(
                True, cx, cy, int(round(outer.diameter)), int(round(inner.diameter)),
                int(round(outer.angle * 100.0)), quality, int(flags),
            )
            source = "full"
        else:
            # 外圆出画后，完整的内圆仍可能保留。要求圆度和中心十字共同成立，
            # 避免把赛道单条黑线当作近地目标。
            partial = self._best_partial(ellipses, binary, w, h)
            if partial is not None:
                ellipse, cross = partial
                flags = FeatureFlag.INNER_VALID | FeatureFlag.CROSS_VALID | FeatureFlag.PARTIAL
                if ellipse.diameter > self.too_close_ratio * min(w, h):
                    flags |= FeatureFlag.TOO_CLOSE
                quality = int(round(min(0.78, 0.38 + 0.40 * cross) * 100))
                candidate = PlatformDetection(
                    True, int(round(ellipse.cx)), int(round(ellipse.cy)), 0,
                    int(round(ellipse.diameter)), int(round(ellipse.angle * 100.0)),
                    quality, int(flags),
                )
                source = "inner_cross"
            else:
                cross_only = self._best_cross(binary, w, h)
                if cross_only is not None:
                    cx, cy, score = cross_only
                    candidate = PlatformDetection(
                        True, cx, cy, 0, 0, 0,
                        int(round(min(0.70, 0.30 + 0.40 * score) * 100)),
                        int(FeatureFlag.CROSS_VALID | FeatureFlag.PARTIAL),
                    )
                    source = "cross"
        if candidate is None or source is None:
            self._pending_source = None
            self._pending_count = 0
            self._note_miss()
            return PlatformDetection(False)
        return self._apply_temporal(candidate, source, w, h)

    def _note_miss(self) -> None:
        self._miss_count += 1
        if self._miss_count >= 5:
            self._last_source = None
            self._last_center = None
            self._pending_source = None
            self._pending_count = 0

    def _apply_temporal(
        self, candidate: PlatformDetection, source: str, width: int, height: int
    ) -> PlatformDetection:
        if self._last_center is not None:
            dx = candidate.cx - self._last_center[0]
            dy = candidate.cy - self._last_center[1]
            max_jump = max(24.0, self.max_center_jump_ratio * min(width, height))
            if (dx * dx + dy * dy) ** 0.5 > max_jump:
                self._note_miss()
                return PlatformDetection(
                    False, candidate.cx, candidate.cy,
                    candidate.outer_px, candidate.inner_px, candidate.angle_cdeg,
                    min(candidate.quality, 35),
                    int(FeatureFlag(candidate.flags) | FeatureFlag.AMBIGUOUS),
                )
        if self._last_source is not None and source != self._last_source:
            if source == self._pending_source:
                self._pending_count += 1
            else:
                self._pending_source = source
                self._pending_count = 1
            if self._pending_count < self.source_confirm_frames:
                return PlatformDetection(
                    True, candidate.cx, candidate.cy,
                    candidate.outer_px, candidate.inner_px, candidate.angle_cdeg,
                    min(candidate.quality, 45),
                    int(FeatureFlag(candidate.flags) | FeatureFlag.AMBIGUOUS),
                    candidate.debug_polygon,
                )
        self._last_source = source
        self._last_center = (candidate.cx, candidate.cy)
        self._miss_count = 0
        self._pending_source = None
        self._pending_count = 0
        return candidate

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
        if self._last_center is None:
            return None
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

    def _best_cross(self, binary: np.ndarray, width: int, height: int):
        """在圆已出画时寻找完整十字交点；单臂或靠边交点不成立。"""
        if self._last_center is None:
            return None
        arm = max(13, int(round(min(width, height) * 0.10)))
        horizontal = cv2.morphologyEx(
            binary, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (arm, 3)),
        )
        vertical = cv2.morphologyEx(
            binary, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, arm)),
        )
        intersections = cv2.bitwise_and(
            cv2.dilate(horizontal, np.ones((5, 5), np.uint8)),
            cv2.dilate(vertical, np.ones((5, 5), np.uint8)),
        )
        count, _, stats, centroids = cv2.connectedComponentsWithStats(intersections)
        candidates = []
        margin = 0.08 * min(width, height)
        for index in range(1, count):
            if stats[index, cv2.CC_STAT_AREA] < 6:
                continue
            cx, cy = centroids[index]
            if not (margin <= cx <= width - margin and margin <= cy <= height - margin):
                continue
            radius = max(arm, int(round(min(width, height) * 0.16)))
            score = self._cross_score(binary, int(round(cx)), int(round(cy)), radius)
            if score < max(0.68, self.min_partial_cross_score):
                continue
            if self._last_center is None:
                center_distance = ((cx - width * 0.5) ** 2 + (cy - height * 0.5) ** 2) ** 0.5
            else:
                center_distance = (
                    (cx - self._last_center[0]) ** 2 + (cy - self._last_center[1]) ** 2
                ) ** 0.5
            candidates.append((center_distance, -score, int(round(cx)), int(round(cy)), score))
        if not candidates:
            return None
        _, _, cx, cy, score = min(candidates)
        return cx, cy, score

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
