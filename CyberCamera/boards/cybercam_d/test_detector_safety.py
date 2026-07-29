import unittest

import cv2
import numpy as np

try:
    from .detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag,
    )
except ImportError:
    from detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag,
    )


def april_frame(tag_id=0, x=260, y=180, side=120):
    image = np.full((480, 640, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, side)
    image[y:y + side, x:x + side] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


def fusion_frame(tag_id=0, blue_center=(320, 240), blue_side=250, tag_side=120):
    image = np.full((480, 640, 3), 220, np.uint8)
    cx, cy = blue_center
    half = blue_side // 2
    cv2.rectangle(
        image, (cx - half, cy - half), (cx + half, cy + half),
        (255, 90, 0), -1,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_side)
    x = cx - tag_side // 2
    y = cy - tag_side // 2
    image[y:y + tag_side, x:x + tag_side] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


class AprilTagFlowSafetyTest(unittest.TestCase):
    def test_direct_decode_then_optical_flow_translation(self):
        detector = AprilTagDetector(redetect_interval=100)
        direct = detector.detect(april_frame())
        self.assertTrue(direct.found)
        self.assertFalse(FeatureFlag(direct.flags) & FeatureFlag.TEMPORAL_TRACKED)

        tracked = detector.detect(april_frame(x=268, y=186))
        self.assertTrue(tracked.found)
        self.assertTrue(FeatureFlag(tracked.flags) & FeatureFlag.APRILTAG_VALID)
        self.assertTrue(FeatureFlag(tracked.flags) & FeatureFlag.TEMPORAL_TRACKED)
        self.assertAlmostEqual(tracked.cx - direct.cx, 8, delta=2)
        self.assertAlmostEqual(tracked.cy - direct.cy, 6, delta=2)

    def test_wrong_id_clears_existing_track(self):
        detector = AprilTagDetector(redetect_interval=1)
        self.assertTrue(detector.detect(april_frame(tag_id=0)).found)
        wrong = detector.detect(april_frame(tag_id=1))
        self.assertFalse(wrong.found)
        self.assertTrue(FeatureFlag(wrong.flags) & FeatureFlag.AMBIGUOUS)
        self.assertIsNone(detector._track_points)


class BlueSquareSafetyTest(unittest.TestCase):
    def test_complete_square(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (110, 70), (210, 170), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        self.assertFalse(FeatureFlag(result.flags) & FeatureFlag.PARTIAL)

    def test_edge_square_is_partial(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (105, 185), (215, 239), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.PARTIAL)
        self.assertEqual(result.outer_px, 0)

    def test_two_candidates_are_ambiguous(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (30, 70), (100, 140), (255, 90, 0), -1)
        cv2.rectangle(image, (210, 70), (280, 140), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertFalse(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.AMBIGUOUS)

    def test_blue_line_is_rejected(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.line(image, (0, 120), (319, 120), (255, 90, 0), 5)
        self.assertFalse(BlueSquareDetector().detect(image).found)


class AprilTagBlueFusionSafetyTest(unittest.TestCase):
    def test_direct_tag_locks_identity_and_blue_drives_center(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        result = detector.detect(fusion_frame())
        flags = FeatureFlag(result.flags)
        self.assertTrue(result.found)
        self.assertTrue(flags & FeatureFlag.APRILTAG_VALID)
        self.assertTrue(flags & FeatureFlag.COLOR_SHAPE_TRACKED)
        self.assertAlmostEqual(result.cx, 320, delta=3)
        self.assertAlmostEqual(result.cy, 240, delta=3)

    def test_blue_cannot_initialize_identity_by_itself(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        image = np.full((480, 640, 3), 220, np.uint8)
        cv2.rectangle(image, (195, 115), (445, 365), (255, 90, 0), -1)
        result = detector.detect(image)
        self.assertFalse(result.found)
        self.assertFalse(detector._identity_locked)

    def test_wrong_tag_clears_identity_lock(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        self.assertTrue(detector.detect(fusion_frame()).found)
        wrong = detector.detect(fusion_frame(tag_id=1))
        self.assertFalse(wrong.found)
        self.assertTrue(FeatureFlag(wrong.flags) & FeatureFlag.AMBIGUOUS)
        self.assertFalse(detector._identity_locked)


if __name__ == "__main__":
    unittest.main()
