import unittest

import cv2
import numpy as np

try:
    from .detector import BlueSquareDetector, FeatureFlag
except ImportError:
    from detector import BlueSquareDetector, FeatureFlag


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


if __name__ == "__main__":
    unittest.main()
