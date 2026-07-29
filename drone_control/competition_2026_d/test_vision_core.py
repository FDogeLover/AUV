import unittest

import cv2
import numpy as np

from CyberCamera.boards.cybercam_d.detector import FeatureFlag, PlatformDetector
from CyberCamera.boards.cybercam_d.protocol import encode
from CyberCamera.boards.cybercam_d.camera_backend import WalnutPiCSICapture
from drone_control.competition_2026_d.control.formation_controller import FormationController
from drone_control.competition_2026_d.vision.camera_model import CameraIntrinsics, DownwardCameraModel
from drone_control.competition_2026_d.vision.cybercam_protocol import ObservationGate, parse_line
from drone_control.competition_2026_d.vision.platform_tracker import PlatformTracker


class VisionCoreTest(unittest.TestCase):
    def test_external_csi_adapter_uses_walnutpi_sensor_contract(self):
        calls = []

        class FakeSensor:
            def __init__(self, width, height):
                calls.append((width, height))
                self.released = False
                self.hmirror = 0

            def isOpened(self):
                return True

            def set_hmirror(self, value):
                self.hmirror = value

            def set_vflip(self, _value):
                pass

            def read(self):
                return True, np.zeros((240, 320, 3), np.uint8)

            def release(self):
                self.released = True

        capture = WalnutPiCSICapture(320, 240, hmirror=True, sensor_factory=FakeSensor)
        self.assertEqual(calls, [(320, 240)])
        self.assertTrue(capture.isOpened())
        ok, frame = capture.read()
        self.assertTrue(ok)
        self.assertEqual(frame.shape, (240, 320, 3))
        capture.release()
        self.assertTrue(capture._sensor.released)

    def test_vs1_roundtrip_and_stream_resync(self):
        line = encode(7, 10, 1000, True, 321, 242, 150, 90, 125, 88, 7)
        obs = parse_line(line, received_monotonic=2.0)
        self.assertEqual((obs.stream_id, obs.seq, obs.angle_cdeg), (7, 10, 125))
        gate = ObservationGate(confirm_frames=3)
        self.assertIsNotNone(gate.accept(obs))
        self.assertIsNone(gate.accept(obs))
        for seq in (0, 1):
            new = parse_line(encode(8, seq, 2000 + seq, True, 10, 20, 0, 80, 0, 70, 14))
            self.assertIsNone(gate.accept(new))
        accepted = gate.accept(parse_line(encode(8, 2, 2002, True, 10, 20, 0, 80, 0, 70, 14)))
        self.assertIsNotNone(accepted)
        self.assertEqual(gate.active_stream, 8)

    def test_concentric_target_detection(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        center = (350, 210)
        cv2.circle(image, center, 100, (0, 0, 0), 10)
        cv2.circle(image, center, 60, (0, 0, 0), 8)
        cv2.line(image, (center[0] - 35, center[1]), (center[0] + 35, center[1]), (0, 0, 0), 7)
        cv2.line(image, (center[0], center[1] - 35), (center[0], center[1] + 35), (0, 0, 0), 7)
        result = PlatformDetector().detect(image)
        self.assertTrue(result.found)
        self.assertLess(abs(result.cx - center[0]), 6)
        self.assertLess(abs(result.cy - center[1]), 6)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.INNER_VALID)

    def test_large_inner_circle_and_cross_support_partial_near_mode(self):
        image = np.full((240, 320, 3), 255, np.uint8)
        center = (160, 120)
        cv2.circle(image, center, 55, (0, 0, 0), 8)
        cv2.line(image, (120, 120), (200, 120), (0, 0, 0), 9)
        cv2.line(image, (160, 80), (160, 160), (0, 0, 0), 9)
        result = PlatformDetector().detect(image)
        self.assertTrue(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.CROSS_VALID)

    def test_camera_signs_tracker_and_controller_limits(self):
        model = DownwardCameraModel(CameraIntrinsics(500, 500, 320, 240))
        forward, right = model.body_relative_xy(370, 290, 1.0)
        self.assertGreater(forward, 0)
        self.assertGreater(right, 0)
        tracker = PlatformTracker()
        tracker.update(0.0, 0.0, 0.0)
        estimate = tracker.update(0.1, 0.0, 0.1)
        self.assertIsNotNone(estimate)
        controller = FormationController()
        controller.reset(0.1)
        command = controller.command(estimate, (0, 0), (0, 0), 0.13)
        self.assertTrue(command.valid)
        self.assertLessEqual((command.vx_m_s ** 2 + command.vy_m_s ** 2) ** 0.5, 0.40)


if __name__ == "__main__":
    unittest.main()
