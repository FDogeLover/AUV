import numpy as np

from Lcode.qr_vision import QRDecoder
from Lcode.vision_servo import VisionServoConfig, servo_command


def test_servo_command_is_bounded_and_adjusts_x_and_z():
    config = VisionServoConfig(
        x_kp_cmd_per_px=0.1,
        z_kp_m_per_px=0.01,
        max_x_command=5.0,
        max_z_adjust_m=0.2,
    )
    x_cmd, z_target = servo_command(config, 100.0, 30.0, 1.4, 1.4, 1)
    assert x_cmd == 5.0
    assert z_target == 1.2


def test_negative_face_reverses_lateral_direction():
    config = VisionServoConfig(x_kp_cmd_per_px=0.1)
    assert config.x_direction("A") == 1
    assert config.x_direction("B") == -1


def test_qr_geometry_is_available_without_bypassing_content_consensus():
    class Detector:
        def detectAndDecode(self, frame):
            points = np.array(
                [[[10, 10], [90, 10], [90, 90], [10, 90]]],
                dtype=np.float32,
            )
            return "", points, None

    mapping = type("Mapping", (), {"content_to_number": {"known": 1}})()
    decoder = QRDecoder(mapping, detection_width=0, detector=Detector())
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    geometry = decoder.detect_geometry(frame)
    assert geometry is not None
    assert geometry.number is None
    assert decoder.detect(frame) is None


def test_fast_geometry_search_uses_native_center_roi():
    class Detector:
        def __init__(self):
            self.shapes = []

        def detect(self, frame):
            self.shapes.append(frame.shape[:2])
            height, width = frame.shape[:2]
            if width != 560:
                return False, None
            return True, np.array(
                [[[210, 180], [350, 180], [350, 320], [210, 320]]],
                dtype=np.float32,
            )

    detector = Detector()
    mapping = type("Mapping", (), {"content_to_number": {}})()
    decoder = QRDecoder(
        mapping,
        detection_width=0,
        detector=detector,
        geometry_roi_width=560,
        geometry_roi_height=600,
    )
    geometry = decoder.detect_geometry(np.zeros((720, 1280, 3), dtype=np.uint8), decode_content=False)
    assert geometry is not None
    assert geometry.center == (640.0, 310.0)
    assert detector.shapes[0] == (600, 560)


def test_fast_geometry_search_upscales_only_roi_after_native_miss():
    class Detector:
        def __init__(self):
            self.shapes = []

        def detect(self, frame):
            self.shapes.append(frame.shape[:2])
            if frame.shape[:2] != (1200, 1120):
                return False, None
            return True, np.array(
                [[[420, 480], [700, 480], [700, 760], [420, 760]]],
                dtype=np.float32,
            )

    detector = Detector()
    mapping = type("Mapping", (), {"content_to_number": {}})()
    decoder = QRDecoder(mapping, detection_width=0, detector=detector)
    geometry = decoder.detect_geometry(
        np.zeros((720, 1280, 3), dtype=np.uint8), decode_content=False
    )

    assert geometry is not None
    assert geometry.center == (640.0, 370.0)
    assert detector.shapes[0] == (600, 560)
    assert (1200, 1120) in detector.shapes


def test_detect_decodes_content_from_localized_geometry():
    class Detector:
        def detect(self, frame):
            if frame.shape[:2] != (1200, 1120):
                return False, None
            return True, np.array(
                [[[420, 480], [700, 480], [700, 760], [420, 760]]],
                dtype=np.float32,
            )

        def detectAndDecode(self, frame):
            return "known", None, None

    mapping = type("Mapping", (), {"content_to_number": {"known": 1}})()
    decoder = QRDecoder(mapping, detection_width=0, detector=Detector())
    detection = decoder.detect(np.zeros((720, 1280, 3), dtype=np.uint8))

    assert detection is not None
    assert detection.number == 1
    assert detection.content == "known"


def test_fast_geometry_search_rejects_degenerate_detector_points():
    class Detector:
        def detect(self, frame):
            return True, np.array(
                [[[100, 180], [240, 180], [240, 180], [100, 180]]],
                dtype=np.float32,
            )

    mapping = type("Mapping", (), {"content_to_number": {}})()
    decoder = QRDecoder(mapping, detection_width=0, detector=Detector())
    geometry = decoder.detect_geometry(
        np.zeros((720, 1280, 3), dtype=np.uint8), decode_content=False
    )
    assert geometry is None


def test_target_point_selects_nearest_qr_candidate():
    mapping = type("Mapping", (), {"content_to_number": {}})()
    candidates = [
        type("Candidate", (), {"center": (300.0, 300.0)})(),
        type("Candidate", (), {"center": (900.0, 520.0)})(),
    ]
    selected = QRDecoder._select_candidate(
        candidates,
        (720, 1280, 3),
        target_point=(960.0, 540.0),
    )
    assert selected is candidates[1]
