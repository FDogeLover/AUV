from types import SimpleNamespace

import numpy as np
import pytest

from Lcode import qr_vision


class Mapping:
    content_to_number = {"near": 11, "far": 20}


def _result(content, left, top, size=80):
    return SimpleNamespace(
        data=content.encode("utf-8"),
        polygon=[
            (left, top),
            (left + size, top),
            (left + size, top + size),
            (left, top + size),
        ],
        rect=None,
    )


def test_target_roi_selects_nearest_qr_not_pyzbar_result_order(monkeypatch):
    # detection_width=800 → scale=0.625 for 1280px frame.
    # Full-frame QR at (620,340,size=80) has downscaled center (413,238).
    # Mapping back to full frame: (413/0.625, 238/0.625) ≈ (661, 381).
    monkeypatch.setattr(
        qr_vision,
        "pyzbar_decode",
        lambda _image: [
            _result("far", 438, 63, size=50),
            _result("near", 388, 213, size=50),
        ],
    )
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center[0] == pytest.approx(661.0, abs=1.0)
    assert detection.center[1] == pytest.approx(381.0, abs=1.0)


def test_opencv_fallback_decodes_when_pyzbar_fails(monkeypatch):
    import cv2
    calls = []

    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    original_detectAndDecode = cv2.QRCodeDetector.detectAndDecode
    def fake_detect_and_decode(self, image):
        calls.append(("ocv", image.shape[:2]))
        if image.shape[:2] == (450, 800):
            return True, "near", np.array([[[0, 0], [100, 0], [100, 100], [0, 100]]], dtype=np.float32)
        return False, "", None

    monkeypatch.setattr(cv2.QRCodeDetector, "detectAndDecode", fake_detect_and_decode)
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert ("ocv", (450, 800)) in calls


def test_airborne_path_does_not_call_slow_decode_search(monkeypatch):
    """When both pyzbar and OpenCV fail, detect() returns None without _decode_search."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    decode_search_called = []
    monkeypatch.setattr(
        decoder,
        "_decode_search",
        lambda *a, **kw: decode_search_called.append(True) or None,
    )

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None
    assert not decode_search_called, "_decode_search must NOT be called in airborne scan loop"


def test_pyzbar_failure_with_no_geometry_does_not_trigger_decode_search(monkeypatch):
    """When both pyzbar and geometry search fail with a target_point, detect()
    must return None immediately without calling the slow _decode_search."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    decode_search_called = []
    monkeypatch.setattr(
        decoder,
        "_decode_search",
        lambda *a, **kw: decode_search_called.append(True) or None,
    )

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None
    assert not decode_search_called, "_decode_search must NOT be called in airborne scan loop"
