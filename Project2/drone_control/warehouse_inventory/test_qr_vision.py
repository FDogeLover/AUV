from types import SimpleNamespace

import numpy as np

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
    # In the 560x600 center ROI, the optical target (640,360 full-frame) is
    # (280,300). Return the far code first to reproduce pyzbar's unspecified
    # multi-code ordering.
    monkeypatch.setattr(
        qr_vision,
        "pyzbar_decode",
        lambda _image: [
            _result("far", 450, 260),
            _result("near", 240, 260),
        ],
    )
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center == (640.0, 360.0)


def test_adaptive_three_x_variant_maps_polygon_back_to_full_frame(monkeypatch):
    calls = []

    def decode(image):
        calls.append((image.ndim, image.shape[:2]))
        # Only the final 3x adaptive-threshold image succeeds.
        if image.ndim == 2 and image.shape[:2] == (1800, 1680):
            return [_result("near", 720, 780, size=240)]
        return []

    monkeypatch.setattr(qr_vision, "pyzbar_decode", decode)
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center == (640.0, 360.0)
    assert (2, (1800, 1680)) in calls


def test_pyzbar_failure_falls_through_to_opencv_geometry_search(monkeypatch):
    """When pyzbar returns nothing, detect() should not return None
    immediately but continue to the OpenCV geometry search path."""
    # pyzbar always fails
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    # Track whether _fast_geometry_search was called
    geo_called = []

    def fake_geometry_search(frame, target_point=None):
        geo_called.append(True)
        # Return a plausible QR quad near the center of the frame
        pts = np.array(
            [[500, 200], [700, 200], [700, 400], [500, 400]],
            dtype=np.float32,
        )
        return SimpleNamespace(corners=pts)

    monkeypatch.setattr(decoder, "_fast_geometry_search", fake_geometry_search)

    # Track whether _decode_localized was called
    local_decode_called = []

    def fake_decode_localized(frame, corners):
        local_decode_called.append(True)
        return "near"

    monkeypatch.setattr(decoder, "_decode_localized", fake_decode_localized)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    # The fallback path was reached
    assert geo_called, "_fast_geometry_search was never called"
    assert local_decode_called, "_decode_localized was never called"
    assert detection is not None
    assert detection.number == 11


def test_pyzbar_failure_with_no_geometry_does_not_trigger_decode_search(monkeypatch):
    """When both pyzbar and geometry search fail with a target_point, detect()
    must return None immediately without calling the slow _decode_search."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    # Geometry search finds nothing (common when QR not in FOV / overexposed)
    monkeypatch.setattr(decoder, "_fast_geometry_search", lambda *a, **kw: None)

    decode_search_called = []
    monkeypatch.setattr(
        decoder,
        "_decode_search",
        lambda *a, **kw: decode_search_called.append(True) or None,
    )

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None
    assert not decode_search_called, "_decode_search must NOT be called in airborne scan loop"
