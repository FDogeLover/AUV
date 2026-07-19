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
