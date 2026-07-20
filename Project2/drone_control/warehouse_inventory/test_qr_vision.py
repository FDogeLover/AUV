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
    # 全帧 adaptiveThreshold + pyzbar 路径中，target_point 应选择最近的 QR。
    monkeypatch.setattr(
        qr_vision,
        "pyzbar_decode",
        lambda _image: [
            _result("far", 700, 100),
            _result("near", 620, 340),
        ],
    )
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center == (660.0, 380.0)


def test_adaptive_full_frame_variant_decodes_qr(monkeypatch):
    calls = []

    def decode(image):
        calls.append((image.ndim, image.shape[:2]))
        # 全帧 adaptiveThreshold(block=31/51) 为灰度且形状为 (720, 1280) 时成功
        if image.ndim == 2 and image.shape == (720, 1280):
            return [_result("near", 620, 340, size=120)]
        return []

    monkeypatch.setattr(qr_vision, "pyzbar_decode", decode)
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center == (680.0, 400.0)
    assert any(shape == (720, 1280) for (_ndim, shape) in calls)


def test_pyzbar_failure_in_airborne_path_does_not_run_opencv_fallback(monkeypatch):
    """Target-aware flight decode must remain bounded when pyzbar finds nothing."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    def forbidden(*args, **kwargs):
        raise AssertionError("slow OpenCV fallback must not run in airborne path")

    monkeypatch.setattr(decoder, "_fast_geometry_search", forbidden)
    monkeypatch.setattr(decoder, "_decode_search", forbidden)

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None


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
