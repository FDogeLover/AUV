"""Cyber Camera到Pi的VS1 ASCII协议。"""

from __future__ import annotations

import binascii


BAUDRATE = 115200
FIELD_COUNT = 13


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def encode(
    stream_id: int,
    seq: int,
    capture_ms: int,
    found: bool,
    cx: int,
    cy: int,
    outer_px: int,
    inner_px: int,
    angle_cdeg: int,
    quality: int,
    flags: int,
) -> bytes:
    values = (
        "VS1",
        str(int(stream_id) & 0xFFFF),
        str(int(seq) & 0xFFFFFFFF),
        str(int(capture_ms) & 0xFFFFFFFF),
        "1" if found else "0",
        str(int(cx)),
        str(int(cy)),
        str(max(0, int(outer_px))),
        str(max(0, int(inner_px))),
        str(int(angle_cdeg)),
        str(max(0, min(100, int(quality)))),
        str(int(flags) & 0xFFFF),
    )
    body = ",".join(values).encode("ascii")
    return body + b"," + f"{crc16(body):04X}".encode("ascii") + b"\n"
