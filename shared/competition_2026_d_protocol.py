"""D题通信协议（DCP v1）的唯一Python参考实现。"""

from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Iterable


MAGIC = 0xAA
TAIL = 0xFF
VERSION = 1
MAX_PAYLOAD = 256
_HEADER = struct.Struct("<BBBBBIHIH")
_MIN_FRAME = 1 + _HEADER.size + 2 + 1


class Device(IntEnum):
    UAV = 0x01
    CAR = 0x02
    GROUND = 0x03
    BROADCAST = 0xFF


class Flag(IntFlag):
    NONE = 0
    ACK_REQUIRED = 1 << 0
    IS_ACK = 1 << 1
    EVENT = 1 << 2
    ERROR = 1 << 3


class MessageType(IntEnum):
    HEARTBEAT = 0x01
    UAV_READY = 0x02
    CAR_START = 0x03
    ACK = 0x04
    CAR_STATE = 0x10
    UAV_STATE = 0x11
    DROP_RELEASED = 0x20
    TOUCHDOWN_CONFIRMED = 0x21
    RETAKEOFF_STARTED = 0x22
    MISSION_COMPLETE = 0x23
    FAULT_EVENT = 0x30


class UavPhase(IntEnum):
    BOOT = 0
    WAIT_T265 = 1
    READY = 2
    TAKEOFF = 3
    HOVER = 4
    INTERCEPT = 5
    FORMATION_FOLLOW = 6
    DROP = 7
    DESCEND = 8
    TOUCHDOWN = 9
    DECK_RIDE = 10
    RETAKEOFF = 11
    RETURN_H = 12
    LAND_H = 13
    COMPLETE = 14
    FAULT = 15
    TERMINAL_PREDICT = 16
    CONTROLLED_ABORT = 17
    SEARCH_TARGET = 18


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    source: int
    dest: int
    session_id: int
    seq: int
    sender_ms: int
    payload: bytes = b""
    version: int = VERSION

    def __post_init__(self) -> None:
        for name in ("message_type", "flags", "source", "dest", "version"):
            value = int(getattr(self, name))
            if not 0 <= value <= 0xFF:
                raise ValueError(f"{name}超出u8范围")
        if not 0 <= int(self.session_id) <= 0xFFFFFFFF:
            raise ValueError("session_id超出u32范围")
        if not 0 <= int(self.seq) <= 0xFFFF:
            raise ValueError("seq超出u16范围")
        if not 0 <= int(self.sender_ms) <= 0xFFFFFFFF:
            raise ValueError("sender_ms超出u32范围")
        if len(bytes(self.payload)) > MAX_PAYLOAD:
            raise ValueError("payload超过DCP v1的256字节上限")


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def encode_frame(frame: Frame) -> bytes:
    payload = bytes(frame.payload)
    body = _HEADER.pack(
        int(frame.version),
        int(frame.message_type),
        int(frame.flags),
        int(frame.source),
        int(frame.dest),
        int(frame.session_id),
        int(frame.seq),
        int(frame.sender_ms),
        len(payload),
    ) + payload
    return bytes((MAGIC,)) + body + struct.pack("<H", crc16(body)) + bytes((TAIL,))


def decode_frame(raw: bytes) -> Frame:
    raw = bytes(raw)
    if len(raw) < _MIN_FRAME or raw[0] != MAGIC or raw[-1] != TAIL:
        raise ValueError("DCP帧头尾或长度无效")
    fields = _HEADER.unpack_from(raw, 1)
    version, msg_type, flags, source, dest, session_id, seq, sender_ms, length = fields
    expected = _MIN_FRAME + length
    if length > MAX_PAYLOAD or len(raw) != expected:
        raise ValueError("DCP载荷长度无效")
    body_end = 1 + _HEADER.size + length
    body = raw[1:body_end]
    expected_crc = struct.unpack_from("<H", raw, body_end)[0]
    if crc16(body) != expected_crc:
        raise ValueError("DCP CRC错误")
    if version != VERSION:
        raise ValueError(f"不支持的DCP版本: {version}")
    return Frame(
        version=version,
        message_type=msg_type,
        flags=flags,
        source=source,
        dest=dest,
        session_id=session_id,
        seq=seq,
        sender_ms=sender_ms,
        payload=raw[1 + _HEADER.size : body_end],
    )


def seq_is_newer(candidate: int, previous: int) -> bool:
    """按u16半区规则比较序号，允许65535→0回绕。"""
    delta = (int(candidate) - int(previous)) & 0xFFFF
    return 0 < delta < 0x8000


class StreamParser:
    """按长度解析字节流；坏帧后搜索下一个0xAA恢复。"""

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buffer = bytearray()
        self.max_buffer = max(max_buffer, _MIN_FRAME)
        self.rejected = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        if len(self._buffer) > self.max_buffer:
            del self._buffer[: len(self._buffer) - self.max_buffer]
            self.rejected += 1
        frames: list[Frame] = []
        while True:
            try:
                start = self._buffer.index(MAGIC)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < _MIN_FRAME:
                break
            try:
                length = _HEADER.unpack_from(self._buffer, 1)[-1]
            except struct.error:
                break
            if length > MAX_PAYLOAD:
                del self._buffer[0]
                self.rejected += 1
                continue
            total = _MIN_FRAME + length
            if len(self._buffer) < total:
                break
            candidate = bytes(self._buffer[:total])
            try:
                frame = decode_frame(candidate)
            except ValueError:
                del self._buffer[0]
                self.rejected += 1
                continue
            del self._buffer[:total]
            frames.append(frame)
        return frames


_PAYLOAD_FORMATS = {
    MessageType.HEARTBEAT: struct.Struct("<BH"),
    MessageType.UAV_READY: struct.Struct("<BHI"),
    MessageType.CAR_START: struct.Struct("<BI"),
    MessageType.ACK: struct.Struct("<BHB"),
    MessageType.CAR_STATE: struct.Struct("<BHhhH"),
    MessageType.UAV_STATE: struct.Struct("<BiihhhBH"),
    MessageType.DROP_RELEASED: struct.Struct("<IB"),
    MessageType.TOUCHDOWN_CONFIRMED: struct.Struct("<IB"),
    MessageType.RETAKEOFF_STARTED: struct.Struct("<I"),
    MessageType.MISSION_COMPLETE: struct.Struct("<BI"),
    MessageType.FAULT_EVENT: struct.Struct("<HBH"),
}


def pack_payload(message_type: MessageType | int, values: Iterable[int]) -> bytes:
    try:
        spec = _PAYLOAD_FORMATS[MessageType(message_type)]
    except (KeyError, ValueError) as exc:
        raise ValueError("未知DCP载荷类型") from exc
    return spec.pack(*tuple(values))


def unpack_payload(message_type: MessageType | int, payload: bytes) -> tuple[int, ...]:
    try:
        spec = _PAYLOAD_FORMATS[MessageType(message_type)]
    except (KeyError, ValueError) as exc:
        raise ValueError("未知DCP载荷类型") from exc
    if len(payload) != spec.size:
        raise ValueError("DCP载荷长度与消息类型不匹配")
    return spec.unpack(payload)
