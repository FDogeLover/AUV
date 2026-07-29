"""基于/dev/bt_serial的DCP v1双向、非阻塞链路。"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    StreamParser,
    encode_frame,
    pack_payload,
    unpack_payload,
)


LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class LinkConfig:
    port: str = "/dev/bt_serial"
    baudrate: int = 115200
    queue_size: int = 128
    read_timeout_s: float = 0.02
    write_timeout_s: float = 0.03
    ack_timeout_s: float = 0.12
    max_retries: int = 4
    seen_events: int = 128


@dataclass
class LinkStats:
    tx_frames: int = 0
    rx_frames: int = 0
    tx_dropped: int = 0
    rx_rejected: int = 0
    retries: int = 0
    ack_timeouts: int = 0
    duplicate_events: int = 0
    io_errors: int = 0


@dataclass
class _Pending:
    frame: Frame
    raw: bytes
    deadline: float
    retries_left: int


class AirGroundLink:
    """收发线程不进入30ms飞行循环；回调只投递已校验帧。"""

    def __init__(self, config: LinkConfig | None = None, serial_factory=None) -> None:
        self.config = config or LinkConfig()
        self._serial_factory = serial_factory
        self._serial = None
        self._parser = StreamParser()
        self._tx_queue: queue.Queue[bytes] = queue.Queue(maxsize=self.config.queue_size)
        self._rx_queue: queue.Queue[Frame] = queue.Queue(maxsize=self.config.queue_size)
        self._callback_queue: queue.Queue[Frame] = queue.Queue(maxsize=self.config.queue_size)
        self._pending: dict[tuple[int, int, int], _Pending] = {}
        self._pending_lock = threading.Lock()
        self._callbacks: list[Callable[[Frame], None]] = []
        self._running = threading.Event()
        self._threads: list[threading.Thread] = []
        self._sequence = 0
        self._seen_order: deque[tuple[int, int, int, int]] = deque()
        self._seen_set: set[tuple[int, int, int, int]] = set()
        self.stats = LinkStats()

    def add_callback(self, callback: Callable[[Frame], None]) -> None:
        self._callbacks.append(callback)

    def start(self) -> bool:
        if self._running.is_set():
            return True
        try:
            factory = self._serial_factory
            if factory is None:
                import serial
                factory = serial.Serial
            self._serial = factory(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.read_timeout_s,
                write_timeout=self.config.write_timeout_s,
            )
        except Exception as exc:
            LOG.error("蓝牙链路打开失败(%s): %s", self.config.port, exc)
            self.stats.io_errors += 1
            return False
        self._running.set()
        self._threads = [
            threading.Thread(target=self._rx_worker, daemon=True, name="dcp-rx"),
            threading.Thread(target=self._tx_worker, daemon=True, name="dcp-tx"),
            threading.Thread(target=self._dispatch_worker, daemon=True, name="dcp-dispatch"),
        ]
        for thread in self._threads:
            thread.start()
        return True

    def close(self) -> None:
        self._running.clear()
        for thread in self._threads:
            thread.join(timeout=0.5)
        self._threads.clear()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def publish(
        self,
        message_type: MessageType | int,
        payload: bytes,
        *,
        session_id: int,
        dest: Device | int = Device.BROADCAST,
        flags: Flag | int = Flag.NONE,
        sender_ms: int | None = None,
    ) -> int | None:
        if not self._running.is_set():
            return None
        seq = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        frame = Frame(
            message_type=int(message_type), flags=int(flags), source=Device.UAV,
            dest=int(dest), session_id=session_id, seq=seq,
            sender_ms=(int(time.monotonic() * 1000) if sender_ms is None else sender_ms) & 0xFFFFFFFF,
            payload=bytes(payload),
        )
        raw = encode_frame(frame)
        if not self._enqueue(raw):
            return None
        if int(flags) & int(Flag.ACK_REQUIRED):
            key = (int(message_type), seq, session_id)
            with self._pending_lock:
                self._pending[key] = _Pending(
                    frame, raw, time.monotonic() + self.config.ack_timeout_s,
                    self.config.max_retries,
                )
        return seq

    def get_nowait(self) -> Frame | None:
        try:
            return self._rx_queue.get_nowait()
        except queue.Empty:
            return None

    def _enqueue(self, raw: bytes) -> bool:
        try:
            self._tx_queue.put_nowait(raw)
            return True
        except queue.Full:
            try:
                self._tx_queue.get_nowait()
                self._tx_queue.task_done()
            except queue.Empty:
                pass
            self.stats.tx_dropped += 1
            try:
                self._tx_queue.put_nowait(raw)
                return True
            except queue.Full:
                self.stats.tx_dropped += 1
                return False

    def _rx_worker(self) -> None:
        while self._running.is_set():
            try:
                data = self._serial.read(256)
                if not data:
                    continue
                frames = self._parser.feed(data)
                self.stats.rx_rejected = self._parser.rejected
                for frame in frames:
                    if frame.dest not in (Device.UAV, Device.BROADCAST):
                        continue
                    self.stats.rx_frames += 1
                    self._handle_frame(frame)
            except Exception as exc:
                self.stats.io_errors += 1
                LOG.error("DCP接收失败，关闭链路: %s", exc)
                self._running.clear()

    def _handle_frame(self, frame: Frame) -> None:
        if frame.message_type == MessageType.ACK or frame.flags & Flag.IS_ACK:
            try:
                acked_type, acked_seq, _result = unpack_payload(MessageType.ACK, frame.payload)
            except ValueError:
                self.stats.rx_rejected += 1
                return
            with self._pending_lock:
                self._pending.pop((acked_type, acked_seq, frame.session_id), None)
            return
        event_key = (frame.source, frame.session_id, frame.message_type, frame.seq)
        duplicate = event_key in self._seen_set
        if frame.flags & Flag.ACK_REQUIRED:
            self._send_ack(frame)
        if duplicate:
            self.stats.duplicate_events += 1
            return
        if frame.flags & Flag.EVENT or frame.flags & Flag.ACK_REQUIRED:
            self._remember(event_key)
        try:
            self._rx_queue.put_nowait(frame)
        except queue.Full:
            try:
                self._rx_queue.get_nowait()
                self._rx_queue.task_done()
            except queue.Empty:
                pass
            self._rx_queue.put_nowait(frame)
        if self._callbacks:
            try:
                self._callback_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._callback_queue.get_nowait()
                    self._callback_queue.task_done()
                except queue.Empty:
                    pass
                self._callback_queue.put_nowait(frame)

    def _send_ack(self, received: Frame) -> None:
        payload = pack_payload(MessageType.ACK, (received.message_type, received.seq, 0))
        self.publish(
            MessageType.ACK, payload, session_id=received.session_id,
            dest=received.source, flags=Flag.IS_ACK,
        )

    def _remember(self, key: tuple[int, int, int, int]) -> None:
        self._seen_order.append(key)
        self._seen_set.add(key)
        while len(self._seen_order) > self.config.seen_events:
            self._seen_set.discard(self._seen_order.popleft())

    def _tx_worker(self) -> None:
        while self._running.is_set() or not self._tx_queue.empty():
            self._service_retries()
            try:
                raw = self._tx_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                self._serial.write(raw)
                self.stats.tx_frames += 1
            except Exception as exc:
                self.stats.io_errors += 1
                LOG.error("DCP发送失败，关闭链路: %s", exc)
                self._running.clear()
            finally:
                self._tx_queue.task_done()

    def _service_retries(self) -> None:
        now = time.monotonic()
        resend: list[bytes] = []
        with self._pending_lock:
            for key, pending in list(self._pending.items()):
                if now < pending.deadline:
                    continue
                if pending.retries_left <= 0:
                    self._pending.pop(key, None)
                    self.stats.ack_timeouts += 1
                    continue
                pending.retries_left -= 1
                pending.deadline = now + self.config.ack_timeout_s
                resend.append(pending.raw)
                self.stats.retries += 1
        for raw in resend:
            self._enqueue(raw)

    def _dispatch_worker(self) -> None:
        while self._running.is_set():
            try:
                frame = self._callback_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                for callback in tuple(self._callbacks):
                    try:
                        callback(frame)
                    except Exception:
                        LOG.exception("DCP回调异常，已隔离")
            finally:
                self._callback_queue.task_done()
