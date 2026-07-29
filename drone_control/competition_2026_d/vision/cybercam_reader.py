"""Cyber Camera串口读取线程；只发布CRC正确且通过stream/seq门禁的新帧。"""

from __future__ import annotations

import logging
import threading
import time

from .cybercam_protocol import ObservationGate, parse_line
from .platform_observation import PlatformObservation


LOG = logging.getLogger(__name__)


class CyberCamReader:
    def __init__(
        self,
        port: str = "/dev/ttyS3",
        baudrate: int = 115200,
        timeout_s: float = 0.02,
        max_line_bytes: int = 256,
        serial_factory=None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.max_line_bytes = max_line_bytes
        self.serial_factory = serial_factory
        self._serial = None
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._gate = ObservationGate()
        self._latest: PlatformObservation | None = None
        self._lock = threading.Lock()
        self.parse_errors = 0

    def start(self) -> bool:
        if self._running.is_set():
            return True
        try:
            factory = self.serial_factory
            if factory is None:
                import serial
                factory = serial.Serial
            self._serial = factory(self.port, self.baudrate, timeout=self.timeout_s)
        except Exception as exc:
            LOG.error("Cyber Camera串口打开失败(%s): %s", self.port, exc)
            return False
        self._running.set()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="cybercam-vs1")
        self._thread.start()
        return True

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def latest(self, now: float | None = None, max_age_s: float = 0.15) -> PlatformObservation | None:
        timestamp = time.monotonic() if now is None else now
        with self._lock:
            observation = self._latest
        if observation is None or observation.age_s(timestamp) > max_age_s:
            return None
        return observation

    def _worker(self) -> None:
        buffer = bytearray()
        while self._running.is_set():
            try:
                data = self._serial.read(128)
                if not data:
                    continue
                buffer.extend(data)
                if len(buffer) > self.max_line_bytes * 2:
                    newline = buffer.rfind(b"\n")
                    buffer[:] = buffer[newline + 1:] if newline >= 0 else b""
                    self.parse_errors += 1
                while b"\n" in buffer:
                    line, remainder = buffer.split(b"\n", 1)
                    buffer[:] = remainder
                    if not line or len(line) > self.max_line_bytes:
                        self.parse_errors += 1
                        continue
                    try:
                        observation = parse_line(line, time.monotonic())
                    except ValueError:
                        self.parse_errors += 1
                        continue
                    accepted = self._gate.accept(observation)
                    if accepted is not None:
                        with self._lock:
                            self._latest = accepted
            except Exception as exc:
                LOG.error("Cyber Camera串口读取失败: %s", exc)
                self._running.clear()
