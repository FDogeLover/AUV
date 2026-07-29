import threading
import time
import unittest

from drone_control.competition_2026_d.Lcode.air_ground_link import AirGroundLink, LinkConfig
from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    decode_frame,
    encode_frame,
    pack_payload,
)


class FakeSerial:
    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.lock = threading.Lock()
        self.closed = False

    def feed(self, data):
        with self.lock:
            self.rx.extend(data)

    def read(self, size):
        with self.lock:
            if not self.rx:
                data = b""
            else:
                data = bytes(self.rx[:size])
                del self.rx[:size]
                return data
        time.sleep(0.002)
        return data

    def write(self, data):
        with self.lock:
            self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


class AirGroundLinkTest(unittest.TestCase):
    def test_ack_and_duplicate_event_are_non_reentrant(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(read_timeout_s=0.002, ack_timeout_s=0.02),
            serial_factory=lambda **_kwargs: serial,
        )
        received = []
        link.add_callback(received.append)
        self.assertTrue(link.start())
        incoming = Frame(
            MessageType.CAR_START,
            int(Flag.ACK_REQUIRED | Flag.EVENT),
            Device.CAR,
            Device.UAV,
            99,
            7,
            100,
            pack_payload(MessageType.CAR_START, (2, 1234)),
        )
        raw = encode_frame(incoming)
        serial.feed(raw + raw)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and (not received or len(serial.writes) < 2):
            time.sleep(0.005)
        link.close()
        self.assertEqual(len(received), 1)
        self.assertEqual(link.stats.duplicate_events, 1)
        self.assertGreaterEqual(len(serial.writes), 2)  # 重复事件仍分别ACK
        ack = decode_frame(serial.writes[0])
        self.assertEqual(ack.message_type, MessageType.ACK)
        self.assertEqual(ack.dest, Device.CAR)


if __name__ == "__main__":
    unittest.main()
