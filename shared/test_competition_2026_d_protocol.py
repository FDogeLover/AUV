import unittest

from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    StreamParser,
    decode_frame,
    encode_frame,
    pack_payload,
    seq_is_newer,
    unpack_payload,
)


class DcpProtocolTest(unittest.TestCase):
    def test_golden_car_start(self):
        frame = Frame(
            message_type=MessageType.CAR_START,
            flags=Flag.ACK_REQUIRED | Flag.EVENT,
            source=Device.CAR,
            dest=Device.UAV,
            session_id=0x12345678,
            seq=0x002A,
            sender_ms=1000,
            payload=pack_payload(MessageType.CAR_START, (2, 0x89ABCDEF)),
        )
        raw = encode_frame(frame)
        self.assertEqual(
            raw.hex(),
            "aa0103050201785634122a00e8030000050002efcdab8982f9ff",
        )
        decoded = decode_frame(raw)
        self.assertEqual(decoded, frame)
        self.assertEqual(unpack_payload(decoded.message_type, decoded.payload), (2, 0x89ABCDEF))

    def test_stream_recovers_after_corrupt_frame(self):
        good = encode_frame(Frame(MessageType.HEARTBEAT, 0, Device.UAV, Device.CAR, 7, 9, 20, b"\x02\x00\x00"))
        bad = bytearray(good)
        bad[-3] ^= 0x40
        parser = StreamParser()
        frames = parser.feed(b"noise" + bytes(bad) + good[:8])
        self.assertEqual(frames, [])
        frames = parser.feed(good[8:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 9)
        self.assertGreaterEqual(parser.rejected, 1)

    def test_sequence_wrap(self):
        self.assertTrue(seq_is_newer(0, 65535))
        self.assertFalse(seq_is_newer(65535, 0))
        self.assertFalse(seq_is_newer(42, 42))

    def test_uav_state_payload_schema(self):
        values = (6, 100, -200, 1500, 20, -30, 88, 3)
        payload = pack_payload(MessageType.UAV_STATE, values)
        self.assertEqual(unpack_payload(MessageType.UAV_STATE, payload), values)


if __name__ == "__main__":
    unittest.main()
