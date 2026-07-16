import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.Lground import build_position_frame, build_fire_frame, FRAME_TYPE_POSITION, FRAME_TYPE_FIRE


class TestBuildPositionFrame:
    def test_frame_starts_with_AA_and_ends_with_FF(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[0] == 0xAA
        assert frame[-1] == 0xFF

    def test_frame_type_byte_is_position(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[1] == FRAME_TYPE_POSITION

    def test_negative_coordinate_roundtrips_as_signed_int16(self):
        frame = build_position_frame(x_cm=-100, y_cm=200)
        x_bytes = frame[2:4]
        y_bytes = frame[4:6]
        x = int.from_bytes(x_bytes, byteorder="little", signed=True)
        y = int.from_bytes(y_bytes, byteorder="little", signed=True)
        assert x == -100
        assert y == 200


class TestBuildFireFrame:
    def test_frame_type_byte_is_fire(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        assert frame[1] == FRAME_TYPE_FIRE

    def test_coordinates_encoded_correctly(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        x = int.from_bytes(frame[2:4], byteorder="little", signed=True)
        y = int.from_bytes(frame[4:6], byteorder="little", signed=True)
        assert x == 80
        assert y == 120
