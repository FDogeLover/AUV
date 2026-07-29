import math

import pytest

from t265_laser_height_compare import HeightZero, compare_height, laser_cm_valid


def test_laser_cm_valid_rejects_sentinel_and_out_of_range():
    assert not laser_cm_valid(0)
    assert not laser_cm_valid(5)
    assert not laser_cm_valid(0xFFFFFFFF)
    assert not laser_cm_valid(float("nan"))
    assert laser_cm_valid(123)


def test_compare_height_aligns_t265_delta_to_initial_laser_height():
    zero = HeightZero(laser_m=0.20, raw_z_m=1.50, filtered_z_m=0.0)
    result = compare_height(
        zero,
        laser_m=0.50,
        raw_z_m=1.80,
        filtered_z_m=0.30,
    )
    assert result["laser_delta_m"] == pytest.approx(0.30)
    assert result["t265_raw_delta_m"] == pytest.approx(0.30)
    assert result["t265_filtered_delta_m"] == pytest.approx(0.30)
    assert result["raw_error_m"] == pytest.approx(0.0)
    assert result["filtered_error_m"] == pytest.approx(0.0)


def test_compare_height_supports_reversed_z_axis():
    zero = HeightZero(laser_m=0.40, raw_z_m=2.0, filtered_z_m=1.0)
    result = compare_height(
        zero,
        laser_m=0.70,
        raw_z_m=1.70,
        filtered_z_m=0.70,
        z_sign=-1,
    )
    assert math.isclose(result["raw_error_m"], 0.0, abs_tol=1e-9)
    assert math.isclose(result["filtered_error_m"], 0.0, abs_tol=1e-9)
