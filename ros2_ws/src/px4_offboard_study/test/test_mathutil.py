import math

import pytest

from px4_offboard_study.mathutil import body_to_ned, shape


def test_deadzone_zeroes_small_input():
    assert shape(0.05, 0.1, 2.0) == 0.0
    assert shape(-0.09, 0.1, 2.0) == 0.0


def test_full_deflection_reaches_limit():
    assert shape(1.0, 0.1, 2.0) == pytest.approx(2.0)
    assert shape(-1.0, 0.1, 2.0) == pytest.approx(-2.0)


def test_rescale_is_continuous_at_deadzone_edge():
    assert shape(0.1000001, 0.1, 2.0) == pytest.approx(0.0, abs=1e-5)


def test_clamps_out_of_range_input():
    assert shape(1.5, 0.1, 2.0) == pytest.approx(2.0)


def test_forward_at_north_heading_is_north():
    assert body_to_ned(1.0, 0.0, 0.0, 0.0) == pytest.approx((1.0, 0.0, 0.0))


def test_forward_at_east_heading_is_east():
    assert body_to_ned(1.0, 0.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_right_at_north_heading_is_east():
    assert body_to_ned(0.0, 1.0, 0.0, 0.0) == pytest.approx((0.0, 1.0, 0.0))


def test_up_is_negative_down():
    assert body_to_ned(0.0, 0.0, 1.0, 0.3)[2] == pytest.approx(-1.0)


def test_rotation_preserves_horizontal_speed():
    vn, ve, _ = body_to_ned(1.2, -0.7, 0.0, 1.1)
    assert math.hypot(vn, ve) == pytest.approx(math.hypot(1.2, 0.7))
