import math

import pytest

from px4_offboard_study.mathutil import body_to_ned, joy_to_input, shape


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


IDX = {'yaw': 0, 'up': 1, 'right': 3, 'fwd': 4}


def pad(**kw):
    a = [0.0] * 6
    for name, v in kw.items():
        a[IDX[name]] = v
    return a


def test_right_stick_pushed_right_moves_right():
    # a real pad reports right as a negative value
    fwd, right, up, yaw = joy_to_input(pad(right=-1.0), IDX, 0.1, 2.0, 1.0, 0.8)
    assert right == pytest.approx(2.0) and fwd == 0.0


def test_right_stick_pushed_up_moves_forward():
    assert joy_to_input(pad(fwd=1.0), IDX, 0.1, 2.0, 1.0, 0.8)[0] == pytest.approx(2.0)


def test_left_stick_up_climbs():
    assert joy_to_input(pad(up=1.0), IDX, 0.1, 2.0, 1.0, 0.8)[2] == pytest.approx(1.0)


def test_left_stick_pushed_right_turns_clockwise():
    assert joy_to_input(pad(yaw=-1.0), IDX, 0.1, 2.0, 1.0, 0.8)[3] == pytest.approx(0.8)


def test_short_axes_list_is_safe():
    assert joy_to_input([], IDX, 0.1, 2.0, 1.0, 0.8) == (0.0, 0.0, 0.0, 0.0)
