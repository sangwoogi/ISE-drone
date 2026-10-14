import math

import pytest

from px4_bridge import frames


@pytest.mark.parametrize('enu,ned', [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((1.0, 2.0, 3.0), (2.0, 1.0, -3.0)),
    ((-0.5, 4.0, 1.5), (4.0, -0.5, -1.5)),
])
def test_enu_ned_position(enu, ned):
    assert frames.enu_to_ned(enu) == pytest.approx(ned)
    assert frames.ned_to_enu(ned) == pytest.approx(enu)
    assert frames.ned_to_enu(frames.enu_to_ned(enu)) == pytest.approx(enu)


@pytest.mark.parametrize('yaw_enu,yaw_ned', [
    (0.0, math.pi / 2),            # facing East  -> NED 90 deg
    (math.pi / 2, 0.0),            # facing North -> NED 0
    (math.pi, -math.pi / 2),       # facing West  -> NED -90
    (-math.pi / 2, math.pi),       # facing South -> NED 180
])
def test_yaw_conversion(yaw_enu, yaw_ned):
    assert frames.yaw_enu_to_ned(yaw_enu) == pytest.approx(frames.wrap_pi(yaw_ned))
    assert frames.yaw_ned_to_enu(yaw_ned) == pytest.approx(frames.wrap_pi(yaw_enu))


def test_wrap_pi():
    # +/-pi are the same heading; only require the magnitude at the boundary.
    assert abs(frames.wrap_pi(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(frames.wrap_pi(-3 * math.pi)) == pytest.approx(math.pi)
    assert frames.wrap_pi(0.1) == pytest.approx(0.1)
    assert frames.wrap_pi(2 * math.pi + 0.5) == pytest.approx(0.5)
    assert frames.wrap_pi(-2 * math.pi - 0.5) == pytest.approx(-0.5)
    for a in (-3.0, -1.0, 0.0, 1.0, 3.0):
        assert -math.pi < frames.wrap_pi(a) <= math.pi


def test_distance():
    assert frames.distance((0, 0, 0), (1, 2, 2)) == pytest.approx(3.0)


def _q_about_z_ned(yaw_ned):
    return (math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2))   # (w,x,y,z)


@pytest.mark.parametrize('yaw_ned_deg', [0, 90, -90, 180, 45])
def test_attitude_conversion_matches_yaw_rule(yaw_ned_deg):
    q_ned = _q_about_z_ned(math.radians(yaw_ned_deg))
    q_enu = frames.quat_ned_frd_to_enu_flu(q_ned)
    yaw_enu = frames.yaw_from_quat_xyzw(q_enu)
    assert frames.wrap_pi(yaw_enu - frames.yaw_ned_to_enu(math.radians(yaw_ned_deg))) == pytest.approx(0.0, abs=1e-9)


def test_attitude_conversion_level_flight_has_no_roll_pitch():
    q_enu = frames.quat_ned_frd_to_enu_flu((1.0, 0.0, 0.0, 0.0))     # identity: nose North, level
    x, y, z, w = q_enu
    # roll / pitch from quaternion must be ~0, yaw = +90 deg (North in ENU)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    assert roll == pytest.approx(0.0, abs=1e-9) and pitch == pytest.approx(0.0, abs=1e-9)
    assert frames.yaw_from_quat_xyzw(q_enu) == pytest.approx(math.pi / 2)


def test_angular_velocity_conversion():
    assert frames.angular_velocity_frd_to_flu((0.1, 0.2, 0.3)) == pytest.approx((0.1, -0.2, -0.3))
