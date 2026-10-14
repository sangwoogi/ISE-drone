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
