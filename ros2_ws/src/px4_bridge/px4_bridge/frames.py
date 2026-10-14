"""Coordinate-frame helpers.

Upper layers of ISE-drone work in ROS ENU (x East, y North, z Up, yaw CCW from +x/East).
PX4 works in NED (x North, y East, z Down, yaw CW from +x/North).
Nothing outside px4_bridge should ever see an NED coordinate.
"""
from __future__ import annotations

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def enu_to_ned(p: Vec3) -> Vec3:
    """(E, N, U) -> (N, E, D)."""
    return (p[1], p[0], -p[2])


def ned_to_enu(p: Vec3) -> Vec3:
    """(N, E, D) -> (E, N, U). The mapping is an involution."""
    return (p[1], p[0], -p[2])


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ENU yaw (CCW from East) -> NED yaw (CW from North)."""
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """NED yaw (CW from North) -> ENU yaw (CCW from East). Also an involution."""
    return wrap_pi(math.pi / 2.0 - yaw_ned)


def distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
