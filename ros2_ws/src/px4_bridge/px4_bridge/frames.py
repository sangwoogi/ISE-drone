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


# --- attitude -----------------------------------------------------------------
# PX4 attitude quaternion: body FRD -> world NED, stored as (w, x, y, z).
# ROS wants body FLU -> world ENU, stored as (x, y, z, w).
#   q_enu_flu = q_NED->ENU * q_ned_frd * q_FLU->FRD
# with q_NED->ENU = rotation (x<->y swap, z flip) = (w=0, x=sqrt(.5), y=sqrt(.5), z=0)
# and  q_FLU->FRD = 180 deg about x = (w=0, x=1, y=0, z=0).
_SQ2 = math.sqrt(0.5)
_Q_NED_ENU = (0.0, _SQ2, _SQ2, 0.0)   # (w, x, y, z)
_Q_FLU_FRD = (0.0, 1.0, 0.0, 0.0)


def quat_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]):
    """Hamilton product of (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_ned_frd_to_enu_flu(q_wxyz: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """PX4 (w,x,y,z) FRD->NED  ->  ROS (x,y,z,w) FLU->ENU."""
    w, x, y, z = quat_mul(quat_mul(_Q_NED_ENU, tuple(float(v) for v in q_wxyz)), _Q_FLU_FRD)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (x / n, y / n, z / n, w / n)


def yaw_from_quat_xyzw(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angular_velocity_frd_to_flu(w: Vec3) -> Vec3:
    """Body rates FRD -> FLU: (p, q, r) -> (p, -q, -r)."""
    return (w[0], -w[1], -w[2])
