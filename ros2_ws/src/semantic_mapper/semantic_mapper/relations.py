"""Spatial relations between objects (plan §23, V1: near / left_of / right_of / in_front_of / behind).

Directions are evaluated in the map frame from a reference heading (yaw of the drone's home or 0 = East).
`on`/`under`/`inside` are deliberately not produced in V1.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Tuple

from .association import TrackedObject


def relations_for(obj: TrackedObject, others: Iterable[TrackedObject], near_threshold: float = 1.5,
                  reference_yaw: float = 0.0) -> List[str]:
    out: List[str] = []
    c, s = math.cos(-reference_yaw), math.sin(-reference_yaw)
    for o in others:
        if o.id == obj.id:
            continue
        dx, dy = o.position[0] - obj.position[0], o.position[1] - obj.position[1]
        d = math.hypot(dx, dy)
        if d > near_threshold:
            continue
        out.append(f'near:{o.id}')
        # rotate into reference heading frame: fx forward, fy left
        fx, fy = c * dx - s * dy, s * dx + c * dy
        if abs(fx) >= abs(fy):
            out.append(f'{"in_front_of" if fx > 0 else "behind"}:{o.id}')
        else:
            out.append(f'{"left_of" if fy > 0 else "right_of"}:{o.id}')
    return out
