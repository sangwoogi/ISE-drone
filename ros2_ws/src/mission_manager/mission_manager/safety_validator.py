"""Safety validator between structured commands and flight primitives (plan §2.4). Pure Python."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class SafetyLimits:
    min_altitude_m: float = 0.5
    max_altitude_m: float = 3.0
    max_distance_from_home_m: float = 15.0
    min_object_confidence: float = 0.5
    object_standoff_m: float = 1.0        # GOTO_OBJECT: stop this far from the object (horizontal)
    object_hover_altitude_m: float = 1.2  # GOTO_OBJECT: flight altitude above home


class SafetyError(ValueError):
    pass


class SafetyValidator:
    def __init__(self, limits: Optional[SafetyLimits] = None):
        self.limits = limits or SafetyLimits()

    def check_altitude(self, rel_z: float) -> None:
        if not (self.limits.min_altitude_m <= rel_z <= self.limits.max_altitude_m):
            raise SafetyError(f'altitude {rel_z:.2f} m outside [{self.limits.min_altitude_m}, {self.limits.max_altitude_m}]')

    def check_goto(self, target: Vec3, home: Vec3) -> None:
        self.check_altitude(target[2] - home[2])
        d = ((target[0] - home[0]) ** 2 + (target[1] - home[1]) ** 2) ** 0.5
        if d > self.limits.max_distance_from_home_m:
            raise SafetyError(f'target {d:.1f} m from home exceeds {self.limits.max_distance_from_home_m} m')

    def object_goal(self, obj_pos: Vec3, drone_pos: Vec3, home: Vec3, confidence: float) -> Vec3:
        """Compute a safe goal near an object: standoff horizontally along the approach direction, fixed altitude."""
        if confidence < self.limits.min_object_confidence:
            raise SafetyError(f'object confidence {confidence:.2f} < {self.limits.min_object_confidence}')
        dx, dy = obj_pos[0] - drone_pos[0], obj_pos[1] - drone_pos[1]
        d = (dx * dx + dy * dy) ** 0.5
        if d <= self.limits.object_standoff_m:
            gx, gy = drone_pos[0], drone_pos[1]
        else:
            k = (d - self.limits.object_standoff_m) / d
            gx, gy = drone_pos[0] + dx * k, drone_pos[1] + dy * k
        goal = (gx, gy, home[2] + self.limits.object_hover_altitude_m)
        self.check_goto(goal, home)
        return goal
