"""Object association: same class + 3D distance < threshold -> same object; position fused by EMA (plan §20-21)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class TrackedObject:
    id: str
    class_name: str
    position: Vec3
    confidence: float
    first_seen: float
    last_seen: float
    observation_count: int = 1
    size: Vec3 = (0.0, 0.0, 0.0)
    relations: list = field(default_factory=list)


class ObjectAssociator:
    def __init__(self, distance_threshold: float = 0.5, alpha: float = 0.3, min_observations: int = 3):
        self.thr = distance_threshold
        self.alpha = alpha
        self.min_obs = min_observations
        self.objects: Dict[str, TrackedObject] = {}
        self._counters: Dict[str, int] = {}

    def _new_id(self, class_name: str) -> str:
        n = self._counters.get(class_name, 0) + 1
        self._counters[class_name] = n
        return f'{class_name.replace(" ", "_")}_{n:02d}'

    def nearest(self, class_name: str, p: Vec3) -> Optional[TrackedObject]:
        best, best_d = None, self.thr
        for o in self.objects.values():
            if o.class_name != class_name:
                continue
            d = math.dist(o.position, p)
            if d < best_d:
                best, best_d = o, d
        return best

    def update(self, class_name: str, p: Vec3, confidence: float, stamp: float,
               size: Vec3 = (0.0, 0.0, 0.0)) -> TrackedObject:
        o = self.nearest(class_name, p)
        if o is None:
            o = TrackedObject(self._new_id(class_name), class_name, tuple(float(v) for v in p), float(confidence),
                              float(stamp), float(stamp), 1, tuple(float(v) for v in size))
            self.objects[o.id] = o
            return o
        a = self.alpha
        o.position = tuple(a * pn + (1 - a) * po for pn, po in zip(p, o.position))
        o.confidence = max(o.confidence, confidence) if o.observation_count < 5 else 0.9 * o.confidence + 0.1 * confidence
        o.last_seen = stamp
        o.observation_count += 1
        if any(size):
            o.size = tuple(a * sn + (1 - a) * so for sn, so in zip(size, o.size))
        return o

    def confirmed(self):
        return [o for o in self.objects.values() if o.observation_count >= self.min_obs]
