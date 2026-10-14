from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Detection:
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center(self):
        return (0.5 * (self.x_min + self.x_max), 0.5 * (self.y_min + self.y_max))

    @property
    def size(self):
        return (self.x_max - self.x_min, self.y_max - self.y_min)


class DetectorBackend:
    """Interface: BGR uint8 image -> list[Detection] (pixel coordinates)."""

    name = 'base'

    def warmup(self) -> None:
        pass

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError
