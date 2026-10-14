"""Mock backend for pipeline tests without a model (Phase 5 / §46-47 synthetic detections)."""
from __future__ import annotations

from typing import List

import numpy as np

from .base import Detection, DetectorBackend


class MockBackend(DetectorBackend):
    name = 'mock'

    def __init__(self, fixed_detections=None, **_):
        # list of dicts {class_name, confidence, bbox:[x1,y1,x2,y2]} in normalized image coords
        # bottom-centre of the image: in an empty world the forward-looking camera only has valid depth on the ground
        self.fixed = fixed_detections or [{'class_name': 'chair', 'confidence': 0.9, 'bbox': [0.4, 0.7, 0.6, 0.95]}]

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        h, w = image_bgr.shape[:2]
        return [Detection(d['class_name'], float(d['confidence']),
                          d['bbox'][0] * w, d['bbox'][1] * h, d['bbox'][2] * w, d['bbox'][3] * h)
                for d in self.fixed]
