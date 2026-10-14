"""ONNX Runtime backend placeholder for the Raspberry Pi (R4). Phase 5/R4 implements pre/post-processing."""
from __future__ import annotations

from typing import List

import numpy as np

from .base import Detection, DetectorBackend


class OnnxBackend(DetectorBackend):
    name = 'onnx'

    def __init__(self, model: str = 'yolov8n.onnx', **_):
        raise NotImplementedError('OnnxBackend is implemented in stage R4 (edge migration)')

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError
