"""Ultralytics YOLO backend (PyTorch/CUDA on the PC). Phase 5 fills in model selection and class filter."""
from __future__ import annotations

from typing import List

import numpy as np

from .base import Detection, DetectorBackend


class UltralyticsBackend(DetectorBackend):
    name = 'ultralytics'

    def __init__(self, model: str = 'yolov8n.pt', device: str = 'cuda:0', conf: float = 0.4,
                 classes=None, imgsz: int = 640, **_):
        from ultralytics import YOLO  # imported lazily: heavy dependency
        self.model = YOLO(model)
        self.device, self.conf, self.imgsz = device, conf, imgsz
        self.class_filter = set(classes) if classes else None

    def warmup(self) -> None:
        self.detect(np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8))

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        res = self.model.predict(image_bgr, device=self.device, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        out = []
        names = res.names
        for box in res.boxes:
            cls = names[int(box.cls.item())]
            if self.class_filter and cls not in self.class_filter:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append(Detection(cls, float(box.conf.item()), x1, y1, x2, y2))
        return out
