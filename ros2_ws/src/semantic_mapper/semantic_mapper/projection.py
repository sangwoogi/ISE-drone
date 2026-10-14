"""2D bbox + depth + intrinsics -> 3D point in the camera optical frame (plan §18, pure functions)."""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def robust_depth(depth: np.ndarray, x_min: float, y_min: float, x_max: float, y_max: float,
                 center_fraction: float = 0.5, min_valid: int = 5,
                 d_min: float = 0.1, d_max: float = 10.0) -> Optional[float]:
    """Median of valid depths inside the central `center_fraction` of the bbox. depth in metres (float32)
    or millimetres (uint16, converted). Returns None when too few valid pixels."""
    h, w = depth.shape[:2]
    cx, cy = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    bw, bh = (x_max - x_min) * center_fraction, (y_max - y_min) * center_fraction
    x0, x1 = int(max(0, cx - bw / 2)), int(min(w, cx + bw / 2 + 1))
    y0, y1 = int(max(0, cy - bh / 2)), int(min(h, cy + bh / 2 + 1))
    if x1 <= x0 or y1 <= y0:
        return None
    roi = depth[y0:y1, x0:x1].astype(np.float64)
    if depth.dtype == np.uint16:
        roi = roi / 1000.0
    valid = roi[np.isfinite(roi) & (roi > d_min) & (roi < d_max)]
    if valid.size < min_valid:
        return None
    return float(np.median(valid))


def pixel_to_camera(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    """Pinhole back-projection in the optical frame (x right, y down, z forward)."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def bbox_to_camera_point(depth: np.ndarray, bbox, K) -> Optional[Tuple[float, float, float]]:
    """bbox = (x_min, y_min, x_max, y_max) px; K = CameraInfo.k (row-major 3x3)."""
    z = robust_depth(depth, *bbox)
    if z is None:
        return None
    u, v = 0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3])
    return pixel_to_camera(u, v, z, K[0], K[4], K[2], K[5])
