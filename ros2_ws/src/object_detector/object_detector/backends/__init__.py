"""Detector backends. The ROS interface never changes when the backend does (Phase 5 / R4)."""
from .base import Detection, DetectorBackend
from .mock import MockBackend

__all__ = ['Detection', 'DetectorBackend', 'MockBackend', 'create_backend']


def create_backend(name: str, **kwargs) -> DetectorBackend:
    if name == 'mock':
        return MockBackend(**kwargs)
    if name == 'ultralytics':
        from .ultralytics_backend import UltralyticsBackend
        return UltralyticsBackend(**kwargs)
    if name == 'onnx':
        from .onnx_backend import OnnxBackend
        return OnnxBackend(**kwargs)
    raise ValueError(f'unknown detector backend: {name}')
