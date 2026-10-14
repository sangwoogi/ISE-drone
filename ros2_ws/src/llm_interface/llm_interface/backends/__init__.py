"""LLM backends (plan §26). The ROS node and tools never depend on a specific vendor."""
from .base import LLMBackend, Message, ToolCall
from .ollama_backend import OllamaBackend
from .mock_backend import MockBackend

__all__ = ['LLMBackend', 'Message', 'ToolCall', 'OllamaBackend', 'MockBackend', 'create_backend']


def create_backend(name: str, **kwargs) -> LLMBackend:
    if name == 'ollama':
        return OllamaBackend(**kwargs)
    if name == 'mock':
        return MockBackend(**kwargs)
    if name == 'cloud':
        from .cloud_backend import CloudBackend
        return CloudBackend(**kwargs)
    raise ValueError(f'unknown LLM backend: {name}')
