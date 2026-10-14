"""Cloud backend placeholder (OpenAI-compatible chat completions). Phase 7 decides whether it is needed."""
from __future__ import annotations

from typing import List, Optional

from .base import LLMBackend, Message


class CloudBackend(LLMBackend):
    name = 'cloud'

    def __init__(self, **_):
        raise NotImplementedError('CloudBackend: configure in Phase 7 if a hosted model is wanted')

    def chat(self, messages: List[Message], tools: Optional[List[dict]] = None) -> Message:
        raise NotImplementedError
