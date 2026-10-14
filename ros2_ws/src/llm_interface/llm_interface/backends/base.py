from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ToolCall:
    name: str
    arguments: Dict[str, Any]
    id: str = ''


@dataclass
class Message:
    role: str                       # system | user | assistant | tool
    content: str = ''
    tool_calls: List[ToolCall] = field(default_factory=list)
    tool_call_id: str = ''
    name: str = ''


class LLMBackend:
    """chat(messages, tools) -> Message. `tools` are OpenAI/Ollama-style JSON schemas."""

    name = 'base'

    def chat(self, messages: List[Message], tools: Optional[List[dict]] = None) -> Message:
        raise NotImplementedError
