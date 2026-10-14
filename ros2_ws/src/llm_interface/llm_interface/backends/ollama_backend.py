"""Ollama HTTP backend (local LLM). Uses /api/chat with tool calling (Ollama >= 0.3)."""
from __future__ import annotations

import json
import urllib.request
from typing import List, Optional

from .base import LLMBackend, Message, ToolCall


class OllamaBackend(LLMBackend):
    name = 'ollama'

    def __init__(self, host: str = 'http://127.0.0.1:11434', model: str = 'qwen2.5:7b', temperature: float = 0.0,
                 timeout_s: float = 120.0, **_):
        self.host, self.model, self.temperature, self.timeout = host.rstrip('/'), model, temperature, timeout_s

    def chat(self, messages: List[Message], tools: Optional[List[dict]] = None) -> Message:
        payload = {
            'model': self.model, 'stream': False, 'options': {'temperature': self.temperature},
            'messages': [self._to_ollama(m) for m in messages],
        }
        if tools:
            payload['tools'] = tools
        req = urllib.request.Request(f'{self.host}/api/chat', data=json.dumps(payload).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            data = json.loads(r.read().decode())
        msg = data.get('message', {})
        calls = [ToolCall(tc['function']['name'], tc['function'].get('arguments') or {}, str(i))
                 for i, tc in enumerate(msg.get('tool_calls') or [])]
        return Message('assistant', msg.get('content', ''), calls)

    @staticmethod
    def _to_ollama(m: Message) -> dict:
        d = {'role': m.role, 'content': m.content}
        if m.tool_calls:
            d['tool_calls'] = [{'function': {'name': c.name, 'arguments': c.arguments}} for c in m.tool_calls]
        return d
