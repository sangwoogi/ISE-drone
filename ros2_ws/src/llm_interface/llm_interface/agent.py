"""QueryAgent: question -> (tool calls against the DB) -> grounded natural-language answer."""
from __future__ import annotations

import json
from typing import List

from .backends import LLMBackend, Message
from .tools import SYSTEM_PROMPT, TOOL_SCHEMAS, SemanticTools


class QueryAgent:
    def __init__(self, backend: LLMBackend, tools: SemanticTools, max_rounds: int = 4):
        self.backend, self.tools, self.max_rounds = backend, tools, max_rounds

    def ask(self, question: str):
        """Returns (answer, tool_call_log). Each DB access is logged for evaluation."""
        messages: List[Message] = [Message('system', SYSTEM_PROMPT), Message('user', question)]
        log = []
        for _ in range(self.max_rounds):
            reply = self.backend.chat(messages, TOOL_SCHEMAS)
            messages.append(reply)
            if not reply.tool_calls:
                return reply.content.strip(), log
            for call in reply.tool_calls:
                result = self.tools.call(call.name, call.arguments)
                log.append(json.dumps({'tool': call.name, 'args': call.arguments, 'result': json.loads(result)}, ensure_ascii=False))
                messages.append(Message('tool', result, tool_call_id=call.id, name=call.name))
        return '도구 호출 횟수를 초과했습니다. 질문을 더 구체적으로 해주세요.', log
