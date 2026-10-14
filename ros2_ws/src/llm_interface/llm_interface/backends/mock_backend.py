"""Deterministic rule-based backend for tests: maps simple Korean/English questions to tool calls (no LLM needed)."""
from __future__ import annotations

import json
import re
from typing import List, Optional

from .base import LLMBackend, Message, ToolCall

CLASS_WORDS = {
    'chair': ['chair', '의자'], 'bottle': ['bottle', '물병', '병'], 'backpack': ['backpack', 'bag', '가방', '백팩'],
    'person': ['person', '사람'], 'bed': ['bed', '침대'], 'tv': ['tv', '티비', '텔레비전'],
    'dining table': ['table', '책상', '테이블', '식탁'], 'couch': ['couch', 'sofa', '소파'], 'book': ['book', '책'],
    'laptop': ['laptop', '노트북'],
}


def guess_class(text: str):
    t = text.lower()
    for cls, words in CLASS_WORDS.items():
        if any(w in t for w in words):
            return cls
    return ''


class MockBackend(LLMBackend):
    name = 'mock'

    def __init__(self, **_):
        pass

    def chat(self, messages: List[Message], tools: Optional[List[dict]] = None) -> Message:
        last = messages[-1]
        if last.role == 'tool':
            data = json.loads(last.content or '{}')
            objs = data.get('objects', [])
            if last.name == 'count_objects':
                return Message('assistant', f'{data.get("count", 0)}개 확인했습니다.')
            if not objs:
                return Message('assistant', '현재 생성된 Semantic Map에서는 해당 객체를 확인하지 못했습니다.')
            o = objs[0]
            return Message('assistant', f'{o["id"]} 확인: 위치 ({o["x"]:.2f}, {o["y"]:.2f}, {o["z"]:.2f}) m.')
        q = last.content
        cls = guess_class(q)
        if re.search(r'몇\s*개|how many|count', q, re.I):
            return Message('assistant', '', [ToolCall('count_objects', {'class_name': cls}, '0')])
        if re.search(r'근처|near|옆', q, re.I):
            return Message('assistant', '', [ToolCall('find_objects', {'near_object_id': cls + '_01', 'radius': 1.5}, '0')])
        return Message('assistant', '', [ToolCall('find_objects', {'class_name': cls}, '0')])
