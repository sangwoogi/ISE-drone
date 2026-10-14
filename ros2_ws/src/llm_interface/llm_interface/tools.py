"""Tool (function-calling) layer between the LLM and the semantic database (plan §24-25).

`SemanticTools` takes any object with `query(class_name, near_object_id, near_point, radius, min_confidence)`
and `count(class_name)` / `classes()` — i.e. `semantic_database.db.SemanticDB` directly (tests) or a thin ROS
service wrapper (llm_node). The LLM never sees anything that is not returned by these tools.
"""
from __future__ import annotations

import json
import math
from typing import Any, Dict, List

TOOL_SCHEMAS: List[dict] = [
    {'type': 'function', 'function': {
        'name': 'find_objects',
        'description': 'Find objects in the semantic map. All arguments optional. Returns id, class, position (map frame, metres) and relations.',
        'parameters': {'type': 'object', 'properties': {
            'class_name': {'type': 'string', 'description': 'object class, e.g. chair, bottle, backpack, person'},
            'near_object_id': {'type': 'string', 'description': 'return objects near this object id'},
            'radius': {'type': 'number', 'description': 'search radius in metres when near_object_id is given'},
        }}}},
    {'type': 'function', 'function': {
        'name': 'count_objects',
        'description': 'Count objects of a class in the semantic map (empty class = all objects).',
        'parameters': {'type': 'object', 'properties': {'class_name': {'type': 'string'}}}}},
    {'type': 'function', 'function': {
        'name': 'list_classes',
        'description': 'List the object classes currently present in the semantic map.',
        'parameters': {'type': 'object', 'properties': {}}}},
]

SYSTEM_PROMPT = (
    'You are the query interface of an indoor exploration drone. Answer ONLY from tool results; never invent objects. '
    'Always call a tool before answering a question about objects. Positions are metres in the map frame whose origin is '
    'the take-off point (x forward/east, y left/north, z up). If nothing is found, say the semantic map does not contain it. '
    'Answer in the language of the question, briefly.'
)


class SemanticTools:
    def __init__(self, db, home=(0.0, 0.0, 0.0)):
        self.db = db
        self.home = home

    def _fmt(self, o: Dict[str, Any]) -> Dict[str, Any]:
        d = math.dist((o['x'], o['y'], o['z']), self.home)
        return {'id': o['id'], 'class': o['class_name'], 'x': round(o['x'], 2), 'y': round(o['y'], 2),
                'z': round(o['z'], 2), 'distance_from_home_m': round(d, 2), 'confidence': round(o['confidence'], 2),
                'observations': o['observation_count'], 'relations': o.get('relations', [])}

    def call(self, name: str, args: Dict[str, Any]) -> str:
        if name == 'find_objects':
            objs = self.db.query(class_name=args.get('class_name', '') or '', near_object_id=args.get('near_object_id', '') or '',
                                 radius=float(args.get('radius', 0) or 0))
            return json.dumps({'objects': [self._fmt(o) for o in objs]}, ensure_ascii=False)
        if name == 'count_objects':
            cls = args.get('class_name', '') or ''
            return json.dumps({'class_name': cls, 'count': self.db.count(cls)}, ensure_ascii=False)
        if name == 'list_classes':
            return json.dumps({'classes': self.db.classes()}, ensure_ascii=False)
        return json.dumps({'error': f'unknown tool {name}'})
