"""LLM layer tested with a mock semantic DB and the rule-based mock backend (plan §46-E)."""
from llm_interface.agent import QueryAgent
from llm_interface.backends import MockBackend
from llm_interface.tools import SemanticTools
from semantic_database.db import SemanticDB


def make_agent():
    db = SemanticDB(':memory:')
    db.upsert_object({'id': 'backpack_01', 'class_name': 'backpack', 'x': 2.81, 'y': -0.92, 'z': 0.41,
                      'confidence': 0.9, 'first_seen': 1, 'last_seen': 2, 'observation_count': 7})
    for i, x in enumerate((1.0, 3.0), 1):
        db.upsert_object({'id': f'chair_0{i}', 'class_name': 'chair', 'x': x, 'y': 1.0, 'z': 0.5,
                          'confidence': 0.8, 'first_seen': 1, 'last_seen': 2, 'observation_count': 4})
    return QueryAgent(MockBackend(), SemanticTools(db))


def test_where_is_bag():
    answer, log = make_agent().ask('방 안에 가방이 있어?')
    assert 'backpack_01' in answer and '2.81' in answer
    assert len(log) == 1 and '"tool": "find_objects"' in log[0]


def test_count_chairs():
    answer, _ = make_agent().ask('의자는 몇 개 있었어?')
    assert answer.startswith('2')


def test_missing_object_is_not_hallucinated():
    answer, _ = make_agent().ask('사람을 발견했어?')
    assert '확인하지 못했습니다' in answer
