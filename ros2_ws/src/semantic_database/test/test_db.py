from semantic_database.db import SemanticDB


def make():
    db = SemanticDB(':memory:')
    db.upsert_object({'id': 'chair_01', 'class_name': 'chair', 'x': 2.0, 'y': 1.0, 'z': 0.5, 'confidence': 0.9,
                      'first_seen': 1.0, 'last_seen': 2.0, 'observation_count': 5, 'relations': ['near:table_01']})
    db.upsert_object({'id': 'table_01', 'class_name': 'dining table', 'x': 2.5, 'y': 1.2, 'z': 0.7, 'confidence': 0.8,
                      'first_seen': 1.0, 'last_seen': 2.0, 'observation_count': 4})
    db.upsert_object({'id': 'bottle_01', 'class_name': 'bottle', 'x': 6.0, 'y': 0.0, 'z': 0.8, 'confidence': 0.6,
                      'first_seen': 1.0, 'last_seen': 3.0, 'observation_count': 3})
    return db


def test_query_class_and_count():
    db = make()
    assert [o['id'] for o in db.query('chair')] == ['chair_01']
    assert db.count() == 3 and db.count('bottle') == 1
    assert db.classes() == ['bottle', 'chair', 'dining table']


def test_query_near_object():
    db = make()
    near = db.query(near_object_id='chair_01', radius=1.0)
    assert [o['id'] for o in near] == ['table_01']
    assert db.query(near_object_id='nope', radius=1.0) == []


def test_relations_and_upsert():
    db = make()
    assert db.get('chair_01')['relations'] == ['near:table_01']
    db.upsert_object({'id': 'chair_01', 'class_name': 'chair', 'x': 2.1, 'y': 1.0, 'z': 0.5, 'confidence': 0.95,
                      'first_seen': 1.0, 'last_seen': 5.0, 'observation_count': 6, 'relations': []})
    o = db.get('chair_01')
    assert o['observation_count'] == 6 and o['relations'] == [] and db.count('chair') == 1
