"""SQLite semantic object store (plan §22). Usable without ROS (LLM tests use it with a mock DB)."""
from __future__ import annotations

import math
import sqlite3
import time
from typing import Iterable, List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS objects (
    id TEXT PRIMARY KEY,
    class_name TEXT NOT NULL,
    x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
    size_x REAL DEFAULT 0, size_y REAL DEFAULT 0, size_z REAL DEFAULT 0,
    confidence REAL NOT NULL,
    first_seen REAL NOT NULL,
    last_seen REAL NOT NULL,
    observation_count INTEGER NOT NULL DEFAULT 1
);
CREATE TABLE IF NOT EXISTS relations (
    subject_id TEXT NOT NULL,
    relation TEXT NOT NULL,
    object_id TEXT NOT NULL,
    confidence REAL DEFAULT 1.0,
    PRIMARY KEY (subject_id, relation, object_id)
);
CREATE TABLE IF NOT EXISTS observations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_id TEXT NOT NULL,
    t REAL NOT NULL,
    x REAL, y REAL, z REAL, confidence REAL
);
CREATE INDEX IF NOT EXISTS idx_objects_class ON objects(class_name);
"""


class SemanticDB:
    def __init__(self, path: str = ':memory:'):
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.conn.executescript(SCHEMA)

    # ---------------------------------------------------------------- write
    def upsert_object(self, obj: dict, log_observation: bool = False) -> None:
        """obj keys: id class_name x y z confidence first_seen last_seen observation_count [size_x..] [relations]"""
        self.conn.execute(
            """INSERT INTO objects (id, class_name, x, y, z, size_x, size_y, size_z, confidence, first_seen, last_seen, observation_count)
               VALUES (:id, :class_name, :x, :y, :z, :size_x, :size_y, :size_z, :confidence, :first_seen, :last_seen, :observation_count)
               ON CONFLICT(id) DO UPDATE SET class_name=excluded.class_name, x=excluded.x, y=excluded.y, z=excluded.z,
                   size_x=excluded.size_x, size_y=excluded.size_y, size_z=excluded.size_z, confidence=excluded.confidence,
                   last_seen=excluded.last_seen, observation_count=excluded.observation_count""",
            {'size_x': 0.0, 'size_y': 0.0, 'size_z': 0.0, **obj})
        rels = obj.get('relations') or []
        self.conn.execute('DELETE FROM relations WHERE subject_id = ?', (obj['id'],))
        for r in rels:
            rel, _, target = r.partition(':')
            if target:
                self.conn.execute('INSERT OR REPLACE INTO relations (subject_id, relation, object_id) VALUES (?, ?, ?)',
                                  (obj['id'], rel, target))
        if log_observation:
            self.conn.execute('INSERT INTO observations (object_id, t, x, y, z, confidence) VALUES (?, ?, ?, ?, ?, ?)',
                              (obj['id'], obj.get('last_seen', time.time()), obj['x'], obj['y'], obj['z'], obj['confidence']))
        self.conn.commit()

    def clear(self) -> None:
        for t in ('objects', 'relations', 'observations'):
            self.conn.execute(f'DELETE FROM {t}')
        self.conn.commit()

    # ----------------------------------------------------------------- read
    def get(self, object_id: str) -> Optional[dict]:
        row = self.conn.execute('SELECT * FROM objects WHERE id = ?', (object_id,)).fetchone()
        return self._with_relations(dict(row)) if row else None

    def query(self, class_name: str = '', near_object_id: str = '', near_point=None, radius: float = 0.0,
              min_confidence: float = 0.0) -> List[dict]:
        sql, args = 'SELECT * FROM objects WHERE confidence >= ?', [min_confidence]
        if class_name:
            sql += ' AND class_name = ?'
            args.append(class_name)
        rows = [dict(r) for r in self.conn.execute(sql, args)]
        centre = None
        if near_object_id:
            ref = self.get(near_object_id)
            if ref is None:
                return []
            centre = (ref['x'], ref['y'], ref['z'])
            rows = [r for r in rows if r['id'] != near_object_id]
        elif near_point is not None and radius > 0:
            centre = tuple(near_point)
        if centre is not None and radius > 0:
            rows = [r for r in rows if math.dist((r['x'], r['y'], r['z']), centre) <= radius]
            rows.sort(key=lambda r: math.dist((r['x'], r['y'], r['z']), centre))
        return [self._with_relations(r) for r in rows]

    def count(self, class_name: str = '') -> int:
        return len(self.query(class_name=class_name))

    def classes(self) -> List[str]:
        return [r[0] for r in self.conn.execute('SELECT DISTINCT class_name FROM objects ORDER BY class_name')]

    def _with_relations(self, row: dict) -> dict:
        row['relations'] = [f'{r["relation"]}:{r["object_id"]}' for r in
                            self.conn.execute('SELECT relation, object_id FROM relations WHERE subject_id = ?', (row['id'],))]
        return row
