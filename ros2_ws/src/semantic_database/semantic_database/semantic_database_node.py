"""semantic_database_node — persists /semantic_map/objects into SQLite and serves QueryObjects (Phase 6)."""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from drone_interfaces.msg import SemanticObject, SemanticObjectArray
from drone_interfaces.srv import QueryObjects

from .db import SemanticDB


def row_to_msg(r: dict) -> SemanticObject:
    m = SemanticObject()
    m.id, m.class_name, m.confidence = r['id'], r['class_name'], float(r['confidence'])
    m.position.x, m.position.y, m.position.z = r['x'], r['y'], r['z']
    m.size.x, m.size.y, m.size.z = r.get('size_x', 0.0), r.get('size_y', 0.0), r.get('size_z', 0.0)
    m.first_seen.sec, m.first_seen.nanosec = int(r['first_seen']), int((r['first_seen'] % 1) * 1e9)
    m.last_seen.sec, m.last_seen.nanosec = int(r['last_seen']), int((r['last_seen'] % 1) * 1e9)
    m.observation_count = int(r['observation_count'])
    m.relations = list(r.get('relations', []))
    return m


class SemanticDatabaseNode(Node):
    def __init__(self):
        super().__init__('semantic_database')
        self.declare_parameter('db_path', 'semantic_map.sqlite')
        self.declare_parameter('clear_on_start', True)
        self.declare_parameter('log_observations', False)
        self.db = SemanticDB(self.get_parameter('db_path').value)
        if self.get_parameter('clear_on_start').value:
            self.db.clear()
        self.log_obs = bool(self.get_parameter('log_observations').value)
        self.create_subscription(SemanticObjectArray, '/semantic_map/objects', self._on_objects, 10)
        self.create_service(QueryObjects, '/semantic_database/query', self._on_query)
        self.get_logger().info(f'semantic_database: {self.get_parameter("db_path").value}')

    def _on_objects(self, msg: SemanticObjectArray):
        for o in msg.objects:
            self.db.upsert_object({
                'id': o.id, 'class_name': o.class_name, 'x': o.position.x, 'y': o.position.y, 'z': o.position.z,
                'size_x': o.size.x, 'size_y': o.size.y, 'size_z': o.size.z, 'confidence': o.confidence,
                'first_seen': o.first_seen.sec + o.first_seen.nanosec * 1e-9,
                'last_seen': o.last_seen.sec + o.last_seen.nanosec * 1e-9,
                'observation_count': o.observation_count, 'relations': list(o.relations)}, self.log_obs)

    def _on_query(self, req, res):
        try:
            near_point = (req.near_point.x, req.near_point.y, req.near_point.z) if req.radius > 0 else None
            rows = self.db.query(req.class_name, req.near_object_id, near_point, req.radius, req.min_confidence)
            res.objects = [row_to_msg(r) for r in rows]
            res.success, res.message = True, f'{len(rows)} objects'
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SemanticDatabaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()
