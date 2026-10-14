"""llm_node — /llm/ask (AskLLM service): grounded Q&A over the semantic database via its query service (Phase 7)."""
from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_interfaces.srv import AskLLM, QueryObjects

from .agent import QueryAgent
from .backends import create_backend
from .tools import SemanticTools


class ServiceDB:
    """Adapts the QueryObjects service to the SemanticDB-like interface used by SemanticTools."""

    def __init__(self, node: Node, client):
        self.node, self.client = node, client

    def _call(self, **kw):
        req = QueryObjects.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        fut = self.client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done() or not fut.result().success:
            return []
        return [{'id': o.id, 'class_name': o.class_name, 'x': o.position.x, 'y': o.position.y, 'z': o.position.z,
                 'confidence': o.confidence, 'observation_count': o.observation_count, 'relations': list(o.relations)}
                for o in fut.result().objects]

    def query(self, class_name='', near_object_id='', near_point=None, radius=0.0, min_confidence=0.0):
        return self._call(class_name=class_name, near_object_id=near_object_id, radius=float(radius),
                          min_confidence=float(min_confidence))

    def count(self, class_name=''):
        return len(self.query(class_name=class_name))

    def classes(self):
        return sorted({o['class_name'] for o in self.query()})


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_interface')
        self.declare_parameter('backend', 'ollama')
        self.declare_parameter('ollama_host', 'http://127.0.0.1:11434')
        self.declare_parameter('model', 'qwen2.5:7b')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('max_rounds', 4)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        cb = ReentrantCallbackGroup()
        self.db_client = self.create_client(QueryObjects, '/semantic_database/query', callback_group=cb)
        backend = create_backend(g('backend'), host=g('ollama_host'), model=g('model'), temperature=float(g('temperature')))
        self.agent = QueryAgent(backend, SemanticTools(ServiceDB(self, self.db_client)), int(g('max_rounds')))
        self.create_service(AskLLM, '/llm/ask', self._on_ask, callback_group=cb)
        self.get_logger().info(f'llm_interface ready: backend={backend.name} model={g("model")}')

    def _on_ask(self, req, res):
        try:
            answer, log = self.agent.ask(req.question)
            res.success, res.answer, res.tool_calls = True, answer, log
            res.structured_command = ''   # Phase 8: command parsing
        except Exception as e:  # noqa: BLE001
            res.success, res.answer = False, f'LLM error: {e}'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()
