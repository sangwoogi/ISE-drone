"""llm_cli — ask questions from the terminal.

    ros2 run llm_interface llm_cli "가방이 어디 있어?"             # via /llm/ask service (needs llm_node + DB node)
    ros2 run llm_interface llm_cli --db semantic_map.sqlite --backend mock "의자는 몇 개야?"   # offline, no ROS
"""
from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('question', nargs='?')
    ap.add_argument('--db', help='SQLite path: run the agent locally instead of calling /llm/ask')
    ap.add_argument('--backend', default='ollama')
    ap.add_argument('--model', default='qwen2.5:7b')
    ap.add_argument('--host', default='http://127.0.0.1:11434')
    a, ros_args = ap.parse_known_args(argv)

    if a.db:
        from semantic_database.db import SemanticDB
        from .agent import QueryAgent
        from .backends import create_backend
        from .tools import SemanticTools
        agent = QueryAgent(create_backend(a.backend, host=a.host, model=a.model), SemanticTools(SemanticDB(a.db)))
        ask = lambda q: agent.ask(q)  # noqa: E731
    else:
        import rclpy
        from drone_interfaces.srv import AskLLM
        rclpy.init(args=ros_args)
        node = rclpy.create_node('llm_cli')
        cli = node.create_client(AskLLM, '/llm/ask')
        if not cli.wait_for_service(timeout_sec=5.0):
            print('/llm/ask not available (start llm_node)', file=sys.stderr)
            return 1

        def ask(q):
            fut = cli.call_async(AskLLM.Request(question=q))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=180.0)
            r = fut.result()
            return (r.answer if r and r.success else f'failed: {r.answer if r else "timeout"}'), (r.tool_calls if r else [])

    questions = [a.question] if a.question else iter(lambda: input('질문> '), 'exit')
    for q in questions:
        answer, log = ask(q)
        for entry in log:
            print(f'  [tool] {entry}')
        print(answer)
    return 0


if __name__ == '__main__':
    sys.exit(main())
