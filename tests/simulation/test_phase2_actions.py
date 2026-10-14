#!/usr/bin/env python3
"""Phase 2 acceptance: drive the flight sequence ONLY through the ROS 2 interface of px4_bridge_node
(FlightCommand action + /drone/* telemetry), N times, plus a cancel-to-hold check.

Requires:  ros2 launch drone_bringup simulation.launch.py headless:=true rviz:=false
Writes:    experiments/002_phase2_bringup/metrics.csv, telemetry_runNN.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_interfaces.action import FlightCommand
from drone_interfaces.msg import VehicleState

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


class Phase2Client(Node):
    def __init__(self):
        super().__init__('phase2_test')
        self.client = ActionClient(self, FlightCommand, '/px4_bridge/flight_command')
        self.state: VehicleState | None = None
        self.state_count = 0
        self.create_subscription(VehicleState, '/drone/vehicle_state', self._on_state, 10)
        self._rec = None

    def _on_state(self, msg):
        self.state, self.state_count = msg, self.state_count + 1
        if self._rec is not None:
            p = msg.pose.position
            self._rec.append((round(time.time(), 3), round(p.x, 3), round(p.y, 3), round(p.z, 3), msg.nav_state, int(msg.armed)))

    def wait_ready(self, timeout=90.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state is not None and self.state.position_valid and self.client.server_is_ready():
                return True
            time.sleep(0.2)
        return False

    def send(self, timeout, **fields):
        """Blocking FlightCommand. Returns (status_str, result, elapsed)."""
        goal = FlightCommand.Goal()
        for k, v in fields.items():
            if k == 'target':
                goal.target.x, goal.target.y, goal.target.z = v
            else:
                setattr(goal, k, v)
        t0 = time.time()
        fut = self.client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.02)
        gh = fut.result()
        if not gh.accepted:
            return 'REJECTED', None, time.time() - t0
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not rf.done():
            gh.cancel_goal_async()
            return 'TIMEOUT', None, time.time() - t0
        status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(rf.result().status, str(rf.result().status))
        return status, rf.result().result, time.time() - t0

    def send_then_cancel(self, after_s, **fields):
        goal = FlightCommand.Goal()
        for k, v in fields.items():
            if k == 'target':
                goal.target.x, goal.target.y, goal.target.z = v
            else:
                setattr(goal, k, v)
        fut = self.client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.02)
        gh = fut.result()
        time.sleep(after_s)
        pos_at_cancel = self.pos()
        gh.cancel_goal_async()
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < 20:
            time.sleep(0.05)
        status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(rf.result().status, str(rf.result().status)) if rf.done() else 'TIMEOUT'
        return status, pos_at_cancel

    def pos(self):
        p = self.state.pose.position
        return (p.x, p.y, p.z)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--height', type=float, default=1.5)
    ap.add_argument('--distance', type=float, default=2.0)
    ap.add_argument('--hover', type=float, default=5.0)
    ap.add_argument('--exp', default='002_phase2_bringup')
    ap.add_argument('--skip-cancel-test', action='store_true')
    args, ros_args = ap.parse_known_args(argv)

    exp = os.path.join(ROOT, 'experiments', args.exp)
    os.makedirs(exp, exist_ok=True)
    session = datetime.now().strftime('%Y%m%d_%H%M%S')
    metrics_path = os.path.join(exp, 'metrics.csv')
    new = not os.path.exists(metrics_path)

    rclpy.init(args=ros_args)
    node = Phase2Client()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    passed = 0
    cancel_ok = None
    try:
        if not node.wait_ready():
            print('px4_bridge_node / telemetry not ready', file=sys.stderr)
            return 1
        print(f'telemetry ok ({node.state_count} VehicleState msgs), action server ready')
        with open(metrics_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['session', 'run', 'ok', 'total_s', 'takeoff', 'hover', 'goto', 'return', 'land',
                            'goto_err_m', 'return_err_m', 'detail'])
            for run in range(1, args.runs + 1):
                print(f'\n===== run {run}/{args.runs} =====')
                node._rec = []
                t0 = time.time()
                home = node.pos()
                steps, errs, ok, detail = {}, {}, True, ''
                plan = [('takeoff', dict(command='TAKEOFF', height=args.height), 60),
                        ('hover', dict(command='HOVER', duration=args.hover), 30),
                        ('goto', dict(command='GOTO', target=(home[0] + args.distance, home[1], home[2] + args.height)), 60),
                        ('hover2', dict(command='HOVER', duration=2.0), 30),
                        ('return', dict(command='RETURN_HOME'), 60),
                        ('land', dict(command='LAND'), 60)]
                for name, fields, to in plan:
                    status, res, el = node.send(to, **fields)
                    steps[name] = status
                    print(f'  {name:<8} {status:<10} {el:5.1f}s  {res.message if res else ""}')
                    if res is not None:
                        errs[name] = res.final_error
                    if status != 'SUCCEEDED':
                        ok, detail = False, f'{name}: {status} {res.message if res else ""}'
                        node.send(60, command='LAND')
                        break
                total = time.time() - t0
                w.writerow([session, run, int(ok), f'{total:.1f}', steps.get('takeoff', ''), steps.get('hover', ''),
                            steps.get('goto', ''), steps.get('return', ''), steps.get('land', ''),
                            f'{errs.get("goto", float("nan")):.2f}', f'{errs.get("return", float("nan")):.2f}', detail])
                f.flush()
                with open(os.path.join(exp, f'telemetry_{session}_run{run:02d}.csv'), 'w', newline='') as tf:
                    cw = csv.writer(tf)
                    cw.writerow(['t', 'x', 'y', 'z', 'nav_state', 'armed'])
                    cw.writerows(node._rec)
                node._rec = None
                passed += int(ok)
                print(f'run {run}: {"PASS" if ok else "FAIL"} ({total:.1f}s)')
                time.sleep(3.0)

        if not args.skip_cancel_test:
            print('\n===== cancel -> hold test =====')
            home = node.pos()
            st, _, _ = node.send(60, command='TAKEOFF', height=args.height)
            status, p_cancel = node.send_then_cancel(1.5, command='GOTO', target=(home[0] + 3.0, home[1], home[2] + args.height))
            time.sleep(3.0)
            p_after = node.pos()
            drift = math.dist(p_cancel, p_after)
            s = node.state
            cancel_ok = status == 'CANCELED' and s.armed and s.in_offboard and drift < 0.5
            print(f'  goto cancelled after 1.5 s: status={status} drift_3s={drift:.2f} m armed={s.armed} nav={s.nav_state} -> {"PASS" if cancel_ok else "FAIL"}')
            node.send(60, command='LAND')
            with open(os.path.join(exp, 'cancel_test.txt'), 'a') as cf:
                cf.write(f'{session} status={status} drift_after_3s={drift:.2f} armed={s.armed} nav={s.nav_state} result={"PASS" if cancel_ok else "FAIL"}\n')
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()

    print(f'\n===== {passed}/{args.runs} runs passed; cancel test: {cancel_ok} =====')
    return 0 if passed == args.runs and cancel_ok in (True, None) else 1


if __name__ == '__main__':
    sys.exit(main())
