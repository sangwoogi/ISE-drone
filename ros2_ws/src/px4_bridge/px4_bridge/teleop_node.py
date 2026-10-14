"""teleop — fly the simulated drone from the keyboard through the FlightCommand action (Phase 2 demo tool).

    ros2 run px4_bridge teleop            # needs simulation.launch.py (px4_bridge_node) running

Keys (terminal must have focus):
    t  take off (1.5 m)        l  land              h  hover / hold here
    w/s  forward/back  a/d  left/right  r/f  up/down   (step: --step, default 0.5 m)
    q/e  yaw left/right 15 deg          c  cancel current goal (hold)
    +/-  change step               x  quit (lands first if airborne)

Moves are relative to the current position in ENU (w = +x East, a = +y North). Every key press is one
GOTO goal; the node never touches px4_msgs, exactly like mission_manager will.
"""
from __future__ import annotations

import math
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_interfaces.action import FlightCommand
from drone_interfaces.msg import VehicleState

from .frames import yaw_from_quat_xyzw


class Teleop(Node):
    def __init__(self, step: float, height: float):
        super().__init__('teleop')
        self.step, self.height = step, height
        self.client = ActionClient(self, FlightCommand, '/px4_bridge/flight_command')
        self.state: VehicleState | None = None
        self.create_subscription(VehicleState, '/drone/vehicle_state', self._on_state, 10)
        self._goal = None
        self._target = None

    def _on_state(self, msg):
        self.state = msg

    # ------------------------------------------------------------ helpers
    def pos(self):
        p = self.state.pose.position
        return [p.x, p.y, p.z]

    def yaw(self):
        q = self.state.pose.orientation
        return yaw_from_quat_xyzw((q.x, q.y, q.z, q.w))

    def send(self, goal: FlightCommand.Goal, wait: bool = False):
        if self._goal is not None and not self._goal_done():
            self._goal.cancel_goal_async()
            time.sleep(0.3)
        fut = self.client.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 2.0:
            time.sleep(0.02)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            self.get_logger().warn(f'{goal.command} rejected')
            return None
        self._goal = gh
        self._result = gh.get_result_async()
        if wait:
            while not self._goal_done():
                time.sleep(0.05)
            self.get_logger().info(f'{goal.command}: {self._result.result().result.message}')
        return gh

    def _goal_done(self):
        return self._goal is None or self._result.done()

    def goto(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        base = self._target if (self._target is not None and not self._goal_done()) else self.pos()
        tgt = [base[0] + dx, base[1] + dy, max(0.5, base[2] + dz)]
        yaw = (self._target_yaw if (self._target is not None and not self._goal_done()) else self.yaw()) + dyaw
        g = FlightCommand.Goal()
        g.command = 'GOTO'
        g.target.x, g.target.y, g.target.z = tgt
        g.yaw, g.use_yaw = float(yaw), True
        self._target, self._target_yaw = tgt, yaw
        self.send(g)
        self.get_logger().info(f'goto ({tgt[0]:.2f}, {tgt[1]:.2f}, {tgt[2]:.2f}) yaw {math.degrees(yaw):.0f}deg')

    def simple(self, command, wait=False, **kw):
        g = FlightCommand.Goal()
        g.command = command
        for k, v in kw.items():
            setattr(g, k, v)
        self._target = None
        return self.send(g, wait=wait)


def read_key(timeout=0.1):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=0.5)
    ap.add_argument('--height', type=float, default=1.5)
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Teleop(args.step, args.height)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(__doc__)
    if not node.client.wait_for_server(timeout_sec=10.0):
        print('px4_bridge action server not found - start simulation.launch.py first', file=sys.stderr)
        return 1
    while node.state is None:
        time.sleep(0.1)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    yaw_step = math.radians(15)
    try:
        last_status = 0.0
        while rclpy.ok():
            k = read_key()
            s = node.state
            if time.time() - last_status > 2.0:
                p = node.pos()
                print(f'\r[{s.nav_state:<12} armed={int(s.armed)}] pos=({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f}) '
                      f'yaw={math.degrees(node.yaw()):+.0f} step={node.step:.2f}m   ', end='', flush=True)
                last_status = time.time()
            if k is None:
                continue
            print()
            flying = s.armed and s.in_offboard
            if k == 'x':
                if flying:
                    node.simple('LAND', wait=True)
                break
            if k == 't' and not flying:
                node.simple('TAKEOFF', height=float(node.height))
            elif k == 'l' and flying:
                node.simple('LAND')
            elif k == 'c' and node._goal is not None:
                node._goal.cancel_goal_async()
                node._target = None
            elif k == 'h' and flying:
                node.goto()
            elif k == '+':
                node.step = min(2.0, node.step + 0.25)
            elif k == '-':
                node.step = max(0.25, node.step - 0.25)
            elif flying and k in 'wsadrfqe':
                st = node.step
                {'w': lambda: node.goto(dx=st), 's': lambda: node.goto(dx=-st),
                 'a': lambda: node.goto(dy=st), 'd': lambda: node.goto(dy=-st),
                 'r': lambda: node.goto(dz=st), 'f': lambda: node.goto(dz=-st),
                 'q': lambda: node.goto(dyaw=yaw_step), 'e': lambda: node.goto(dyaw=-yaw_step)}[k]()
            elif not flying and k in 'wsadrfqehl':
                print('not flying - press t to take off')
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
