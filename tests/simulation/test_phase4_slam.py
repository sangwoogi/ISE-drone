#!/usr/bin/env python3
"""Phase 4 acceptance: fly a scripted loop through indoor_v1 while RTAB-Map maps, then evaluate.

Route (plan §29 scripted trajectory): Room A -> hallway -> Room B -> back, `--laps` times at 1.2 m, yaw facing travel.
Needs: simulation.launch.py (gz_x500_rgbd, indoor_v1, use_sim_time, px4_tf:=false for vo) +
       sensor_adapter gazebo_sensors.launch.py + drone_slam rtabmap.launch.py record_dir:=<run_dir>
After the flight it evaluates <run_dir>/{est,odom,px4,gt}.csv with drone_slam.slam_eval (ATE/RPE),
saves the occupancy grid (/slam/map) as PNG, a trajectory plot and report.json.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.action import FlightCommand
from drone_interfaces.msg import VehicleState

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'drone_slam'))
from drone_slam.slam_eval import associate, ate, load_csv, rpe  # noqa: E402

# Room A (origin) -> doorway x=3 -> hallway -> doorway x=7 -> Room B -> back. All |y| < 1 at the doors.
ROUTE = [(1.5, 0.0), (3.0, 0.0), (5.0, 0.0), (7.0, 0.0), (9.0, 0.5), (10.5, -0.5), (9.0, 1.2), (7.0, 0.0),
         (5.0, 0.0), (3.0, 0.0), (0.5, 1.2), (-1.5, 0.5), (0.0, 0.0)]


class SlamFlight(Node):
    def __init__(self):
        super().__init__('phase4_slam_flight')
        self.client = ActionClient(self, FlightCommand, '/px4_bridge/flight_command')
        self.state = None
        self.grid = None
        self.create_subscription(VehicleState, '/drone/vehicle_state', self._on_state, 10)
        self.create_subscription(OccupancyGrid, '/slam/map', self._on_grid,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_state(self, m):
        self.state = m

    def _on_grid(self, m):
        self.grid = m

    def send(self, timeout=150.0, **fields):
        g = FlightCommand.Goal()
        for k, v in fields.items():
            if k == 'target':
                g.target.x, g.target.y, g.target.z = v
            else:
                setattr(g, k, v)
        fut = self.client.send_goal_async(g)
        while not fut.done():
            time.sleep(0.02)
        gh = fut.result()
        if not gh.accepted:
            return 'REJECTED', None
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not rf.done():
            gh.cancel_goal_async()
            return 'TIMEOUT', None
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(rf.result().status, '?'), rf.result().result


def save_grid_png(grid: OccupancyGrid, path: str):
    import cv2
    data = np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    img = np.full(data.shape, 128, dtype=np.uint8)
    img[data == 0] = 255
    img[data > 50] = 0
    cv2.imwrite(path, cv2.flip(img, 0))
    return data


def plot(run_dir, rep):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5))
    for key, style, lbl in (('gt', 'k-', 'ground truth'), ('est', 'r-', 'SLAM map->base_link'), ('px4', 'b--', 'PX4 EKF')):
        p = os.path.join(run_dir, f'{key}.csv')
        if os.path.exists(p) and os.path.getsize(p) > 40:
            _, xyz = load_csv(p)
            ax.plot(xyz[:, 0], xyz[:, 1], style, lw=1, label=lbl)
    for x0, x1, y0, y1 in ((-3, 3, -2.5, 2.5), (3, 7, -1, 1), (7, 12, -2.5, 2.5)):
        ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color='0.6', lw=0.8)
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.set_xlabel('x East [m]'); ax.set_ylabel('y North [m]')
    a = rep.get('ate_est_vs_gt', {})
    ax.set_title(f'{os.path.basename(run_dir)}: ATE rmse {a.get("rmse", float("nan")):.3f} m, '
                 f'loop closures {rep.get("stats", {}).get("loop_closures", "?")}, lost {rep.get("stats", {}).get("tracking_lost_events", "?")}')
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(run_dir, 'trajectory.png'), dpi=130)


def evaluate(run_dir, max_dt=0.1, rpe_delta=1.0):
    rep = {}
    gt_path = os.path.join(run_dir, 'gt.csv')
    t_g, p_g = load_csv(gt_path)
    for key in ('est', 'px4', 'odom'):
        p = os.path.join(run_dir, f'{key}.csv')
        if not os.path.exists(p) or os.path.getsize(p) < 40:
            continue
        t_e, p_e = load_csv(p)
        pairs = associate(t_e, t_g, max_dt)
        if len(pairs) < 10:
            rep[f'ate_{key}_vs_gt'] = {'error': f'only {len(pairs)} associated samples'}
            continue
        ie, ig = zip(*pairs)
        est, gt, t = p_e[list(ie)], p_g[list(ig)], t_e[list(ie)]
        a_rmse, a_mean, a_max = ate(est, gt)
        a_rmse_al, _, _ = ate(est, gt, align=True)
        r_rmse, r_mean = rpe(t, est, gt, rpe_delta)
        rep[f'ate_{key}_vs_gt'] = {'samples': len(pairs), 'rmse': round(a_rmse, 3), 'mean': round(a_mean, 3), 'max': round(a_max, 3),
                                   'rmse_aligned': round(a_rmse_al, 3), 'rpe_1s_rmse': round(r_rmse, 3), 'rpe_1s_mean': round(r_mean, 3)}
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True, help='directory the trajectory_recorder writes to')
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--height', type=float, default=1.2)
    ap.add_argument('--skip-flight', action='store_true', help='only evaluate an existing run dir')
    ap.add_argument('--ate-gate', type=float, default=0.5, help='PASS if ATE rmse (est vs gt) <= gate [m]')
    args, ros_args = ap.parse_known_args(argv)
    run_dir = os.path.abspath(args.run_dir)
    os.makedirs(run_dir, exist_ok=True)
    rep = {'laps': args.laps, 'height': args.height}

    if not args.skip_flight:
        rclpy.init(args=ros_args)
        node = SlamFlight()
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < 90 and not (node.state and node.state.position_valid and node.client.server_is_ready()):
            time.sleep(0.2)
        home = node.state.pose.position
        z = home.z + args.height
        st, _ = node.send(command='TAKEOFF', height=args.height)
        print('takeoff', st)
        ok = st == 'SUCCEEDED'
        flight_log = []
        if ok:
            node.send(command='HOVER', duration=3.0)
            prev = (home.x, home.y)
            for lap in range(1, args.laps + 1):
                for i, (x, y) in enumerate(ROUTE):
                    yaw = math.atan2(y - prev[1], x - prev[0])
                    # turn in place first (visual odometry dislikes translating while yawing fast)
                    node.send(command='GOTO', target=(prev[0], prev[1], z), yaw=float(yaw), use_yaw=True)
                    st, res = node.send(command='GOTO', target=(x, y, z), yaw=float(yaw), use_yaw=True)
                    flight_log.append({'lap': lap, 'wp': i, 'xy': [x, y], 'status': st, 'err': getattr(res, 'final_error', None)})
                    print(f'lap {lap} wp {i:2d} ({x:5.1f},{y:5.1f}) {st}')
                    if st != 'SUCCEEDED':
                        ok = False
                        break
                    prev = (x, y)
                    node.send(command='HOVER', duration=0.5)
                if not ok:
                    break
            node.send(command='RETURN_HOME')
            node.send(command='HOVER', duration=2.0)
        st, _ = node.send(command='LAND')
        print('land', st)
        rep['flight_ok'] = ok
        rep['flight_time_s'] = round(time.time() - t0, 1)
        rep['waypoints'] = flight_log
        if node.grid is not None:
            data = save_grid_png(node.grid, os.path.join(run_dir, 'map.png'))
            rep['map'] = {'width': node.grid.info.width, 'height': node.grid.info.height, 'resolution': node.grid.info.resolution,
                          'free_cells': int((data == 0).sum()), 'occupied_cells': int((data > 50).sum())}
        ex.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()
        time.sleep(2.0)   # let the recorder flush

    stats_path = os.path.join(run_dir, 'stats.json')
    rep['stats'] = json.load(open(stats_path)) if os.path.exists(stats_path) else {}
    try:
        rep.update(evaluate(run_dir))
    except Exception as e:  # noqa: BLE001
        rep['eval_error'] = str(e)
    a = rep.get('ate_est_vs_gt', {})
    rep['pass'] = bool(rep.get('flight_ok', True) and 'rmse' in a and a['rmse'] <= args.ate_gate
                       and rep['stats'].get('tracking_lost_events', 0) == 0)
    with open(os.path.join(run_dir, 'report.json'), 'w') as f:
        json.dump(rep, f, indent=2)
    try:
        plot(run_dir, rep)
    except Exception as e:  # noqa: BLE001
        print('plot failed:', e)
    print(json.dumps({k: v for k, v in rep.items() if k != 'waypoints'}, indent=2))
    print('PHASE4 SLAM:', 'PASS' if rep['pass'] else 'FAIL')
    sys.stdout.flush()
    os._exit(0 if rep['pass'] else 1)


if __name__ == '__main__':
    main()
