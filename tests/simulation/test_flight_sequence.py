#!/usr/bin/env python3
"""Phase 1 acceptance test: fly the reference sequence N times in a row.

    Arm -> Takeoff 1.5 m -> Hover 5 s -> Move 2 m -> Hover -> Return -> Land

Requires a running simulation:
    ros2 launch drone_bringup sitl.launch.py headless:=true
Then:
    python3 tests/simulation/test_flight_sequence.py --runs 10

Each run is appended to experiments/<exp>/metrics.csv and a trajectory CSV is written
per run (for plotting with tools/plot_flight.py). Exit code 0 only when every run passed.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from datetime import datetime

import rclpy

from px4_bridge.flight_controller import FlightController, FlightError
from px4_bridge.flight_demo_node import print_results, run_sequence

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


class TrajectoryRecorder:
    """Samples the vehicle position in a background thread while a run is in progress."""

    def __init__(self, fc: FlightController, path: str, period: float = 0.1):
        self._fc, self._path, self._period = fc, path, period
        self._stop = threading.Event()
        self._rows = []
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def _loop(self):
        t0 = time.time()
        while not self._stop.is_set():
            s = self._fc.state
            sp = self._fc.px4.setpoint_enu()
            self._rows.append((round(time.time() - t0, 3), *[round(v, 3) for v in s.position_enu],
                               *[round(v, 3) for v in sp], s.nav_state_name, int(s.armed), int(s.landed)))
            time.sleep(self._period)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2.0)
        with open(self._path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'sp_x', 'sp_y', 'sp_z', 'nav_state', 'armed', 'landed'])
            w.writerows(self._rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=10)
    ap.add_argument('--height', type=float, default=1.5)
    ap.add_argument('--distance', type=float, default=2.0)
    ap.add_argument('--hover', type=float, default=5.0)
    ap.add_argument('--pause', type=float, default=3.0, help='seconds to wait on the ground between runs')
    ap.add_argument('--exp', default='001_phase1_flight', help='experiments/<exp>/ output folder')
    ap.add_argument('--stop-on-fail', action='store_true')
    args, ros_args = ap.parse_known_args(argv)

    exp_dir = os.path.join(ROOT, 'experiments', args.exp)
    traj_dir = os.path.join(exp_dir, 'trajectories')
    os.makedirs(traj_dir, exist_ok=True)
    metrics_path = os.path.join(exp_dir, 'metrics.csv')
    new_file = not os.path.exists(metrics_path)
    session = datetime.now().strftime('%Y%m%d_%H%M%S')

    rclpy.init(args=ros_args)
    fc = FlightController(node_name='phase1_flight_test')
    passed = 0
    try:
        fc.wait_until_ready()
        with open(metrics_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(['session', 'run', 'ok', 'total_s', 'takeoff_s', 'goto_s', 'return_s', 'land_s',
                            'hover_max_err_m', 'goto_final_err_m', 'return_final_err_m', 'error'])
            for run in range(1, args.runs + 1):
                print(f'\n===== run {run}/{args.runs} =====')
                t0 = time.time()
                err_msg = ''
                results = []
                try:
                    with TrajectoryRecorder(fc, os.path.join(traj_dir, f'{session}_run{run:02d}.csv')):
                        results = run_sequence(fc, args.height, args.distance, args.hover, verbose=False)
                    ok = all(r.ok for r in results)
                except FlightError as e:
                    ok = False
                    err_msg = str(e)
                    print(f'run {run} FAILED: {e}', file=sys.stderr)
                    fc.abort()
                total = time.time() - t0
                by = {r.name: r for r in results}

                def g(name, attr):
                    r = by.get(name)
                    return f'{getattr(r, attr):.2f}' if r else ''
                hovers = [r for r in results if r.name == 'hover']
                w.writerow([session, run, int(ok), f'{total:.1f}', g('takeoff', 'elapsed_s'), g('goto', 'elapsed_s'),
                            g('return_home', 'elapsed_s'), g('land', 'elapsed_s'),
                            f'{max(h.max_error_m for h in hovers):.2f}' if hovers else '',
                            g('goto', 'final_error_m'), g('return_home', 'final_error_m'), err_msg])
                f.flush()
                if results:
                    print_results(results)
                print(f'run {run}: {"PASS" if ok else "FAIL"} ({total:.1f}s)')
                passed += int(ok)
                if not ok and args.stop_on_fail:
                    break
                time.sleep(args.pause)
    except FlightError as e:
        print(f'setup failed: {e}', file=sys.stderr)
    finally:
        fc.shutdown()
        rclpy.shutdown()

    print(f'\n===== {passed}/{args.runs} runs passed  (metrics: {metrics_path}) =====')
    return 0 if passed == args.runs else 1


if __name__ == '__main__':
    sys.exit(main())
