"""`ros2 run px4_bridge flight_demo` — fly the Phase 1 sequence once.

    Arm -> Takeoff h -> Hover t -> goto(+d m, +x) -> Hover -> Return home -> Land
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy

from .flight_controller import FlightController, FlightError


def run_sequence(fc: FlightController, height: float, distance: float, hover_s: float, verbose: bool = True):
    """Fly one sequence; return list of StepResult. Raises FlightError on failure."""
    results = []
    results.append(fc.takeoff(height))
    results.append(fc.hover(hover_s))
    hx, hy, hz = fc.home_enu
    results.append(fc.goto(hx + distance, hy, hz + height))
    results.append(fc.hover(2.0))
    results.append(fc.return_home())
    results.append(fc.hover(1.0))
    results.append(fc.land())
    if verbose:
        print_results(results)
    return results


def print_results(results) -> None:
    print(f'{"step":<12}{"ok":<5}{"time[s]":>8}{"final_err[m]":>14}{"max_err[m]":>12}')
    for r in results:
        print(f'{r.name:<12}{str(r.ok):<5}{r.elapsed_s:>8.1f}{r.final_error_m:>14.2f}{r.max_error_m:>12.2f}')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='PX4 offboard flight demo (Phase 1)')
    parser.add_argument('--height', type=float, default=1.5, help='takeoff height [m]')
    parser.add_argument('--distance', type=float, default=2.0, help='forward move along +x (East) [m]')
    parser.add_argument('--hover', type=float, default=5.0, help='hover duration after takeoff [s]')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    fc = FlightController()
    rc = 0
    t0 = time.time()
    try:
        run_sequence(fc, args.height, args.distance, args.hover)
        print(f'SEQUENCE OK in {time.time() - t0:.1f}s')
    except FlightError as e:
        print(f'SEQUENCE FAILED: {e}', file=sys.stderr)
        fc.abort()
        rc = 1
    except KeyboardInterrupt:
        fc.abort()
        rc = 130
    finally:
        fc.shutdown()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
