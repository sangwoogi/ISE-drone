#!/usr/bin/env python3
"""Phase 1 failsafe check: stop the offboard heartbeat mid-hover and verify PX4 reacts.

Expected PX4 behaviour (COM_OF_LOSS_T = 3 s in our SITL launch): leave OFFBOARD within ~3-4 s and
execute the offboard-loss failsafe action (COM_OBL_ACT, default: Land). The test passes
when OFFBOARD is left within --max-react seconds and the vehicle ends up landed+disarmed.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy

from px4_bridge.flight_controller import FlightController, FlightError


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--height', type=float, default=1.5)
    ap.add_argument('--max-react', type=float, default=5.0, help='max seconds allowed to leave OFFBOARD (COM_OF_LOSS_T=3 in SITL + margin)')
    ap.add_argument('--land-timeout', type=float, default=60.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    fc = FlightController(node_name='phase1_failsafe_test')
    rc = 1
    try:
        fc.takeoff(args.height)
        fc.hover(3.0)
        print('>>> stopping OffboardControlMode / TrajectorySetpoint stream')
        t0 = time.time()
        fc.px4.stop_stream()
        left_offboard_at = None
        transitions = []
        last = fc.state.nav_state_name
        while time.time() - t0 < args.land_timeout:
            s = fc.state
            if s.nav_state_name != last:
                transitions.append((round(time.time() - t0, 2), s.nav_state_name))
                last = s.nav_state_name
            if left_offboard_at is None and not s.in_offboard:
                left_offboard_at = time.time() - t0
            if s.landed and not s.armed:
                break
            time.sleep(0.05)
        s = fc.state
        print(f'nav_state transitions after stream stop: {transitions}')
        print(f'left OFFBOARD after: {left_offboard_at}s   landed={s.landed} armed={s.armed}')
        ok = (left_offboard_at is not None and left_offboard_at <= args.max_react and s.landed and not s.armed)
        print('FAILSAFE OK' if ok else 'FAILSAFE CHECK FAILED')
        rc = 0 if ok else 1
        if not ok:
            fc.abort()
    except FlightError as e:
        print(f'failed: {e}', file=sys.stderr)
        fc.abort()
    finally:
        fc.shutdown()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
