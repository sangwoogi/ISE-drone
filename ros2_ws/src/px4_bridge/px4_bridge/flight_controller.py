"""FlightController: the stable Mission API of ISE-drone (Phase 1 contract).

    takeoff(height)        climb straight up to `height` metres above the arming point
    land()                 land at the current position and wait for disarm
    goto(x, y, z, yaw)     fly to an ENU position (metres, relative to PX4 local origin)
    hover(duration)        hold the current setpoint for `duration` seconds
    return_home()          fly back above the arming point at the current altitude

All blocking calls raise FlightError on timeout or when PX4 leaves OFFBOARD/armed state.
The controller spins its own rclpy executor in a background thread, so it can be used
from plain Python scripts as well as from ROS nodes.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .frames import Vec3, distance, wrap_pi
from .px4_interface import PX4Interface, VehicleState


class FlightError(RuntimeError):
    """Raised when a flight primitive cannot be completed safely."""


class FlightCancelled(FlightError):
    """Raised when the caller cancelled the primitive; the vehicle is left hovering in place."""


@dataclass
class FlightParams:
    stream_rate_hz: float = 10.0
    position_tolerance_m: float = 0.20
    velocity_tolerance_mps: float = 0.20
    settle_time_s: float = 0.5
    connection_timeout_s: float = 60.0
    arm_timeout_s: float = 15.0
    goto_timeout_s: float = 120.0
    land_timeout_s: float = 60.0
    # minimal safety envelope (Phase 8 will replace this with a real Safety Validator)
    min_altitude_m: float = 0.5
    max_altitude_m: float = 5.0
    max_horizontal_distance_m: float = 15.0


@dataclass
class StepResult:
    name: str
    ok: bool
    elapsed_s: float
    final_error_m: float
    max_error_m: float
    detail: str = ''


class FlightController:
    """High-level, blocking flight API on top of PX4Interface."""

    def __init__(self, node: Optional[Node] = None, node_name: str = 'px4_bridge',
                 params: Optional[FlightParams] = None, spin: bool = True):
        """spin=False: the caller spins `node` itself (e.g. a MultiThreadedExecutor in px4_bridge_node)."""
        self._own_node = node is None
        self.node = node or Node(node_name)
        self.params = params or self._declare_params(self.node)
        topics = self._declare_topics(self.node)
        self.px4 = PX4Interface(self.node, topics=topics, stream_rate_hz=self.params.stream_rate_hz)
        self.log = self.node.get_logger()

        self.home_enu: Optional[Vec3] = None
        self.home_yaw: Optional[float] = None

        self._executor = None
        self._spin_thread = None
        if spin:
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
            self._spin_thread = threading.Thread(target=self._executor.spin, name='px4_bridge_spin', daemon=True)
            self._spin_thread.start()

    # ------------------------------------------------------------- parameters
    @staticmethod
    def _declare_params(node: Node) -> FlightParams:
        p = FlightParams()
        for name, default in p.__dict__.items():
            node.declare_parameter(name, default)
            setattr(p, name, float(node.get_parameter(name).value))
        return p

    @staticmethod
    def _declare_topics(node: Node) -> dict:
        from .px4_interface import DEFAULT_TOPICS
        topics = {}
        for key, default in DEFAULT_TOPICS.items():
            node.declare_parameter(f'topics.{key}', default)
            topics[key] = str(node.get_parameter(f'topics.{key}').value)
        return topics

    # ----------------------------------------------------------------- state
    @property
    def state(self) -> VehicleState:
        return self.px4.state

    @property
    def position(self) -> Vec3:
        return self.px4.state.position_enu

    def shutdown(self) -> None:
        """Stop streaming, stop the spin thread, then destroy the node (in that order)."""
        self.px4.stop_stream()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
            self._spin_thread.join(timeout=3.0)
            self._executor.remove_node(self.node)
        if self._own_node:
            self.node.destroy_node()

    # ------------------------------------------------------------ primitives
    def wait_until_ready(self, timeout: Optional[float] = None) -> None:
        """Block until PX4 telemetry flows and the local position estimate is valid."""
        timeout = timeout or self.params.connection_timeout_s
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < timeout:
            s = self.px4.state
            if self.px4.is_connected() and s.position_valid:
                self.log.info(f'PX4 ready: nav_state={s.nav_state_name} armed={s.armed} '
                              f'pos={self._fmt(s.position_enu)}')
                return
            if time.time() - last_log > 5.0:
                self.log.info('waiting for PX4 telemetry / valid local position ...')
                last_log = time.time()
            time.sleep(0.1)
        raise FlightError(f'PX4 not ready after {timeout:.0f}s (topics: {self.px4.topics})')

    def takeoff(self, height: float, timeout: Optional[float] = None) -> StepResult:
        """Arm in OFFBOARD and climb to `height` m above the current position."""
        self._check_altitude(height)
        self.wait_until_ready()
        s = self.px4.state
        if s.armed:
            raise FlightError('takeoff() called while already armed')

        self.home_enu = s.position_enu
        self.home_yaw = s.yaw_enu
        target = (self.home_enu[0], self.home_enu[1], self.home_enu[2] + height)
        self.log.info(f'takeoff: home={self._fmt(self.home_enu)} target={self._fmt(target)}')

        # PX4 requires a setpoint stream *before* it accepts OFFBOARD (official sequence).
        self.px4.set_setpoint_enu(target, self.home_yaw)
        self.px4.start_stream()
        time.sleep(max(1.0, 12.0 / self.params.stream_rate_hz))

        self._request_until(lambda: self.px4.state.in_offboard, self.px4.set_offboard_mode,
                            'enter OFFBOARD', self.params.arm_timeout_s)
        self._request_until(lambda: self.px4.state.armed, self.px4.arm,
                            'arm', self.params.arm_timeout_s)

        return self._wait_reached('takeoff', target, timeout or self.params.goto_timeout_s)

    def goto(self, x: float, y: float, z: float, yaw: Optional[float] = None,
             timeout: Optional[float] = None) -> StepResult:
        """Fly to ENU (x, y, z). yaw is ENU radians (CCW from East); None keeps current yaw."""
        self._require_flying('goto')
        self._check_altitude(z - (self.home_enu[2] if self.home_enu else 0.0))
        if self.home_enu is not None:
            d = math.hypot(x - self.home_enu[0], y - self.home_enu[1])
            if d > self.params.max_horizontal_distance_m:
                raise FlightError(f'goto target {d:.1f} m from home exceeds limit '
                                  f'{self.params.max_horizontal_distance_m} m')
        target = (float(x), float(y), float(z))
        yaw_cmd = wrap_pi(yaw) if yaw is not None else self.px4.state.yaw_enu
        self.log.info(f'goto: target={self._fmt(target)} yaw={math.degrees(yaw_cmd):.0f}deg')
        self.px4.set_setpoint_enu(target, yaw_cmd)
        return self._wait_reached('goto', target, timeout or self.params.goto_timeout_s)

    def hover(self, duration: float) -> StepResult:
        """Hold the current setpoint for `duration` seconds and report max deviation."""
        self._require_flying('hover')
        target = self.px4.setpoint_enu()
        self.log.info(f'hover: {duration:.1f}s at {self._fmt(target)}')
        t0 = time.time()
        max_err = 0.0
        while time.time() - t0 < duration:
            s = self.px4.state
            self._assert_offboard_armed(s, 'hover')
            self._check_cancel('hover')
            max_err = max(max_err, distance(s.position_enu, target))
            time.sleep(0.05)
        final = distance(self.px4.state.position_enu, target)
        return StepResult('hover', True, time.time() - t0, final, max_err)

    def return_home(self, timeout: Optional[float] = None) -> StepResult:
        """Fly back above the arming point, keeping the current altitude."""
        self._require_flying('return_home')
        if self.home_enu is None:
            raise FlightError('return_home() without a recorded home (call takeoff first)')
        z = self.px4.setpoint_enu()[2]
        res = self.goto(self.home_enu[0], self.home_enu[1], z, self.home_yaw, timeout)
        res.name = 'return_home'
        return res

    def land(self, timeout: Optional[float] = None) -> StepResult:
        """Switch PX4 to AUTO_LAND at the current position, wait for touchdown + disarm."""
        timeout = timeout or self.params.land_timeout_s
        t0 = time.time()
        s = self.px4.state
        if not s.armed:
            self.px4.stop_stream()
            return StepResult('land', True, 0.0, 0.0, 0.0, 'already disarmed')
        self.log.info('land: requesting AUTO_LAND')
        self._request_until(lambda: self.px4.state.nav_state == PX4Interface.NAV_AUTO_LAND,
                            self.px4.land, 'enter AUTO_LAND', self.params.arm_timeout_s)
        # The mode switch is latched; drop the offboard stream so a later failsafe can't re-enter it.
        self.px4.stop_stream()
        while time.time() - t0 < timeout:
            s = self.px4.state
            if s.landed and not s.armed:
                self.log.info(f'landed and disarmed at {self._fmt(s.position_enu)}')
                return StepResult('land', True, time.time() - t0, 0.0, 0.0)
            time.sleep(0.1)
        raise FlightError(f'land: not landed/disarmed after {timeout:.0f}s '
                          f'(landed={s.landed} armed={s.armed} nav={s.nav_state_name})')

    def abort(self) -> None:
        """Best-effort emergency stop: land, then force-disarm if still armed."""
        self.log.error('ABORT requested')
        try:
            self.land(timeout=20.0)
        except FlightError as e:
            self.log.error(f'abort: land failed ({e}); forcing disarm')
            self.px4.stop_stream()
            self.px4.disarm(force=True)

    # --------------------------------------------------------------- helpers
    def _request_until(self, predicate, request, what: str, timeout: float, period: float = 1.0) -> None:
        t0 = time.time()
        next_send = 0.0
        while time.time() - t0 < timeout:
            if predicate():
                self.log.info(f'{what}: ok ({time.time() - t0:.1f}s)')
                return
            if time.time() >= next_send:
                request()
                next_send = time.time() + period
            time.sleep(0.05)
        s = self.px4.state
        raise FlightError(f'{what}: timeout after {timeout:.0f}s '
                          f'(nav={s.nav_state_name} armed={s.armed} acks={s.acks})')

    # Optional callback(name, distance_remaining_m, elapsed_s) used by px4_bridge_node for action feedback.
    progress_cb = None
    # Optional callable() -> bool polled by the blocking primitives; True aborts the primitive with
    # FlightCancelled after re-targeting the stream to the current position (hover in place).
    cancel_check = None

    def _check_cancel(self, what: str) -> None:
        if self.cancel_check is not None and self.cancel_check():
            held = self.px4.hold_current_position()
            self.log.warn(f'{what}: cancelled -> holding at {self._fmt(held)}')
            raise FlightCancelled(f'{what}: cancelled by client, holding position')

    def hold(self) -> Vec3:
        """Hover at the current position (keeps OFFBOARD). Returns the held position."""
        self._require_flying('hold')
        return self.px4.hold_current_position()

    def _wait_reached(self, name: str, target: Vec3, timeout: float) -> StepResult:
        t0 = time.time()
        last_fb = 0.0
        settled_since: Optional[float] = None
        max_err = 0.0
        err = float('inf')
        while time.time() - t0 < timeout:
            s = self.px4.state
            self._assert_offboard_armed(s, name)
            self._check_cancel(name)
            err = distance(s.position_enu, target)
            max_err = max(max_err, err)
            if self.progress_cb and time.time() - last_fb > 0.2:
                last_fb = time.time()
                self.progress_cb(name, err, time.time() - t0)
            if err < self.params.position_tolerance_m and s.speed < self.params.velocity_tolerance_mps:
                settled_since = settled_since or time.time()
                if time.time() - settled_since >= self.params.settle_time_s:
                    el = time.time() - t0
                    self.log.info(f'{name}: reached {self._fmt(target)} err={err:.2f}m in {el:.1f}s')
                    return StepResult(name, True, el, err, max_err)
            else:
                settled_since = None
            time.sleep(0.05)
        raise FlightError(f'{name}: target {self._fmt(target)} not reached in {timeout:.0f}s '
                          f'(err={err:.2f}m pos={self._fmt(self.px4.state.position_enu)})')

    def _assert_offboard_armed(self, s: VehicleState, what: str) -> None:
        if not s.armed:
            raise FlightError(f'{what}: vehicle disarmed unexpectedly (nav={s.nav_state_name})')
        if not s.in_offboard:
            raise FlightError(f'{what}: PX4 left OFFBOARD (nav={s.nav_state_name}) - failsafe?')

    def _require_flying(self, what: str) -> None:
        s = self.px4.state
        if not (s.armed and s.in_offboard and self.px4.streaming):
            raise FlightError(f'{what}: not flying in OFFBOARD (armed={s.armed} nav={s.nav_state_name})')

    def _check_altitude(self, rel_height: float) -> None:
        if not (self.params.min_altitude_m <= rel_height <= self.params.max_altitude_m):
            raise FlightError(f'altitude {rel_height:.2f} m outside '
                              f'[{self.params.min_altitude_m}, {self.params.max_altitude_m}] m')

    @staticmethod
    def _fmt(p: Vec3) -> str:
        return f'({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})'


def create_controller(args=None, **kwargs) -> FlightController:
    """Convenience: rclpy.init + FlightController for scripts."""
    if not rclpy.ok():
        rclpy.init(args=args)
    return FlightController(**kwargs)
