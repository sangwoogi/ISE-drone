"""Thin, thread-safe wrapper around the PX4 uXRCE-DDS topics.

This module is the ONLY place in ISE-drone that imports px4_msgs. Everything it
exposes to callers is expressed in ROS ENU coordinates; NED conversion happens here.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from . import frames
from .frames import Vec3

# PX4 publishes /fmu/out/* with BEST_EFFORT + TRANSIENT_LOCAL, depth 1.
QOS_PX4_OUT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Publishers to /fmu/in/*: default reliable QoS is compatible with the agent's readers.
QOS_PX4_IN = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)

DEFAULT_TOPICS = {
    'vehicle_status': '/fmu/out/vehicle_status_v1',
    'vehicle_local_position': '/fmu/out/vehicle_local_position_v1',
    'vehicle_land_detected': '/fmu/out/vehicle_land_detected',
    'vehicle_command_ack': '/fmu/out/vehicle_command_ack',
    'offboard_control_mode': '/fmu/in/offboard_control_mode',
    'trajectory_setpoint': '/fmu/in/trajectory_setpoint',
    'vehicle_command': '/fmu/in/vehicle_command',
}

NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: 'MANUAL',
    VehicleStatus.NAVIGATION_STATE_ALTCTL: 'ALTCTL',
    VehicleStatus.NAVIGATION_STATE_POSCTL: 'POSCTL',
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: 'AUTO_MISSION',
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'AUTO_LOITER',
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: 'AUTO_RTL',
    VehicleStatus.NAVIGATION_STATE_ACRO: 'ACRO',
    VehicleStatus.NAVIGATION_STATE_DESCEND: 'DESCEND',
    VehicleStatus.NAVIGATION_STATE_TERMINATION: 'TERMINATION',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'OFFBOARD',
    VehicleStatus.NAVIGATION_STATE_STAB: 'STAB',
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: 'AUTO_TAKEOFF',
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 'AUTO_LAND',
    VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET: 'AUTO_FOLLOW_TARGET',
    VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND: 'AUTO_PRECLAND',
    VehicleStatus.NAVIGATION_STATE_ORBIT: 'ORBIT',
    VehicleStatus.NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: 'AUTO_VTOL_TAKEOFF',
}

ACK_RESULT_NAMES = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'IN_PROGRESS',
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: 'CANCELLED',
}


@dataclass
class VehicleState:
    """Snapshot of what the bridge knows about the vehicle, in ENU."""
    status_time: float = 0.0          # wall time of last VehicleStatus
    position_time: float = 0.0        # wall time of last VehicleLocalPosition
    armed: bool = False
    nav_state: int = -1
    position_valid: bool = False
    position_enu: Vec3 = (0.0, 0.0, 0.0)
    velocity_enu: Vec3 = (0.0, 0.0, 0.0)
    yaw_enu: float = 0.0
    landed: bool = True
    acks: dict = field(default_factory=dict)   # command id -> (result, wall time)

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f'UNKNOWN({self.nav_state})')

    @property
    def in_offboard(self) -> bool:
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def speed(self) -> float:
        vx, vy, vz = self.velocity_enu
        return math.sqrt(vx * vx + vy * vy + vz * vz)


class PX4Interface:
    """Publishes the offboard stream and mirrors PX4 telemetry into a VehicleState."""

    def __init__(self, node: Node, topics: Optional[dict] = None, stream_rate_hz: float = 10.0):
        self._node = node
        self._log = node.get_logger()
        self._lock = threading.Lock()
        self._state = VehicleState()
        t = dict(DEFAULT_TOPICS, **(topics or {}))
        self.topics = t

        # --- setpoint stream (NED, internal only) ---
        self._streaming = False
        self._sp_ned: Vec3 = (0.0, 0.0, 0.0)
        self._sp_yaw_ned: float = float('nan')

        self._pub_mode = node.create_publisher(OffboardControlMode, t['offboard_control_mode'], QOS_PX4_IN)
        self._pub_sp = node.create_publisher(TrajectorySetpoint, t['trajectory_setpoint'], QOS_PX4_IN)
        self._pub_cmd = node.create_publisher(VehicleCommand, t['vehicle_command'], QOS_PX4_IN)

        node.create_subscription(VehicleStatus, t['vehicle_status'], self._on_status, QOS_PX4_OUT)
        node.create_subscription(VehicleLocalPosition, t['vehicle_local_position'], self._on_local_position, QOS_PX4_OUT)
        node.create_subscription(VehicleLandDetected, t['vehicle_land_detected'], self._on_land_detected, QOS_PX4_OUT)
        node.create_subscription(VehicleCommandAck, t['vehicle_command_ack'], self._on_ack, QOS_PX4_OUT)

        # Steady clock: the heartbeat cadence must not stall when the wall clock is stepped
        # (seen on WSL2: backward steps of ~0.8 s made PX4 declare offboard signal loss).
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = node.create_timer(1.0 / stream_rate_hz, self._on_stream_timer, clock=self._steady_clock)

    # ------------------------------------------------------------------ state
    @property
    def state(self) -> VehicleState:
        """Return a copy of the latest state (safe to read from any thread)."""
        with self._lock:
            s = VehicleState(**{k: v for k, v in self._state.__dict__.items() if k != 'acks'})
            s.acks = dict(self._state.acks)
            return s

    def is_connected(self, max_age_s: float = 2.0) -> bool:
        s = self.state
        now = time.time()
        return (now - s.status_time) < max_age_s and (now - s.position_time) < max_age_s

    # -------------------------------------------------------------- callbacks
    def _on_status(self, msg: VehicleStatus) -> None:
        with self._lock:
            self._state.status_time = time.time()
            self._state.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
            self._state.nav_state = int(msg.nav_state)

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._state.position_time = time.time()
            self._state.position_valid = bool(msg.xy_valid and msg.z_valid)
            self._state.position_enu = frames.ned_to_enu((float(msg.x), float(msg.y), float(msg.z)))
            self._state.velocity_enu = frames.ned_to_enu((float(msg.vx), float(msg.vy), float(msg.vz)))
            self._state.yaw_enu = frames.yaw_ned_to_enu(float(msg.heading))

    def _on_land_detected(self, msg: VehicleLandDetected) -> None:
        with self._lock:
            self._state.landed = bool(msg.landed)

    def _on_ack(self, msg: VehicleCommandAck) -> None:
        with self._lock:
            self._state.acks[int(msg.command)] = (int(msg.result), time.time())
        name = ACK_RESULT_NAMES.get(int(msg.result), str(msg.result))
        if msg.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self._log.warn(f'VehicleCommand {msg.command} -> {name}')
        else:
            self._log.debug(f'VehicleCommand {msg.command} -> {name}')

    # ---------------------------------------------------------------- stream
    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds // 1000)

    def set_setpoint_enu(self, position_enu: Vec3, yaw_enu: Optional[float] = None) -> None:
        """Set the position (and optional yaw) that the stream will hold."""
        with self._lock:
            self._sp_ned = frames.enu_to_ned(position_enu)
            self._sp_yaw_ned = frames.yaw_enu_to_ned(yaw_enu) if yaw_enu is not None else float('nan')

    def setpoint_enu(self) -> Vec3:
        with self._lock:
            return frames.ned_to_enu(self._sp_ned)

    def start_stream(self) -> None:
        with self._lock:
            self._streaming = True

    def stop_stream(self) -> None:
        """Stop the heartbeat. PX4 will leave OFFBOARD (failsafe) within ~1 s."""
        with self._lock:
            self._streaming = False

    @property
    def streaming(self) -> bool:
        return self._streaming

    def _on_stream_timer(self) -> None:
        with self._lock:
            if not self._streaming:
                return
            sp_ned, yaw_ned = self._sp_ned, self._sp_yaw_ned
        ts = self._now_us()

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        self._pub_mode.publish(mode)

        nan = float('nan')
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(sp_ned[0]), float(sp_ned[1]), float(sp_ned[2])]
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = float(yaw_ned)
        sp.yawspeed = nan
        self._pub_sp.publish(sp)

    # -------------------------------------------------------------- commands
    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
                     param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = int(command)
        msg.param1, msg.param2, msg.param3, msg.param4 = float(param1), float(param2), float(param3), float(param4)
        msg.param5, msg.param6, msg.param7 = float(param5), float(param6), float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        with self._lock:
            self._state.acks.pop(int(command), None)
        self._pub_cmd.publish(msg)

    def arm(self) -> None:
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self, force: bool = False) -> None:
        # 21196 is the MAVLink magic value that forces disarm in flight.
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0,
                          param2=21196.0 if force else 0.0)

    def set_offboard_mode(self) -> None:
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1), PX4 custom main mode OFFBOARD (6)
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self) -> None:
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def last_ack(self, command: int) -> Optional[int]:
        with self._lock:
            entry = self._state.acks.get(int(command))
        return entry[0] if entry else None

    # Command ids re-exported so callers never import px4_msgs.
    CMD_ARM_DISARM = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    CMD_SET_MODE = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    CMD_LAND = VehicleCommand.VEHICLE_CMD_NAV_LAND
    ACK_ACCEPTED = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
    NAV_OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    NAV_AUTO_LAND = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
