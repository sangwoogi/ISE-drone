"""px4_bridge_node — ROS 2 face of the flight layer (Phase 2).

Publishes (ENU, frame odom = PX4 local origin):
    /drone/vehicle_state   drone_interfaces/VehicleState   20 Hz
    /drone/pose            geometry_msgs/PoseStamped       20 Hz
    TF odom -> base_link

Action server:
    /px4_bridge/flight_command   drone_interfaces/action/FlightCommand
        TAKEOFF(height) | LAND | GOTO(target, yaw, use_yaw) | HOVER(duration) | RETURN_HOME

Only one goal runs at a time; a new goal while one is active is rejected. Cancel -> abort (land).
Upper layers (mission_manager) use this action and never touch px4_msgs.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from drone_interfaces.action import FlightCommand
from drone_interfaces.msg import VehicleState

from .flight_controller import FlightController, FlightError


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PX4BridgeNode(Node):
    def __init__(self):
        super().__init__('px4_bridge')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('state_rate_hz', 20.0)
        self.declare_parameter('publish_tf', True)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.fc = FlightController(node=self, spin=False)
        self.fc.progress_cb = self._on_progress

        self.pub_state = self.create_publisher(VehicleState, '/drone/vehicle_state', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / float(self.get_parameter('state_rate_hz').value), self._publish_state)

        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._feedback = None
        self._action = ActionServer(
            self, FlightCommand, '/px4_bridge/flight_command',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('px4_bridge_node ready: action /px4_bridge/flight_command, topics /drone/*')

    # ------------------------------------------------------------ telemetry
    def _publish_state(self):
        s = self.fc.state
        now = self.get_clock().now().to_msg()
        q = yaw_to_quat(s.yaw_enu)
        msg = VehicleState()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.armed, msg.in_offboard, msg.landed = s.armed, s.in_offboard, s.landed
        msg.position_valid = s.position_valid
        msg.nav_state = s.nav_state_name
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = s.position_enu
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        msg.velocity.x, msg.velocity.y, msg.velocity.z = s.velocity_enu
        self.pub_state.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.pub_pose.publish(pose)

        if self.get_parameter('publish_tf').value and s.position_valid:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = s.position_enu
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------------------- action
    def _on_goal(self, goal_request):
        if goal_request.command not in ('TAKEOFF', 'LAND', 'GOTO', 'HOVER', 'RETURN_HOME'):
            self.get_logger().warn(f'rejecting unknown command {goal_request.command}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warn('rejecting goal: another flight command is active')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().warn('cancel requested -> abort (land)')
        return CancelResponse.ACCEPT

    def _on_progress(self, name, dist, elapsed):
        gh = self._active_goal
        if gh is None:
            return
        fb = FlightCommand.Feedback()
        fb.state, fb.distance_remaining, fb.elapsed = name, float(dist), float(elapsed)
        gh.publish_feedback(fb)
        if gh.is_cancel_requested:
            raise FlightError('cancelled by client')

    def _execute(self, goal_handle):
        req = goal_handle.request
        with self._goal_lock:
            self._active_goal = goal_handle
        result = FlightCommand.Result()
        t0 = time.time()
        try:
            if req.command == 'TAKEOFF':
                step = self.fc.takeoff(float(req.height))
            elif req.command == 'LAND':
                step = self.fc.land()
            elif req.command == 'GOTO':
                step = self.fc.goto(req.target.x, req.target.y, req.target.z,
                                    float(req.yaw) if req.use_yaw else None)
            elif req.command == 'HOVER':
                step = self.fc.hover(float(req.duration))
            else:
                step = self.fc.return_home()
            result.success, result.message = True, f'{step.name} ok'
            result.final_error, result.elapsed = float(step.final_error_m), float(step.elapsed_s)
            goal_handle.succeed()
        except FlightError as e:
            result.success, result.message = False, str(e)
            result.elapsed = float(time.time() - t0)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            if self.fc.state.armed:
                self.fc.abort()
        finally:
            p = self.fc.position
            result.final_position.x, result.final_position.y, result.final_position.z = p
            with self._goal_lock:
                self._active_goal = None
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PX4BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=1.0)
            node.fc.shutdown()
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
