"""mission_manager_node — Mission action server: validates, sequences and executes mission steps (Phase 8).

    LLM / CLI --(drone_interfaces/action/Mission)--> mission_manager --(FlightCommand action)--> px4_bridge
                                                     |-- QueryObjects srv (semantic_database) for GOTO_OBJECT

Baseline implements TAKEOFF / LAND / HOVER / GOTO / RETURN_HOME / GOTO_OBJECT (V1+V2) through the state machine
and safety validator. EXPLORE / SEARCH_OBJECT (V3) call the exploration package (stub) and are rejected until Phase 8.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_interfaces.action import FlightCommand, Mission
from drone_interfaces.msg import MissionStatus, VehicleState
from drone_interfaces.srv import QueryObjects

from .safety_validator import SafetyError, SafetyLimits, SafetyValidator
from .state_machine import MissionStateMachine, State


class ObjectNotFound(Exception):
    """Semantic map does not contain the requested object (not a built-in lookup error: KeyError is one)."""


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        lim = SafetyLimits()
        for k, v in lim.__dict__.items():
            self.declare_parameter(k, v)
            setattr(lim, k, float(self.get_parameter(k).value))
        self.declare_parameter('default_takeoff_height_m', 1.2)
        self.declare_parameter('status_rate_hz', 5.0)
        self.validator = SafetyValidator(lim)
        self.sm = MissionStateMachine()
        self.vehicle: VehicleState | None = None
        self.home = None
        self._lock = threading.Lock()
        cb = ReentrantCallbackGroup()

        self.create_subscription(VehicleState, '/drone/vehicle_state', self._on_vehicle, 10, callback_group=cb)
        self.pub_status = self.create_publisher(MissionStatus, '/mission/status', 10)
        self.create_timer(1.0 / float(self.get_parameter('status_rate_hz').value), self._publish_status, callback_group=cb)
        self.flight = ActionClient(self, FlightCommand, '/px4_bridge/flight_command', callback_group=cb)
        self.db = self.create_client(QueryObjects, '/semantic_database/query', callback_group=cb)
        self.server = ActionServer(self, Mission, '/mission/execute', execute_callback=self._execute,
                                   goal_callback=self._on_goal, cancel_callback=lambda gh: CancelResponse.ACCEPT,
                                   callback_group=cb)
        self._status_msg = ''
        self.get_logger().info('mission_manager ready: action /mission/execute')

    # ------------------------------------------------------------------ io
    def _on_vehicle(self, msg: VehicleState):
        self.vehicle = msg
        if self.home is None and msg.position_valid:
            self.home = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.sm.state in (State.LANDING, State.ERROR) and msg.landed and not msg.armed:
            self.sm.landed()

    def _publish_status(self):
        m = MissionStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.current_action, m.message = self.sm.state.value, self.sm.current_action, self._status_msg
        self.pub_status.publish(m)

    def _on_goal(self, req):
        if not self.sm.can_start(req.action):
            self.get_logger().warn(f'reject {req.action}: not allowed in {self.sm.state.value}')
            return GoalResponse.REJECT
        if req.action in ('EXPLORE', 'SEARCH_OBJECT', 'LOOK_AT_OBJECT'):
            self.get_logger().warn(f'reject {req.action}: implemented in Phase 8 (exploration)')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------- execute
    def _execute(self, gh):
        req = gh.request
        result = Mission.Result()
        with self._lock:
            self.sm.start(req.action)
            self._status_msg = f'{req.action} running'
            try:
                fc = self._to_flight_command(req)
                ok, msg = self._run_flight(fc, gh)
                self.sm.finish(ok, 'FLIGHT_ERROR')
            except SafetyError as e:          # rejected before any flight command: vehicle state unchanged
                ok, msg = False, f'rejected by safety validator: {e}'
                self.sm.cancel()
            except ObjectNotFound as e:
                ok, msg = False, str(e)
                self.sm.cancel()
            except Exception as e:  # noqa: BLE001 - never leave the state machine inconsistent
                self.get_logger().error(f'{req.action} failed with {type(e).__name__}: {e}')
                ok, msg = False, f'internal error: {type(e).__name__}: {e}'
                self.sm.finish(False, 'FLIGHT_ERROR')
            self._status_msg = msg
        result.success, result.message, result.final_state = ok, msg, self.sm.state.value
        (gh.succeed if ok else gh.abort)()
        return result

    def _to_flight_command(self, req) -> FlightCommand.Goal:
        g = FlightCommand.Goal()
        home = self.home or (0.0, 0.0, 0.0)
        if req.action == 'TAKEOFF':
            g.command, g.height = 'TAKEOFF', float(req.param or self.get_parameter('default_takeoff_height_m').value)
            self.validator.check_altitude(g.height)
        elif req.action == 'LAND':
            g.command = 'LAND'
        elif req.action == 'HOVER':
            g.command, g.duration = 'HOVER', float(req.param or 2.0)
        elif req.action == 'RETURN_HOME':
            g.command = 'RETURN_HOME'
        elif req.action == 'GOTO':
            tgt = (req.target.x, req.target.y, req.target.z)
            self.validator.check_goto(tgt, home)
            g.command = 'GOTO'
            g.target.x, g.target.y, g.target.z = tgt
        elif req.action == 'GOTO_OBJECT':
            obj = self._lookup_object(req.target_id)
            drone = self.vehicle.pose.position if self.vehicle else None
            dpos = (drone.x, drone.y, drone.z) if drone else home
            goal = self.validator.object_goal((obj.position.x, obj.position.y, obj.position.z), dpos, home, obj.confidence)
            g.command = 'GOTO'
            g.target.x, g.target.y, g.target.z = goal
        else:
            raise SafetyError(f'unsupported action {req.action}')
        return g

    def _lookup_object(self, target_id: str):
        if not self.db.wait_for_service(timeout_sec=2.0):
            raise ObjectNotFound('semantic database service unavailable')
        req = QueryObjects.Request()
        if '_' in target_id and target_id.rsplit('_', 1)[-1].isdigit():
            req.class_name = ''
        else:
            req.class_name = target_id
        fut = self.db.call_async(req)
        t0 = time.time()   # the MultiThreadedExecutor services the future while we wait
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done() or not fut.result().success:
            raise ObjectNotFound('semantic database query failed')
        objs = fut.result().objects
        if req.class_name == '':
            objs = [o for o in objs if o.id == target_id]
        if not objs:
            raise ObjectNotFound(f'object "{target_id}" not in semantic map')
        return max(objs, key=lambda o: o.confidence)

    def _run_flight(self, goal: FlightCommand.Goal, gh):
        if not self.flight.wait_for_server(timeout_sec=3.0):
            return False, 'px4_bridge flight_command action unavailable'
        send = self.flight.send_goal_async(goal, feedback_callback=lambda fb: self._forward_feedback(gh, fb))
        while not send.done():
            time.sleep(0.02)
        handle = send.result()
        if not handle.accepted:
            return False, f'{goal.command} rejected by px4_bridge'
        res_fut = handle.get_result_async()
        while not res_fut.done():
            if gh.is_cancel_requested:
                handle.cancel_goal_async()
            time.sleep(0.05)
        res = res_fut.result().result
        return bool(res.success), res.message

    def _forward_feedback(self, gh, fb_msg):
        fb = Mission.Feedback()
        fb.state = self.sm.state.value
        fb.message = f'{fb_msg.feedback.state} {fb_msg.feedback.distance_remaining:.2f} m'
        gh.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()
