"""gt_pose_node — Gazebo ground-truth pose for EVALUATION ONLY (never consumed by SLAM/mapper).

Baseline: subscribes a ros_gz-bridged `tf_msgs/TFMessage` or `geometry_msgs/PoseArray` is model-specific;
the simplest robust source in PX4's gz worlds is the `/world/<world>/pose/info` topic bridged as
`tf2_msgs/TFMessage`. Phase 4 wires the bridge; this node extracts the vehicle model pose and republishes
`/ground_truth/pose` (PoseStamped, frame map_gt) and logs a CSV for ATE/RPE.
"""
from __future__ import annotations

import csv
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GroundTruthPoseNode(Node):
    def __init__(self):
        super().__init__('gt_pose_node')
        self.declare_parameter('model_name', 'x500_depth_0')
        self.declare_parameter('pose_info_topic', '/gz/pose_info')
        self.declare_parameter('csv_path', '')
        self.model = self.get_parameter('model_name').value
        self.pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.create_subscription(TFMessage, self.get_parameter('pose_info_topic').value, self._on_pose_info, 10)
        csv_path = self.get_parameter('csv_path').value
        self._csv = None
        if csv_path:
            os.makedirs(os.path.dirname(csv_path) or '.', exist_ok=True)
            self._csv = csv.writer(open(csv_path, 'w', newline=''))
            self._csv.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.get_logger().info(f'ground truth for model {self.model} -> /ground_truth/pose (evaluation only)')

    def _on_pose_info(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == self.model:
                p = PoseStamped()
                p.header.stamp = t.header.stamp
                p.header.frame_id = 'map_gt'
                p.pose.position.x = t.transform.translation.x
                p.pose.position.y = t.transform.translation.y
                p.pose.position.z = t.transform.translation.z
                p.pose.orientation = t.transform.rotation
                self.pub.publish(p)
                if self._csv:
                    q = t.transform.rotation
                    self._csv.writerow([time.time(), p.pose.position.x, p.pose.position.y, p.pose.position.z,
                                        q.x, q.y, q.z, q.w])
                return


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()
