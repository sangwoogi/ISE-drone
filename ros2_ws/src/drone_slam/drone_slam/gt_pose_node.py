"""gt_pose_node — Gazebo ground-truth pose of the vehicle for EVALUATION ONLY (never consumed by SLAM/mapper).

Reads `gz topic -e -t /world/<world>/dynamic_pose/info` (protobuf text stream) in a subprocess and republishes
the pose of `model_name` as /ground_truth/pose (PoseStamped, frame map_gt, stamp = Gazebo sim time).
This avoids depending on a ros_gz_bridge Pose_V conversion. Optional CSV log (t,x,y,z,qx,qy,qz,qw).
"""
from __future__ import annotations

import csv
import os
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

NUM = re.compile(r'(-?[\d.]+(?:e-?\d+)?)')


class GroundTruthPoseNode(Node):
    def __init__(self):
        super().__init__('gt_pose_node')
        self.declare_parameter('model_name', 'x500_rgbd_0')
        self.declare_parameter('world', 'indoor_v1')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('max_rate_hz', 50.0)
        self.model = self.get_parameter('model_name').value
        self.world = self.get_parameter('world').value
        self.min_period = 1.0 / float(self.get_parameter('max_rate_hz').value)
        self.pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self._csv = None
        csv_path = self.get_parameter('csv_path').value
        if csv_path:
            os.makedirs(os.path.dirname(csv_path) or '.', exist_ok=True)
            self._csv = csv.writer(open(csv_path, 'w', newline=''))
            self._csv.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.count = 0
        self._last_pub = 0.0
        self._proc = None
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.create_timer(10.0, lambda: self.get_logger().info(f'{self.count} ground-truth poses published'))
        self.get_logger().info(f'ground truth for {self.model} from gz /world/{self.world}/dynamic_pose/info -> /ground_truth/pose')

    def _reader(self):
        env = dict(os.environ)
        env.setdefault('GZ_IP', '127.0.0.1')
        topic = f'/world/{self.world}/dynamic_pose/info'
        while rclpy.ok():
            self._proc = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                          text=True, env=env, bufsize=1)
            stamp, in_pose, name, pos, ori, depth = None, False, None, {}, {}, 0
            sec, nsec = 0, 0
            for line in self._proc.stdout:
                ls = line.strip()
                if ls.startswith('header {'):
                    sec, nsec = 0, 0
                elif ls.startswith('sec:'):
                    sec = int(ls.split(':')[1])
                elif ls.startswith('nsec:'):
                    nsec = int(ls.split(':')[1])
                elif ls.startswith('pose {'):
                    in_pose, name, pos, ori, depth = True, None, {}, {}, 1
                elif in_pose:
                    if ls.startswith('name:'):
                        name = ls.split('"')[1]
                    elif ls.startswith('position {'):
                        depth = 2; cur = pos
                    elif ls.startswith('orientation {'):
                        depth = 2; cur = ori
                    elif depth == 2 and ls[:2] in ('x:', 'y:', 'z:', 'w:'):
                        cur[ls[0]] = float(NUM.search(ls).group(1))
                    elif ls == '}':
                        if depth == 2:
                            depth = 1
                        else:
                            in_pose = False
                            if name == self.model and len(pos) == 3 and len(ori) == 4:
                                self._publish(sec, nsec, pos, ori)
            time.sleep(1.0)   # gz topic exited (sim gone?) -> retry

    def _publish(self, sec, nsec, pos, ori):
        now = time.monotonic()
        if now - self._last_pub < self.min_period:
            return
        self._last_pub = now
        p = PoseStamped()
        p.header.stamp.sec, p.header.stamp.nanosec = int(sec), int(nsec)
        p.header.frame_id = 'map_gt'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = pos['x'], pos['y'], pos['z']
        p.pose.orientation.x, p.pose.orientation.y = ori.get('x', 0.0), ori.get('y', 0.0)
        p.pose.orientation.z, p.pose.orientation.w = ori.get('z', 0.0), ori.get('w', 1.0)
        self.pub.publish(p)
        self.count += 1
        if self._csv:
            self._csv.writerow([sec + nsec * 1e-9, pos['x'], pos['y'], pos['z'], p.pose.orientation.x, p.pose.orientation.y,
                                p.pose.orientation.z, p.pose.orientation.w])

    def stop(self):
        if self._proc:
            self._proc.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        try:
            node.destroy_node()
        except BaseException:
            pass
        rclpy.try_shutdown()
