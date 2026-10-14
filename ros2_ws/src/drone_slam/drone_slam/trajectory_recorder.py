"""trajectory_recorder — write estimated / PX4 / ground-truth trajectories to CSV for ATE/RPE (Phase 4).

    ros2 run drone_slam trajectory_recorder --ros-args -p out_dir:=experiments/004_phase4_slam/run1

Columns t,x,y,z,qx,qy,qz,qw (t = sim time if use_sim_time). Files:
    est.csv  TF map -> base_link (SLAM estimate, sampled at `rate_hz`)
    odom.csv TF odom -> base_link (odometry only: VO or PX4 depending on who publishes it)
    px4.csv  /drone/odom pose (PX4 EKF, always)
    gt.csv   /ground_truth/pose (Gazebo, evaluation only)
Also counts odometry tracking losses (/slam/odom_info.lost) and loop closures (/slam/info.loop_closure_id)
and writes stats.json on shutdown.
"""
from __future__ import annotations

import csv
import json
import os

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('out_dir', 'experiments/004_phase4_slam/latest')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.out = self.get_parameter('out_dir').value
        os.makedirs(self.out, exist_ok=True)
        self.handles = {k: open(os.path.join(self.out, f'{k}.csv'), 'w', newline='') for k in ('est', 'odom', 'px4', 'gt')}
        self.files = {k: csv.writer(h) for k, h in self.handles.items()}
        for w in self.files.values():
            w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.counts = {k: 0 for k in self.files}
        self.stats = {'tracking_lost_events': 0, 'odom_lost_samples': 0, 'loop_closures': 0, 'tf_est_failures': 0}
        self._was_lost = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_node = rclpy.create_node('trajectory_recorder_tf', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=bool(self.get_parameter('use_sim_time').value))])
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.create_subscription(Odometry, '/drone/odom', lambda m: self._pose('px4', m.header, m.pose.pose), 10)
        self.create_subscription(PoseStamped, '/ground_truth/pose', lambda m: self._pose('gt', m.header, m.pose), 10)
        try:
            from rtabmap_msgs.msg import Info, OdomInfo
            self.create_subscription(OdomInfo, '/slam/odom_info', self._on_odom_info, 10)
            self.create_subscription(Info, '/slam/info', self._on_info, 10)
        except ImportError:
            self.get_logger().warn('rtabmap_msgs not available: no lost/loop-closure statistics')
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._sample_tf)
        self.create_timer(10.0, self._log)
        self.create_timer(5.0, self._flush)   # stats + CSV flushed periodically, not only at shutdown
        self.get_logger().info(f'recording to {self.out}')

    def _t(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _pose(self, key, header, pose):
        p, q = pose.position, pose.orientation
        self.files[key].writerow([round(self._t(header), 4), round(p.x, 4), round(p.y, 4), round(p.z, 4),
                                  round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5)])
        self.counts[key] += 1

    def _sample_tf(self):
        now = self.get_clock().now()
        for key, parent in (('est', self.get_parameter('map_frame').value), ('odom', self.get_parameter('odom_frame').value)):
            try:
                tf = self.tf_buffer.lookup_transform(parent, self.get_parameter('base_frame').value, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if key == 'est':
                    self.stats['tf_est_failures'] += 1
                continue
            tr, q = tf.transform.translation, tf.transform.rotation
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            self.files[key].writerow([round(t if t > 0 else now.nanoseconds * 1e-9, 4), round(tr.x, 4), round(tr.y, 4), round(tr.z, 4),
                                      round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5)])
            self.counts[key] += 1

    def _on_odom_info(self, m):
        if m.lost:
            self.stats['odom_lost_samples'] += 1
            if not self._was_lost:
                self.stats['tracking_lost_events'] += 1
                self.get_logger().warn('odometry tracking LOST')
        self._was_lost = bool(m.lost)

    def _on_info(self, m):
        if m.loop_closure_id > 0:
            self.stats['loop_closures'] += 1
            self.get_logger().info(f'loop closure #{self.stats["loop_closures"]} (node {m.loop_closure_id})')

    def _log(self):
        self.get_logger().info(f'samples {self.counts} stats {self.stats}')

    def _flush(self):
        for h in self.handles.values():
            h.flush()
        with open(os.path.join(self.out, 'stats.json'), 'w') as f:
            json.dump({**self.stats, 'samples': dict(self.counts)}, f, indent=2)

    def finish(self):
        self._flush()
        self.get_logger().info(f'stats written: {self.stats}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        try:
            node.destroy_node()
        except BaseException:
            pass
        rclpy.try_shutdown()
