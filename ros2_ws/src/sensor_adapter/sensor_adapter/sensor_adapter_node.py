"""sensor_adapter_node — normalises camera topics into the common sensor contract (Phase 3).

Baseline behaviour: relay color / depth / camera_info, optionally rewriting frame_id and
checking timestamp consistency (color vs depth skew is published on /sensor_adapter/skew_ms).
The Gazebo-specific part is ros_gz_bridge (config/gz_bridge_*.yaml); the real-camera part will be
a libcamera/camera_ros driver. Both feed this node, so downstream never knows the source.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32


class SensorAdapterNode(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('rewrite_frame_id', True)
        self.declare_parameter('restamp', False)
        self.declare_parameter('registered_depth', True)   # no depth CameraInfo source: reuse the colour one
        for key, default in {'color_image': '/camera/color/image_raw_gz', 'depth_image': '/camera/depth/image_raw_gz',
                             'camera_info': '/camera/color/camera_info_gz', 'depth_info': '/camera/depth/camera_info_gz'}.items():
            self.declare_parameter(f'input.{key}', default)
        for key, default in {'color_image': '/camera/color/image_raw', 'depth_image': '/camera/depth/image_raw',
                             'camera_info': '/camera/color/camera_info', 'depth_info': '/camera/depth/camera_info'}.items():
            self.declare_parameter(f'output.{key}', default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.optical_frame = p('optical_frame')
        self.rewrite = bool(p('rewrite_frame_id'))
        self.restamp = bool(p('restamp'))
        self.registered = bool(p('registered_depth'))
        self._last_dinfo_t = 0.0

        self.pub_color = self.create_publisher(Image, p('output.color_image'), qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, p('output.depth_image'), qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, p('output.camera_info'), qos_profile_sensor_data)
        self.pub_dinfo = self.create_publisher(CameraInfo, p('output.depth_info'), qos_profile_sensor_data)
        self.pub_skew = self.create_publisher(Float32, '/sensor_adapter/skew_ms', 10)
        self.create_subscription(Image, p('input.color_image'), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, p('input.depth_image'), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('input.camera_info'), self._on_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('input.depth_info'), self._on_dinfo, qos_profile_sensor_data)
        self.stats = {'color': 0, 'depth': 0, 'info': 0, 'skew_abs_ms': []}
        self.create_timer(5.0, self._log_stats)
        self._last_color_t = None
        self.get_logger().info('sensor_adapter relaying to common /camera/* topics')

    def _fix(self, msg):
        if self.rewrite:
            msg.header.frame_id = self.optical_frame
        if self.restamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _on_color(self, msg: Image):
        self._last_color_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stats['color'] += 1
        self.pub_color.publish(self._fix(msg))

    def _on_depth(self, msg: Image):
        if self._last_color_t is not None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.pub_skew.publish(Float32(data=float((t - self._last_color_t) * 1e3)))
            self.stats['skew_abs_ms'].append(abs(t - self._last_color_t) * 1e3)
        self.stats['depth'] += 1
        self.pub_depth.publish(self._fix(msg))

    def _on_info(self, msg: CameraInfo):
        self.stats['info'] += 1
        fixed = self._fix(msg)
        self.pub_info.publish(fixed)
        if self.registered and (self.get_clock().now().nanoseconds * 1e-9 - self._last_dinfo_t) > 1.0:
            self.pub_dinfo.publish(fixed)      # registered RGB-D: identical intrinsics for the depth image

    def _on_dinfo(self, msg: CameraInfo):
        self._last_dinfo_t = self.get_clock().now().nanoseconds * 1e-9
        self.pub_dinfo.publish(self._fix(msg))

    def _log_stats(self):
        sk = self.stats['skew_abs_ms']
        if self.stats['color'] or self.stats['depth']:
            med = sorted(sk)[len(sk) // 2] if sk else float('nan')
            self.get_logger().info(f'5 s: color {self.stats["color"] / 5:.1f} Hz, depth {self.stats["depth"] / 5:.1f} Hz, '
                                   f'camera_info {self.stats["info"] / 5:.1f} Hz, |color-depth| skew median {med:.1f} ms '
                                   f'max {max(sk) if sk else float("nan"):.1f} ms')
        self.stats = {'color': 0, 'depth': 0, 'info': 0, 'skew_abs_ms': []}


def main(args=None):
    rclpy.init(args=args)
    node = SensorAdapterNode()
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
