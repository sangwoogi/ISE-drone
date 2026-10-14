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
        for key, default in {'color_image': '/camera/color/image_raw_gz', 'depth_image': '/camera/depth/image_raw_gz',
                             'camera_info': '/camera/color/camera_info_gz'}.items():
            self.declare_parameter(f'input.{key}', default)
        for key, default in {'color_image': '/camera/color/image_raw', 'depth_image': '/camera/depth/image_raw',
                             'camera_info': '/camera/color/camera_info'}.items():
            self.declare_parameter(f'output.{key}', default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.optical_frame = p('optical_frame')
        self.rewrite = bool(p('rewrite_frame_id'))
        self.restamp = bool(p('restamp'))

        self.pub_color = self.create_publisher(Image, p('output.color_image'), qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, p('output.depth_image'), qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, p('output.camera_info'), qos_profile_sensor_data)
        self.pub_skew = self.create_publisher(Float32, '/sensor_adapter/skew_ms', 10)
        self.create_subscription(Image, p('input.color_image'), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, p('input.depth_image'), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('input.camera_info'), self._on_info, qos_profile_sensor_data)
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
        self.pub_color.publish(self._fix(msg))

    def _on_depth(self, msg: Image):
        if self._last_color_t is not None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.pub_skew.publish(Float32(data=float((t - self._last_color_t) * 1e3)))
        self.pub_depth.publish(self._fix(msg))

    def _on_info(self, msg: CameraInfo):
        self.pub_info.publish(self._fix(msg))


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
