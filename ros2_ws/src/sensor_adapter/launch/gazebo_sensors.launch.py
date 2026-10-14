"""Gazebo -> common sensor topics: ros_gz_bridge + sensor_adapter + static camera TF (Phase 3)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('sensor_adapter')
    bridge_cfg = LaunchConfiguration('bridge_config')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('bridge_config', default_value=os.path.join(share, 'config', 'gz_bridge_x500_depth.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Gazebo topics -> *_gz ROS topics (remapped so the adapter owns the final names)
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_sensor_bridge', output='screen',
             parameters=[{'config_file': bridge_cfg}, sim],
             remappings=[('/camera/color/image_raw', '/camera/color/image_raw_gz'),
                         ('/camera/depth/image_raw', '/camera/depth/image_raw_gz'),
                         ('/camera/color/camera_info', '/camera/color/camera_info_gz')]),
        Node(package='sensor_adapter', executable='sensor_adapter_node', output='screen',
             parameters=[os.path.join(share, 'config', 'sensor_adapter.yaml'), sim]),
        # base_link -> camera_link (x500_depth: camera 0.12 m forward, 0.03 m down; verify against model.sdf in Phase 3)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_camera', parameters=[sim],
             arguments=['--x', '0.12', '--y', '0.0', '--z', '-0.03', '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        # camera_link -> camera_optical_frame (REP-103 -> REP-104 optical: z forward, x right, y down)
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_to_optical', parameters=[sim],
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '-1.5707963', '--pitch', '0', '--roll', '-1.5707963',
                        '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']),
    ])
