"""RTAB-Map RGB-D SLAM on the common sensor topics (Phase 4).

V1: PX4 odometry (odom->base_link from px4_bridge) + RTAB-Map mapping/loop closure (map->odom).
Set use_rgbd_odometry:=true to let rtabmap_odom estimate odom instead (then px4_bridge publish_tf:=false).
Requires: sudo apt install ros-jazzy-rtabmap-ros
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_slam')
    params = os.path.join(share, 'config', 'rtabmap.yaml')
    remaps = [('rgb/image', '/camera/color/image_raw'),
              ('depth/image', '/camera/depth/image_raw'),
              ('rgb/camera_info', '/camera/color/camera_info'),
              ('odom', '/slam/odom')]
    use_rgbd_odom = LaunchConfiguration('use_rgbd_odometry')
    return LaunchDescription([
        DeclareLaunchArgument('use_rgbd_odometry', default_value='false'),
        DeclareLaunchArgument('delete_db', default_value='true'),
        Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
             parameters=[params], remappings=remaps, condition=IfCondition(use_rgbd_odom)),
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[params], remappings=remaps + [('grid_map', '/slam/map'), ('cloud_map', '/slam/pointcloud')],
             arguments=['-d']),  # TODO Phase 4: make -d conditional on delete_db
    ])
