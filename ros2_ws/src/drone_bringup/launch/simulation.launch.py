"""Phase 2 bringup: PX4 SITL + Gazebo + XRCE agent + px4_bridge node + RViz.

    ros2 launch drone_bringup simulation.launch.py [vehicle:=gz_x500_depth] [headless:=true] [rviz:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('drone_bringup')
    px4_bridge_cfg = os.path.join(get_package_share_directory('px4_bridge'), 'config', 'px4_bridge.yaml')
    rviz_cfg = os.path.join(bringup, 'config', 'drone.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='gz_x500_depth'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sitl.launch.py')),
            launch_arguments={'vehicle': LaunchConfiguration('vehicle'), 'headless': LaunchConfiguration('headless')}.items()),
        # give PX4 + agent a few seconds before the bridge starts streaming
        TimerAction(period=5.0, actions=[
            Node(package='px4_bridge', executable='px4_bridge_node', name='px4_bridge', output='screen',
                 parameters=[px4_bridge_cfg, {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        ]),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg], output='log',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        # map -> odom identity until SLAM (Phase 4) owns it
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_static',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             condition=IfCondition(LaunchConfiguration('static_map_odom', default='true'))),
    ])
