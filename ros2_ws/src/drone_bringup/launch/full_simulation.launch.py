"""Full stack (plan §33): Gazebo + PX4 + agent + sensor adapter + SLAM + detector + semantic mapper + DB
+ mission manager + LLM + system monitor + RViz. Every stage can be toggled: enable_<stage>:=false.

    ros2 launch drone_bringup full_simulation.launch.py                     # everything (SLAM needs rtabmap_ros)
    ros2 launch drone_bringup full_simulation.launch.py enable_slam:=false detector_backend:=mock
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def generate_launch_description():
    lc = LaunchConfiguration
    stages = ['sensors', 'slam', 'detector', 'tracker', 'mapper', 'database', 'mission', 'llm', 'monitor', 'exploration']
    args = [DeclareLaunchArgument(f'enable_{s}', default_value='true') for s in stages]
    args += [
        DeclareLaunchArgument('vehicle', default_value='gz_x500_rgbd'),
        DeclareLaunchArgument('world', default_value='indoor_v1'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('detector_backend', default_value='ultralytics'),
        DeclareLaunchArgument('llm_backend', default_value='ollama'),
        DeclareLaunchArgument('db_path', default_value='semantic_map.sqlite'),
        # Gazebo publishes /clock (ros_gz_bridge); all stamps (camera, TF, detections) must share it.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Phase 4 result: pure RGB-D visual odometry loses tracking 50-110x per lap in indoor_v1 (ATE 8 m);
        # PX4 EKF odometry + RTAB-Map loop closure gives ATE 0.075 m. Keep 'vo' / 'vo_guess' for experiments.
        DeclareLaunchArgument('odom_source', default_value='px4', description='SLAM odometry: px4 (EKF TF, default) | vo (rgbd_odometry)'),
    ]
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('drone_bringup', 'launch', 'simulation.launch.py')),
        launch_arguments={'vehicle': lc('vehicle'), 'headless': lc('headless'), 'rviz': lc('rviz'), 'world': lc('world'),
                          'static_map_odom': 'false', 'use_sim_time': lc('use_sim_time'),
                          # visual odometry owns odom->base_link; PX4 EKF mode keeps px4_bridge TF
                          'px4_tf': PythonExpression(["'false' if '", lc('odom_source'), "' == 'vo' and '", lc('enable_slam'), "'.lower() in ('true','1','yes') else 'true'"])}.items())
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('sensor_adapter', 'launch', 'gazebo_sensors.launch.py')),
        launch_arguments={'use_sim_time': lc('use_sim_time')}.items(),
        condition=IfCondition(lc('enable_sensors')))
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('drone_slam', 'launch', 'rtabmap.launch.py')),
        launch_arguments={'use_sim_time': lc('use_sim_time'), 'odom_source': lc('odom_source'), 'world': lc('world')}.items(),
        condition=IfCondition(lc('enable_slam')))
    # Without SLAM, keep map == odom so the semantic mapper still works (Phase 5/6 tests)
    map_odom = Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_static',
                    arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                    condition=IfCondition(PythonExpressionNot(lc('enable_slam'))))
    sim = {'use_sim_time': lc('use_sim_time')}
    nodes = [
        Node(package='object_detector', executable='detector_node', output='screen',
             parameters=[share('object_detector', 'config', 'detector.yaml'), {'backend': lc('detector_backend')}, sim],
             condition=IfCondition(lc('enable_detector'))),
        Node(package='object_tracker', executable='tracker_node', output='screen', parameters=[sim], condition=IfCondition(lc('enable_tracker'))),
        Node(package='semantic_mapper', executable='semantic_mapper_node', output='screen',
             parameters=[share('semantic_mapper', 'config', 'semantic_mapper.yaml'), sim], condition=IfCondition(lc('enable_mapper'))),
        Node(package='semantic_database', executable='semantic_database_node', output='screen',
             parameters=[{'db_path': lc('db_path')}, sim], condition=IfCondition(lc('enable_database'))),
        Node(package='mission_manager', executable='mission_manager_node', output='screen',
             parameters=[share('mission_manager', 'config', 'mission_manager.yaml'), sim], condition=IfCondition(lc('enable_mission'))),
        Node(package='llm_interface', executable='llm_node', output='screen',
             parameters=[share('llm_interface', 'config', 'llm_interface.yaml'), {'backend': lc('llm_backend')}, sim],
             condition=IfCondition(lc('enable_llm'))),
        Node(package='system_monitor', executable='system_monitor_node', output='screen',
             parameters=[share('system_monitor', 'config', 'system_monitor.yaml'), sim], condition=IfCondition(lc('enable_monitor'))),
        Node(package='exploration', executable='exploration_node', output='screen', parameters=[sim], condition=IfCondition(lc('enable_exploration'))),
    ]
    return LaunchDescription(args + [base, sensors, map_odom, TimerAction(period=8.0, actions=[slam] + nodes)])


class PythonExpressionNot:
    """Tiny helper: IfCondition(PythonExpressionNot(x)) == UnlessCondition(x) with a substitution."""

    def __new__(cls, sub):
        from launch.substitutions import PythonExpression
        return PythonExpression(["'false' if '", sub, "'.lower() in ('true', '1', 'yes') else 'true'"])
