"""RTAB-Map RGB-D SLAM on the common sensor topics (Phase 4).

    ros2 launch drone_slam rtabmap.launch.py odom_source:=vo|px4 [db_path:=...] [record_dir:=...] [gt:=true]

odom_source=vo  : rtabmap_odom/rgbd_odometry estimates odom->base_link from RGB-D (plan §14 V1). Start the
                  simulation with px4_tf:=false so px4_bridge does not also publish that transform.
odom_source=px4 : PX4 EKF owns odom->base_link (px4_bridge TF); rtabmap only maps + closes loops (map->odom).
gt=true         : gt_pose_node streams Gazebo /world/<world>/dynamic_pose/info (gz topic -e) and publishes
                  /ground_truth/pose (evaluation only).
record_dir      : if set, trajectory_recorder writes est/odom/px4/gt CSVs + stats.json there.
Requires: sudo apt install ros-jazzy-rtabmap-ros
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    odom_source = LC('odom_source').perform(context)
    db_path = LC('db_path').perform(context)
    record_dir = LC('record_dir').perform(context)
    gt = LC('gt').perform(context).lower() in ('1', 'true', 'yes')
    world = LC('world').perform(context)
    model = LC('gt_model').perform(context)
    use_sim_time = LC('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')
    share = get_package_share_directory('drone_slam')
    params_file = os.path.join(share, 'config', 'rtabmap.yaml')
    sim = {'use_sim_time': use_sim_time}

    remaps = [('rgb/image', '/camera/color/image_raw'),
              ('depth/image', '/camera/depth/image_raw'),
              ('rgb/camera_info', '/camera/color/camera_info')]
    nodes = []
    vo_guess = LC('vo_guess').perform(context).lower() in ('1', 'true', 'yes')
    if odom_source == 'vo':
        # vo_guess: PX4 EKF (TF odom_px4->base_link from px4_bridge) is the motion prior; rgbd_odometry then
        # publishes the visual correction odom->odom_px4 (RTAB-Map guess_frame_id pattern, like wheel odometry).
        guess = {'guess_frame_id': 'odom_px4'} if vo_guess else {}
        nodes.append(Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
                          parameters=[params_file, sim, {'frame_id': 'base_link', 'odom_frame_id': 'odom',
                                                         'publish_tf': True, 'wait_for_transform': 0.3,
                                                         'approx_sync': True, 'queue_size': 10,
                                                         'Vis/MinInliers': '12'}, guess],   # odometry stays permissive
                          remappings=remaps + [('odom', '/slam/odom'), ('odom_info', '/slam/odom_info')]))
        odom_params = {'subscribe_odom_info': True, 'odom_frame_id': ''}
        odom_remaps = [('odom', '/slam/odom'), ('odom_info', '/slam/odom_info')]
    else:
        # PX4 EKF: take odom from TF (odom_frame_id) - no odom topic subscription. The EKF is far more accurate than
        # visual loop closures in this synthetic world, so tell the graph optimiser to trust it (tiny TF variances),
        # do not refine neighbour links visually, and reject any loop closure that would move the graph > 0.3 m.
        odom_params = {'odom_frame_id': 'odom', 'subscribe_odom_info': False,
                       'odom_tf_linear_variance': 0.0001, 'odom_tf_angular_variance': 0.0001,
                       'RGBD/NeighborLinkRefining': 'false', 'RGBD/OptimizeMaxError': '0.3'}
        odom_remaps = [('odom', '/slam/odom_unused')]
    nodes.append(Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
                      parameters=[params_file, sim, odom_params, {'database_path': db_path, 'frame_id': 'base_link',
                                                                   'map_frame_id': 'map', 'publish_tf': True,
                                                                   'approx_sync': True, 'queue_size': 10}],
                      remappings=remaps + odom_remaps + [('map', '/slam/map'), ('grid_prob_map', '/slam/map_prob'), ('cloud_map', '/slam/pointcloud'),
                                                         ('info', '/slam/info'), ('mapData', '/slam/map_data')],
                      arguments=['--delete_db_on_start']))
    if gt:
        # reads `gz topic -e` directly (ros_gz_bridge Pose_V->TFMessage delivered nothing in Jazzy/Harmonic)
        nodes.append(Node(package='drone_slam', executable='gt_pose_node', output='screen',
                          parameters=[sim, {'model_name': model, 'world': world}]))
    if record_dir:
        nodes.append(Node(package='drone_slam', executable='trajectory_recorder', output='screen',
                          parameters=[sim, {'out_dir': record_dir}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='px4', description='px4 (EKF TF, default after Phase 4 evaluation) | vo (rgbd_odometry)'),
        DeclareLaunchArgument('vo_guess', default_value='false', description='vo: use PX4 EKF TF odom_px4->base_link as motion guess'),
        DeclareLaunchArgument('db_path', default_value=os.path.expanduser('~/.ros/rtabmap.db')),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument('gt', default_value='true'),
        DeclareLaunchArgument('world', default_value='indoor_v1'),
        DeclareLaunchArgument('gt_model', default_value='x500_rgbd_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
