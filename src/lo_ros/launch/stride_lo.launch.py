"""STRIDE LiDAR odometry: lifecycle node (respawned on crash) + lifecycle supervisor.

  ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c use_sim_time:=true
  args: robot (anymal_c | stride_rig | default), config (extra params file), use_sim_time,
        requests (true: emit.mode request on /stride_lo/node_request), supervisor, log_level,
        rviz (default true: RViz with rviz/stride_lo.rviz + lo_viz; rviz:=false on a headless robot),
        gt_topic (nav_msgs/Odometry ground truth for lo_viz, e.g. /sim/raw/odom; '' = none),
        viz_base_tf (lo_viz publishes stride_odom -> base; false if another node owns base's TF),
        deskewed (publish /stride_lo/deskewed for RViz; default: same as rviz)
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('lo_ros')
    robot = LaunchConfiguration('robot').perform(context)
    extra = LaunchConfiguration('config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    requests = LaunchConfiguration('requests').perform(context).lower() == 'true'
    supervisor = LaunchConfiguration('supervisor').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    gt_topic = LaunchConfiguration('gt_topic').perform(context)
    viz_base_tf = LaunchConfiguration('viz_base_tf').perform(context).lower() == 'true'
    deskewed = LaunchConfiguration('deskewed').perform(context).lower()
    deskewed = rviz if deskewed in ('', 'auto') else deskewed == 'true'
    respawn = LaunchConfiguration('respawn').perform(context).lower() == 'true'

    params = [os.path.join(share, 'config', 'default.yaml')]
    robot_file = os.path.join(share, 'config', f'{robot}.yaml')
    if robot != 'default':
        if not os.path.exists(robot_file):
            raise RuntimeError(f'no config for robot "{robot}": {robot_file}')
        params.append(robot_file)
    if extra:
        params.append(extra)
    overrides = {'use_sim_time': use_sim_time, 'debug.publish_deskewed': deskewed}
    if requests:
        overrides.update({'emit.mode': 'request', 'topics.node_request': '/stride_lo/node_request'})
    params.append(overrides)

    nodes = [Node(package='lo_ros', executable='lo_node', name='stride_lo', output='screen',
                  parameters=params, respawn=respawn, respawn_delay=2.0,
                  arguments=['--ros-args', '--log-level', log_level])]
    if supervisor:
        nodes.append(Node(package='lo_ros', executable='lo_supervisor', name='lo_supervisor',
                          output='screen',
                          parameters=[{'node_name': 'stride_lo', 'use_sim_time': use_sim_time}]))
    if rviz:
        try:   # lo_viz lives in lo_sim (no hard dependency: lo_sim depends on lo_ros)
            get_package_share_directory('lo_sim')
            nodes.append(Node(package='lo_sim', executable='lo_viz.py', name='lo_viz', output='log',
                              parameters=[{'use_sim_time': use_sim_time, 'gt_topic': gt_topic,
                                           'publish_base_tf': viz_base_tf}]))
        except PackageNotFoundError:
            pass
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'stride_lo.rviz')],
                          parameters=[{'use_sim_time': use_sim_time}],
                          # software GL: 2 render threads, below STRIDE's priority (I37)
                          prefix='nice -n 10', additional_env={'LP_NUM_THREADS': '2'}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='anymal_c'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('requests', default_value='false'),
        DeclareLaunchArgument('supervisor', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gt_topic', default_value=''),
        DeclareLaunchArgument('viz_base_tf', default_value='true'),
        DeclareLaunchArgument('deskewed', default_value='auto'),
        # crash respawn (robot). Demos / sims pass false: ros2 launch SIGINTs its nodes on
        # shutdown before it marks itself shutting down, and would respawn lo_node then.
        DeclareLaunchArgument('respawn', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
