"""STRIDE LiDAR odometry: lifecycle node (respawned on crash) + lifecycle supervisor.

  ros2 launch lo_ros stride_lo.launch.py robot:=anymal_c use_sim_time:=true
  args: robot (anymal_c | stride_rig | default), config (extra params file), use_sim_time,
        requests (true: emit.mode request on /stride_lo/node_request), supervisor, log_level
"""
import os

from ament_index_python.packages import get_package_share_directory
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

    params = [os.path.join(share, 'config', 'default.yaml')]
    robot_file = os.path.join(share, 'config', f'{robot}.yaml')
    if robot != 'default':
        if not os.path.exists(robot_file):
            raise RuntimeError(f'no config for robot "{robot}": {robot_file}')
        params.append(robot_file)
    if extra:
        params.append(extra)
    overrides = {'use_sim_time': use_sim_time}
    if requests:
        overrides.update({'emit.mode': 'request', 'topics.node_request': '/stride_lo/node_request'})
    params.append(overrides)

    nodes = [Node(package='lo_ros', executable='lo_node', name='stride_lo', output='screen',
                  parameters=params, respawn=True, respawn_delay=2.0,
                  arguments=['--ros-args', '--log-level', log_level])]
    if supervisor:
        nodes.append(Node(package='lo_ros', executable='lo_supervisor', name='lo_supervisor',
                          output='screen',
                          parameters=[{'node_name': 'stride_lo', 'use_sim_time': use_sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='anymal_c'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('requests', default_value='false'),
        DeclareLaunchArgument('supervisor', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
