"""STRIDE sensor rig, live (lo_sim).

  ros2 launch lo_sim sim.launch.py scenario:=crowd lo:=true record:=true gazebo:=true gui:=false

  scenario   lo_synth scenario (room warehouse corridor open_field stairs crowd turn180 ...)
  gazebo     true: Gazebo Harmonic runs the world + KinematicRig and provides /clock;
             false: lo_sim_rig publishes /clock itself (headless, no Gazebo)
  gui        Gazebo GUI (needs a display / GPU); the server alone is enough otherwise
  lo         start the STRIDE node (stride_rig.yaml, request mode) and lo_graph_demo
  record     ros2 bag record the inputs, STRIDE outputs and ground truth to sim_<scenario>_<time>
  rate       playback rate when gazebo:=false
"""
import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    gazebo = LaunchConfiguration('gazebo').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    lo = LaunchConfiguration('lo').perform(context).lower() == 'true'
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    rate = LaunchConfiguration('rate').perform(context)
    delay = '3.0'
    actions = []
    prefix = get_package_prefix('lo_sim')
    if gazebo:
        world = f'/tmp/stride_{scenario}_{seed}.sdf'
        with open(world, 'w') as f:
            f.write(subprocess.check_output(
                [os.path.join(prefix, 'lib', 'lo_sim', 'lo_sim_world'), scenario, '--seed', seed,
                 '--start-delay', delay], text=True))
        actions.append(SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.path.join(prefix, 'lib') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')))
        gz_args = ['gz', 'sim', '-r', world] + ([] if gui else ['-s', '--headless-rendering'])
        actions.append(ExecuteProcess(cmd=gz_args, output='screen'))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='clock_bridge', output='screen',
                            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']))
    actions.append(Node(package='lo_sim', executable='lo_sim_rig', name='lo_sim_rig', output='screen',
                        parameters=[{'scenario': scenario, 'seed': int(seed),
                                     'clock': 'external' if gazebo else 'internal',
                                     'use_sim_time': gazebo, 'rate': float(rate),
                                     'start_delay': float(delay)}]))
    if lo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lo_ros'), 'launch', 'stride_lo.launch.py')),
            launch_arguments={'robot': 'stride_rig', 'use_sim_time': 'true',
                              'requests': 'true'}.items()))
        actions.append(Node(package='lo_gtsam_adapter', executable='lo_graph_demo',
                            name='lo_graph_demo', output='screen',
                            parameters=[{'use_sim_time': True, 'imu_topic': '/livox/imu'}]))
    if record:
        out = f'sim_{scenario}_{time.strftime("%Y%m%d_%H%M%S")}'
        actions.append(ExecuteProcess(cmd=[
            'ros2', 'bag', 'record', '-s', 'mcap', '-o', out, '/livox/lidar', '/livox/imu',
            '/joint_states', '/tf_static', '/robot_description', '/ground_truth/odom',
            '/stride_lo/factor', '/stride_lo/odometry', '/stride_lo/node_request', '/diagnostics',
            '/lo_graph_demo/pose'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='room'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('lo', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=_setup),
    ])
