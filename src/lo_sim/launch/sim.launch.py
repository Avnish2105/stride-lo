"""STRIDE sensor rig, live (lo_sim).

  ros2 launch lo_sim sim.launch.py scenario:=crowd record:=true
  headless: ... gazebo:=false rviz:=false
  easiest:  the VS Code "STRIDE: Launch" button (scripts/launcher) sets all of these from one
            window and shows Gazebo / RViz on the shared screen (scripts/screen, port 6080)

  scenario   lo_synth scenario (room warehouse corridor open_field stairs crowd turn180)
  seed       scenario seed
  gazebo     true: Gazebo Harmonic shows the world; its KinematicRig follows lo_sim_rig's clock
             (display only, DECISIONS I37); false: no Gazebo
  gui        Gazebo window (default true); gui:=false runs the Gazebo server only
  rviz       RViz (rviz/sim.rviz) + lo_viz + the rig's robot model (default true)
  lo         start the STRIDE node (stride_rig.yaml, request mode) and lo_graph_demo
  record     ros2 bag record the inputs, STRIDE outputs and ground truth to sim_<scenario>_<time>
  rate       scenario playback rate (1.0 = real time)

Time: lo_sim_rig always publishes /clock (scenario time x rate, from a monotonic wall timer);
every node runs on it. Gazebo never drives time, so a slow or stalled Gazebo GUI cannot disturb
the sensor streams or STRIDE's clock checks. Windows open on $DISPLAY (the launcher sets the
shared screen :99). Each launch uses its own GZ_PARTITION, so a second Gazebo on the machine
cannot interfere.
"""
import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Ask the Gazebo GUI camera to follow the rig from behind/above (CameraTracking plugin in
# config/gui.config), low enough to stay under a room's ceiling (crowd: 3.5 m). Retried: the GUI
# may take a few seconds to come up.
_FOLLOW = r'''
for i in $(seq 1 30); do
  if gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
       --timeout 1000 --req 'data: "stride_rig"' 2>/dev/null | grep -q true; then
    gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
      --timeout 1000 --req 'x: -5, y: -2.5, z: 1.8' >/dev/null 2>&1
    exit 0
  fi
  sleep 1
done
echo "lo_sim: Gazebo camera follow not available (GUI not up?)" >&2
'''


# The viewers render in software (llvmpipe uses every core by default: Gazebo's GUI took 631 %
# CPU and starved the sensor rig, whose scans then left up to 69 ms late). Viewers get 2 render
# threads and a lower priority than the rig and STRIDE (DECISIONS I37).
_VIEWER_ENV = {'LP_NUM_THREADS': '2'}
_NICE = ['nice', '-n', '10']


def _setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    gazebo = LaunchConfiguration('gazebo').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    lo = LaunchConfiguration('lo').perform(context).lower() == 'true'
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    rate = LaunchConfiguration('rate').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    delay = '3.0'
    actions = []
    prefix = get_package_prefix('lo_sim')
    share = get_package_share_directory('lo_sim')
    world_tool = os.path.join(prefix, 'lib', 'lo_sim', 'lo_sim_world')
    if gazebo:
        world = f'/tmp/stride_{scenario}_{seed}_{os.getpid()}.sdf'
        with open(world, 'w') as f:
            f.write(subprocess.check_output(
                [world_tool, scenario, '--seed', seed, '--start-delay', delay], text=True))
        actions.append(SetEnvironmentVariable(
            'GZ_PARTITION', os.environ.get('GZ_PARTITION') or f'stride_{os.getpid()}'))
        actions.append(SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.path.join(prefix, 'lib') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')))
        if gui:
            gz_args = ['gz', 'sim', '-r', world, '--gui-config',
                       os.path.join(share, 'config', 'gui.config')]
            actions.append(TimerAction(period=4.0, actions=[
                ExecuteProcess(cmd=['bash', '-c', _FOLLOW], output='log')]))
        else:
            gz_args = ['gz', 'sim', '-r', '-s', world]
        actions.append(ExecuteProcess(cmd=_NICE + gz_args, output='screen',
                                      additional_env=_VIEWER_ENV))
        # the rig's clock -> Gazebo (KinematicRig poses follow it; Gazebo is display only)
        # (gz topic != /clock: Gazebo publishes its own sim time there)
        bridge = f'/tmp/stride_clock_bridge_{os.getpid()}.yaml'
        with open(bridge, 'w') as f:
            f.write('- ros_topic_name: /clock\n  gz_topic_name: /stride/scenario_clock\n'
                    '  ros_type_name: rosgraph_msgs/msg/Clock\n  gz_type_name: gz.msgs.Clock\n'
                    '  direction: ROS_TO_GZ\n')
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='scenario_clock_bridge', output='log',
                            parameters=[{'config_file': bridge}]))
    actions.append(Node(package='lo_sim', executable='lo_sim_rig', name='lo_sim_rig', output='screen',
                        parameters=[{'scenario': scenario, 'seed': int(seed),
                                     'rate': float(rate), 'start_delay': float(delay)}]))
    if lo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lo_ros'), 'launch', 'stride_lo.launch.py')),
            launch_arguments={'robot': 'stride_rig', 'use_sim_time': 'true',
                              'requests': 'true', 'rviz': 'false', 'respawn': 'false',
                              'deskewed': 'true' if rviz else 'false'}.items()))
        actions.append(Node(package='lo_gtsam_adapter', executable='lo_graph_demo',
                            name='lo_graph_demo', output='screen',
                            parameters=[{'use_sim_time': True, 'imu_topic': '/livox/imu'}]))
    if rviz:
        # robot model with visuals (viz-only URDF on its own topic: the self-filter's
        # /robot_description from lo_sim_rig is not touched)
        urdf = subprocess.check_output([world_tool, scenario, '--seed', seed, '--urdf'], text=True)
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name='lo_viz_robot_state_publisher', output='log',
                            parameters=[{'robot_description': urdf, 'use_sim_time': True}],
                            remappings=[('robot_description', '/lo_viz/robot_description')]))
        actions.append(Node(package='lo_sim', executable='lo_viz.py', name='lo_viz', output='log',
                            parameters=[{'use_sim_time': True, 'gt_topic': '/ground_truth/odom'}]))
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
                            parameters=[{'use_sim_time': True}],
                            prefix=' '.join(_NICE), additional_env=_VIEWER_ENV))
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
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lo', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=_setup),
    ])
