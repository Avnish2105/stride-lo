"""lo_graph_demo next to the STRIDE node: graph nodes every 100 ms on IMU stamps, answered by
STRIDE in request mode (ros2 launch lo_ros stride_lo.launch.py requests:=true ...)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('node_period', default_value='0.1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lo_gtsam_adapter', executable='lo_graph_demo', name='lo_graph_demo',
             output='screen',
             parameters=[{'imu_topic': LaunchConfiguration('imu_topic'),
                          'node_period': LaunchConfiguration('node_period'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
