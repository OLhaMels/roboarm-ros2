"""Perception layer only: camera_node + vision_node (+ stream_node).

For a Pi or container that runs just the camera/QR part of the robot,
without MoveIt and ros2_control:

    ros2 launch roboarm_bringup perception.launch.py \
        perception_config:=/config/perception.yaml video_stream:=true
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('roboarm_bringup'))
    config = LaunchConfiguration('perception_config')
    return LaunchDescription([
        DeclareLaunchArgument('perception_config', default_value=str(share / 'config/perception.yaml')),
        DeclareLaunchArgument('video_stream', default_value='false'),
        Node(package='roboarm_nodes', executable='camera_node', output='screen', parameters=[config]),
        Node(package='roboarm_nodes', executable='vision_node', output='screen', parameters=[config]),
        Node(package='roboarm_nodes', executable='stream_node', output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('video_stream'))),
    ])
