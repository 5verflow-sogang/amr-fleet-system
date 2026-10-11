"""YOLO 인식 노드. 로봇 네임스페이스는 bringup이 붙인다 (예: /amr_01/perception/objects)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_detection'), 'config', 'detection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='map',
                              description='출력 좌표계. 위치 추정 없이 시험할 때 odom'),
        DeclareLaunchArgument('device', default_value='cpu', description='cpu 또는 cuda:0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_detection', executable='detection_node', name='detection',
            parameters=[config, {
                'target_frame': LaunchConfiguration('target_frame'),
                'device': LaunchConfiguration('device'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'),
    ])
