"""장애물 추적 노드. use_synthetic:=true면 시험용 측정 발행기도 함께 띄운다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_tracking'), 'config', 'tracking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_synthetic', default_value='false',
            description='LiDAR 군집화 대신 시험용 장애물 측정 발행'),
        Node(
            package='amr_tracking', executable='tracking_node', name='tracking',
            parameters=[config], output='screen'),
        Node(
            package='amr_tracking', executable='synthetic_obstacles_node',
            name='synthetic_obstacles', parameters=[config], output='screen',
            condition=IfCondition(LaunchConfiguration('use_synthetic'))),
    ])
