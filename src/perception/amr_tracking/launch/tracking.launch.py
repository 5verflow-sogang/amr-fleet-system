"""장애물 추적 노드. 기본은 LiDAR 스캔 입력. use_synthetic:=true면 시험용 측정 발행기를 함께 띄우고 그 측정을 쓴다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_tracking'), 'config', 'tracking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_synthetic', default_value='false',
            description='LiDAR 스캔 대신 시험용 장애물 측정(PoseArray) 발행·구독'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_tracking', executable='tracking_node', name='tracking',
            parameters=[config, {
                # 시험용 측정 발행기를 쓰면 스캔 대신 PoseArray 측정을 받는다
                'measurement_source': PythonExpression([
                    "'poses' if '", LaunchConfiguration('use_synthetic'),
                    "' == 'true' else 'scan'"]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'),
        Node(
            package='amr_tracking', executable='synthetic_obstacles_node',
            name='synthetic_obstacles', parameters=[config], output='screen',
            condition=IfCondition(LaunchConfiguration('use_synthetic'))),
    ])
