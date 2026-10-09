"""안전 노드. 로봇 네임스페이스 안에서 띄운다 (예: ros2 launch amr_safety safety.launch.py namespace:=amr_01)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_safety'), 'config', 'safety.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'cmd_vel_in_topic', default_value='cmd_vel_zone_limited',
            description='게이트 입력. 구역 허가 없이 시험할 때는 cmd_vel_nav나 teleop 토픽'),
        Node(
            package='amr_safety', executable='safety_node', name='safety',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_in_topic': LaunchConfiguration('cmd_vel_in_topic'),
            }],
            output='screen'),
    ])
