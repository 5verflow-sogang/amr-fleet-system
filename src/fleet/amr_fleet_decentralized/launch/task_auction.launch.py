"""분산 경매 노드. 로봇 네임스페이스 안에서 띄운다.

    ros2 launch amr_fleet_decentralized task_auction.launch.py namespace:=amr_01 robot_id:=amr_01
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amr_fleet_decentralized'), 'config', 'task_auction.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('robot_id', default_value='amr_01', description='TaskBid.robot_id'),
        Node(
            package='amr_fleet_decentralized',
            executable='task_auction_node',
            name='task_auction',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config, {
                'robot_id': LaunchConfiguration('robot_id'),
            }],
            output='screen',
        ),
    ])
