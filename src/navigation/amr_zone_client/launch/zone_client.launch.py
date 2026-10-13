"""구역 진입 처리 노드. 로봇 네임스페이스 안에서 띄운다.

    ros2 launch amr_zone_client zone_client.launch.py namespace:=amr_01 robot_id:=amr_01
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_zone_client'), 'config', 'zone_client.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('robot_id', default_value='amr_01', description='ZoneRequest.robot_id'),
        DeclareLaunchArgument(
            'zones_file', default_value='', description='zones.yaml 경로 (서버와 같은 파일)'),
        DeclareLaunchArgument(
            'zones_frame', default_value='',
            description='zones.yaml area_world의 좌표계. 시뮬레이션은 world (TF map ← world로 바꿔 쓴다). 비우면 map 그대로'),
        Node(
            package='amr_zone_client',
            executable='zone_client_node',
            name='zone_client',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config, {
                'robot_id': LaunchConfiguration('robot_id'),
                'zones_file': LaunchConfiguration('zones_file'),
                'zones_frame': LaunchConfiguration('zones_frame'),
            }],
            output='screen',
        ),
    ])
