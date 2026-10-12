"""구역 진입 처리 노드. 로봇 네임스페이스 안에서 띄운다.

    ros2 launch amr_zone_client zone_client.launch.py namespace:=amr_01 robot_id:=amr_01
    # amr_bringup(시뮬레이션): 프레임에 로봇 접두어가 붙고 시계는 시뮬레이션 시간
    ros2 launch amr_zone_client zone_client.launch.py namespace:=amr_02 robot_id:=amr_02 \\
        base_frame:=amr_02/base_footprint use_sim_time:=true
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
            'base_frame', default_value='base_footprint',
            description='로봇 프레임. 시뮬레이션에서는 <네임스페이스>/base_footprint'),
        DeclareLaunchArgument(
            'cmd_vel_out_topic', default_value='cmd_vel_zone_limited',
            description='구역 속도 제한 출력. 안전 노드(amr_safety)가 없으면 cmd_vel'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='서버 lease와 같은 시계로 비교하려면 시뮬레이션 시간(true)'),
        Node(
            package='amr_zone_client',
            executable='zone_client_node',
            name='zone_client',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config, {
                'robot_id': LaunchConfiguration('robot_id'),
                'zones_file': LaunchConfiguration('zones_file'),
                'base_frame': LaunchConfiguration('base_frame'),
                'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_out_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
    ])
