"""가짜 로봇 N대. Fleet 서버·장애 주입기·하네스를 BT 없이 돌려볼 때 쓴다.

    ros2 launch amr_fleet_manager fake_robots.launch.py count:=3
    ros2 launch amr_fleet_manager fake_robots.launch.py count:=5 relayed:=true   # 장애 주입기 중계본 구독

로봇 이름은 amr_01부터 차례로 붙는다. 시작 위치는 config/fake_robots.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robots(context):
    config = os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'config', 'fake_robots.yaml')
    count = int(LaunchConfiguration('count').perform(context))
    relayed = LaunchConfiguration('relayed').perform(context).lower() == 'true'
    # 장애 주입기는 /<robot_id>/fleet/<topic>으로 중계한다. 네임스페이스 안의 상대 이름이 그 토픽이 된다
    prefix = 'fleet' if relayed else '/fleet'
    nodes = []
    for i in range(1, count + 1):
        robot_id = f'amr_{i:02d}'
        nodes.append(Node(
            package='amr_fleet_manager',
            executable='fake_robot',
            namespace=robot_id,
            parameters=[config, {'robot_id': robot_id, 'server_topic_prefix': prefix}],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='5', description='로봇 수 (1~5)'),
        DeclareLaunchArgument(
            'relayed', default_value='false',
            description='true면 장애 주입기 중계본(/<robot_id>/fleet/*)을 구독한다'),
        OpaqueFunction(function=_robots),
    ])
