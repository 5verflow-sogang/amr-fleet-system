"""분산 경매 노드. 로봇 네임스페이스 안에서 띄운다.

    ros2 launch amr_fleet_decentralized task_auction.launch.py namespace:=amr_01 robot_id:=amr_01
    ros2 launch amr_fleet_decentralized task_auction.launch.py relayed:=true   # 장애 주입기 중계본(fleet/...) 구독
    ros2 launch amr_fleet_decentralized task_auction.launch.py fleet_mode:=hybrid relayed:=true   # 하이브리드

공고·입찰 구독: <prefix>/task_auctions, <prefix>/task_bids (prefix: /fleet, relayed면 fleet) -
amr_bt·가짜 로봇의 relayed 인자와 같은 이름·동작(10/9 결정). relayed면 하이브리드의 heartbeat·작업표도
중계본(fleet/heartbeat, fleet/task_table)을 구독한다 - 장애 주입기가 거는 끊김을 그대로 겪어야 폴백한다.
fleet_mode: decentral(기본, 늘 분산) / hybrid(평시 중앙, heartbeat가 끊기면 분산, architecture.md 5-2절).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _task_auction(context):
    ns = LaunchConfiguration('namespace').perform(context)
    relayed = LaunchConfiguration('relayed').perform(context).lower() == 'true'
    config = os.path.join(
        get_package_share_directory('amr_fleet_decentralized'), 'config', 'task_auction.yaml')
    overrides = {
        'robot_id': LaunchConfiguration('robot_id').perform(context),
        'fleet_mode': LaunchConfiguration('fleet_mode').perform(context),
    }
    if relayed:
        overrides['auction_topic_prefix'] = 'fleet'
        overrides['server_topic_prefix'] = 'fleet'
    return [
        Node(
            package='amr_fleet_decentralized',
            executable='task_auction_node',
            name='task_auction',
            namespace=ns,
            parameters=[config, overrides],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('robot_id', default_value='amr_01', description='TaskBid.robot_id'),
        DeclareLaunchArgument(
            'relayed', default_value='false',
            description='true면 장애 주입기 중계본(fleet/task_auctions, fleet/task_bids)을 구독한다'),
        DeclareLaunchArgument(
            'fleet_mode', default_value='decentral',
            description='decentral(늘 분산) 또는 hybrid(평시 중앙, 서버 heartbeat가 끊기면 분산)'),
        OpaqueFunction(function=_task_auction),
    ])
