"""로봇 1대의 작업 수행 노드(amr_bt_node)를 띄운다.

    ros2 launch amr_bt amr_bt.launch.py namespace:=amr_01
    ros2 launch amr_bt amr_bt.launch.py relayed:=true   # 장애 주입기 중계본(/<ns>/fleet/...) 구독

서버 → 로봇: <prefix>/task_assignments, <prefix>/heartbeat (prefix: /fleet, relayed면 fleet)
로봇 → 서버: /fleet/task_assignment_acks, /fleet/task_status, /fleet/robot_states
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bt(context):
    ns = LaunchConfiguration('namespace').perform(context)
    relayed = LaunchConfiguration('relayed').perform(context).lower() == 'true'
    config = os.path.join(get_package_share_directory('amr_bt'), 'config', 'amr_bt.yaml')
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    }
    if relayed:
        overrides['server_topic_prefix'] = 'fleet'
    return [
        Node(
            package='amr_bt',
            executable='amr_bt_node',
            name='amr_bt',
            namespace=ns,
            parameters=[config, overrides],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스 = robot_id'),
        DeclareLaunchArgument(
            'relayed', default_value='false',
            description='true면 장애 주입기 중계본(/<ns>/fleet/...)을 구독한다'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_bt),
    ])
