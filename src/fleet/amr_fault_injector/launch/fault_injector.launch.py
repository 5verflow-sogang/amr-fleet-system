"""서버 → 로봇 토픽 중계 + 지연·손실 주입.

    ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=3

구독: /fleet/{task_assignments,heartbeat,zone_grants,task_table}
발행: /<robot_id>/fleet/{task_assignments,heartbeat,zone_grants,task_table}
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amr_fault_injector'), 'config', 'fault_injector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'delay_ms', default_value='100.0',
            description='고정 지연 [ms]. 기본값은 명세 9절 "통신 지연 100ms 반영"'),
        DeclareLaunchArgument(
            'loss_pct', default_value='0.0',
            description='손실률 [%], 로봇·토픽마다 독립'),
        DeclareLaunchArgument(
            'seed', default_value='0',
            description='손실 난수 seed. 하네스 반복 번호를 넣는다'),
        Node(
            package='amr_fault_injector',
            executable='fault_injector',
            parameters=[config, {
                'delay_ms': ParameterValue(LaunchConfiguration('delay_ms'), value_type=float),
                'loss_pct': ParameterValue(LaunchConfiguration('loss_pct'), value_type=float),
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            }],
            output='screen',
        ),
    ])
