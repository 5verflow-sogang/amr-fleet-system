"""중앙집중 Fleet 서버.

    ros2 launch amr_fleet_manager fleet_manager.launch.py algorithm:=hungarian assignments_csv:=/tmp/run1/assignments.csv

구독: /fleet/{task_requests,robot_states,task_assignment_acks,task_status}
발행: /fleet/{task_assignments,heartbeat,task_table}
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
        get_package_share_directory('amr_fleet_manager'), 'config', 'fleet_manager.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm', default_value='nearest',
            description='할당 알고리즘: nearest / load_balance / hungarian'),
        DeclareLaunchArgument(
            'assignments_csv', default_value='',
            description='할당 호출 기록 파일. 비우면 기록하지 않는다 (하네스가 실행마다 경로를 준다)'),
        Node(
            package='amr_fleet_manager',
            executable='fleet_manager',
            parameters=[config, {
                'assignment.algorithm': ParameterValue(LaunchConfiguration('algorithm'), value_type=str),
                'assignments_csv': ParameterValue(LaunchConfiguration('assignments_csv'), value_type=str),
            }],
            output='screen',
        ),
    ])
