"""작업 생성기. 시나리오 일정대로 /fleet/task_requests에 Task를 발행한다.

    ros2 launch amr_fleet_manager task_generator.launch.py seed:=3 schedule_out:=/tmp/run1/tasks.json
    ros2 launch amr_fleet_manager task_generator.launch.py tasks_file:=/tmp/run1/tasks.json   # 같은 일정 다시 돌리기
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file', default_value=os.path.join(config, 'scenarios', 'basic.yaml'),
            description='작업 시나리오. tasks_file을 주면 무시한다'),
        DeclareLaunchArgument(
            'tasks_file', default_value='',
            description='JSON Task 파일. 주면 시나리오 대신 이 일정을 그대로 발행한다'),
        DeclareLaunchArgument('seed', default_value='0', description='작업 일정 난수 seed. 하네스 반복 번호'),
        DeclareLaunchArgument(
            'stations_file', default_value=os.path.join(config, 'stations.yaml'),
            description='스테이션 자세 (world 좌표)'),
        DeclareLaunchArgument(
            'schedule_out', default_value='',
            description='만든 일정을 JSON Task 파일로 남길 경로. 하네스 실행 폴더의 tasks.json'),
        Node(
            package='amr_fleet_manager',
            executable='task_generator',
            parameters=[{
                'scenario_file': ParameterValue(LaunchConfiguration('scenario_file'), value_type=str),
                'tasks_file': ParameterValue(LaunchConfiguration('tasks_file'), value_type=str),
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                'stations_file': ParameterValue(LaunchConfiguration('stations_file'), value_type=str),
                'schedule_out': ParameterValue(LaunchConfiguration('schedule_out'), value_type=str),
            }],
            output='screen',
        ),
    ])
