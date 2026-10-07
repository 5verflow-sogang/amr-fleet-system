"""로봇 1대의 경로 추종 노드(path_tracker_node)를 띄운다.

    ros2 launch amr_control path_tracker.launch.py namespace:=amr_01
    ros2 launch amr_control path_tracker.launch.py controller:=stanley cte_csv_path:=/tmp/cte.csv

구독: plan (nav_msgs/Path), odom (nav_msgs/Odometry)
발행: cmd_vel_nav (geometry_msgs/Twist) → amr_zone_client → amr_safety → cmd_vel
Nav2 controller_server와 같은 cmd_vel_nav를 내므로 둘을 같이 띄우지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _tracker(context):
    config = os.path.join(get_package_share_directory('amr_control'), 'config', 'path_tracker.yaml')
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    }
    # 비운 인자는 yaml 값을 그대로 쓴다
    for name in ('controller', 'cte_csv_path'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    return [
        Node(
            package='amr_control',
            executable='path_tracker_node',
            name='path_tracker',
            namespace=LaunchConfiguration('namespace').perform(context),
            parameters=[config, overrides],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스 = robot_id'),
        DeclareLaunchArgument(
            'controller', default_value='', description='pure_pursuit | stanley (비우면 yaml 값)'),
        DeclareLaunchArgument(
            'cte_csv_path', default_value='', description='cte.csv 경로 (비우면 yaml 값, 기본 기록 안 함)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_tracker),
    ])
