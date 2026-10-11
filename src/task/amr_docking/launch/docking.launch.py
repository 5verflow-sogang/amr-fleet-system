"""로봇 1대의 도킹 액션 서버(docking/dock)를 띄운다.

    ros2 launch amr_docking docking.launch.py namespace:=amr_01
    ros2 launch amr_docking docking.launch.py cmd_vel_topic:=cmd_vel   # Nav2·구역·안전 체인 없이 단독 시험

카메라가 켜져 있어야 한다 (sim enable_camera:=true). 마커판은 aruco_markers.launch.py로 띄운다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _server(context):
    config = os.path.join(get_package_share_directory('amr_docking'), 'config', 'docking.yaml')
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic').perform(context),
    }
    return [Node(
        package='amr_docking', executable='docking_server', name='docking_server',
        namespace=LaunchConfiguration('namespace').perform(context),
        parameters=[config, overrides], output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel_nav',
            description='도킹 속도 명령 토픽. 기본은 Nav2 컨트롤러와 같은 자리(cmd_vel_nav)'),
        OpaqueFunction(function=_server),
    ])
