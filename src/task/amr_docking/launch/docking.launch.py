"""로봇 1대의 도킹 액션 서버(docking/dock)를 띄운다.

    ros2 launch amr_docking docking.launch.py namespace:=amr_01
    ros2 launch amr_docking docking.launch.py cmd_vel_topic:=cmd_vel   # Nav2·구역·안전 체인 없이 단독 시험
    ros2 launch amr_docking docking.launch.py aruco_localizer:=true    # Kidnapped 복구용 ArUco 위치 노드도

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
    ns = LaunchConfiguration('namespace').perform(context)
    nodes = [Node(
        package='amr_docking', executable='docking_server', name='docking_server',
        namespace=ns, parameters=[config, overrides], output='screen',
    )]
    if LaunchConfiguration('aruco_localizer').perform(context).lower() == 'true':
        nodes.append(Node(
            package='amr_docking', executable='aruco_localizer', name='aruco_localizer', namespace=ns,
            parameters=[os.path.join(get_package_share_directory('amr_docking'), 'config', 'aruco_localizer.yaml'),
                        {'use_sim_time': overrides['use_sim_time']}],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel_nav',
            description='도킹 속도 명령 토픽. 기본은 Nav2 컨트롤러와 같은 자리(cmd_vel_nav)'),
        DeclareLaunchArgument(
            'aruco_localizer', default_value='false',
            description='true면 마커판으로 로봇 map 자세(aruco_pose)를 내는 노드도 띄운다 (Kidnapped 복구용)'),
        OpaqueFunction(function=_server),
    ])
