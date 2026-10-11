"""YOLO 인식 노드. 로봇 네임스페이스 안에서 띄운다 (예: namespace:=amr_01 → /amr_01/perception/objects).

amr_bringup은 로봇마다 namespace:=<이름>을 launch 인자로 넘긴다. 인자로 받아 노드에 붙이지 않으면
노드가 루트(/camera/..., /perception/objects)에 떠서 로봇 카메라(/amr_01/camera/...)를 받지 못한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_detection'), 'config', 'detection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='map',
                              description='출력 좌표계. 위치 추정 없이 시험할 때 odom'),
        DeclareLaunchArgument('device', default_value='cpu', description='cpu 또는 cuda:0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        Node(
            package='amr_detection', executable='detection_node', name='detection',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config, {
                'target_frame': LaunchConfiguration('target_frame'),
                'device': LaunchConfiguration('device'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'),
    ])
