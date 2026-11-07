"""안전 노드와 RViz2 E-Stop 버튼. 로봇 네임스페이스 안에서 띄운다 (예: ros2 launch amr_safety safety.launch.py namespace:=amr_01).

E-Stop 버튼(estop_button)은 기본으로 함께 뜬다. 원 명세 7장의 필수 기능이고, 클릭하기 전에는 마커만 내므로
비용이 작다. RViz를 쓰지 않는 헤드리스 실험에서는 estop_button:=false로 끈다.

profile:=restaurant(amr_bringup body:=serving이 넘긴다)면 safety.yaml 뒤에 safety_restaurant.yaml을 얹어
식당 값(서빙 로봇 몸체, 근접 저속)만 바꾼다. 기본 warehouse는 safety.yaml 그대로다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROFILES = ('warehouse', 'restaurant')


def config_files(profile):
    """안전 노드 설정 파일 목록. ROS2 Node parameters는 나중 파일이 이긴다."""
    config_dir = os.path.join(get_package_share_directory('amr_safety'), 'config')
    configs = [os.path.join(config_dir, 'safety.yaml')]
    if profile == 'restaurant':
        configs.append(os.path.join(config_dir, 'safety_restaurant.yaml'))
    return configs


def _safety(context):
    return [Node(
        package='amr_safety', executable='safety_node', name='safety',
        namespace=LaunchConfiguration('namespace'),
        parameters=config_files(LaunchConfiguration('profile').perform(context)) + [{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_in_topic': LaunchConfiguration('cmd_vel_in_topic'),
        }],
        output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'cmd_vel_in_topic', default_value='cmd_vel_zone_limited',
            description='게이트 입력. 구역 허가 없이 시험할 때는 cmd_vel_nav나 teleop 토픽'),
        DeclareLaunchArgument(
            'estop_button', default_value='true',
            description='RViz2 E-Stop 버튼(interactive marker)을 함께 띄울지'),
        DeclareLaunchArgument(
            'profile', default_value='warehouse', choices=list(PROFILES),
            description='restaurant면 safety_restaurant.yaml을 얹는다 (서빙 로봇 몸체, 근접 저속)'),
        OpaqueFunction(function=_safety),
        Node(
            package='amr_safety', executable='estop_button', name='estop_button',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('estop_button')),
            output='screen'),
    ])
