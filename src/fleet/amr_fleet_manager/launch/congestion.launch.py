"""혼잡도 노드. 손님 정답 위치로 구역별 혼잡도(/fleet/congestion)와 costmap 마스크를 발행한다 (api.md 4-5절).

    ros2 launch amr_fleet_manager congestion.launch.py
    ros2 launch amr_fleet_manager congestion.launch.py zones_file:=/path/restaurant_zones.yaml obstacles_file:=/path/peak.yaml
"""

import os
from typing import List

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config')
    # Fleet 컨테이너처럼 amr_gazebo가 없으면 obstacles_file을 직접 준다
    try:
        obstacles = os.path.join(get_package_share_directory('amr_gazebo'), 'config', 'dynamic_obstacles.yaml')
    except PackageNotFoundError:
        obstacles = ''

    return LaunchDescription([
        DeclareLaunchArgument(
            'zones_file', default_value=os.path.join(config, 'congestion_zones.yaml'),
            description='혼잡도 구역 파일 (id, area, capacity)'),
        DeclareLaunchArgument(
            'obstacles_file', default_value=obstacles,
            description='손님 id를 읽을 동적 장애물 설정 (amr_gazebo dynamic_obstacles.yaml 형식)'),
        DeclareLaunchArgument(
            'map_origin_in_world', default_value='[-0.0439, 1.4712, -0.00088]',
            description='world에서 본 map 원점 [x, y, yaw] (실수로 적는다). 정답 위치(world)를 map으로 옮긴다'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_fleet_manager',
            executable='congestion',
            parameters=[{
                'zones_file': ParameterValue(LaunchConfiguration('zones_file'), value_type=str),
                'obstacles_file': ParameterValue(LaunchConfiguration('obstacles_file'), value_type=str),
                'map_origin_in_world': ParameterValue(LaunchConfiguration('map_origin_in_world'), value_type=List[float]),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            output='screen',
        ),
    ])
