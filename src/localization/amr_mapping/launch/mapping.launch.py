"""slam_toolbox로 지도를 만든다 (온라인 비동기 매핑).

    ros2 launch amr_mapping mapping.launch.py                     # amr_01, /amr_01/scan
    ros2 launch amr_mapping mapping.launch.py scan_topic:=scan_raw # 필터 없이 원본 스캔으로

발행: /map (nav_msgs/OccupancyGrid), TF map → <ns>/odom
저장: ros2 run nav2_map_server map_saver_cli -f <경로/이름> --ros-args -p use_sim_time:=true
     (README "지도 저장" 참고)

odom → base_footprint TF는 다른 노드(지금은 Gazebo DiffDrive, D4 이후 ekf_local)가 발행해야 한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _slam(context):
    ns = LaunchConfiguration('namespace').perform(context)
    prefix = f'{ns}/' if ns else ''
    scan = LaunchConfiguration('scan_topic').perform(context)
    defaults = os.path.join(
        get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async.yaml')
    ours = os.path.join(
        get_package_share_directory('amr_mapping'), 'config', 'slam_toolbox_mapping.yaml')
    return [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[defaults, ours, {
                'odom_frame': prefix + 'odom',
                'base_frame': prefix + 'base_footprint',
                'scan_topic': f'/{prefix}{scan}',
                'use_sim_time':
                    LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='지도를 만들 로봇'),
        DeclareLaunchArgument(
            'scan_topic', default_value='scan',
            description='로봇 네임스페이스 아래 스캔 토픽. 필터 출력 scan, 원본 scan_raw'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_slam),
    ])
