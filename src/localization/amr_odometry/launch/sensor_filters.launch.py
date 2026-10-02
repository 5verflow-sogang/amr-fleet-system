"""LiDAR·IMU 전처리 필터를 띄운다.

    ros2 launch amr_odometry sensor_filters.launch.py namespace:=amr_01

/<ns>/scan_raw → /<ns>/scan         (거리·각도·아웃라이어)
/<ns>/imu/data_raw → /<ns>/imu/data (바이어스 보정·저역통과·공분산). 시작 후 2초간 로봇을 세워 둔다
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _filters(context):
    ns = LaunchConfiguration('namespace').perform(context)
    config = os.path.join(
        get_package_share_directory('amr_odometry'), 'config', 'sensor_filters.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        Node(
            package='amr_odometry',
            executable=executable,
            name=name,
            namespace=ns,
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        )
        for executable, name in (('scan_filter_node', 'scan_filter'),
                                 ('imu_filter_node', 'imu_filter'))
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_filters),
    ])
