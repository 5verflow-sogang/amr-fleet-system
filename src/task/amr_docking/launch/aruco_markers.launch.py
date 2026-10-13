"""실행 중인 Gazebo에 도킹 스테이션 ArUco 마커판을 스폰한다 (config/dock_stations.yaml).

    ros2 launch amr_docking aruco_markers.launch.py

판은 static 모델 하나(aruco_markers)의 링크로 들어간다. LiDAR 스캔 높이를 피해 바닥 근처에 붙여
지도·위치추정에는 영향이 없다 (markers.py).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_docking.markers import clears_lidar, load_stations, markers_sdf


def _spawn(context):
    path = LaunchConfiguration('stations_file').perform(context) or os.path.join(
        get_package_share_directory('amr_docking'), 'config', 'dock_stations.yaml')
    stations = list(load_stations(path).values())
    blocked = [s.name for s in stations if not clears_lidar(s.board)]
    if blocked:
        raise RuntimeError(f'마커판이 LiDAR 스캔 높이에 걸린다: {blocked} (markers.py BOARDS)')
    # 판 28개의 칸 상자가 수백 개라 -string으로 넘기면 명령 인자 길이 한도를 넘는다 → 파일로 넘긴다
    sdf = tempfile.NamedTemporaryFile('w', suffix='.sdf', prefix='aruco_markers_', delete=False)
    sdf.write(markers_sdf(stations))
    sdf.close()
    return [Node(
        package='ros_gz_sim', executable='create',
        arguments=['-file', sdf.name, '-name', 'aruco_markers'],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stations_file', default_value='',
                              description='스테이션 yaml (기본 config/dock_stations.yaml)'),
        OpaqueFunction(function=_spawn),
    ])
