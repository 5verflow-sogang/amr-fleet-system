"""실행 중인 Gazebo에 화물 상자를 스폰한다 (config/cargo_boxes.yaml, YOLO 인식용).

    ros2 launch amr_gazebo cargo_boxes.launch.py                 # yaml의 상자 전부
    ros2 launch amr_gazebo sim.launch.py cargo:=true             # sim과 같이

상자마다 static 모델 하나(이름 = 상자 id)로 스폰한다. 라벨 카메라가 모델 단위로 상자를 내기 때문이다.
위치는 yaml 좌표 그대로라 GT 라벨로도 쓸 수 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_gazebo.cargo import cargo_model_sdf, check_lidar_clearance, load_boxes


def _spawn(context):
    config = LaunchConfiguration('config').perform(context) or os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'cargo_boxes.yaml')
    boxes = load_boxes(config)
    blocked = check_lidar_clearance(boxes)
    if blocked:
        raise RuntimeError(f'LiDAR 스캔 높이에 걸리는 상자: {blocked} (config/cargo_boxes.yaml 배치 규칙)')
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', cargo_model_sdf(b), '-name', b.id],
        output='screen',
    ) for b in boxes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='상자 배치 yaml (기본 config/cargo_boxes.yaml)'),
        OpaqueFunction(function=_spawn),
    ])
