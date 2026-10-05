"""물류센터 월드 + AMR 1대 (+ RViz)를 한 번에 띄운다.

    ros2 launch amr_gazebo sim.launch.py                       # gui 컨테이너에서
    ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false  # 서버만
    ros2 launch amr_gazebo sim.launch.py publish_odom_tf:=false  # EKF가 odom TF를 낼 때 (D4)
    ros2 launch amr_gazebo sim.launch.py payload_mass:=25.0      # 적재 질량 [kg]
    ros2 launch amr_gazebo sim.launch.py enable_camera:=false    # 카메라 끔 (CPU 렌더링 부담 줄이기)
    ros2 launch amr_gazebo sim.launch.py dynamic_obstacles:=true # 동적 장애물 5개 (config/dynamic_obstacles.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('amr_gazebo'), 'launch')
    rviz_config = os.path.join(
        get_package_share_directory('amr_description'), 'rviz', 'amr.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2 실행 여부 (gui 이미지에만 설치됨)'),
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('payload_mass', default_value='0.0', description='적재물 질량 [kg]'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true',
            description='odom → base_footprint TF를 DiffDrive에서 낼지. EKF를 띄우면 false (D4)'),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='RGBD 카메라 센서와 브리지를 만들지. CPU sim에서 여러 대를 띄울 때 false'),
        DeclareLaunchArgument(
            'dynamic_obstacles', default_value='false',
            description='동적 장애물을 띄울지. 지도 매핑 때는 끈다'),
        DeclareLaunchArgument(
            'obstacle_seed', default_value='0',
            description='동적 장애물 속도·무작위 경로 seed. 0이면 실행마다 무작위'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'world.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'payload_mass': LaunchConfiguration('payload_mass'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
                'enable_camera': LaunchConfiguration('enable_camera'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'dynamic_obstacles.launch.py')),
            launch_arguments={'seed': LaunchConfiguration('obstacle_seed')}.items(),
            condition=IfCondition(LaunchConfiguration('dynamic_obstacles')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
