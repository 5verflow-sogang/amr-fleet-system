"""실행 중인 Gazebo에 동적 장애물을 스폰하고 움직인다 (config/dynamic_obstacles.yaml).

    ros2 launch amr_gazebo dynamic_obstacles.launch.py                    # yaml의 5개 전부
    ros2 launch amr_gazebo dynamic_obstacles.launch.py only:=forklift_1   # 1개만 (쉼표로 여러 개)
    ros2 launch amr_gazebo dynamic_obstacles.launch.py seed:=42           # 속도·무작위 경로 재현

ROS 토픽 (<id> = yaml의 id):
    /dynamic_obstacles/<id>/ground_truth/odom  nav_msgs/Odometry  Gazebo → ROS (진짜 위치·속도, frame "world", 평가 전용)
    /dynamic_obstacles/<id>/cmd_vel            geometry_msgs/Twist ROS → Gazebo (dynamic_obstacles_node가 낸다)
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# yaml의 type → models/ 안의 모델
MODELS = {'worker': 'dynamic_worker', 'forklift': 'dynamic_forklift', 'robot': 'dynamic_robot'}


def _obstacles(context):
    share = get_package_share_directory('amr_gazebo')
    config = LaunchConfiguration('config').perform(context) \
        or os.path.join(share, 'config', 'dynamic_obstacles.yaml')
    only = [s.strip() for s in LaunchConfiguration('only').perform(context).split(',') if s.strip()]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    with open(config, encoding='utf-8') as f:
        specs = yaml.safe_load(f)['obstacles']
    if only:
        specs = [s for s in specs if s['id'] in only]

    actions, bridge_args, remappings = [], [], []
    for spec in specs:
        oid, spawn = spec['id'], spec['spawn']
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', os.path.join(share, 'models', MODELS[spec['type']], 'model.sdf'),
                '-name', oid,
                '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', '0.0',
                '-Y', str(spawn.get('yaw', 0.0)),
            ],
            output='screen',
        ))
        bridge_args += [
            f'/model/{oid}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/model/{oid}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ]
        remappings += [
            (f'/model/{oid}/cmd_vel', f'/dynamic_obstacles/{oid}/cmd_vel'),
            (f'/model/{oid}/odometry', f'/dynamic_obstacles/{oid}/ground_truth/odom'),
        ]

    return actions + [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='dynamic_obstacles_bridge',
            arguments=bridge_args,
            remappings=remappings,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='amr_gazebo',
            executable='dynamic_obstacles_node.py',
            name='dynamic_obstacles',
            parameters=[{
                'config': config,
                'seed': int(LaunchConfiguration('seed').perform(context)),
                'only': only or [''],
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='',
            description='장애물 yaml 경로. 비우면 config/dynamic_obstacles.yaml'),
        DeclareLaunchArgument(
            'only', default_value='',
            description='움직일 장애물 id (쉼표로 구분). 비우면 전부'),
        DeclareLaunchArgument(
            'seed', default_value='0',
            description='속도·무작위 경로 seed. 0이면 실행마다 무작위'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_obstacles),
    ])
