"""휠 오도메트리 노드를 띄운다.

    ros2 launch amr_odometry odometry.launch.py namespace:=amr_01
    ros2 launch amr_odometry odometry.launch.py publish_tf:=true      # EKF 없이 단독 시험
    ros2 launch amr_odometry odometry.launch.py encoder_seed:=42      # 계통 오차 재현
    ros2 launch amr_odometry odometry.launch.py encoder_noise:=false  # 이상적 엔코더 (양자화만)

구독: /<ns>/joint_states  발행: /<ns>/<odom_topic> (기본 odom)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _odometry(context):
    ns = LaunchConfiguration('namespace').perform(context)
    config = os.path.join(get_package_share_directory('amr_odometry'), 'config', 'odometry.yaml')
    overrides = {
        # TF 프레임 이름에는 네임스페이스 접두어가 붙는다 (docs/api.md TF 트리)
        'frame_prefix': f'{ns}/' if ns else '',
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        'publish_tf': LaunchConfiguration('publish_tf').perform(context).lower() == 'true',
        'odom_topic': LaunchConfiguration('odom_topic').perform(context),
        'encoder.seed': int(LaunchConfiguration('encoder_seed').perform(context)),
        'encoder.noise_enabled':
            LaunchConfiguration('encoder_noise').perform(context).lower() == 'true',
    }
    return [
        Node(
            package='amr_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            namespace=ns,
            parameters=[config, overrides],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'publish_tf', default_value='false',
            description='odom → base_footprint TF 발행. EKF(ekf_local)가 발행하면 false (D4)'),
        DeclareLaunchArgument(
            'odom_topic', default_value='odom',
            description='출력 토픽 (상대 이름). Gazebo DiffDrive odom과 겹치면 바꿔서 쓴다'),
        DeclareLaunchArgument(
            'encoder_seed', default_value='0',
            description='엔코더 노이즈 seed. 0이면 실행마다 무작위'),
        DeclareLaunchArgument(
            'encoder_noise', default_value='true',
            description='false면 엔코더 양자화만 하고 계통·무작위 오차를 넣지 않는다'),
        OpaqueFunction(function=_odometry),
    ])
