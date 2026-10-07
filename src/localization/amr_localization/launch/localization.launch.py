"""위치추정 스택: 지도 서버 + AMCL + EKF 두 개 (D4 A안, docs/api.md 2-1절).

    ros2 launch amr_localization localization.launch.py                          # amr_01, 지도 v0
    ros2 launch amr_localization localization.launch.py use_imu:=false           # IMU 없이 (센서 플러그인 전)
    ros2 launch amr_localization localization.launch.py odom_topic:=wheel/odom   # DiffDrive odom과 겹칠 때 (#35)

구독 (<ns> = namespace):
    /<ns>/<odom_topic>  nav_msgs/Odometry   휠 오도메트리 (amr_odometry)
    /<ns>/imu/data      sensor_msgs/Imu     IMU 필터 출력 (amr_odometry, use_imu:=true일 때)
    /<ns>/scan          sensor_msgs/LaserScan  스캔 필터 출력 (AMCL)
    /map                nav_msgs/OccupancyGrid  지도 (start_map_server:=true면 여기서 띄움)
발행:
    /<ns>/amcl_pose          AMCL 추정 (ekf_global 입력)
    /<ns>/odometry/local     ekf_local 출력, odom 기준
    /<ns>/odometry/filtered  ekf_global 출력, map 기준 (위치 오차 KPI 측정 대상)
    /tf                      map → <ns>/odom (ekf_global), <ns>/odom → <ns>/base_footprint (ekf_local)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _localization(context):
    share = get_package_share_directory('amr_localization')
    ekf_config = os.path.join(share, 'config', 'ekf.yaml')
    amcl_config = os.path.join(share, 'config', 'amcl.yaml')

    ns = LaunchConfiguration('namespace').perform(context)
    prefix = f'{ns}/' if ns else ''
    use_sim_time = _is_true(context, 'use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    # TF 프레임 이름에는 로봇 접두어가 붙는다 (docs/api.md TF 트리). map은 로봇들이 같이 쓴다
    frames = {
        'map_frame': 'map',
        'odom_frame': prefix + 'odom',
        'base_link_frame': prefix + 'base_footprint',
    }
    inputs = {'odom0': odom_topic}
    if _is_true(context, 'use_imu'):
        inputs['imu0'] = imu_topic

    actions = []
    if _is_true(context, 'start_map_server'):
        actions += [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{
                    'yaml_filename': LaunchConfiguration('map').perform(context),
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                             'node_names': ['map_server']}],
                output='screen',
            ),
        ]

    actions += [
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            parameters=[amcl_config, {
                'use_sim_time': use_sim_time,
                'base_frame_id': prefix + 'base_footprint',
                'odom_frame_id': prefix + 'odom',
                # 초기 위치는 map 기준. 지도 v0은 world (0, 1.5)에서 매핑을 시작해
                # 기본 스폰 위치 world (0, 1.5)가 map (0, 0)이다 (amr_mapping maps/warehouse_v1.yaml, v0과 같다)
                'initial_pose.x': float(LaunchConfiguration('initial_x').perform(context)),
                'initial_pose.y': float(LaunchConfiguration('initial_y').perform(context)),
                'initial_pose.yaw': float(LaunchConfiguration('initial_yaw').perform(context)),
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['amcl']}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            namespace=ns,
            parameters=[ekf_config, frames, inputs, {
                'use_sim_time': use_sim_time,
                'world_frame': frames['odom_frame'],
            }],
            # 두 EKF가 같은 네임스페이스라 출력 이름을 나눈다
            remappings=[('odometry/filtered', 'odometry/local')],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=ns,
            parameters=[ekf_config, frames, inputs, {
                'use_sim_time': use_sim_time,
                'world_frame': frames['map_frame'],
                'pose0': 'amcl_pose',
            }],
            output='screen',
        ),
    ]
    return actions


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('amr_mapping'), 'maps', 'warehouse_v1.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map, description='지도 yaml (map_server)'),
        DeclareLaunchArgument(
            'start_map_server', default_value='true',
            description='지도 서버를 여기서 띄울지. 로봇 여러 대면 한 번만 띄우고 나머지는 false'),
        DeclareLaunchArgument(
            'odom_topic', default_value='odom',
            description='휠 오도메트리 토픽 (상대 이름). DiffDrive odom과 겹치면 wheel/odom (#35)'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data', description='IMU 토픽 (상대 이름)'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='EKF에 IMU를 넣을지. IMU가 없는 환경(센서 플러그인 전, sandbox)에서는 false'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='AMCL 초기 위치 x (map 기준) [m]'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='AMCL 초기 위치 y (map 기준) [m]'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='AMCL 초기 방향 (map 기준) [rad]'),
        OpaqueFunction(function=_localization),
    ])
