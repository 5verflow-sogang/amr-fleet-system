"""시뮬레이터 정답 위치를 localization.launch.py(AMCL+EKF) 대신 쓴다 - 식당 시연 녹화용(README 참고).

    ros2 launch amr_localization ground_truth_localization.launch.py namespace:=amr_01

map → <ns>/odom은 identity 정적 TF, <ns>/odom → <ns>/base_footprint는 ground_truth_localization
노드가 ground_truth/odom(world 기준)을 map 기준으로 바꿔 매 메시지 낸다. localization.launch.py와
같은 토픽(odometry/local·odometry/filtered·amcl_pose)·TF를 내므로 나머지 노드는 바뀐 걸 모른다.

물류센터 KPI에는 쓰지 않는다 - 위치추정 성능을 전혀 반영하지 않는다(ground_truth_localization_node.py).
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _ground_truth_localization(context):
    ns = LaunchConfiguration('namespace').perform(context)
    prefix = f'{ns}/' if ns else ''
    use_sim_time = _is_true(context, 'use_sim_time')
    # 노드의 map_origin_in_world 파라미터는 float 배열이라 문자열 그대로 넘기면 타입이 안 맞는다
    # (system.launch.py도 같은 식으로 연다)
    origin = [float(v) for v in yaml.safe_load(LaunchConfiguration('map_origin_in_world').perform(context))]

    return [
        # map → <ns>/odom: identity. ground_truth_localization_node가 <ns>/odom → <ns>/base_footprint
        # 하나로 정답 위치를 그대로 옮긴다 (localization.launch.py의 ekf_global 자리)
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_identity',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', prefix + 'odom'],
            parameters=[{'use_sim_time': use_sim_time}], output='screen'),
        Node(
            package='amr_localization', executable='ground_truth_localization',
            name='ground_truth_localization', namespace=ns,
            parameters=[{'use_sim_time': use_sim_time, 'map_origin_in_world': origin}],
            output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_origin_in_world', default_value='[-0.0439, 1.4712, -0.00088]',
            description='world에서 본 map 원점 [x, y, yaw] (system.launch.py와 같은 값으로 맞춘다)'),
        OpaqueFunction(function=_ground_truth_localization),
    ])
