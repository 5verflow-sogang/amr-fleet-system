"""분산 경매 노드 N대 (amr_01부터). 가짜 로봇 N대와 같이 띄운다 (하네스 fleet_fake fleet_mode).

    ros2 launch amr_fleet_decentralized task_auctions.launch.py count:=5 relayed:=true
    ros2 launch amr_fleet_decentralized task_auctions.launch.py count:=5 relayed:=true fleet_mode:=hybrid

로봇마다 task_auction.launch.py와 같은 노드(네임스페이스·robot_id = amr_NN)를 띄운다.
relayed면 장애 주입기 relay_auction 중계본을 구독한다 (fault_injector.launch.py relay_auction:=true와 같이 쓴다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 가짜 로봇 시작 위치 수와 같다 (amr_fleet_manager fake_robots.launch.py MAX_ROBOTS)
MAX_ROBOTS = 10


def _auctions(context):
    count = int(LaunchConfiguration('count').perform(context))
    if not 1 <= count <= MAX_ROBOTS:
        raise RuntimeError(f'count는 1~{MAX_ROBOTS}: {count}')
    relayed = LaunchConfiguration('relayed').perform(context)
    fleet_mode = LaunchConfiguration('fleet_mode').perform(context)
    fallback_timeout_s = LaunchConfiguration('fallback_timeout_s').perform(context)
    single = PathJoinSubstitution([FindPackageShare('amr_fleet_decentralized'), 'launch', 'task_auction.launch.py'])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single),
            launch_arguments={'namespace': robot_id, 'robot_id': robot_id, 'relayed': relayed,
                              'fleet_mode': fleet_mode, 'fallback_timeout_s': fallback_timeout_s}.items())
        for robot_id in (f'amr_{i:02d}' for i in range(1, count + 1))
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='5', description='로봇 수 (amr_01~amr_NN)'),
        DeclareLaunchArgument(
            'relayed', default_value='false',
            description='true면 장애 주입기 중계본(fleet/task_auctions, fleet/task_bids)을 구독한다'),
        DeclareLaunchArgument(
            'fleet_mode', default_value='decentral',
            description='decentral(늘 분산) 또는 hybrid(평시 중앙, 서버 heartbeat가 끊기면 분산). task_auction.launch.py로 넘긴다'),
        DeclareLaunchArgument(
            'fallback_timeout_s', default_value='2.0',
            description='하이브리드: heartbeat가 이 시간(s) 동안 안 오면 분산으로 전환. task_auction.launch.py로 넘긴다'),
        OpaqueFunction(function=_auctions),
    ])
