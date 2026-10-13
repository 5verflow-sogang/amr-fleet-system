"""Fleet 토픽 중계 + 지연·손실 주입.

    ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=3
    # 분산·하이브리드 비교군: 경매 토픽에도 같은 조건을 건다
    ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 relay_auction:=true
    # 로봇 수 (중계 대상 amr_01~amr_NN). 기본 5대는 config/fault_injector.yaml의 robot_ids
    ros2 launch amr_fault_injector fault_injector.launch.py num_robots:=10

구독: /fleet/{task_assignments,heartbeat,zone_grants,task_table}, relay_auction이면 /fleet/{task_auctions,task_bids}도
발행: /<robot_id>/fleet/<같은 토픽>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _injector(context):
    config = os.path.join(
        get_package_share_directory('amr_fault_injector'), 'config', 'fault_injector.yaml')
    overrides = {
        'delay_ms': ParameterValue(LaunchConfiguration('delay_ms'), value_type=float),
        'loss_pct': ParameterValue(LaunchConfiguration('loss_pct'), value_type=float),
        'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
        'relay_auction': ParameterValue(LaunchConfiguration('relay_auction'), value_type=bool),
    }
    num_robots = LaunchConfiguration('num_robots').perform(context)
    if num_robots:
        # 비우면 fault_injector.yaml의 robot_ids. 주면 amr_01부터 그 수만큼 (가짜 로봇·system.launch와 같은 이름 규칙)
        count = int(num_robots)
        if count < 1:
            raise RuntimeError(f'num_robots는 1 이상: {count}')
        overrides['robot_ids'] = [f'amr_{i:02d}' for i in range(1, count + 1)]
    return [Node(
        package='amr_fault_injector',
        executable='fault_injector',
        parameters=[config, overrides],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay_ms', default_value='100.0',
            description='고정 지연 [ms]. 기본값은 명세 9절 "통신 지연 100ms 반영"'),
        DeclareLaunchArgument(
            'loss_pct', default_value='0.0',
            description='손실률 [%], 로봇·토픽마다 독립'),
        DeclareLaunchArgument(
            'seed', default_value='0',
            description='손실 난수 seed. 하네스 반복 번호를 넣는다'),
        DeclareLaunchArgument(
            'relay_auction', default_value='false',
            description='true면 분산 경매 토픽(task_auctions, task_bids, 로봇 → 로봇)도 중계한다. 분산·하이브리드 비교군에서 켠다'),
        DeclareLaunchArgument(
            'num_robots', default_value='',
            description='중계할 로봇 수 (amr_01~). 비우면 fault_injector.yaml의 robot_ids (5대)'),
        OpaqueFunction(function=_injector),
    ])
