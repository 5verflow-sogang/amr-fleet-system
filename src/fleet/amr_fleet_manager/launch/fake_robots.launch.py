"""가짜 로봇 N대. Fleet 서버·장애 주입기·하네스를 BT 없이 돌려볼 때 쓴다.

    ros2 launch amr_fleet_manager fake_robots.launch.py count:=3
    ros2 launch amr_fleet_manager fake_robots.launch.py count:=5 relayed:=true   # 장애 주입기 중계본 구독
    # 10대: 장애 주입기도 같은 수로 띄워야 6~10번에 중계본이 간다 (fault_injector.launch.py num_robots:=10)
    ros2 launch amr_fleet_manager fake_robots.launch.py count:=10 relayed:=true
    # 120초에 amr_03을 SIGKILL로 죽인다 (연구 1 로봇 장애). 시각은 fault_events.csv에 남는다
    ros2 launch amr_fleet_manager fake_robots.launch.py kill_robot:=amr_03 kill_robot_at_s:=120.0 \\
        fault_events_file:=/tmp/run1/fault_events.csv

로봇 이름은 amr_01부터 차례로 붙는다. 시작 위치는 config/fake_robots.yaml에 있는 대수(10대)까지다.
kill_robot_at_s는 이 launch를 시작한 때부터 잰 실제 시간이다. 다시 띄운 로봇은 작업 없이 새로 시작한다.
"""

import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, TimerAction
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_manager.records import append_fault_event

# config/fake_robots.yaml에 시작 위치가 있는 로봇 수
MAX_ROBOTS = 10


def _record(context, path, event):
    now = append_fault_event(path, event)
    return [LogInfo(msg=f'[장애 주입] 가짜 로봇 {event} (time={now:.3f})')]


def _robots(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'config', 'fake_robots.yaml')
    count = int(arg('count'))
    if not 1 <= count <= MAX_ROBOTS:
        # 시작 위치가 없는 로봇은 원점에 겹쳐 생겨 할당(거리 기준)이 틀어진다. 늘리려면 fake_robots.yaml에 위치를 더한다
        raise RuntimeError(f'count는 1~{MAX_ROBOTS} (fake_robots.yaml 시작 위치 수): {count}')
    relayed = arg('relayed').lower() == 'true'
    # 장애 주입기는 /<robot_id>/fleet/<topic>으로 중계한다. 네임스페이스 안의 상대 이름이 그 토픽이 된다
    prefix = 'fleet' if relayed else '/fleet'

    def robot(robot_id):
        return Node(
            package='amr_fleet_manager',
            executable='fake_robot',
            namespace=robot_id,
            parameters=[config, {'robot_id': robot_id, 'server_topic_prefix': prefix,
                                 'pause_on_link_loss_s': float(arg('pause_on_link_loss_s')),
                                 'speed_mps': float(arg('speed_mps')),
                                 'pause_mode': arg('pause_mode')}],
            output='screen',
        )

    robot_ids = [f'amr_{i:02d}' for i in range(1, count + 1)]
    nodes = {robot_id: robot(robot_id) for robot_id in robot_ids}
    actions = list(nodes.values())

    victim = arg('kill_robot')
    kill_at = float(arg('kill_robot_at_s'))
    restart_after = float(arg('restart_robot_after_s'))
    events_file = arg('fault_events_file')
    if victim not in ('', 'none') and kill_at >= 0:
        if victim not in nodes:
            raise ValueError(f'kill_robot {victim}가 없다 (가능: {robot_ids})')
        actions.append(TimerAction(period=kill_at, actions=[
            OpaqueFunction(function=_record, args=[events_file, f'robot_kill:{victim}']),
            # 정리 과정 없이 죽인다 (로봇 장애). 서버는 RobotState가 끊기는 것으로 알아챈다
            EmitEvent(event=SignalProcess(signal_number=signal.SIGKILL, process_matcher=matches_action(nodes[victim]))),
        ]))
        if restart_after >= 0:
            actions.append(TimerAction(period=kill_at + restart_after, actions=[
                OpaqueFunction(function=_record, args=[events_file, f'robot_restart:{victim}']),
                robot(victim),
            ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='5', description='로봇 수 (1~10)'),
        DeclareLaunchArgument(
            'relayed', default_value='false',
            description='true면 장애 주입기 중계본(/<robot_id>/fleet/*)을 구독한다'),
        DeclareLaunchArgument('kill_robot', default_value='', description='죽일 로봇 (예: amr_03). 비우거나 none이면 끄기'),
        DeclareLaunchArgument(
            'kill_robot_at_s', default_value='-1',
            description='launch 시작 후 이 시각(실제 시간, s)에 SIGKILL. 음수면 끄기'),
        DeclareLaunchArgument(
            'restart_robot_after_s', default_value='-1',
            description='죽인 뒤 이 시간(s)이 지나면 다시 띄운다. 음수면 죽은 채로 둔다'),
        DeclareLaunchArgument(
            'fault_events_file', default_value='',
            description='강제종료·재시작 시각 기록 파일 (fault_events.csv). 비우면 기록하지 않는다'),
        DeclareLaunchArgument(
            'pause_mode', default_value='stop',
            description='pause_on_link_loss_s > 0일 때 끊긴 동안: stop(그 자리에서 멈춤) / leg(지금 구간은 마저 가고 다음 구간 앞에서 대기)'),
        DeclareLaunchArgument(
            'speed_mps', default_value='1.0',
            description='직선 이동 속도 (m/s, config 기본 1.0). 올리면 작업 수행 시간이 짧아진다 (연구 1 짧은 작업 예측 검증)'),
        DeclareLaunchArgument(
            'pause_on_link_loss_s', default_value='0.0',
            description='서버 heartbeat가 이 시간(실제 시간, s) 동안 안 오면 하던 작업을 멈추고 기다린다(끊기면 멈춤 정책). 0이면 끄기'),
        OpaqueFunction(function=_robots),
    ])
