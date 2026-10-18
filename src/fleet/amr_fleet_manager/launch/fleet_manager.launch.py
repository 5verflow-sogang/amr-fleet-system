"""중앙집중 Fleet 서버. 서버 강제종료 장애(연구 1)도 여기서 건다.

    ros2 launch amr_fleet_manager fleet_manager.launch.py algorithm:=hungarian assignments_csv:=/tmp/run1/assignments.csv
    # 60초에 서버를 SIGKILL로 죽이고 10초 뒤 다시 띄운다. 시각은 fault_events.csv에 남는다
    ros2 launch amr_fleet_manager fleet_manager.launch.py kill_at_s:=60.0 restart_after_s:=10.0 \\
        fault_events_file:=/tmp/run1/fault_events.csv

구독: /fleet/{task_requests,robot_states,task_assignment_acks,task_status,zone_requests}
발행: /fleet/{task_assignments,heartbeat,task_table,zone_grants}

launch는 자기가 띄운 프로세스에만 신호를 보낼 수 있으므로 강제종료를 장애 주입기 노드가 아니라 이 launch에 둔다.
kill_at_s는 이 launch를 시작한 때부터 잰 실제 시간이다.
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
from launch_ros.parameter_descriptions import ParameterValue

from amr_fleet_manager.records import append_fault_event

# Node 파라미터 이름 ← launch 인자 이름
SERVER_ARGS = [
    ('assignment.algorithm', 'algorithm'),
    ('assignments_csv', 'assignments_csv'),
    ('traffic.zones_file', 'zones_file'),
    ('traffic.deadlock_strategy', 'deadlock_strategy'),
    ('epoch_file', 'epoch_file'),
]


def _record(context, path, event):
    now = append_fault_event(path, event)
    return [LogInfo(msg=f'[장애 주입] Fleet 서버 {event} (time={now:.3f})')]


def _server(context):
    config = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config', 'fleet_manager.yaml')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def server():
        return Node(
            package='amr_fleet_manager',
            executable='fleet_manager',
            parameters=[config, {
                param: ParameterValue(arg(launch_arg), value_type=str) for param, launch_arg in SERVER_ARGS
            }, {
                # 구역 lease는 로봇과 같은 시계로 잰다. Gazebo와 같이 돌리면 true (amr_bringup system.launch.py)
                # arg()는 이미 문자열이라 ParameterValue(value_type=bool)로 넘기면 타입 검사에서 실패한다
                'use_sim_time': arg('use_sim_time').lower() == 'true',
                # 할당 명령 재전송 한도 (연구 1 재전송 비교). 정수 파라미터라 문자열 묶음(SERVER_ARGS)과 따로 넘긴다
                'resend.max_resends': int(arg('max_resends')),
                # 재전송 간격 (연구 1 재시도 간격 비교). 실수 파라미터라 따로 넘긴다
                'resend.interval_s': float(arg('resend_interval_s')),
                # 상태 보고가 이 시간 끊긴 로봇의 작업을 회수 (연구 1 회수 기준 비교, robot_timeout_s 이상이어야 한다)
                'assignment.reclaim_timeout_s': float(arg('reclaim_timeout_s')),
            }],
            output='screen',
        )

    first = server()
    actions = [first]
    kill_at = float(arg('kill_at_s'))
    restart_after = float(arg('restart_after_s'))
    events_file = arg('fault_events_file')
    if kill_at >= 0:
        actions.append(TimerAction(period=kill_at, actions=[
            OpaqueFunction(function=_record, args=[events_file, 'kill']),
            # 정리 과정 없이 죽인다 (서버 장애). 로봇은 heartbeat가 끊기는 것으로 알아챈다
            EmitEvent(event=SignalProcess(signal_number=signal.SIGKILL, process_matcher=matches_action(first))),
        ]))
        if restart_after >= 0:
            # 새 프로세스는 epoch_file의 값을 1 올려 쓰므로 로봇이 명령 번호 기준을 초기화한다
            actions.append(TimerAction(period=kill_at + restart_after, actions=[
                OpaqueFunction(function=_record, args=[events_file, 'restart']),
                server(),
            ]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('amr_fleet_manager')

    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm', default_value='nearest',
            description='할당 알고리즘: nearest / load_balance / hungarian'),
        DeclareLaunchArgument(
            'assignments_csv', default_value='',
            description='할당 호출 기록 파일. 비우면 기록하지 않는다 (하네스가 실행마다 경로를 준다)'),
        DeclareLaunchArgument(
            'zones_file', default_value=os.path.join(share, 'config', 'zones.yaml'),
            description='구역 허가 대상 구역 목록'),
        DeclareLaunchArgument(
            'deadlock_strategy', default_value='revoke',
            description='교착 해소 전략: revoke(우선순위 낮은 로봇 허가 회수·후진) / avoid(순환을 닫는 요청 거부)'),
        DeclareLaunchArgument(
            'epoch_file', default_value='/tmp/amr_fleet_manager/epoch',
            description='서버 재시작 번호 파일. 서버가 뜰 때마다 1씩 올린다'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='시뮬레이션 시간을 쓸지. 구역 lease를 로봇과 같은 시계로 재야 하므로 Gazebo와 같이 돌리면 true'),
        DeclareLaunchArgument(
            'resend_interval_s', default_value='1.0',
            description='Ack가 없을 때 할당 명령을 다시 보내는 간격 (s, config 기본 1.0)'),
        DeclareLaunchArgument(
            'reclaim_timeout_s', default_value='5.0',
            description='RobotState가 이 시간(s) 끊기면 그 로봇의 작업을 회수해 다시 할당한다 (config 기본 5.0)'),
        DeclareLaunchArgument(
            'max_resends', default_value='5',
            description='Ack가 없을 때 할당 명령 재전송 한도. 넘으면 작업을 큐로 되돌려 다른 로봇에 맡긴다 (config 기본 5)'),
        DeclareLaunchArgument(
            'kill_at_s', default_value='-1',
            description='launch 시작 후 이 시각(실제 시간, s)에 서버를 SIGKILL. 음수면 죽이지 않는다'),
        DeclareLaunchArgument(
            'restart_after_s', default_value='-1',
            description='죽인 뒤 이 시간(s)이 지나면 서버를 다시 띄운다. 음수면 다시 띄우지 않는다'),
        DeclareLaunchArgument(
            'fault_events_file', default_value='',
            description='강제종료·재시작 시각 기록 파일 (fault_events.csv). 비우면 기록하지 않는다'),
        OpaqueFunction(function=_server),
    ])
