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

import yaml

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
    ('assignment.cost_fn', 'cost_fn'),
    ('assignments_csv', 'assignments_csv'),
    ('traffic.zones_file', 'zones_file'),
    ('traffic.deadlock_strategy', 'deadlock_strategy'),
    ('epoch_file', 'epoch_file'),
    ('charging.stations_file', 'stations_file'),
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
            # extra_params_file(시연별 덮어쓰기, 예: config/fleet_manager_restaurant.yaml)는 기본 설정 위에 얹는다
            parameters=[config, *([arg('extra_params_file')] if arg('extra_params_file') else []), {
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
                # 스테이션 예약 거리 (연구 1 가짜 로봇 실험은 지난 조건대로 0으로 끈다)
                'assignment.station_clearance_m': float(arg('station_clearance_m')),
                # 혼잡도 가중치 (식당 시연 1-1). cost_fn:=congestion일 때만 쓴다
                'assignment.congestion_weight': float(arg('congestion_weight')),
                # 자동 충전 (배터리 인지 운영). 충전기·대기 위치는 stations_file에서 이름으로 고른다
                'charging.enabled': arg('charging').lower() == 'true',
                'charging.low_percent': float(arg('charge_low_percent')),
                'charging.min_in_service': int(arg('min_in_service')),
                'charging.map_origin_in_world': [float(v) for v in yaml.safe_load(arg('map_origin_in_world'))],
                # 피크 대비 (4단계). peaks는 [시작, 끝, ...] (서버 시작부터 s), 비우면 끔
                'charging.peaks': [float(v) for v in (yaml.safe_load(arg('peaks')) or [0.0])],
                'charging.pre_peak_s': float(arg('pre_peak_s')),
                'charging.peak_max_charging': int(arg('peak_max_charging')),
                'charging.peak_target_percent': float(arg('peak_target_percent')),
                # 에너지 예측 배정 (2단계). 소모율은 로봇 쪽과 같게 (가짜 로봇 fake_robots_battery.yaml이면 0.1·0.01·1.0·1.0)
                'energy.enabled': arg('energy_check').lower() == 'true',
                'energy.move_per_m': float(arg('energy_move_per_m')),
                'energy.idle_per_s': float(arg('energy_idle_per_s')),
                'energy.speed_mps': float(arg('energy_speed_mps')),
                'energy.path_factor': float(arg('energy_path_factor')),
                'assignment.battery_weight': float(arg('battery_weight')),
            }, *([{
                # 우선순위 aging 비교용 (설정 파일 값을 이 실행만 바꾼다). 비우면 설정 파일(extra_params_file 포함) 값
                'assignment.aging_rate': float(arg('aging_rate')),
            }] if arg('aging_rate') else []), *([{
                # 구역 밖 대치 양보 켬/끔. 비우면 설정 파일(extra_params_file 포함, 식당은 끔) 값. 연구 1 가짜 로봇 실험은
                # false - 가짜 로봇은 지시를 못 따르고 조건도 바뀐다
                'traffic.standoff.enabled': arg('standoff').lower() == 'true',
            }] if arg('standoff') else [])],
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
            'cost_fn', default_value='euclidean',
            description='할당 비용: euclidean / congestion (직선 거리 + 혼잡 구역 통과 벌점, /fleet/congestion 구독)'),
        DeclareLaunchArgument(
            'congestion_weight', default_value='3.0',
            description='혼잡도 1.0인 구역 1 m를 (1 + 이 값) m로 본다 (cost_fn:=congestion일 때)'),
        DeclareLaunchArgument(
            'charging', default_value='false',
            description='자동 충전 배정: 배터리 낮은 로봇을 빈 충전기로 보내고, 최소 운영 대수를 지키며 동시 충전을 묶는다'),
        DeclareLaunchArgument(
            'charge_low_percent', default_value='30.0',
            description='놀고 있는 로봇이 이보다 낮으면 충전하러 보낸다 (%, charging:=true일 때)'),
        DeclareLaunchArgument(
            'min_in_service', default_value='0',
            description='운영 중 로봇 최소 대수 N. 동시 충전 ≤ 살아 있는 로봇 − N (0이면 충전기 수로만 묶는다)'),
        DeclareLaunchArgument(
            'peaks', default_value='[]',
            description='피크 시간대 [시작, 끝, ...] (서버 시작부터 s). 비우면 피크 대비를 끈다 (charging:=true일 때)'),
        DeclareLaunchArgument(
            'pre_peak_s', default_value='0.0', description='피크 이만큼 전부터 80 % 밑이면 미리 충전 (s, 0이면 끔)'),
        DeclareLaunchArgument(
            'peak_max_charging', default_value='0', description='피크 중 동시 충전 상한 (0이면 최소 운영 대수 규칙만)'),
        DeclareLaunchArgument(
            'peak_target_percent', default_value='100.0', description='피크 중 충전 목표 (%). 닿으면 충전을 끝낸다'),
        DeclareLaunchArgument(
            'energy_check', default_value='false',
            description='에너지 예측 배정: 작업을 끝내고 가장 가까운 충전기까지 갈 배터리가 없는 로봇은 그 작업에서 뺀다'),
        DeclareLaunchArgument('energy_move_per_m', default_value='0.05', description='이동 1 m당 소모 (%, 로봇과 같게)'),
        DeclareLaunchArgument('energy_idle_per_s', default_value='0.002', description='정지 소모 (%/s, 로봇과 같게)'),
        DeclareLaunchArgument('energy_speed_mps', default_value='0.5', description='평균 주행 속도 (m/s)'),
        DeclareLaunchArgument(
            'energy_path_factor', default_value='1.3', description='직선 거리 → 경로 길이 비율 (가짜 로봇 1.0)'),
        DeclareLaunchArgument(
            'battery_weight', default_value='0.0',
            description='비용 + 이 값 × (필요 배터리 / 남은 배터리) [m]. 배터리가 많은 로봇이 먼 작업을 맡는다. 0이면 끔'),
        DeclareLaunchArgument(
            'stations_file', default_value=os.path.join(share, 'config', 'stations.yaml'),
            description='충전기(charger_*)·대기 위치(wait_*) 스테이션 파일 (world 좌표). 식당은 stations_restaurant.yaml'),
        DeclareLaunchArgument(
            'map_origin_in_world', default_value='[-0.0439, 1.4712, -0.00088]',
            description='world에서 본 map 원점 [x, y, yaw] (작업 생성기와 같게). 식당은 [0, 0, 0]'),
        DeclareLaunchArgument(
            'assignments_csv', default_value='',
            description='할당 호출 기록 파일. 비우면 기록하지 않는다 (하네스가 실행마다 경로를 준다)'),
        DeclareLaunchArgument(
            'aging_rate', default_value='',
            description='우선순위 aging 비율 (s당). 비우면 설정 파일 값 (기본 0.0, 식당 1.667). 비교 실험에서만 준다'),
        DeclareLaunchArgument(
            'extra_params_file', default_value='',
            description='fleet_manager.yaml 위에 얹을 파라미터 파일 (식당 시연: config/fleet_manager_restaurant.yaml). 비우면 없음'),
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
            'station_clearance_m', default_value='1.0',
            description='스테이션 예약 거리 (m). 맡겨진 작업의 스테이션 근처를 쓰는 작업은 끝날 때까지 미룬다. 0이면 끔 (config 기본 1.0)'),
        DeclareLaunchArgument(
            'standoff', default_value='',
            description='구역 밖 대치 양보 켬/끔 (true/false). 비우면 설정 파일 값 (fleet_manager.yaml 켬, 식당 끔). 가짜 로봇 실험은 false'),
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
