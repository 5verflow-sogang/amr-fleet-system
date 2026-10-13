"""전체 시스템 (1단계): Gazebo 월드 + 로봇 N대(스폰·오도메트리·센서 필터·위치추정·BT) + Fleet 서버·장애 주입기.

    ros2 launch amr_bringup system.launch.py                              # 로봇 5대, 화면 없이, 카메라 끔
    ros2 launch amr_bringup system.launch.py num_robots:=3 gui:=true
    ros2 launch amr_bringup system.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=3 dynamic_obstacles:=true

1단계에는 주행(Nav2), 구역 클라이언트, 인지·안전이 없다. BT의 이동은 아직 가짜(stub)라 로봇이 움직이지 않는다.
각 패키지가 준비되면 인자로 켜는 방식으로 붙인다 (README "단계").

로봇마다 (<ns> = robots.yaml의 이름):
    amr_gazebo spawn_robot      스폰, ROS ↔ Gazebo 브리지. odom TF는 EKF가 내므로 끈다
    amr_odometry odometry       휠 오도메트리 /<ns>/odom (TF 안 냄)
    amr_odometry sensor_filters /<ns>/scan, /<ns>/imu/data
    amr_localization            AMCL + EKF 두 개. map → <ns>/odom → <ns>/base_footprint. 지도 서버는 첫 로봇에서 한 번
    amr_bt                      Fleet 작업 수신·Ack·상태 보고. fleet:=true면 장애 주입기 중계본을 구독한다
전역:
    amr_gazebo world            월드, /clock
    amr_gazebo dynamic_obstacles  (dynamic_obstacles:=true)
    amr_fleet_manager           Fleet 서버 (use_sim_time:=true: 구역 lease를 로봇과 같은 시계로)
    amr_fault_injector          서버 → 로봇 지연·손실 (로봇 목록을 robots.yaml에서 넘긴다)
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup.robots import encoder_seed, load_robots, world_to_map, world_to_map_tf_args


def _launch(package, name, **arguments):
    """다른 패키지 launch를 include한다. 넘긴 인자가 뒤의 include로 새지 않게 GroupAction으로 가둔다."""
    return GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', name)),
        launch_arguments={k: str(v) for k, v in arguments.items()}.items(),
    )])


def _system(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() == 'true'

    robots = load_robots(arg('robots_file'), int(arg('num_robots')))
    origin = [float(v) for v in yaml.safe_load(arg('map_origin_in_world'))]
    seed = int(arg('seed'))
    localization = flag('localization')
    fleet = flag('fleet')
    interval = float(arg('spawn_interval_s'))

    actions = [_launch('amr_gazebo', 'world.launch.py', gui=arg('gui'))]
    if localization:
        # world → map 정적 TF. 지도 원점(map_origin_in_world)을 여기 한 곳에서만 발행하고, world 좌표 설정
        # (zones.yaml 등)을 쓰는 노드는 이 TF로 바꾼다. 원점 값을 노드마다 복사하지 않기 위해서다
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
            arguments=world_to_map_tf_args(origin), parameters=[{'use_sim_time': True}], output='screen'))
    if flag('dynamic_obstacles'):
        actions.append(_launch('amr_gazebo', 'dynamic_obstacles.launch.py', seed=seed, use_sim_time='true'))

    for i, robot in enumerate(robots):
        ns = robot.name
        mx, my, myaw = world_to_map(robot.x, robot.y, robot.yaw, origin)
        stack = [
            _launch('amr_gazebo', 'spawn_robot.launch.py',
                    namespace=ns, x=robot.x, y=robot.y, yaw=robot.yaw,
                    # EKF(ekf_local)가 odom → base_footprint를 내므로 DiffDrive TF는 끈다 (docs/api.md 2-1절)
                    publish_odom_tf='false' if localization else 'true',
                    enable_camera=arg('enable_camera'), payload_mass=arg('payload_mass'), use_sim_time='true'),
            _launch('amr_odometry', 'odometry.launch.py',
                    namespace=ns, encoder_seed=encoder_seed(seed, i), use_sim_time='true'),
            _launch('amr_odometry', 'sensor_filters.launch.py', namespace=ns, use_sim_time='true'),
        ]
        if localization:
            stack.append(_launch(
                'amr_localization', 'localization.launch.py',
                namespace=ns, map=arg('map'), start_map_server='true' if i == 0 else 'false',
                use_imu=arg('use_imu'), initial_x=mx, initial_y=my, initial_yaw=myaw, use_sim_time='true'))
        if flag('bt'):
            stack.append(_launch('amr_bt', 'amr_bt.launch.py',
                                 namespace=ns, relayed='true' if fleet else 'false', use_sim_time='true'))
        # 한꺼번에 스폰하면 Gazebo가 수십 초 동안 크게 느려진다(명세서 11-3절 측정). 로봇마다 시차를 둔다
        actions.append(TimerAction(period=i * interval, actions=[GroupAction(stack)]))

    if fleet:
        actions.append(_launch(
            'amr_fleet_manager', 'fleet_manager.launch.py',
            algorithm=arg('algorithm'), assignments_csv=arg('assignments_csv'), epoch_file=arg('epoch_file'),
            kill_at_s=arg('kill_at_s'), restart_after_s=arg('restart_after_s'),
            fault_events_file=arg('fault_events_file'), use_sim_time='true'))
        injector_config = os.path.join(
            get_package_share_directory('amr_fault_injector'), 'config', 'fault_injector.yaml')
        actions.append(Node(
            package='amr_fault_injector',
            executable='fault_injector',
            parameters=[injector_config, {
                # 로봇 수가 5대가 아니어도 중계 대상이 맞도록 직접 넘긴다
                'robot_ids': [r.name for r in robots],
                'delay_ms': float(arg('delay_ms')),
                'loss_pct': float(arg('loss_pct')),
                'seed': seed,
                'relay_auction': flag('relay_auction'),
            }],
            output='screen',
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('amr_bringup')
    # 지도 v1 (#96, 0.05 m). 0.025 m는 위치 오차에 효과가 없었다 (민지 하네스 측정, 10/8). 매핑 시작 자세가 v0과 같아
    # map_origin_in_world [0, 1.5, 0]은 그대로다
    default_map = os.path.join(get_package_share_directory('amr_mapping'), 'maps', 'warehouse_v1.yaml')

    args = [
        # 로봇·시뮬레이터
        ('num_robots', '5', '로봇 수. robots_file 앞에서부터 쓴다'),
        ('robots_file', os.path.join(share, 'config', 'robots.yaml'), '로봇 이름과 스폰 자세 (world)'),
        ('gui', 'false', 'Gazebo 화면. 러너·CI에서는 false'),
        ('enable_camera', 'false', 'RGBD 카메라. CPU로 5대를 그리면 RTF 0.37로 떨어진다 (명세서 11-3절)'),
        ('dynamic_obstacles', 'false', '동적 장애물 5개'),
        ('spawn_interval_s', '3.0', '로봇 스폰 시차 (실제 시간, s)'),
        ('payload_mass', '0.0', '모든 로봇의 적재물 질량 [kg] (연구 2 Gazebo 평가)'),
        ('seed', '0', '실행 seed. 엔코더 노이즈·동적 장애물·손실 난수에 쓴다. 하네스는 1부터 준다'),
        # 위치추정
        ('localization', 'true', 'AMCL + EKF. 끄면 DiffDrive odom TF를 쓴다'),
        ('map', default_map, '지도 yaml'),
        ('map_origin_in_world', '[0.0, 1.5, 0.0]', 'world에서 본 map 원점 [x, y, yaw]. AMCL 초기 위치 계산'),
        ('use_imu', 'true', 'EKF에 IMU를 넣을지'),
        # 작업 수행
        ('bt', 'true', 'amr_bt (Fleet 작업 수신). 이동은 아직 가짜'),
        # Fleet
        ('fleet', 'true', 'Fleet 서버 + 장애 주입기'),
        ('algorithm', 'nearest', '할당 알고리즘'),
        ('delay_ms', '100.0', '서버 → 로봇 지연 (원 명세 "통신 지연 100ms 반영")'),
        ('loss_pct', '0.0', '서버 → 로봇 손실률'),
        ('relay_auction', 'false', '분산 경매 토픽도 중계 (분산·하이브리드 비교군)'),
        ('assignments_csv', '', '할당 기록 (하네스가 실행 폴더를 준다)'),
        ('epoch_file', '/tmp/amr_fleet_manager/epoch', '서버 재시작 번호 파일'),
        ('kill_at_s', '-1', '서버 강제종료 시각 (launch 시작 기준, 실제 시간). 음수면 끔'),
        ('restart_after_s', '-1', '강제종료 후 재시작까지'),
        ('fault_events_file', '', '강제종료·재시작 시각 기록'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in args]
        + [OpaqueFunction(function=_system)])
