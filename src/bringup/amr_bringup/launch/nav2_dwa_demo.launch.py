"""amr_nav2_plugins::DwaController 증명용 데모: Gazebo 월드에서 목표점 1곳으로 주행.
controller:=teb로 TEB와도 바꿔 띌 수 있다 ((10) DWA vs TEB 비교, navigation.md 6절).

로봇 1대(네임스페이스 amr_01). nav2_params.yaml은 네임스페이스를 가정하지 않으므로
(CONTRIBUTING 4-5절), TF 프레임 이름이 들어가는 파라미터(costmap의 robot_base_frame,
local costmap의 global_frame=odom)는 nav2_common.launch.RewrittenYaml로 덮어쓴다
(민지 #40 약속). planner_server·controller_server는 각자 자기 costmap 섹션만 읽으므로
(global_costmap/local_costmap은 같은 파일 안에 있어도 서로의 파라미터를 보지 않는다),
같은 "global_frame" 키라도 두 노드에 서로 다른 재작성 파일을 따로 만들어 주면 된다 -
한 파일로 둘 다 맞추려 하면 global_costmap(map, 공유)과 local_costmap(odom, 로봇별)이
충돌한다.

지도 v1(warehouse_v1.yaml, #103 민지 제안)과 DiffDrive 이상적 odom을 그대로 쓰고,
map->odom은 AMCL/EKF 없이 정지 변환(항등)으로 둔다 - 지도 원점이 스폰 위치와 같아서
(v0과 매핑 시작 자세가 같아 map_origin_in_world [0, 1.5, 0]은 그대로다, amr_mapping
README) 짧은 데모에는 충분하고, AMCL/EKF 없이 컨트롤러 플러그인 자체를 빨리 검증할 수
있다. LiDAR 스캔 필터(amr_odometry/sensor_filters)도 띄운다 - 안 띄우고 scan_raw를
바로 obstacle_layer에 물렸더니 로봇을 2x2m 네모 모양의 lethal/inscribed 셀이 거의
완전히 둘러싸 어느 쪽으로도 못 나가는 현상이 있었다(global_costmap에서 확인,
local_costmap도 같은 raw 토픽을 보니 DWA가 "Failed to make progress"로 멈추는 원인도
같았다). 아웃라이어 제거(scan_filter) 뒤의 /<ns>/scan을 구독하면 없어진다 - scan_topic 참고.

global costmap은 지도 static_layer를 그대로 쓴다. 한때 "NavFn이 큰 지도(60x40m)에서
멈춘다"고 보고 로봇 중심 rolling_window로 바꿔봤었는데, 진짜 원인은 그게 아니라
obstacle_layer가 구독하는 토픽 이름이 실제 LiDAR 토픽과 달라 costmap이 영영 준비되지
않던 것이었다(scan_topic 참고) - 고치고 나니 static_layer로도 바로 된다. 자세한 경위는
docs/algorithms/navigation.md "DWA·Nav2 연결" 절 참고.

    ros2 launch amr_bringup nav2_dwa_demo.launch.py
    ros2 launch amr_bringup nav2_dwa_demo.launch.py controller:=teb
    ros2 launch amr_bringup nav2_dwa_demo.launch.py gui:=true goal_x:=3.0 goal_y:=1.5
    ros2 launch amr_bringup nav2_dwa_demo.launch.py goal_yaw:=90.0
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('amr_bringup')
    gazebo_launch_dir = os.path.join(get_package_share_directory('amr_gazebo'), 'launch')
    odometry_launch_dir = os.path.join(get_package_share_directory('amr_odometry'), 'launch')
    map_yaml = os.path.join(
        get_package_share_directory('amr_mapping'), 'maps', 'warehouse_v1.yaml')
    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    ns = 'amr_01'
    base_footprint = f'{ns}/base_footprint'
    odom_frame = f'{ns}/odom'

    # controller:=dwa(기본)|teb - (10) DWA vs TEB 비교. nav2_params.yaml의 컨트롤러 이름
    # ("DWA"·"teb")별 설정 블록 키 - 아래에서 그 블록 하나를 고른다.
    controller = LaunchConfiguration('controller').perform(context)
    controller_key = {'dwa': 'DWA', 'teb': 'teb'}.get(controller)
    if controller_key is None:
        raise ValueError(f"controller는 'dwa' 또는 'teb'여야 한다: {controller!r}")

    # bt_navigator 기본 BT(커스텀 xml을 안 줬다)는 컨트롤러를 "FollowPath"라는 이름으로
    # 부른다(하드코딩) - nav2_params.yaml에는 "FollowPath"(DWB)·"DWA"·"teb" 다 있지만, 이
    # 데모는 고른 것 하나만 올리려고 controller_plugins를 좁힌다. 그러면 BT가 찾는
    # "FollowPath"가 목록에서 사라져 매번 "FollowPath ... does not exist. Available
    # controllers are: DWA."로 바로 Aborting된다. DWB(dwb_core 추가 의존성)·다른 쪽은 안
    # 띄우려는 의도를 유지하면서, 고른 컨트롤러 설정을 "FollowPath"라는 이름으로도 등록해
    # BT가 그대로 찾게 한다.
    with open(nav2_params) as f:
        chosen_config = yaml.safe_load(f)['controller_server']['ros__parameters'][controller_key]
    controller_overrides = {ns: {'controller_server': {'ros__parameters': {
        'controller_plugins': ['FollowPath'],
        'FollowPath': chosen_config,
    }}}}
    controller_overrides_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(controller_overrides, controller_overrides_file)
    controller_overrides_file.close()

    # obstacle_layer가 구독하는 "scan"(상대 토픽)은 로봇 네임스페이스 밑으로만 붙고(예:
    # amr_01/global_costmap/scan), Gazebo LiDAR 브리지가 실제로 내는 토픽은 그 이름이
    # 아니라 절대 경로 /<ns>/scan_raw다(amr_gazebo/launch/spawn_robot.launch.py의 gz_bridge) -
    # 그래서 observation source가 평생 비어서 costmap이 isCurrent()가 안 되고, planner_server
    # ·controller_server가 전부 그걸 기다리며 조용히 멈춰 있었다(ComputePathToPose가 costmap
    # 준비를 기다림). 레이어를 빼는 대신(DWA가 장애물을 봐야 한다) 절대 토픽으로 맞춰 구독한다.
    #
    # scan_raw를 바로 물리면 아웃라이어 때문으로 보이는 lethal 셀이 로봇을 네모로 둘러싸
    # 전혀 못 움직였다 - amr_odometry/sensor_filters(scan_filter)가 걸러낸 /<ns>/scan을 쓴다.
    scan_topic = f'/{ns}/scan'

    # global_costmap 재작성: robot_base_frame은 항상 필요하다(공유 map, 로봇 프레임만
    # 다름). map_topic은 절대 경로로 맞춘다(map_server가 네임스페이스 없이 뜬다).
    global_costmap_rewrites = {
        'robot_base_frame': base_footprint, 'topic': scan_topic, 'map_topic': '/map',
    }

    # planner_server는 global_costmap만 읽는다. controller_server는 local_costmap만 읽는다
    # (odom은 로봇마다 다른 좌표계라 global_frame도 같이 바꿔야 한다). 둘 다 같은 원본 파일
    # (nav2_params.yaml)에서 따로 만들어서, 서로 안 읽는 섹션이 잘못 바뀌어도 상관없다.
    planner_params = RewrittenYaml(
        source_file=nav2_params, root_key=ns,
        param_rewrites=global_costmap_rewrites,
        convert_types=True)
    controller_params = RewrittenYaml(
        source_file=nav2_params, root_key=ns,
        param_rewrites={
            'robot_base_frame': base_footprint, 'global_frame': odom_frame,
            'map_topic': '/map', 'topic': scan_topic,
        },
        convert_types=True)

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    # 최종 접근(제자리 회전) 테스트용. 도(度) 단위로 받아 여기서 바로 쿼터니언으로
    # 바꾼다 - goal_x/y처럼 LaunchConfiguration을 ExecuteProcess cmd에 그대로 넘기면
    # 실행 시점에야 문자열로 풀리는데, sin/cos는 지금(launch 설정 시점) 계산해야 한다.
    goal_yaw_deg = float(LaunchConfiguration('goal_yaw').perform(context))
    goal_yaw_rad = math.radians(goal_yaw_deg)
    goal_qz = math.sin(goal_yaw_rad / 2.0)
    goal_qw = math.cos(goal_yaw_rad / 2.0)
    use_sim_time = {'use_sim_time': True}

    planner_parameters = [planner_params, use_sim_time, {'GridBased.use_astar': True}]

    lifecycle_nodes_nav = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']

    # send_goal:=false로 끄면 이 launch는 목표를 안 보낸다 - (10) 실험 스크립트가 액션
    # 클라이언트로 직접 보내 구조화된 결과(성공·시각)를 받을 때 쓴다(로그 문자열 grep 대신).
    # condition은 TimerAction이 아니라 안쪽 ExecuteProcess에 건다 - TimerAction(condition=...)은
    # 타이머 자체가 조건과 상관없이 등록되고 나중에 안쪽 action들을 조건 없이 실행해 버려서
    # send_goal:=false로도 목표가 나가는 문제가 있었다(실험 중 발견, (10) 작업).
    send_goal = TimerAction(period=15.0, actions=[ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal', f'/{ns}/navigate_to_pose',
            'nav2_msgs/action/NavigateToPose',
            [
                '{pose: {header: {frame_id: map}, pose: {position: {x: ', goal_x,
                ', y: ', goal_y, ', z: 0.0}, orientation: {z: ', str(goal_qz),
                ', w: ', str(goal_qw), '}}}}',
            ],
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('send_goal')),
    )])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'sim.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'namespace': ns,
                'enable_camera': 'false',
                'dynamic_obstacles': 'false',
                'publish_odom_tf': 'true',  # DiffDrive 이상적 odom -> base_footprint TF
            }.items(),
        ),

        # scan_raw -> scan (아웃라이어 제거). costmap의 obstacle_layer가 구독하는 scan_topic이
        # 바로 이 출력이다 - 위 docstring·scan_topic 주석 참고.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(odometry_launch_dir, 'sensor_filters.launch.py')),
            launch_arguments={'namespace': ns, 'use_sim_time': 'true'}.items(),
        ),

        # map -> <ns>/odom: AMCL/EKF 없이 정지 변환(항등) (데모 전용, 위 docstring 참고)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
            parameters=[use_sim_time],
            output='screen',
        ),

        # 지도는 로봇과 상관없이 공유라 네임스페이스를 안 붙인다 (절대 토픽 /map).
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': map_yaml, **use_sim_time}], output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server'], **use_sim_time}],
            output='screen'),

        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            namespace=ns,
            parameters=planner_parameters,
            output='screen'),
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            namespace=ns,
            parameters=[
                controller_params, use_sim_time,
                # DiffDrive 이상적 odom은 odom이 아니라 diff_drive/odom으로 브리지된다
                # (spawn_robot.launch.py). 이 데모는 DwaController 자체를 보는 게 목적이라
                # DWB baseline(dwb_core 등 추가 의존성)은 안 띄우고 DWA만 올린다 - BT가 찾는
                # 이름("FollowPath")으로 등록하는 덮어쓰기는 위 controller_overrides_file 참고.
                {'odom_topic': 'diff_drive/odom'},
                controller_overrides_file.name,
            ],
            output='screen'),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            namespace=ns,
            # bt_navigator 기본 BT가 spin/backup/wait 복구 행동을 참조한다 - 없으면 BT 자체를
            # 못 읽어 bringup이 멈춘다. 이 데모는 장애물 회피 자체가 목적이 아니라 기본값만 준다.
            parameters=[{'robot_base_frame': base_footprint, 'global_frame': odom_frame,
                        **use_sim_time}],
            output='screen'),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            namespace=ns,
            parameters=[{'global_frame': 'map', 'robot_base_frame': base_footprint,
                        **use_sim_time}],
            output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=ns,
            parameters=[{'autostart': True, 'node_names': lifecycle_nodes_nav, **use_sim_time}],
            output='screen'),

        # Gazebo·map_server·Nav2 lifecycle이 active로 올라올 시간을 준 뒤 목표를 한 번 보낸다
        send_goal,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo GUI 실행 여부'),
        DeclareLaunchArgument(
            'controller', default_value='dwa',
            description="로컬 컨트롤러 'dwa' 또는 'teb' ((10) DWA vs TEB 비교)"),
        DeclareLaunchArgument('goal_x', default_value='3.0', description='목표 x [m] (map 기준)'),
        DeclareLaunchArgument('goal_y', default_value='1.5', description='목표 y [m] (map 기준)'),
        DeclareLaunchArgument(
            'goal_yaw', default_value='0.0',
            description='목표 각도 [도]. 최종 접근(제자리 회전) 확인용'),
        DeclareLaunchArgument(
            'send_goal', default_value='true',
            description='15초 뒤 목표를 이 launch가 보낼지. false면 (10) 실험 스크립트처럼 '
                        '액션 클라이언트로 직접 보내는 쪽이 대신 보낸다'),
        OpaqueFunction(function=_launch_setup),
    ])
