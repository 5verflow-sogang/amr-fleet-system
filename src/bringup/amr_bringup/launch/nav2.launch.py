"""로봇 1대의 Nav2 (경로 계획 planner_server, 주행 controller_server, behavior_server, bt_navigator).

system.launch.py가 로봇마다 부른다. 월드·지도·위치추정은 안 띄운다 (map → <ns>/odom TF는 AMCL + EKF가 낸다).

    ros2 launch amr_bringup nav2.launch.py namespace:=amr_02

nav2_params.yaml은 네임스페이스를 가정하지 않는다 (CONTRIBUTING 4-5절). 프레임 이름이 들어가는 값은 RewrittenYaml로 덮어쓴다.
planner_server는 global_costmap만, controller_server는 local_costmap만 읽으므로 둘에 서로 다른 재작성 파일을 준다
(global_frame이 map / <ns>/odom으로 다르다). 자세한 경위는 nav2_dwa_demo.launch.py와 docs/algorithms/navigation.md "DWA·Nav2 연결".

컨트롤러: bt_navigator 기본 트리가 "FollowPath"라는 이름을 하드코딩해서 부르므로, nav2_params.yaml의 DWA 설정을 FollowPath 이름으로 등록한다.
속도 명령은 cmd_vel이 아니라 cmd_vel_nav로 낸다 (cmd_vel 체인: cmd_vel_nav → 구역 클라이언트 → cmd_vel_zone_limited → 안전 노드 → cmd_vel).
전역 planner: 기본은 GridBased(NavFn). planner:=astar면 우리 A*(nav2_params.yaml planner_server.AStar)를 GridBased 이름에
얹는다 - 비교용 (amr_bringup.planner, docs/algorithms/navigation.md 3절).

nav2_profile:=restaurant면 footprint를 서빙 로봇 크기(#299)로, congestion_filter:=keepout|speed면 전역
costmap에 민지 혼잡도 노드(#296)용 KeepoutFilter/SpeedFilter를 붙인다 (amr_bringup.nav2_profile). 둘 다
기본(warehouse·none)은 바뀌는 게 없어 지금 물류센터 동작 그대로다 - nav2_params.yaml을 복사하지 않고
controller_server override와 같은 방식(바뀌는 키만 임시 yaml에 더해 얹는다)으로 적용한다.

nav2_profile:=restaurant는 bt_navigator의 default_nav_to_pose_bt_xml도 nav2 제공 1초 재계획 BT
(navigate_to_pose_w_replanning_and_recovery.xml)로 되돌린다. #292가 bt_navigator 기본 트리를
"막혔을 때만 재계획" BT로 바꾸는데, 식당 혼잡 비용(229)으로는 그 BT가 재계획을 안 해 1-2 우회가
안 나온다(로컬 비교 3회씩, #292 PR 댓글). warehouse는 #292가 정한 기본값을 그대로 쓴다.

이 경로는 amr_bringup.nav2_profile(ROS 비의존, 문자열만 다룬다)이 아니라 여기서 바로
FindPackageShare(실제 launch 실행 때만 nav2_bt_navigator를 찾는 launch substitution)로 만든다 -
CI는 exec_depend(nav2_bt_navigator 포함)를 안 설치하므로(.github/workflows/ci.yml, 무거운 런타임
의존성을 줄이려는 의도), get_package_share_directory로 즉시(Python 레벨에서) 찾으면
test_nav2_launch.py가 _nav2()를 직접 호출할 때 PackageNotFoundError로 죽는다(#307 CI 실패).
restaurant일 때만 이 substitution을 bt_navigator Node parameters 맨 뒤에 추가해서, warehouse는
애초에 찾지도 않고 restaurant는 실제 ros2 launch가 그 Node를 띄우는 순간에만 찾는다(#292가 그
앞에 dict로 넣을 값보다 이 값이 이긴다 - ROS2 Node parameters는 나중 소스가 이긴다).
"""

import os
import tempfile

import yaml
from amr_bringup.nav2_profile import CONGESTION_FILTERS, PROFILES, build_overrides
from amr_bringup.planner import PLANNERS, planner_overrides
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
# lifecycle_manager가 관리 노드의 생존 신호(bond)를 기다리는 시간. 기본 4 s면 5대가 함께 뜰 때 CPU가 밀려
# 살아 있는 노드를 죽었다고 보고 꺼 버린다 (11/6: 식당·fleet_stack 실패 19건 중 17건이 이 기동 실패, 16코어에서도)
BOND_TIMEOUT_S = 30.0


def _nav2(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = arg('namespace')
    use_sim_time = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    odom_topic = arg('odom_topic')
    cmd_vel_topic = arg('cmd_vel_topic')
    params_file = arg('params_file')
    base_footprint = f'{ns}/base_footprint'
    odom_frame = f'{ns}/odom'
    scan_topic = f'/{ns}/scan'

    with open(params_file) as f:
        nav2_params = yaml.safe_load(f)
    planner_params_extra = planner_overrides(nav2_params, arg('planner'))
    # SpeedFilter 기본 토픽은 costmap 자신의 네임스페이스(<ns>/global_costmap/speed_limit)인데
    # controller_server는 <ns>/speed_limit을 구독해서 절대 경로로 맞춰 줘야 만난다(실측 확인 -
    # 안 맞추면 SpeedFilter publisher count 0, DWA가 평생 제한을 못 받는다)
    overrides = {ns: build_overrides(
        nav2_params, arg('nav2_profile'), arg('congestion_filter'), odom_topic,
        speed_limit_topic=f'/{ns}/speed_limit')}
    # local_costmap 기준 프레임. 따로 뜰 때는 controller_params 재작성과 같은 값이고, 컴포지션(한 프로세스에 지도 기준
    # 파일 하나)일 때는 이것만 local_costmap을 <ns>/odom으로 맞춘다
    overrides[ns]['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = odom_frame
    # 로봇마다 다른 임시 파일이 필요하다 (프로세스가 끝나면 OS가 지운다)
    overrides_file = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'nav2_{ns}_', suffix='.yaml', delete=False)
    yaml.dump(overrides, overrides_file)
    overrides_file.close()

    planner_params = RewrittenYaml(
        source_file=params_file, root_key=ns,
        param_rewrites={'robot_base_frame': base_footprint, 'topic': scan_topic, 'map_topic': '/map'},
        convert_types=True)
    controller_params = RewrittenYaml(
        source_file=params_file, root_key=ns,
        param_rewrites={'robot_base_frame': base_footprint, 'global_frame': odom_frame,
                        'map_topic': '/map', 'topic': scan_topic},
        convert_types=True)

    # 컨트롤러와 복구 행동(spin/backup)이 낸 속도는 모두 cmd_vel_nav로 모은다
    cmd_vel_remap = [('cmd_vel', cmd_vel_topic)]

    # restaurant일 때만 만든다 - FindPackageShare는 생성만 해도 아무 것도 안 찾지만(evaluate() 때만
    # 찾는다), warehouse에서는 아예 만들지도 않아 "warehouse는 nav2_bt_navigator를 찾지 않는다"가
    # 코드로도 분명하다
    restaurant_bt_override = []
    if arg('nav2_profile') == 'restaurant':
        restaurant_bt_override.append({'default_nav_to_pose_bt_xml': PathJoinSubstitution([
            FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'])})

    # (패키지, 실행 파일, 컴포넌트 플러그인, 노드 이름, 파라미터, 리맵)
    servers = [
        ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', 'planner_server',
         [planner_params, use_sim_time, {'GridBased.use_astar': True}, planner_params_extra, overrides_file.name], []),
        ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer', 'controller_server',
         [controller_params, use_sim_time, overrides_file.name], cmd_vel_remap),
        ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer', 'behavior_server',
         [{'robot_base_frame': base_footprint, 'global_frame': odom_frame, **use_sim_time}], cmd_vel_remap),
        # bt_loop_duration 50ms(20Hz) - 기본 10ms(100Hz)는 5대 통합 실행에서 "tick rate 100
        # exceeded" 경고가 1,400번 나오고 로봇당 CPU를 0.25코어씩 더 썼다.
        # restaurant_bt_override를 맨 뒤에 둬서(비어 있지 않을 때만) #292가 넣을 dict보다 이긴다
        ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator',
         [{'global_frame': 'map', 'robot_base_frame': base_footprint,
           'odom_topic': odom_topic, 'bt_loop_duration': 50, **use_sim_time}, *restaurant_bt_override], []),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager',
         'lifecycle_manager_navigation',
         [{'autostart': True, 'node_names': LIFECYCLE_NODES, 'bond_timeout': BOND_TIMEOUT_S, **use_sim_time}], []),
    ]

    if arg('use_composition').lower() != 'true':
        return [Node(package=pkg, executable=exe, name=name, namespace=ns, parameters=params,
                     remappings=remaps, output='screen')
                for pkg, exe, _, name, params, remaps in servers]

    # 컴포지션: 5개를 프로세스 하나로 (DDS 참가자 5 → 1). 식당 시연 5대에 추적을 켜면 프로세스가 120개를 넘어
    # 뒤늦게 뜬 노드가 다른 노드를 못 찾았다 (11/6 demo_smoke track: 녹화 하네스가 /clock을 못 받음).
    # costmap 노드는 컴포넌트 파라미터가 아니라 프로세스 --params-file을 읽으므로 컨테이너에 파일을 준다
    # (nav2_bringup과 같은 방식). 파일이 하나라 지도 기준(planner_params)으로 주고 local_costmap은 overrides가 맞춘다
    return [ComposableNodeContainer(
        name='nav2_container', namespace=ns, package='rclcpp_components',
        executable='component_container_isolated', output='screen',
        parameters=[planner_params, use_sim_time, overrides_file.name],
        composable_node_descriptions=[
            ComposableNode(package=pkg, plugin=plugin, name=name, namespace=ns, remappings=remaps,
                           parameters=[planner_params if p is controller_params else p for p in params])
            for pkg, _, plugin, name, params, remaps in servers])]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('amr_bringup'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Nav2 파라미터'),
        DeclareLaunchArgument(
            'odom_topic', default_value='odometry/local',
            description='컨트롤러가 속도를 읽는 odom (상대 이름). 위치추정 없이는 diff_drive/odom'),
        DeclareLaunchArgument(
            'planner', default_value='navfn', choices=list(PLANNERS),
            description='전역 planner. navfn(기본) 또는 astar(우리 A* + 평활화, 비교용)'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel_nav',
            description='Nav2 속도 명령 출력. 구역 클라이언트·안전 노드가 없으면 cmd_vel'),
        DeclareLaunchArgument(
            'nav2_profile', default_value='warehouse', choices=list(PROFILES),
            description='footprint 등 로봇/장소 프로필. restaurant면 서빙 로봇 크기(#299)'),
        DeclareLaunchArgument(
            'congestion_filter', default_value='none', choices=list(CONGESTION_FILTERS),
            description='전역 costmap에 붙일 혼잡도 필터(#296). keepout(돌아가기)·speed(느려짐)'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='Nav2 서버 5개를 컨테이너 프로세스 하나로 (DDS 참가자를 줄인다, 식당 시연 녹화)'),
        OpaqueFunction(function=_nav2),
    ])
