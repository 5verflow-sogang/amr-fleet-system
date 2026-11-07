"""system.launch.py를 불러 인자와 로봇별 액션이 만들어지는지 본다. 설치된 ROS 워크스페이스(CI)에서만 돈다."""

import importlib.util
import os

import pytest

# ROS 환경에서만 있는 모듈로 판단한다 (이름이 같은 다른 'launch' 패키지가 있을 수 있다)
pytest.importorskip('launch_ros')
pytest.importorskip('ament_index_python')
import launch  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.actions import IncludeLaunchDescription, TimerAction  # noqa: E402
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

# 이 launch가 include하는 파일. 이름이 바뀌면 여기서 먼저 깨진다
INCLUDED = [
    ('amr_gazebo', 'world.launch.py'),
    ('amr_gazebo', 'dynamic_obstacles.launch.py'),
    ('amr_gazebo', 'spawn_robot.launch.py'),
    ('amr_odometry', 'odometry.launch.py'),
    ('amr_odometry', 'sensor_filters.launch.py'),
    ('amr_localization', 'localization.launch.py'),
    ('amr_localization', 'ground_truth_localization.launch.py'),
    ('amr_bt', 'amr_bt.launch.py'),
    ('amr_fleet_manager', 'fleet_manager.launch.py'),
    ('amr_fleet_manager', 'congestion.launch.py'),
    ('amr_bringup', 'nav2.launch.py'),
    ('amr_zone_client', 'zone_client.launch.py'),
    ('amr_tracking', 'tracking.launch.py'),
    ('amr_safety', 'safety.launch.py'),
    ('amr_detection', 'detection.launch.py'),
    ('amr_docking', 'docking.launch.py'),
    ('amr_docking', 'aruco_markers.launch.py'),
]


def load_module():
    path = os.path.join(get_package_share_directory('amr_bringup'), 'launch', 'system.launch.py')
    spec = importlib.util.spec_from_file_location('system_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with_defaults(module, **overrides):
    context = launch.LaunchContext()
    for declared in module.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(context, declared.default_value)
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    return context


def test_included_launch_files_exist():
    for package, name in INCLUDED:
        assert os.path.exists(os.path.join(get_package_share_directory(package), 'launch', name)), (package, name)


@pytest.mark.parametrize('num_robots', [1, 5])
def test_actions_per_robot(num_robots):
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=num_robots))
    # 월드 1 + world → map TF Node 1 + 로봇마다 TimerAction 1 + Fleet 서버 include 1 + 장애 주입기 Node 1
    assert len(actions) == 1 + 1 + num_robots + 2
    assert sum(isinstance(a, TimerAction) for a in actions) == num_robots
    assert sum(isinstance(a, Node) for a in actions) == 2


def test_options_off():
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=2, fleet='false', dynamic_obstacles='true'))
    # 월드 + world → map TF + 동적 장애물 + 로봇 2대, Fleet 없음 (Node는 TF 하나)
    assert len(actions) == 1 + 1 + 1 + 2
    assert sum(isinstance(a, Node) for a in actions) == 1


def test_world_to_map_tf_only_with_localization():
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=1, fleet='false', localization='off'))
    assert not any(isinstance(a, Node) for a in actions)


def test_world_to_map_tf_published_for_ground_truth_too():
    # ground_truth도 world → map TF가 필요하다 (ground_truth_localization_node가 정답을 map 기준으로 바꿀 때 같은 원점을 씀)
    module = load_module()
    actions = module._system(context_with_defaults(
        module, num_robots=1, fleet='false', localization='ground_truth'))
    assert any(isinstance(a, Node) for a in actions)



@pytest.mark.parametrize('old, has_tf', [('true', True), ('True', True), ('false', False), ('False', False)])
def test_old_boolean_localization_values_still_launch(old, has_tf):
    # localization이 true/false → amcl/ground_truth/off로 바뀌었다. 예전 값은 true = amcl, false = off
    module = load_module()
    declared = {a.name: a for a in module.generate_launch_description().get_launch_arguments()}
    assert old in declared['localization'].choices
    actions = module._system(context_with_defaults(module, num_robots=1, fleet='false', localization=old))
    assert any(isinstance(a, Node) for a in actions) == has_tf   # world → map TF는 위치추정이 있을 때만

def test_stage_flags_do_not_change_robot_count():
    # 2·3단계를 켜도 로봇마다 TimerAction 하나 (스택 내용은 test_stack.py가 ROS 없이 본다)
    module = load_module()
    actions = module._system(context_with_defaults(
        module, num_robots=2, nav2=True, zone_client=True, tracking=True, safety=True))
    assert sum(isinstance(a, TimerAction) for a in actions) == 2


def test_nav2_needs_localization():
    module = load_module()
    with pytest.raises(RuntimeError, match='localization'):
        module._system(context_with_defaults(module, nav2=True, localization='off'))


@pytest.mark.parametrize('name', ['detection', 'docking'])
def test_camera_features_need_camera(name):
    module = load_module()
    with pytest.raises(RuntimeError, match='enable_camera'):
        module._system(context_with_defaults(module, nav2=True, **{name: True}))


def test_docking_needs_nav2():
    # nav2:=false면 BT 이동이 자리채움이라 로봇이 스테이션에 가지 않는다
    module = load_module()
    with pytest.raises(RuntimeError, match='nav2'):
        module._system(context_with_defaults(module, docking=True, enable_camera=True))


def test_docking_spawns_markers_once():
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=3, nav2=True, docking=True,
                                                                  enable_camera=True))
    # 로봇마다 TimerAction 1 + 마커판 TimerAction 1
    assert sum(isinstance(a, TimerAction) for a in actions) == 3 + 1


def include_args(context, actions, key):
    """actions(로봇 스택 TimerAction 제외) 중 인자 key를 받는 include의 인자 {이름: 값}.

    include 경로는 Substitution으로만 들고 있어(launch Humble) 인자 이름으로 찾는다.
    """
    for action in actions:
        for sub in getattr(action, 'get_sub_entities', lambda: [])():
            if isinstance(sub, IncludeLaunchDescription):
                args = {perform_substitutions(context, normalize_to_list_of_substitutions(k)):
                        perform_substitutions(context, normalize_to_list_of_substitutions(v))
                        for k, v in sub.launch_arguments}
                if key in args:
                    return args
    raise AssertionError(f'{key} 인자를 받는 include 없음')


def test_restaurant_args_reach_world_obstacles_and_fleet():
    # 식당 5대 혼잡도 켬/끔 측정(작업 5): 월드, 손님 파일(config/ 아래 이름), Fleet 할당 비용이 하위 launch에 닿는다
    module = load_module()
    context = context_with_defaults(
        module, num_robots=1, world='restaurant.sdf', dynamic_obstacles=True,
        dynamic_obstacles_file='dynamic_obstacles_restaurant_peak.yaml', cost_fn='congestion', congestion_weight=2.0)
    actions = module._system(context)
    assert include_args(context, actions, 'world')['world'] == 'restaurant.sdf'
    config = include_args(context, actions, 'config')['config']
    assert config == os.path.join(get_package_share_directory('amr_gazebo'), 'config',
                                  'dynamic_obstacles_restaurant_peak.yaml')
    assert os.path.exists(config)
    fleet = include_args(context, actions, 'cost_fn')
    assert (fleet['cost_fn'], fleet['congestion_weight']) == ('congestion', '2.0')
    # 기본값은 물류센터 그대로 (손님 파일을 비우면 dynamic_obstacles.launch.py 기본 파일)
    context = context_with_defaults(module, num_robots=1, dynamic_obstacles=True)
    actions = module._system(context)
    assert include_args(context, actions, 'world')['world'] == 'warehouse.sdf'
    assert include_args(context, actions, 'config')['config'] == ''
    assert include_args(context, actions, 'cost_fn')['cost_fn'] == 'euclidean'
    assert include_args(context, actions, 'cost_fn')['extra_params_file'] == ''


def test_fleet_params_file_reaches_fleet_manager():
    # 식당 장면 2(회수 + aging): 시연 설정 덮어쓰기가 Fleet 서버 launch에 닿는다. 상대 이름은 amr_fleet_manager config/
    module = load_module()
    context = context_with_defaults(module, num_robots=1, fleet_params_file='fleet_manager_restaurant.yaml')
    actions = module._system(context)
    assert include_args(context, actions, 'cost_fn')['extra_params_file'] == os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'config', 'fleet_manager_restaurant.yaml')


def test_congestion_node_uses_spawned_guests():
    # 혼잡도 노드는 실제로 스폰한 손님 파일과 지도 원점으로 센다 (식당 시연 "명령 한 줄")
    module = load_module()
    context = context_with_defaults(
        module, num_robots=1, dynamic_obstacles=True, congestion='restaurant', map_origin_in_world='[0, 0, 0]',
        dynamic_obstacles_file='dynamic_obstacles_restaurant_peak.yaml')
    actions = module._system(context)
    congestion = include_args(context, actions, 'site')
    assert congestion['site'] == 'restaurant'
    assert congestion['obstacles_file'] == include_args(context, actions, 'config')['config']
    assert congestion['map_origin_in_world'] == '[0, 0, 0]'
    # 구역 클라이언트용 zones_file이 새어 들어가지 않게 비워 넘긴다 (혼잡도 노드는 site 기본 구역)
    assert congestion['zones_file'] == ''
    # 기본은 혼잡도 노드 없음
    context = context_with_defaults(module, num_robots=1)
    with pytest.raises(AssertionError, match='site'):
        include_args(context, module._system(context), 'site')


def test_staff_spawned_separately_and_not_counted():
    # 직원은 두 번째 동적 장애물 launch(이름 staff_obstacles)로 띄우고, 혼잡도 노드는 손님 파일만 센다
    module = load_module()
    context = context_with_defaults(
        module, num_robots=1, dynamic_obstacles=True, congestion='restaurant',
        dynamic_obstacles_file='dynamic_obstacles_restaurant_peak.yaml',
        staff_file='dynamic_obstacles_restaurant_staff.yaml')
    actions = module._system(context)
    staff = include_args(context, actions, 'name')
    assert staff['name'] == 'staff_obstacles'
    assert staff['config'].endswith('config/dynamic_obstacles_restaurant_staff.yaml') and os.path.exists(staff['config'])
    assert include_args(context, actions, 'site')['obstacles_file'].endswith('dynamic_obstacles_restaurant_peak.yaml')
    # 기본은 직원 없음
    context = context_with_defaults(module, num_robots=1)
    with pytest.raises(AssertionError, match='name'):
        include_args(context, module._system(context), 'name')


def test_camera_robot_names_checked():
    module = load_module()
    module._system(context_with_defaults(module, num_robots=2, enable_camera='amr_02'))
    with pytest.raises(RuntimeError, match='amr_03'):
        module._system(context_with_defaults(module, num_robots=2, enable_camera='amr_03'))


def test_nav2_launch_description_builds():
    path = os.path.join(get_package_share_directory('amr_bringup'), 'launch', 'nav2.launch.py')
    spec = importlib.util.spec_from_file_location('nav2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = {a.name for a in module.generate_launch_description().get_launch_arguments()}
    assert {'namespace', 'use_sim_time', 'params_file', 'odom_topic', 'cmd_vel_topic',
            'nav2_profile', 'congestion_filter', 'use_composition'} <= names
