"""시나리오 파일 불러오기 (ROS 없이). extends로 물려받은 시나리오가 기반과 같은 프로세스를 쓰고 params만 바뀌는지 본다."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run  # noqa: E402
from harness.runner import apply_param_sets, enabled, render  # noqa: E402

SCENARIOS = sorted(p.stem for p in (Path(run.HERE) / 'scenarios').glob('*.yaml'))


@pytest.mark.parametrize('name', SCENARIOS)
def test_every_scenario_loads_and_renders(name):
    s = run.load_scenario(name)
    params = apply_param_sets(s, {**(s.get('params') or {}), 'run_dir': '/tmp/r', 'seed': 1, 'repeat': 0, 'label': ''})
    # 명령 안의 {이름}이 모두 params로 채워져야 한다 (오타면 KeyError)
    render(s['processes'], params)
    render(s['driver'], params)


@pytest.mark.parametrize('name, origin', [('loc_kpi_b0025', '[-0.0562, 1.5317, 0.00009]'),
                                          ('loc_kpi_b005', '[-0.0571, 1.5167, 0.00013]')])
def test_kpi_scenarios_extend_loc_basic(name, origin):
    base, s = run.load_scenario('loc_basic'), run.load_scenario(name)
    assert s['name'] == name and 'extends' not in s
    assert s['processes'] == base['processes'] and s['driver'] == base['driver']
    assert s['params']['map_origin'] == origin
    # 초기 위치 = 스폰 world (0, 1.5)를 원점 기준 map 좌표로
    ox, oy, _ = (float(v) for v in origin.strip('[]').split(','))
    assert float(s['params']['initial_x']) == pytest.approx(0.0 - ox)
    assert float(s['params']['initial_y']) == pytest.approx(1.5 - oy)
    assert s['params']['dynamic_obstacles'] == base['params']['dynamic_obstacles']


@pytest.mark.parametrize('name', ['nav_avoidance', 'nav_avoidance_headon'])
def test_nav_avoidance_stack_and_eval_share_map_origin(name):
    # 스택(지도·AMCL 초기 위치)과 회피 평가(기준 경로 map → world)가 같은 원점을 써야 경로 이탈에 원점 차이가 안 섞인다.
    # #249로 기본 지도가 v2가 됐을 때 평가만 v1 원점으로 남았다 (#262 이진 리뷰)
    s = run.load_scenario(name)
    params = apply_param_sets(s, {**s['params'], 'run_dir': '/tmp/r', 'seed': 1, 'repeat': 0, 'label': ''})
    cmds = {p['name']: p['cmd'] for p in render(s['processes'], params)}
    origin = s['params']['map_origin']
    assert f'map:={s["params"]["map"]} map_origin_in_world:="{origin}"' in cmds['system']
    assert f'"map_origin_in_world:={origin}"' in cmds['avoidance_eval']


def test_nav_avoidance_headon_differs_from_crossing_only_in_worker_and_goal():
    # 정면 접근은 가로지르기를 복사한 시나리오다. 작업자 움직임(--mode headon, 원 명세 1.0 m/s)과 목표(map x 9 m, 작업자가
    # 첫 전역 경로에 끼지 않게) 말고는 스택·기록·평가·드라이버가 같아야 두 결과를 나란히 볼 수 있다
    base, s = run.load_scenario('nav_avoidance'), run.load_scenario('nav_avoidance_headon')
    params = apply_param_sets(s, {**s['params'], 'run_dir': '/tmp/r', 'seed': 1, 'repeat': 0, 'label': ''})
    cmds = {p['name']: p['cmd'] for p in render(s['processes'], params)}
    assert '--mode headon --speed 1.0' in cmds['obstacles']
    assert float(s['params']['goal_x']) == 9.0
    others = [k for k in s['params'] if k not in ('goal_x', 'worker_speed')]
    assert {k: s['params'][k] for k in others} == {k: base['params'][k] for k in others}
    assert set(base['params']) == set(others) | {'goal_x'}
    rest = [p for p in s['processes'] if p['name'] != 'obstacles']
    assert rest == [p for p in base['processes'] if p['name'] != 'obstacles']
    assert s['driver'] == base['driver']


def test_config_patch_values_are_filled_from_params():
    # 규모 곡선 B안: 10대면 120건·5초
    sets = run.load_scenario('fleet_fake')['config_patch']['amr_fleet_manager/config/scenarios/basic.yaml']
    params = {**run.load_scenario('fleet_fake')['params'], 'task_count': '120', 'task_interval_s': '5.0'}
    assert run.render_patch(sets, params) == {'count': 120, 'interval_s': 5.0}
    assert run.render_patch({'a.b': 3, 'c': 'plain'}, {}) == {'a.b': 3, 'c': 'plain'}


def _fleet_fake_procs(allow_blocked=True, **sets):
    s = run.load_scenario('fleet_fake')
    params = apply_param_sets(s, {**s['params'], **sets, 'run_dir': '/tmp/r', 'seed': 1, 'repeat': 0, 'label': ''},
                              explicit=sets, allow_blocked=allow_blocked)
    return {p['name']: p for p in render(s['processes'], params) if enabled(p)}


def test_resend_and_link_loss_policy_reach_processes():
    # 연구 1 원인·정책 비교: 재전송 한도는 서버로, 끊기면 멈춤은 가짜 로봇으로 간다. 기본은 지금 동작(5회, 계속 수행)
    procs = _fleet_fake_procs()
    assert 'max_resends:=5' in procs['fleet_manager']['cmd']
    assert 'pause_on_link_loss_s:=0' in procs['fake_robots']['cmd']
    procs = _fleet_fake_procs(max_resends='0', pause_on_link_loss_s='2', robot_speed_mps='3')
    assert 'max_resends:=0' in procs['fleet_manager']['cmd']
    assert 'resend_interval_s:=1.0' in procs['fleet_manager']['cmd']
    assert 'resend_interval_s:=3' in _fleet_fake_procs(resend_interval_s='3')['fleet_manager']['cmd']
    assert 'pause_on_link_loss_s:=2' in procs['fake_robots']['cmd']
    assert 'speed_mps:=3' in procs['fake_robots']['cmd']
    assert 'speed_mps:=1.0' in _fleet_fake_procs()['fake_robots']['cmd']
    assert 'pause_mode:=stop' in _fleet_fake_procs()['fake_robots']['cmd']
    assert 'pause_mode:=leg' in _fleet_fake_procs(pause_mode='leg')['fake_robots']['cmd']
    up = _fleet_fake_procs(relay_uplink='true')
    assert 'relay_uplink:=true' in up['fault_injector']['cmd'] and 'uplink_relayed:=true' in up['fake_robots']['cmd']
    assert 'uplink_relayed:=false' in _fleet_fake_procs()['fake_robots']['cmd']


def test_reclaim_and_fallback_timeouts_reach_processes():
    # 연구 1 회수 기준·하이브리드 전환 기준 비교. 기본은 지금 동작(회수 5초, 전환 2초)
    base = _fleet_fake_procs(fleet_mode='hybrid')
    assert 'reclaim_timeout_s:=5.0' in base['fleet_manager']['cmd']
    assert 'fallback_timeout_s:=2.0' in base['task_auctions']['cmd']
    sets = _fleet_fake_procs(fleet_mode='hybrid', reclaim_timeout_s='40', fallback_timeout_s='6')
    assert 'reclaim_timeout_s:=40' in sets['fleet_manager']['cmd']
    assert 'fallback_timeout_s:=6' in sets['task_auctions']['cmd']


def test_fleet_mode_processes():
    central = _fleet_fake_procs()
    assert 'task_auctions' not in central and 'fleet_manager' in central
    assert 'relay_auction:=false' in central['fault_injector']['cmd']
    assert 'server_node:=fleet_manager server_count:=1' in central['task_generator']['cmd']
    # 분산: 서버 없음, 작업 생성기는 경매 노드 N개를 기다린다 ({num_robots}가 param_sets 안에서 채워진다)
    dec = _fleet_fake_procs(fleet_mode='decentral', num_robots='7')
    assert 'fleet_manager' not in dec
    assert 'count:=7' in dec['task_auctions']['cmd'] and 'fleet_mode:=decentral' in dec['task_auctions']['cmd']
    assert 'relay_auction:=true' in dec['fault_injector']['cmd']
    assert 'server_node:=task_auction server_count:=7' in dec['task_generator']['cmd']
    # 하이브리드: 서버 + 경매 노드, 경매 노드에 fleet_mode:=hybrid
    hyb = _fleet_fake_procs(fleet_mode='hybrid')
    assert 'fleet_manager' in hyb and 'fleet_mode:=hybrid' in hyb['task_auctions']['cmd']
    assert 'server_node:=fleet_manager server_count:=1' in hyb['task_generator']['cmd']
    # --set으로 직접 준 값은 param_sets가 덮어쓰지 않는다
    assert 'relay_auction:=true' in _fleet_fake_procs(relay_auction='true')['fault_injector']['cmd']
    with pytest.raises(ValueError, match='fleet_mode=decentralized'):
        _fleet_fake_procs(fleet_mode='decentralized')


@pytest.mark.parametrize('mode', ['decentral', 'hybrid'])
def test_fleet_mode_runs_after_prerequisites_merge(mode):
    # 선행 PR(#213 작업 생성기 server_count, #215 경매 노드 fleet_mode·하이브리드 폴백)이 들어가 막지 않는다
    procs = _fleet_fake_procs(allow_blocked=False, fleet_mode=mode)
    assert f'fleet_mode:={mode}' in procs['task_auctions']['cmd']
    assert 'task_auctions' not in _fleet_fake_procs(allow_blocked=False)      # central은 그대로


# ---------------------------------------------------------------- 식당 시연 장면 2 (demo_s2a·demo_s2b)
def _demo_procs(name, **sets):
    s = run.load_scenario(name)
    params = apply_param_sets(s, {**s['params'], **sets, 'run_dir': '/tmp/r', 'seed': 1, 'repeat': 0, 'label': ''},
                              explicit=sets)
    return {p['name']: p for p in render(s['processes'], params) if enabled(p)}


def _args(cmd):
    """stack cmd의 $(test -n "값" && echo 인자)를 bash처럼 풀어 쓴다 (값이 비면 인자 없음)."""
    import re
    return re.sub(r'\$\(test -n "([^"]*)" && echo ([^)]*)\)', lambda m: m[2] if m[1] else '', cmd)


def _own_params(name):
    """Actions가 받는 이름: 장면 파일 자체의 params 절 (extends 값은 안 본다)."""
    import yaml
    return set(yaml.safe_load((Path(run.HERE) / 'scenarios' / f'{name}.yaml').read_text(encoding='utf-8'))['params'])


@pytest.mark.parametrize('name', ['demo_s2a', 'demo_s2b'])
def test_scene2_runs_the_scene_script_instead_of_the_task_generator(name):
    s = run.load_scenario(name)
    assert s['processes'] == run.load_scenario('demo_restaurant')['processes']   # 공통 프로세스 그대로
    procs = _demo_procs(name)
    assert 'task_generator' not in procs
    cmd = procs['scene']['cmd']
    assert cmd.startswith('python3 /repo/tools/harness/demo_scene.py ')
    assert f'/repo/tools/harness/tasks/{name}.yaml' in cmd and '--out /tmp/r/scene.json' in cmd
    assert '--num-robots 5' in cmd
    stack = procs['stack']['cmd']
    assert 'dynamic_obstacles_file:=dynamic_obstacles_restaurant.yaml' in stack     # 손님 평상시
    assert 'num_robots:=5' in stack and 'body:=serving' in stack
    # 측정 bag에 #326 측정 스크립트가 읽는 토픽과 /demo/t0
    for topic in ('/fleet/task_status', '/fleet/robot_states', '/fleet/task_table', '/demo/t0'):
        assert topic in procs['rosbag']['cmd']
    # Actions로 넘길 이름 (켬/끔, 위치추정)이 장면 파일 params에 있다
    assert {'collect_priority', 'fleet_params', 'localization'} <= _own_params(name)


@pytest.mark.parametrize('name', ['demo_s2a', 'demo_s2b'])
def test_scene2_scene_script_is_one_line_to_swap(name):
    # 연우 장면 스크립트로 바꿀 때 장면 파일의 scene_script 한 줄만 바꾼다 (같은 인자를 받는 명령)
    assert 'scene_script' in _own_params(name)
    cmd = _demo_procs(name, scene_script='python3 /repo/tools/harness/other_scene.py')['scene']['cmd']
    assert cmd.startswith(f'python3 /repo/tools/harness/other_scene.py /repo/tools/harness/tasks/{name}.yaml ')
    assert 'scene' not in _demo_procs(name, scene_script='')        # 비우면 안 띄운다


def test_scene2_act1_on_off_is_collect_priority_only():
    on, off = _demo_procs('demo_s2a', collect_priority='200'), _demo_procs('demo_s2a', collect_priority='100')
    assert '--set collect_priority=200' in on['scene']['cmd'] and '--set collect_priority=100' in off['scene']['cmd']
    assert on['stack']['cmd'] == off['stack']['cmd']
    assert 'fleet_params_file' not in _args(on['stack']['cmd'])        # aging은 둘 다 끔


def test_scene2_act2_on_off_is_aging_only():
    on = _demo_procs('demo_s2b', fleet_params='fleet_manager_restaurant.yaml')
    off = _demo_procs('demo_s2b')
    assert 'fleet_params_file:=fleet_manager_restaurant.yaml' in _args(on['stack']['cmd'])
    assert 'fleet_params_file' not in _args(off['stack']['cmd'])
    assert on['scene']['cmd'] == off['scene']['cmd']


def test_scene2_localization_can_be_ground_truth():
    assert 'localization:=ground_truth' in _demo_procs('demo_s2a', localization='ground_truth')['stack']['cmd']
    assert 'localization:=amcl' in _demo_procs('demo_s2b')['stack']['cmd']


@pytest.mark.parametrize('name', ['demo_s11', 'demo_s12', 'demo_smoke'])
def test_other_demo_scenes_keep_the_task_generator(name):
    procs = _demo_procs(name)
    assert 'task_generator' in procs and 'scene' not in procs
    assert 'fleet_params_file' not in _args(procs['stack']['cmd'])     # 서버 설정 덮어쓰기 없음 (지금과 같다)


# ---------------------------------------------------------------- 식당 시연 장면 3 (demo_s3 충전 스케줄링)
def test_scene3_runs_the_scene_script_with_battery_and_charging_on_the_stack():
    s = run.load_scenario('demo_s3')
    assert s['processes'] == run.load_scenario('demo_restaurant')['processes']   # 공통 프로세스 그대로
    procs = _demo_procs('demo_s3')
    assert 'task_generator' not in procs
    assert '/repo/tools/harness/tasks/demo_s3.yaml' in procs['scene']['cmd']
    stack = procs['stack']['cmd']
    assert 'bt_params_file:=amr_bt_battery_demo_s3.yaml' in _args(stack)
    assert 'stations_file:=stations_restaurant.yaml' in stack
    assert 'charging:=true' in stack and 'localization:=ground_truth' in stack
    assert 'dynamic_obstacles_file:=dynamic_obstacles_restaurant_s3.yaml' in stack    # 손님은 서쪽에만


def test_scene3_on_off_is_the_charging_policy_only():
    on, off = _demo_procs('demo_s3', policy='on'), _demo_procs('demo_s3', policy='off')
    on_policy, off_policy = 'charge_low_percent:=30.0 min_in_service:=3', 'charge_low_percent:=20.0 min_in_service:=0'
    assert on_policy in on['stack']['cmd'] and off_policy in off['stack']['cmd']
    assert on['stack']['cmd'].replace(on_policy, '') == off['stack']['cmd'].replace(off_policy, '')
    assert on['scene']['cmd'] == off['scene']['cmd']
    with pytest.raises(ValueError, match='policy'):
        _demo_procs('demo_s3', policy='maybe')


def test_scene3_other_scenes_keep_charging_off():
    for name in ('demo_s2a', 'demo_s11'):
        stack = _demo_procs(name)['stack']['cmd']
        assert 'charging:=false' in stack and 'bt_params_file' not in _args(stack)


def test_scene3_fake_robots_match_the_gazebo_battery_and_policy():
    import yaml
    repo = Path(run.HERE).parents[1]
    bt = yaml.safe_load((repo / 'src/task/amr_bt/config/amr_bt_battery_demo_s3.yaml').read_text(encoding='utf-8'))
    fake = yaml.safe_load((Path(run.HERE) / 'tasks/demo_s3_fake_robots.yaml').read_text(encoding='utf-8'))
    assert fake['/**']['ros__parameters']['battery'] == bt['/**']['ros__parameters']['battery']
    for i in range(1, 6):
        assert (fake[f'/amr_{i:02d}/fake_robot']['ros__parameters']['battery']
                == bt[f'/amr_{i:02d}/amr_bt']['ros__parameters']['battery'])
    assert run.load_scenario('demo_s3_fake')['param_sets'] == run.load_scenario('demo_s3')['param_sets']


def test_demo_stack_never_passes_an_empty_launch_argument():
    # Humble ros2 launch는 'name:='(빈 값)을 malformed launch argument로 거부해 스택이 안 뜬다 (11/8 demo_s3 맥 스모크)
    import re
    for name in ('demo_restaurant', 'demo_s11', 'demo_s2a', 'demo_s2b', 'demo_s3'):
        cmd = _args(_demo_procs(name)['stack']['cmd'])
        assert not re.search(r':=(""|\s|$)', cmd), (name, re.findall(r'\S+:=(?:""|(?=\s)|$)', cmd))


def test_scene3_fake_robots_start_where_the_gazebo_robots_spawn():
    import yaml
    repo = Path(run.HERE).parents[1]
    robots = yaml.safe_load((repo / 'src/bringup/amr_bringup/config/robots_restaurant_s3.yaml').read_text(encoding='utf-8'))
    fake = yaml.safe_load((Path(run.HERE) / 'tasks/demo_s3_fake_robots.yaml').read_text(encoding='utf-8'))
    for name, (x, y, _) in robots['robots'].items():
        assert fake[f'/{name}/fake_robot']['ros__parameters']['start_xy'] == [x, y], name
    assert run.load_scenario('demo_s3')['params']['robots'] == 'robots_restaurant_s3.yaml'
