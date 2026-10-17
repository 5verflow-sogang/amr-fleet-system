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
def test_fleet_mode_blocked_until_prerequisites_merge(mode):
    # 선행 PR(#213·#215) 머지 전에는 분산 이름의 중앙 데이터가 쌓이지 않게 막는다. 머지되면 _blocked 줄을 지우고 이 테스트도 지운다
    with pytest.raises(ValueError, match='아직 돌릴 수 없다'):
        _fleet_fake_procs(allow_blocked=False, fleet_mode=mode)
    assert 'task_auctions' not in _fleet_fake_procs(allow_blocked=False)      # central은 그대로
