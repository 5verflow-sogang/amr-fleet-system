"""작업 생성기 일정 만들기. rclpy 없이 돈다."""

import math
from pathlib import Path

import pytest
import yaml

from amr_fleet_manager.task_gen import (TaskSpec, count_ready, dump_tasks_json, expand, generate, load_stations,
                                        load_tasks_json, select_load, world_to_map, yaw_to_quaternion)

CONFIG = Path(__file__).resolve().parents[1] / 'config'


@pytest.fixture(scope='module')
def stations():
    return load_stations(str(CONFIG / 'stations.yaml'))


@pytest.fixture(scope='module')
def basic():
    return yaml.safe_load((CONFIG / 'scenarios' / 'basic.yaml').read_text(encoding='utf-8'))


def test_shipped_stations(stations):
    assert {'dock_1', 'dock_2', 'dock_a', 'dock_b', 'charger_c1', 'wait_1'} <= set(stations)
    assert len([s for s in stations if s.startswith('rack_')]) == 21
    # 모든 스테이션이 창고 안 (x −30~30, y −20~20)
    assert all(-30 < x < 30 and -20 < y < 20 for x, y, _ in stations.values())


def test_world_to_map():
    assert world_to_map((0.0, 1.5, 0.3), [0.0, 1.5, 0.0]) == pytest.approx((0.0, 0.0, 0.3))
    # map이 world에서 90도 돌아 있으면 world +x는 map −y
    x, y, yaw = world_to_map((1.0, 0.0, 0.0), [0.0, 0.0, math.pi / 2])
    assert (x, y, yaw) == pytest.approx((0.0, -1.0, -math.pi / 2))
    assert yaw_to_quaternion(math.pi) == pytest.approx((1.0, 0.0))


def test_expand(stations):
    assert expand(['dock_?'], sorted(stations)) == ['dock_1', 'dock_2', 'dock_a', 'dock_b']
    with pytest.raises(ValueError):
        expand(['nowhere_*'], sorted(stations))


def test_generate_basic(stations, basic):
    tasks = generate(basic, stations, seed=3)
    assert len(tasks) == basic['count']
    assert tasks[0].at_s == basic['start_delay_s']
    assert tasks[1].at_s - tasks[0].at_s == pytest.approx(basic['interval_s'])
    assert len({t.task_id for t in tasks}) == len(tasks)
    assert tasks[0].task_id == 'task_s3_0001'
    for t in tasks:
        assert t.type == 'delivery'
        assert t.pickup != t.dropoff
        assert t.pickup in stations and t.dropoff in stations
        # 입고 도크 → 랙, 랙 → 출고 도크
        assert (t.pickup in ('dock_1', 'dock_2') and t.dropoff.startswith('rack_')) or \
            (t.pickup.startswith('rack_') and t.dropoff in ('dock_a', 'dock_b'))
    # 두 흐름과 세 물품이 모두 나온다
    assert {t.pickup.startswith('dock') for t in tasks} == {True, False}
    assert {t.item for t in tasks} == {'small', 'medium', 'large'}


def test_same_seed_same_schedule(stations, basic):
    assert generate(basic, stations, seed=7) == generate(basic, stations, seed=7)
    assert generate(basic, stations, seed=7) != generate(basic, stations, seed=8)


def test_poisson_arrival(stations, basic):
    scenario = dict(basic, arrival='poisson', count=200, interval_s=2.0)
    tasks = generate(scenario, stations, seed=1)
    gaps = [b.at_s - a.at_s for a, b in zip(tasks, tasks[1:])]
    assert len(set(gaps)) > 1
    assert sum(gaps) / len(gaps) == pytest.approx(2.0, rel=0.25)


def test_charge_flow_has_no_pickup(stations):
    scenario = {'count': 3, 'interval_s': 1.0, 'flows': [{'type': 'charge', 'dropoff': ['charger_*']}]}
    for t in generate(scenario, stations, seed=0):
        assert (t.type, t.pickup, t.item) == ('charge', '', 'none')
        assert t.dropoff.startswith('charger_')


def test_collect_flow_has_pickup_and_its_own_priority(stations):
    scenario = {'count': 20, 'interval_s': 1.0, 'priority': [100, 100],
                'flows': [{'type': 'delivery', 'pickup': ['dock_1'], 'dropoff': ['rack_*']},
                          {'type': 'collect', 'pickup': ['rack_c*'], 'dropoff': ['dock_b'], 'priority': [200, 200]}]}
    tasks = generate(scenario, stations, seed=3)
    collects = [t for t in tasks if t.type == 'collect']
    assert collects and len(collects) < len(tasks)
    for t in collects:
        assert t.pickup.startswith('rack_c') and t.dropoff == 'dock_b' and t.item == 'none' and t.priority == 200
    assert {t.priority for t in tasks if t.type == 'delivery'} == {100}


def test_flow_priority_keeps_the_random_sequence(stations, basic):
    # 흐름에 시나리오와 같은 priority를 적어도 일정(시각·스테이션·물품)은 그대로다 - 난수 순서가 안 바뀐다
    flows = [dict(f, priority=basic['priority']) for f in basic['flows']]
    assert generate(dict(basic, flows=flows), stations, seed=4) == generate(basic, stations, seed=4)


def test_collect_aging_scenario_mix(stations):
    # 시연 2 리허설 일정: 회수 호출 약 20 %, 일반 100 / 회수 200, 평균 간격 6 s
    scenario = yaml.safe_load((CONFIG / 'scenarios' / 'collect_aging.yaml').read_text(encoding='utf-8'))
    tasks = [t for seed in range(1, 6) for t in generate(scenario, stations, seed=seed)]
    share = sum(t.type == 'collect' for t in tasks) / len(tasks)
    assert 0.1 < share < 0.3
    assert {(t.type, t.priority) for t in tasks} == {('delivery', 100), ('collect', 200)}


def test_task_spec_validation():
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'fly', 'small', 'a', 'b')
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'charge', 'none', 'dock_1', 'charger_c1')
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'delivery', 'small', 'a', 'b', priority=300)
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'collect', 'none', '', 'dock_b')  # 회수는 테이블(pickup)이 있어야 한다


def test_json_round_trip(tmp_path, stations, basic):
    tasks = generate(dict(basic, count=5), stations, seed=2)
    path = tmp_path / 'run' / 'tasks.json'
    dump_tasks_json(list(reversed(tasks)), str(path), meta={'seed': 2})
    # 시각 순으로 다시 읽는다
    assert load_tasks_json(str(path)) == tasks


def test_response_short_uses_nearby_racks(stations):
    """응답 시간 KPI용 시나리오: B–C 통로 동쪽 랙 8곳만 쓰고(로봇 5대보다 많게), 작업 하나가 짧다 (픽업 ↔ 하차 25 m 이하, rack_b4만 x −1.9)."""
    scenario = yaml.safe_load((CONFIG / 'scenarios' / 'response_short.yaml').read_text(encoding='utf-8'))
    tasks = generate(scenario, stations, seed=1)
    assert len(tasks) == 60
    racks = {f'rack_{row}{i}' for row in 'bc' for i in range(4, 8)}
    for t in tasks:
        assert {t.pickup, t.dropoff} <= racks
        (px, py, _), (dx, dy, _) = stations[t.pickup], stations[t.dropoff]
        assert math.hypot(px - dx, py - dy) <= 25.0


class _Sub:
    def __init__(self, node_name):
        self.node_name = node_name


def test_count_ready_server_and_decentral():
    """중앙: fleet_manager 하나면 시작. 분산: 로봇마다 task_auction이 다 붙어야 시작 (rosbag 녹화는 세지 않는다)."""
    central = [_Sub('rosbag2_recorder'), _Sub('fleet_manager')]
    assert count_ready(central, 'fleet_manager') == 1
    decentral = [_Sub('rosbag2_recorder')] + [_Sub('task_auction')] * 4
    assert count_ready(decentral, 'task_auction') == 4      # 5대 중 4대만 떴으면 아직 시작하지 않는다 (server_count 5)
    assert count_ready(decentral, '') == 5                  # 비우면 아무 구독자나 (녹화 포함)
    assert count_ready([], 'fleet_manager') == 0


def test_profile_changes_interval_by_time(stations, basic):
    # 한가함(10 s) → 피크(2 s) → 한가함(10 s). count 없이 마지막 until_s까지
    scenario = {k: v for k, v in basic.items() if k not in ('count', 'interval_s')}
    scenario.update(start_delay_s=0.0, profile=[{'until_s': 30, 'interval_s': 10}, {'until_s': 40, 'interval_s': 2},
                                                {'until_s': 60, 'interval_s': 10}])
    times = [t.at_s for t in generate(scenario, stations, seed=1)]
    assert times == [0.0, 10.0, 20.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 50.0]
    # count가 있으면 상한
    assert len(generate(dict(scenario, count=4), stations, seed=1)) == 4
    with pytest.raises(ValueError):
        generate(dict(scenario, profile=[{'until_s': 10, 'interval_s': 0}]), stations, seed=1)
    with pytest.raises(ValueError):
        generate(dict(scenario, profile=[{'until_s': 10, 'interval_s': 1}, {'until_s': 5, 'interval_s': 1}]),
                 stations, seed=1)


def test_select_load_overlays_and_rejects_unknown():
    scenario = {'count': 60, 'interval_s': 6.0, 'flows': [], 'loads': {'calm': {'interval_s': 12.5}, 'same': {}}}
    assert select_load(scenario) == {'count': 60, 'interval_s': 6.0, 'flows': []}
    assert select_load(scenario, 'same') == select_load(scenario)
    assert select_load(scenario, 'calm')['interval_s'] == 12.5
    with pytest.raises(ValueError):
        select_load(scenario, 'storm')


def test_burst_adds_only_its_flow_inside_the_window(stations, basic):
    burst = {'start_s': 60.0, 'duration_s': 120.0, 'interval_s': 6.0,
             'flow': {'type': 'collect', 'pickup': ['rack_c*'], 'dropoff': ['dock_b'], 'priority': [200, 200]}}
    plain = generate(dict(basic, count=20), stations, seed=7)
    tasks = generate(dict(basic, count=20, bursts=[burst]), stations, seed=7)
    extra = [t for t in tasks if '_b' in t.task_id]
    # 본 일정은 그대로이고, 몰림 작업은 구간 안에만 그 흐름으로 들어간다. 결과는 시각 순
    assert [t for t in tasks if '_b' not in t.task_id] == plain
    assert 10 <= len(extra) <= 35
    assert all(60.0 <= t.at_s < 180.0 and t.type == 'collect' and t.priority == 200 for t in extra)
    assert [t.at_s for t in tasks] == sorted(t.at_s for t in tasks)


def test_collect_aging_loads(stations):
    # 첫 측정(overload)은 loads 없이 만든 일정과 같다. normal은 간격 12.5 s, collect_burst는 normal + 60~180 s 회수 몰림
    scenario = yaml.safe_load((CONFIG / 'scenarios' / 'collect_aging.yaml').read_text(encoding='utf-8'))
    base = {k: v for k, v in scenario.items() if k != 'loads'}
    assert generate(select_load(scenario, 'overload'), stations, seed=1) == generate(base, stations, seed=1)
    normal = generate(select_load(scenario, 'normal'), stations, seed=1)
    burst = generate(select_load(scenario, 'collect_burst'), stations, seed=1)
    assert [t for t in burst if '_b' not in t.task_id] == normal
    extra = [t for t in burst if '_b' in t.task_id]
    assert extra and all(t.type == 'collect' and 60.0 <= t.at_s < 180.0 for t in extra)
