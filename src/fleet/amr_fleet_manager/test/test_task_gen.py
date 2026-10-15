"""작업 생성기 일정 만들기. rclpy 없이 돈다."""

import math
from pathlib import Path

import pytest
import yaml

from amr_fleet_manager.task_gen import (TaskSpec, dump_tasks_json, expand, generate, load_stations, load_tasks_json,
                                        world_to_map, yaw_to_quaternion)

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


def test_task_spec_validation():
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'fly', 'small', 'a', 'b')
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'charge', 'none', 'dock_1', 'charger_c1')
    with pytest.raises(ValueError):
        TaskSpec(0.0, 't', 'delivery', 'small', 'a', 'b', priority=300)


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

