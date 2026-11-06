"""에너지 예측 배정 (energy)과 할당 알고리즘의 feasible. ROS 없이 돈다."""

import pytest

from amr_fleet_core.assignment import (ALGORITHMS, RobotInfo, TaskInfo, assign_hungarian, assign_nearest,
                                       euclidean_cost)
from amr_fleet_core.charging import Station
from amr_fleet_core.energy import (ChargeQueue, EnergyModel, charge_remaining_s, make_battery_cost,
                                   make_energy_feasible, required_percent, trip_length)

CHARGERS = [Station('c1', (0.0, -10.0)), Station('c2', (20.0, -10.0))]
# 계산하기 쉬운 모델: 1 m당 1 %, 직선 그대로, 여유 0
UNIT = EnergyModel(move_per_m=1.0, idle_per_s=0.0, speed_mps=1.0, path_factor=1.0, reserve_percent=0.0)


def robot(robot_id, battery, xy=(0.0, 0.0)):
    return RobotInfo(robot_id=robot_id, xy=xy, battery_percent=battery, mode=0)


def delivery(task_id, pickup, dropoff, priority=0):
    return TaskInfo(task_id=task_id, task_type=0, pickup_xy=pickup, dropoff_xy=dropoff, priority=priority, created=0.0)


def test_model_validation_and_per_m():
    assert EnergyModel(move_per_m=0.05, idle_per_s=0.002, speed_mps=0.5).per_m == pytest.approx(0.054)
    for kwargs in ({'move_per_m': -1.0}, {'speed_mps': 0.0}, {'path_factor': 0.9}, {'reserve_percent': -1.0}):
        with pytest.raises(ValueError):
            EnergyModel(**kwargs)


def test_trip_includes_return_to_nearest_charger():
    # (0,0) → 픽업 (0,10) 10 m → 배달 (20,10) 20 m → 가장 가까운 충전기 c2 (20,-10) 20 m
    task = delivery('t', (0.0, 10.0), (20.0, 10.0))
    assert trip_length((0.0, 0.0), task, CHARGERS) == pytest.approx(50.0)
    assert trip_length((0.0, 0.0), task, []) == pytest.approx(30.0)        # 충전기가 없으면 배달지까지
    model = EnergyModel(move_per_m=0.1, idle_per_s=0.01, speed_mps=1.0, path_factor=1.2, reserve_percent=5.0)
    assert required_percent((0.0, 0.0), task, CHARGERS, model) == pytest.approx(50.0 * 1.2 * 0.11 + 5.0)


def test_charge_task_needs_only_the_way_there():
    charge = TaskInfo(task_id='c', task_type=1, dropoff_xy=(0.0, -10.0), priority=0, created=0.0)
    assert trip_length((0.0, 0.0), charge, CHARGERS) == pytest.approx(10.0)


def test_low_robot_skips_long_task_but_takes_short_one():
    # 첫 Docker 실행(#312)의 실패: 22 %에서 긴 배송을 받아 충전기 가는 길에 0 %가 됐다
    feasible = make_energy_feasible(CHARGERS, UNIT)
    long_task = delivery('long', (0.0, 10.0), (20.0, 10.0))     # 50 %
    short_task = delivery('short', (0.0, -2.0), (0.0, -5.0))    # 2 + 3 + 5 = 10 %
    low = robot('amr_01', 22.0)
    assert not feasible(low, long_task) and feasible(low, short_task)


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_algorithms_skip_infeasible_pairs(name):
    feasible = make_energy_feasible(CHARGERS, UNIT)
    tasks = [delivery('long', (0.0, 10.0), (20.0, 10.0), priority=5), delivery('short', (0.0, -2.0), (0.0, -5.0))]
    # 가까운 amr_01은 배터리가 모자라 긴 작업을 못 받는다 → 먼 amr_02(충분)가 받고, amr_01은 짧은 작업
    robots = [robot('amr_01', 22.0, (0.0, 1.0)), robot('amr_02', 90.0, (5.0, 0.0))]
    result = ALGORITHMS[name](tasks, robots, euclidean_cost, feasible)
    assert result.assignments == {'long': 'amr_02', 'short': 'amr_01'}
    assert result.unassigned == []
    # feasible 없이(기존 동작)는 우선순위 높은 긴 작업이 가까운 amr_01에게 간다 (nearest·load_balance)
    if name != 'hungarian':
        assert ALGORITHMS[name](tasks, robots, euclidean_cost).assignments['long'] == 'amr_01'


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_task_nobody_can_do_stays_queued_and_robot_is_kept(name):
    feasible = make_energy_feasible(CHARGERS, UNIT)
    tasks = [delivery('long', (0.0, 10.0), (20.0, 10.0), priority=5), delivery('short', (0.0, -2.0), (0.0, -5.0))]
    result = ALGORITHMS[name](tasks, [robot('amr_01', 22.0)], euclidean_cost, feasible)
    assert result.assignments == {'short': 'amr_01'} and result.unassigned == ['long']


def test_hungarian_drops_forced_infeasible_match():
    # 작업 2개 다 로봇 1대만 할 수 있다 → 하나만 맡기고 하나는 남긴다 (큰 비용 짝을 맡기지 않는다)
    feasible = make_energy_feasible(CHARGERS, UNIT)
    tasks = [delivery('a', (0.0, -2.0), (0.0, -5.0), priority=1), delivery('b', (0.0, -3.0), (0.0, -6.0))]
    robots = [robot('amr_01', 90.0), robot('amr_02', 1.0)]
    result = assign_hungarian(tasks, robots, euclidean_cost, feasible)
    assert result.assignments == {'a': 'amr_01'} and result.unassigned == ['b']


def test_battery_cost_prefers_fuller_robot_for_long_task():
    cost = make_battery_cost(euclidean_cost, CHARGERS, UNIT, weight=20.0)
    task = delivery('long', (0.0, 10.0), (20.0, 10.0))
    # 거리는 amr_01이 2 m 가깝지만, 필요량 50 %가 남은 배터리(60 %)의 대부분이라 벌점이 크다
    near_low, far_full = robot('amr_01', 60.0, (0.0, 2.0)), robot('amr_02', 100.0, (0.0, 0.0))
    assert euclidean_cost(near_low, task) < euclidean_cost(far_full, task)
    assert assign_nearest([task], [near_low, far_full], cost).assignments == {'long': 'amr_02'}
    assert make_battery_cost(euclidean_cost, CHARGERS, UNIT, 0.0) is euclidean_cost
    with pytest.raises(ValueError):
        make_battery_cost(euclidean_cost, CHARGERS, UNIT, -1.0)


# ---- 충전 차례 대기 시간 ----

def test_queue_wait_free_slot_is_zero():
    assert ChargeQueue((100.0,), waiting=0, free_slots=1, cycle_s=140.0).wait_s() == 0.0
    assert ChargeQueue((100.0,), waiting=1, free_slots=2, cycle_s=140.0).wait_s() == 0.0
    # 빈 자리 하나는 앞에서 기다리던 로봇이 쓴다 → 충전 중인 로봇이 끝날 때까지
    assert ChargeQueue((100.0,), waiting=1, free_slots=1, cycle_s=140.0).wait_s() == pytest.approx(100.0)


def test_queue_wait_uses_earliest_free_and_rounds():
    q = ChargeQueue((90.0, 30.0), waiting=0, free_slots=0, cycle_s=140.0)
    assert q.wait_s() == pytest.approx(30.0)                  # 먼저 끝나는 자리
    assert q.wait_s(after_s=20.0) == pytest.approx(10.0)      # 작업하는 동안 흐른 시간은 뺀다
    assert q.wait_s(after_s=50.0) == 0.0
    q = ChargeQueue((90.0, 30.0), waiting=1, free_slots=0, cycle_s=140.0)
    assert q.wait_s() == pytest.approx(90.0)                  # 앞에 한 대: 두 번째로 끝나는 자리
    q = ChargeQueue((90.0, 30.0), waiting=2, free_slots=0, cycle_s=140.0)
    assert q.wait_s() == pytest.approx(30.0 + 140.0)          # 앞에 두 대: 한 바퀴 더
    # 충전 중인 로봇도 자리도 없으면 (살아 있는 로봇 ≤ N) 추정하지 않는다
    assert ChargeQueue((), waiting=3, free_slots=0).wait_s() == 0.0


def test_charge_remaining_travel_plus_charge():
    model = EnergyModel(speed_mps=1.0, path_factor=1.0, charge_per_s=0.5, charge_target_percent=100.0)
    assert charge_remaining_s(robot('amr_01', 30.0, (0.0, 0.0)), (0.0, -10.0), model) == pytest.approx(10.0 + 140.0)


def test_wait_added_only_when_robot_ends_below_low():
    # 1 m당 1 % (정지 0.1 %/s, 1 m/s → per_m 1.1). 충전 자리가 100 s 뒤에 난다
    model = EnergyModel(move_per_m=1.0, idle_per_s=0.1, speed_mps=1.0, path_factor=1.0, reserve_percent=0.0)
    queue = ChargeQueue((100.0,), waiting=0, free_slots=0, cycle_s=140.0)
    short = delivery('short', (0.0, -2.0), (0.0, -5.0))     # 일 5 m (5.5 %) + 충전기까지 5 m → 11 %
    base = required_percent((0.0, 0.0), short, CHARGERS, model)
    assert base == pytest.approx(11.0)
    # 90 %면 끝나도 30 % 위라 바로 충전하러 가지 않는다 → 대기 없음
    assert required_percent((0.0, 0.0), short, CHARGERS, model, 90.0, queue, 30.0) == pytest.approx(base)
    # 20 %면 끝나고 충전 차례를 (100 − 5) s 기다린다 → + 0.1 × 95
    assert required_percent((0.0, 0.0), short, CHARGERS, model, 20.0, queue, 30.0) == pytest.approx(base + 9.5)
    feasible = make_energy_feasible(CHARGERS, model, queue, 30.0)
    assert feasible(robot('amr_01', 15.0), short) is False      # 대기 없이는 되지만(11 %) 기다리면 모자란다
    assert make_energy_feasible(CHARGERS, model)(robot('amr_01', 15.0), short) is True
