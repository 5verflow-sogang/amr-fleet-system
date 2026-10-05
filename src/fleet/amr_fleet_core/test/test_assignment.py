import pytest

from amr_fleet_core.assignment import (
    ALGORITHMS,
    RobotInfo,
    TaskInfo,
    check_unique_ids,
    euclidean_cost,
    filter_candidates,
    sort_by_priority,
)
from amr_fleet_core.assignment.types import MODE_BUSY, MODE_IDLE, TYPE_CHARGE, TYPE_DELIVERY


def task(task_id, priority=0, created=0.0, deadline=None, pickup=(0.0, 0.0), dropoff=(1.0, 1.0)):
    return TaskInfo(
        task_id=task_id, task_type=TYPE_DELIVERY, pickup_xy=pickup, dropoff_xy=dropoff,
        priority=priority, created=created, deadline=deadline,
    )


def robot(robot_id, xy=(0.0, 0.0), battery=100.0, mode=MODE_IDLE, load=0):
    return RobotInfo(robot_id=robot_id, xy=xy, battery_percent=battery, mode=mode, load=load)


def test_start_xy_falls_back_to_dropoff():
    charge = TaskInfo(task_id='c', task_type=TYPE_CHARGE, dropoff_xy=(5.0, 0.0), priority=0, created=0.0)
    assert charge.start_xy == (5.0, 0.0)


def test_euclidean_cost_uses_start_xy():
    assert euclidean_cost(robot('amr_01', xy=(0.0, 0.0)), task('t', pickup=(3.0, 4.0))) == 5.0


def test_filter_candidates():
    robots = [
        robot('amr_03'),
        robot('amr_01'),
        robot('amr_02', mode=MODE_BUSY),
        robot('amr_04', battery=19.9),
        robot('amr_05', battery=20.0),
    ]
    result = filter_candidates(robots, min_battery_percent=20.0)
    assert [r.robot_id for r in result] == ['amr_01', 'amr_03', 'amr_05']


def test_sort_by_priority():
    tasks = [
        task('low', priority=1),
        task('no_deadline', priority=5),
        task('late', priority=5, deadline=20.0),
        task('early', priority=5, deadline=10.0),
        task('early_b', priority=5, deadline=10.0, created=1.0),
    ]
    order = [t.task_id for t in sort_by_priority(tasks)]
    assert order == ['early', 'early_b', 'late', 'no_deadline', 'low']


def test_check_unique_ids():
    with pytest.raises(ValueError):
        check_unique_ids([task('a'), task('a')], [])
    with pytest.raises(ValueError):
        check_unique_ids([], [robot('amr_01'), robot('amr_01')])


def test_algorithm_registry():
    assert set(ALGORITHMS) == {'nearest', 'load_balance', 'hungarian'}


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_empty_input(name):
    result = ALGORITHMS[name]([], [])
    assert result.assignments == {} and result.unassigned == []
    assert result.total_cost == 0.0


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_no_robots_leaves_all_unassigned(name):
    result = ALGORITHMS[name]([task('low', priority=1), task('high', priority=9)], [])
    assert result.assignments == {}
    assert result.unassigned == ['high', 'low']


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_duplicate_ids_raise(name):
    with pytest.raises(ValueError):
        ALGORITHMS[name]([task('a'), task('a')], [robot('amr_01')])


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_more_robots_than_tasks(name):
    tasks = [task('t1', pickup=(10.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0)), robot('amr_02', xy=(9.0, 0.0)), robot('amr_03', xy=(30.0, 0.0))]
    result = ALGORITHMS[name](tasks, robots)
    assert result.assignments == {'t1': 'amr_02'}
    assert result.unassigned == []
    assert result.total_cost == pytest.approx(1.0)


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_more_tasks_than_robots(name):
    tasks = [task('t1', priority=3), task('t2', priority=2), task('t3', priority=1)]
    robots = [robot('amr_01'), robot('amr_02')]
    result = ALGORITHMS[name](tasks, robots)
    assert set(result.assignments) == {'t1', 't2'}
    assert set(result.assignments.values()) == {'amr_01', 'amr_02'}
    assert result.unassigned == ['t3']


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_tie_broken_by_robot_id(name):
    # 비용이 모두 같으면 우선순위 순 작업이 robot_id 사전순 로봇을 받는다
    tasks = [task('t_b', priority=1), task('t_a', priority=2)]
    robots = [robot('amr_02'), robot('amr_01')]
    result = ALGORITHMS[name](tasks, robots)
    assert result.assignments == {'t_a': 'amr_01', 't_b': 'amr_02'}


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_deterministic(name):
    tasks = [task(f't{i}', priority=i % 3, pickup=(float(i), 0.0)) for i in range(6)]
    robots = [robot(f'amr_0{i}', xy=(float(i * 2), 1.0)) for i in range(4)]
    first = ALGORITHMS[name](tasks, robots)
    second = ALGORITHMS[name](list(reversed(tasks)), list(reversed(robots)))
    assert first == second


def test_nearest_picks_closest():
    tasks = [task('t1', pickup=(5.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0)), robot('amr_02', xy=(4.0, 0.0))]
    assert ALGORITHMS['nearest'](tasks, robots).assignments == {'t1': 'amr_02'}


def test_load_balance_prefers_low_load():
    # amr_02가 더 가깝지만 load가 커서 amr_01이 받는다
    tasks = [task('t1', pickup=(5.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0), load=0), robot('amr_02', xy=(4.0, 0.0), load=2)]
    result = ALGORITHMS['load_balance'](tasks, robots)
    assert result.assignments == {'t1': 'amr_01'}
    assert result.total_cost == pytest.approx(5.0)


def test_load_balance_same_load_uses_cost():
    tasks = [task('t1', pickup=(5.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0), load=1), robot('amr_02', xy=(4.0, 0.0), load=1)]
    assert ALGORITHMS['load_balance'](tasks, robots).assignments == {'t1': 'amr_02'}


def test_load_is_not_mutated():
    robots = [robot('amr_01', load=3)]
    ALGORITHMS['load_balance']([task('t1')], robots)
    assert robots[0].load == 3


def test_hungarian_beats_greedy():
    # 탐욕은 t1(급함)에 가까운 amr_01을 주고 t2에 먼 amr_02를 준다: 1 + 5 = 6
    # 최적은 t1←amr_02, t2←amr_01: 2 + 2 = 4
    tasks = [task('t1', priority=9, pickup=(1.0, 0.0)), task('t2', priority=1, pickup=(-2.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0)), robot('amr_02', xy=(3.0, 0.0))]
    greedy = ALGORITHMS['nearest'](tasks, robots)
    optimal = ALGORITHMS['hungarian'](tasks, robots)
    assert greedy.total_cost == pytest.approx(6.0)
    assert optimal.total_cost == pytest.approx(4.0)


@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_priority_wins_when_robots_short(name):
    tasks = [task('urgent', priority=9, pickup=(100.0, 0.0)), task('near', priority=0)]
    result = ALGORITHMS[name](tasks, [robot('amr_01')])
    assert result.assignments == {'urgent': 'amr_01'}
    assert result.unassigned == ['near']
