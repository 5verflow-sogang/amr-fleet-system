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


# ---- 2주차 구현 후 skip 해제 ----

@pytest.mark.skip(reason='2주차 구현')
@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_empty_input(name):
    result = ALGORITHMS[name]([], [])
    assert result.assignments == {} and result.unassigned == []


@pytest.mark.skip(reason='2주차 구현')
def test_hungarian_beats_greedy():
    # 탐욕은 t1(급함)에 가까운 amr_01을 주고 t2에 먼 amr_02를 준다: 1 + 5 = 6
    # 최적은 t1←amr_02, t2←amr_01: 2 + 2 = 4
    tasks = [task('t1', priority=9, pickup=(1.0, 0.0)), task('t2', priority=1, pickup=(-2.0, 0.0))]
    robots = [robot('amr_01', xy=(0.0, 0.0)), robot('amr_02', xy=(3.0, 0.0))]
    greedy = ALGORITHMS['nearest'](tasks, robots)
    optimal = ALGORITHMS['hungarian'](tasks, robots)
    assert greedy.total_cost == pytest.approx(6.0)
    assert optimal.total_cost == pytest.approx(4.0)


@pytest.mark.skip(reason='2주차 구현')
@pytest.mark.parametrize('name', sorted(ALGORITHMS))
def test_priority_wins_when_robots_short(name):
    tasks = [task('urgent', priority=9, pickup=(100.0, 0.0)), task('near', priority=0)]
    result = ALGORITHMS[name](tasks, [robot('amr_01')])
    assert result.assignments == {'urgent': 'amr_01'}
    assert result.unassigned == ['near']
