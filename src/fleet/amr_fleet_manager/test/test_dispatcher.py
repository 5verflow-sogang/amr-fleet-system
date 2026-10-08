"""Dispatcher 장부·재전송 로직. rclpy 없이 돈다."""

import csv
from pathlib import Path

import pytest
from amr_fleet_core.assignment import RobotInfo, TaskInfo

from amr_fleet_manager.dispatcher import (STATE_COMPLETED, STATE_FAILED, STATE_IN_PROGRESS, STATE_PENDING,
                                          Dispatcher, DispatcherConfig)
from amr_fleet_manager.records import (ASSIGNMENTS_HEADER, AssignmentLog, append_fault_event, load_zones,
                                       next_epoch)

IDLE, BUSY = 0, 1


def task(task_id, xy=(0.0, 0.0), priority=0, created=0.0):
    return TaskInfo(task_id=task_id, task_type=0, dropoff_xy=(50.0, 50.0), pickup_xy=xy,
                    priority=priority, created=created)


def robot(robot_id, xy=(0.0, 0.0), mode=IDLE, battery=100.0):
    return RobotInfo(robot_id=robot_id, xy=xy, battery_percent=battery, mode=mode)


def make(**kwargs):
    return Dispatcher(DispatcherConfig(**kwargs))


def test_unknown_algorithm_rejected():
    with pytest.raises(ValueError):
        make(algorithm='random')


def test_no_call_without_tasks_or_robots():
    d = make()
    assert d.assign(0.0) == ([], [])
    d.add_task(task('t1'))
    assert d.assign(0.0) == ([], [])


def test_duplicate_task_ignored():
    d = make()
    assert d.add_task(task('t1'))
    assert not d.add_task(task('t1'))


def test_assigns_nearest_and_logs_rows():
    d = make()
    d.update_robot(robot('amr_01', (0.0, 0.0)), '', 0.0)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 0.0)
    d.add_task(task('t1', (9.0, 0.0)))
    commands, rows = d.assign(0.0)

    assert [(c.assignment_id, c.robot_id, c.task_id, c.first) for c in commands] == [(1, 'amr_02', 't1', True)]
    assert len(rows) == 1
    assert (rows[0].call_id, rows[0].robot_id, rows[0].cost) == (1, 'amr_02', pytest.approx(1.0))
    assert d.tasks['t1'].assignee == 'amr_02'
    assert d.robots['amr_02'].load == 1


def test_unassigned_rows_when_tasks_exceed_robots():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('low', priority=1))
    d.add_task(task('high', priority=9))
    commands, rows = d.assign(0.0)

    assert [c.task_id for c in commands] == ['high']
    assert [(r.task_id, r.robot_id, r.cost) for r in rows if not r.robot_id] == [('low', '', None)]
    # 로봇이 바쁘면 다음 호출은 하지 않는다
    assert d.assign(0.0) == ([], [])


def test_busy_stale_and_low_battery_robots_skipped():
    d = make(robot_timeout_s=3.0, min_battery_percent=20.0)
    d.update_robot(robot('busy', mode=BUSY), '', 0.0)
    d.update_robot(robot('low', battery=10.0), '', 0.0)
    d.update_robot(robot('stale'), '', 0.0)
    d.add_task(task('t1'))
    assert d.assign(5.0) == ([], [])


def test_resend_same_id_until_ack():
    d = make(resend_interval_s=1.0, max_resends=5)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)

    assert d.due_resends(0.5) == ([], [])
    resend, _ = d.due_resends(1.0)
    assert [(c.assignment_id, c.first) for c in resend] == [(1, False)]

    assert d.on_ack('amr_01', 1, accepted=True)
    assert not d.on_ack('amr_01', 1, accepted=True)
    assert d.due_resends(10.0) == ([], [])


def test_ack_from_other_robot_ignored():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert not d.on_ack('amr_02', 1, accepted=True)
    assert d.pending_ids() == {1}


def test_give_up_returns_task_to_queue():
    d = make(resend_interval_s=1.0, max_resends=2)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert len(d.due_resends(1.0)[0]) == 1
    assert len(d.due_resends(2.0)[0]) == 1
    resend, given_up = d.due_resends(3.0)

    assert (resend, given_up) == ([], ['t1'])
    assert d.tasks['t1'].assignee == ''
    assert d.robots['amr_01'].load == 0
    # 다시 할당하면 새 번호를 쓴다
    d.update_robot(robot('amr_01'), '', 3.0)
    commands, _ = d.assign(3.0)
    assert [c.assignment_id for c in commands] == [2]


def test_reject_returns_task_to_queue():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert d.on_ack('amr_01', 1, accepted=False)
    assert d.tasks['t1'].assignee == ''
    assert d.robots['amr_01'].task_id == ''


def test_robot_state_confirms_when_ack_lost():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    d.update_robot(robot('amr_01', mode=BUSY), 't1', 0.5)
    assert d.pending_ids() == set()


def test_status_flow_frees_robot():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.add_task(task('t2', created=1.0))
    d.assign(0.0)

    # 상태 보고만 와도 Ack로 본다
    assert d.on_status('t1', 'amr_01', STATE_IN_PROGRESS)
    assert d.pending_ids() == set()
    assert not d.on_status('t1', 'amr_02', STATE_COMPLETED)
    assert d.on_status('t1', 'amr_01', STATE_COMPLETED)
    assert not d.on_status('t1', 'amr_01', STATE_FAILED)
    assert d.tasks['t1'].state == STATE_COMPLETED

    commands, _ = d.assign(1.0)
    assert [c.task_id for c in commands] == ['t2']
    assert d.tasks['t2'].state == STATE_PENDING


def test_next_epoch(tmp_path):
    path = tmp_path / 'sub' / 'epoch'
    assert next_epoch('') == 1
    assert next_epoch(str(path)) == 1
    assert next_epoch(str(path)) == 2
    path.write_text('garbage')
    assert next_epoch(str(path)) == 1


def test_assignment_log(tmp_path):
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1', (3.0, 4.0)))
    d.add_task(task('t2'))
    _, rows = d.assign(0.0)

    path = tmp_path / 'run' / 'assignments.csv'
    log = AssignmentLog(str(path))
    log.write(12.5, rows)
    log.close()

    with path.open(newline='') as f:
        lines = list(csv.reader(f))
    assert lines[0] == ASSIGNMENTS_HEADER
    assert lines[1:] == [
        # 우선순위·요청 시각이 같으면 task_id 순으로 먼저 맡는다
        ['12.500000', '1', 'nearest', 'euclidean', 't1', 'amr_01', '5.000000'],
        ['12.500000', '1', 'nearest', 'euclidean', 't2', '', ''],
    ]


def test_load_zones(tmp_path):
    shipped = Path(__file__).resolve().parents[1] / 'config' / 'zones.yaml'
    zones = load_zones(str(shipped))
    assert [z.zone_id for z in zones] == ['narrow_01', 'narrow_02']
    assert all(z.capacity == 1 for z in zones)

    path = tmp_path / 'zones.yaml'
    path.write_text('zones:\n  - id: wide\n    capacity: 2\n  - id: junction_01\n')
    assert [(z.zone_id, z.capacity) for z in load_zones(str(path))] == [('wide', 2), ('junction_01', 1)]
    assert load_zones('') == []


def test_append_fault_event(tmp_path):
    path = tmp_path / 'run' / 'fault_events.csv'
    assert append_fault_event('', 'kill', now=1.0) == 1.0
    append_fault_event(str(path), 'kill', now=100.25)
    append_fault_event(str(path), 'restart', now=110.5)
    with path.open(newline='') as f:
        assert list(csv.reader(f)) == [['time', 'event'], ['100.250000', 'kill'], ['110.500000', 'restart']]


def test_failed_task_retried_once():
    d = make(failed_retries=1)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.update_robot(robot('amr_02', (5.0, 0.0)), '', 0.0)
    d.add_task(task('t1'))
    [first], _ = d.assign(0.0)

    # 1회째 실패: 큐로 돌아가 다시 할당된다 (새 assignment_id)
    assert d.on_status('t1', first.robot_id, STATE_FAILED)
    assert (d.tasks['t1'].state, d.tasks['t1'].assignee, d.tasks['t1'].failures) == (STATE_PENDING, '', 1)
    assert d.robots[first.robot_id].task_id == ''
    [second], _ = d.assign(1.0)
    assert second.assignment_id == first.assignment_id + 1

    # 예전 로봇의 늦은 보고는 무시한다
    if second.robot_id != first.robot_id:
        assert not d.on_status('t1', first.robot_id, STATE_COMPLETED)

    # 2회째 실패: 실패로 남는다
    assert d.on_status('t1', second.robot_id, STATE_FAILED)
    assert d.tasks['t1'].state == STATE_FAILED
    assert d.assign(2.0) == ([], [])


def test_failed_retries_zero_keeps_failed():
    d = make(failed_retries=0)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    d.on_status('t1', 'amr_01', STATE_FAILED)
    assert d.tasks['t1'].state == STATE_FAILED
    with pytest.raises(ValueError):
        make(failed_retries=-1)
