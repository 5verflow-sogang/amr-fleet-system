"""Dispatcher 장부·재전송 로직. rclpy 없이 돈다."""

import csv
from pathlib import Path

import pytest
from amr_fleet_core.assignment import RobotInfo, TaskInfo

from amr_fleet_manager.dispatcher import (ERROR_CANCELED, ERROR_TASK_TAKEN, STATE_COMPLETED, STATE_FAILED,
                                          STATE_IN_PROGRESS, STATE_PENDING, Dispatcher, DispatcherConfig)
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
    # 늦게 수락한 다른 로봇의 보고는 장부를 바꾸지 않는다 (먼저 수락한 쪽이 주인)
    assert not d.on_status('t1', 'amr_02', STATE_IN_PROGRESS, start=1.0)
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


def test_canceled_task_not_retried():
    # 서버가 취소해 온 FAILED는 재시도 횟수가 남아도 다시 할당하지 않는다 (#107 리뷰)
    d = make(failed_retries=1)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert d.on_status('t1', 'amr_01', STATE_FAILED, error_code=ERROR_CANCELED)
    assert (d.tasks['t1'].state, d.tasks['t1'].failures) == (STATE_FAILED, 0)
    assert d.robots['amr_01'].task_id == ''
    assert d.assign(1.0) == ([], [])


def test_robot_priority_from_assigned_task():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    assert d.robot_priority('amr_01') == 0
    assert d.robot_priority('unknown') == 0
    d.add_task(task('t1', priority=7))
    d.assign(0.0)
    assert d.robot_priority('amr_01') == 7
    d.on_status('t1', 'amr_01', STATE_COMPLETED)
    assert d.robot_priority('amr_01') == 0


# ---- 로봇 장애: 작업 회수 ----

def test_dead_robot_task_reclaimed_and_reassigned():
    d = make(robot_timeout_s=3.0, reclaim_timeout_s=5.0)
    d.update_robot(robot('amr_01', (0.0, 0.0)), '', 0.0)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 0.0)
    d.add_task(task('t1', (1.0, 0.0)))
    [first], _ = d.assign(0.0)
    assert first.robot_id == 'amr_01'
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS)

    # amr_01은 1초 뒤 죽고(RobotState 끊김), amr_02는 계속 보낸다
    d.update_robot(robot('amr_01', mode=BUSY), 't1', 1.0)
    for t in (2.0, 4.0, 5.5):
        d.update_robot(robot('amr_02', (10.0, 0.0)), '', t)
    assert d.reclaim_lost(5.9) == []          # 마지막 RobotState(1.0) 후 5초 전
    assert d.reclaim_lost(6.0) == [('t1', 'amr_01')]
    assert (d.tasks['t1'].state, d.tasks['t1'].assignee, d.tasks['t1'].reclaims) == (STATE_PENDING, '', 1)
    assert d.tasks['t1'].failures == 0        # FAILED 재시도 횟수와 따로 센다
    assert d.reclaim_lost(7.0) == []          # 한 번만

    [second], _ = d.assign(6.0)
    assert (second.robot_id, second.task_id, second.assignment_id) == ('amr_02', 't1', first.assignment_id + 1)
    # 죽은 줄 알았던 로봇이 끝냈다고 보고하면 끝난 작업으로 본다 (끝낸 쪽이 주인)
    assert d.on_status('t1', 'amr_01', STATE_COMPLETED, now=6.5)
    assert (d.tasks['t1'].state, d.tasks['t1'].assignee) == (STATE_COMPLETED, 'amr_01')
    assert d.robots['amr_02'].task_id == ''   # 다시 맡았던 로봇은 풀린다
    assert d.pending_ids() == set()
    assert not d.on_status('t1', 'amr_02', STATE_COMPLETED)


def test_reclaim_drops_pending_ack_of_dead_robot():
    d = make(reclaim_timeout_s=5.0)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert d.pending_ids() == {1}
    assert d.reclaim_lost(5.0) == [('t1', 'amr_01')]
    assert d.pending_ids() == set()
    assert d.robots['amr_01'].load == 0


def test_idle_or_disabled_robot_not_reclaimed():
    d = make(reclaim_timeout_s=5.0)
    d.update_robot(robot('amr_01'), '', 0.0)   # 맡은 작업 없음
    assert d.reclaim_lost(100.0) == []
    off = make(reclaim_timeout_s=0.0)
    off.update_robot(robot('amr_01'), '', 0.0)
    off.add_task(task('t1'))
    off.assign(0.0)
    assert off.reclaim_lost(100.0) == []
    with pytest.raises(ValueError):
        make(robot_timeout_s=3.0, reclaim_timeout_s=2.0)


# ---- 먼저 수락한 쪽이 주인 (architecture.md 5-2절) ----

def test_unassigned_robot_that_accepted_first_takes_queued_task():
    """분산 경매로 맡은 로봇의 보고: 큐에 있던 작업을 그 로봇에게 넘기고 그 로봇에는 새 작업을 주지 않는다."""
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    assert d.on_status('t1', 'amr_03', STATE_PENDING, start=0.5, now=0.1)
    assert (d.tasks['t1'].assignee, d.tasks['t1'].assignment_id, d.tasks['t1'].state) == ('amr_03', 0, STATE_PENDING)
    assert d.assign(0.2) == ([], [])          # t1은 이미 주인이 있다
    assert d.on_status('t1', 'amr_03', STATE_COMPLETED, start=0.5, now=3.0)
    assert d.tasks['t1'].state == STATE_COMPLETED


def test_earlier_acceptor_takes_over_from_unconfirmed_assignment():
    """서버 명령이 아직 확인되지 않았는데 다른 로봇이 먼저 수락했다: 재전송을 그만두고 장부를 넘긴다."""
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.update_robot(robot('amr_02'), '', 0.0)
    d.add_task(task('t1'))
    [cmd], _ = d.assign(0.0)
    assert cmd.robot_id == 'amr_01'
    assert d.on_status('t1', 'amr_02', STATE_IN_PROGRESS, start=0.0, now=0.3)
    assert d.tasks['t1'].assignee == 'amr_02'
    assert d.pending_ids() == set()
    assert d.robots['amr_01'].task_id == '' and d.robots['amr_01'].load == 0
    assert d.robots['amr_02'].task_id == 't1'
    # 늦게 온 amr_01의 수락(늦게 수락한 쪽)은 장부를 바꾸지 않는다
    assert not d.on_ack('amr_01', cmd.assignment_id, True)
    assert not d.on_status('t1', 'amr_01', STATE_PENDING, start=0.8, now=0.9)
    assert d.tasks['t1'].assignee == 'amr_02'


def test_server_assignee_that_accepted_first_keeps_task():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert d.on_status('t1', 'amr_01', STATE_PENDING, start=0.2, now=0.3)
    assert not d.on_status('t1', 'amr_03', STATE_PENDING, start=0.4, now=0.5)
    assert d.tasks['t1'].assignee == 'amr_01'
    # 같은 시각이면 robot_id 순
    assert not d.on_status('t1', 'amr_02', STATE_PENDING, start=0.2, now=0.5)
    assert d.tasks['t1'].assignee == 'amr_01'


def test_task_taken_failure_hands_over_without_retry_count():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, start=1.0, now=1.0)
    # amr_02가 더 먼저 수락했다는 보고가 서버에 늦게 왔고, amr_01은 그걸 보고 이미 물러났다
    d.claims.observe('t1', 'amr_02', STATE_IN_PROGRESS, 0.5, 1.1)
    assert d.on_status('t1', 'amr_01', STATE_FAILED, ERROR_TASK_TAKEN, start=1.0, now=1.2)
    assert (d.tasks['t1'].assignee, d.tasks['t1'].state, d.tasks['t1'].failures) == ('amr_02', STATE_IN_PROGRESS, 0)


def test_task_taken_failure_without_known_owner_requeues():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    assert d.on_status('t1', 'amr_01', STATE_FAILED, ERROR_TASK_TAKEN, now=0.5)
    assert (d.tasks['t1'].assignee, d.tasks['t1'].state, d.tasks['t1'].failures) == ('', STATE_PENDING, 0)


def test_task_taken_reject_ack_hands_over_to_owner():
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    [cmd], _ = d.assign(0.0)
    # 서버가 아직 반영하지 못한 주인(amr_02)을 amr_01이 먼저 알고 거부했다
    d.claims.observe('t1', 'amr_02', STATE_COMPLETED, 0.1, 0.2)
    assert d.on_ack('amr_01', cmd.assignment_id, False, ERROR_TASK_TAKEN, now=0.3)
    assert (d.tasks['t1'].assignee, d.tasks['t1'].state) == ('amr_02', STATE_COMPLETED)
    assert d.assign(0.4) == ([], [])


def test_returning_robot_that_accepted_first_takes_task_back():
    """회수 뒤 살아 돌아온 로봇이 옛 작업을 계속 보고하면 그 로봇이 주인이다. 다시 맡은 로봇은 스스로 물러난다."""
    d = make(reclaim_timeout_s=5.0)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 0.0)
    d.add_task(task('t1'))
    d.assign(0.0)
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, start=0.0, now=0.1)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 5.5)
    assert d.reclaim_lost(6.0) == [('t1', 'amr_01')]
    [second], _ = d.assign(6.0)
    d.on_status('t1', 'amr_02', STATE_PENDING, start=6.0, now=6.1)
    assert d.claims.owner('t1', 6.2) == 'amr_02'   # 죽은 로봇의 주장은 빠졌다

    d.update_robot(robot('amr_01', mode=BUSY), 't1', 7.0)
    assert d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, start=0.0, now=7.1)
    assert d.tasks['t1'].assignee == 'amr_01'


def test_restarted_robot_drops_its_claim():
    """재시작한 로봇(RobotState current_task_id가 빔)의 옛 주장은 세지 않는다."""
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('t1'))
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, start=0.0, now=0.1)
    d.update_robot(robot('amr_01'), '', 1.0)
    assert d.claims.owner('t1', 1.1) is None


def test_report_before_request_is_kept_for_assign():
    """분산 경매로 맡은 로봇의 보고가 작업 요청보다 먼저 왔다: 요청이 오면 맡기지 않고 그 로봇에게 넘긴다."""
    d = make()
    d.update_robot(robot('amr_01'), '', 0.0)
    assert not d.on_status('t1', 'amr_03', STATE_PENDING, start=0.1, now=0.1)
    d.add_task(task('t1'))
    assert d.assign(0.2) == ([], [])
    assert (d.tasks['t1'].assignee, d.tasks['t1'].state) == ('amr_03', STATE_IN_PROGRESS)
