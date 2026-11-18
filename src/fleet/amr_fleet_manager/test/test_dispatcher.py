"""Dispatcher 장부·재전송 로직. rclpy 없이 돈다."""

import csv
from dataclasses import replace
from pathlib import Path

import pytest
from amr_fleet_core.assignment import RobotInfo, TaskInfo
from amr_fleet_core.charging import ChargeConfig, Station
from amr_fleet_core.energy import EnergyModel

from amr_fleet_manager.dispatcher import (ERROR_CANCELED, ERROR_TASK_TAKEN, STATE_COMPLETED, STATE_FAILED,
                                          STATE_IN_PROGRESS, STATE_PENDING, TYPE_CHARGE, TYPE_RETURN, Dispatcher,
                                          DispatcherConfig)
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


def test_unknown_cost_fn_rejected():
    with pytest.raises(ValueError):
        make(cost_fn='random')
    with pytest.raises(ValueError):
        make(cost_fn='congestion', congestion_weight=-1.0)


def _crowded_choice(at=0.0, **kwargs):
    # 시연 1-1: 가까운 amr_01(5 m)은 손님이 몰린 구역(x 2~4)을 지나야 하고, amr_02(6 m)는 안 지난다.
    # 혼잡도는 0 s에 받고, at 초에 할당한다
    d = make(**kwargs)
    d.update_robot(robot('amr_01', (0.0, 0.0)), '', at)
    d.update_robot(robot('amr_02', (5.0, 6.0)), '', at)
    d.set_congestion([((2.0, 4.0, -1.0, 1.0), 1.0)], 0.0)
    d.add_task(task('t1', (5.0, 0.0)))
    commands, rows = d.assign(at)
    return commands[0].robot_id, rows[0]


def test_congestion_cost_fn_avoids_crowded_robot_and_logs_name():
    robot_id, row = _crowded_choice(cost_fn='congestion', congestion_weight=3.0)
    assert (robot_id, row.cost_fn, row.cost) == ('amr_02', 'congestion', pytest.approx(6.0))


def test_euclidean_ignores_congestion():
    # 끈 쪽(기본): 혼잡도를 넣어도 가까운 로봇
    robot_id, row = _crowded_choice()
    assert (robot_id, row.cost_fn, row.cost) == ('amr_01', 'euclidean', pytest.approx(5.0))


def test_stale_congestion_falls_back_to_distance():
    # 혼잡도 노드가 죽어 congestion_timeout_s 넘게 안 오면 마지막 값으로 벌점을 주지 않는다.
    # assignments.csv에는 실제로 쓴 비용 함수가 남는다 (켬/끔 측정에서 섞인 호출을 가려내게)
    robot_id, row = _crowded_choice(at=3.5, cost_fn='congestion', congestion_timeout_s=3.0)
    assert (robot_id, row.cost_fn, row.cost) == ('amr_01', 'euclidean', pytest.approx(5.0))
    assert _crowded_choice(at=2.9, cost_fn='congestion', congestion_timeout_s=3.0)[0] == 'amr_02'
    with pytest.raises(ValueError):
        make(cost_fn='congestion', congestion_timeout_s=0.0)


def _one_robot_picks(aging_rate, aging_now):
    # 로봇 1대, 일반 작업(100, 0 s 접수)과 회수 호출(200, 70 s 접수). 할당 시각 now(실제 시간)와 aging_now(작업 시계)는 따로다
    d = make(aging_rate=aging_rate)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.add_task(task('normal', (1.0, 0.0), priority=100, created=0.0))
    d.add_task(task('collect', (2.0, 0.0), priority=200, created=70.0))
    commands, _ = d.assign(0.0, aging_now=aging_now)
    return commands[0].task_id


def test_aging_lets_long_waiting_task_go_before_new_urgent_one():
    assert _one_robot_picks(0.0, 75.0) == 'collect'          # 끔: 우선순위 그대로
    assert _one_robot_picks(100.0 / 60.0, 75.0) == 'normal'  # 75 s 기다린 일반 작업(225) > 5 s 된 회수 호출(208)
    assert _one_robot_picks(100.0 / 60.0, None) == 'collect'  # 작업 시계를 안 주면 aging 없음


def test_negative_aging_rate_rejected():
    with pytest.raises(ValueError):
        make(aging_rate=-1.0)


def test_add_task_ages_by_received_at_not_wall_clock_created_time():
    # 대시보드가 created_time에 실제 시각(~1.7e9)을 넣어도, aging은 add_task가 받은 시각(received_at, 서버 시계)으로
    # 잰다. created_time 그대로 쟀으면 now(시뮬레이션 시간, 600)보다 항상 뒤라 대기가 0으로 고정돼, 75 s나 기다린
    # 회수 호출(200)이 막 들어온 더 급한 작업(250)에 영영 밀렸을 것이다
    d = make(aging_rate=100.0 / 60.0)
    d.update_robot(robot('amr_01'), '', 600.0)
    d.add_task(task('collect', (2.0, 0.0), priority=200, created=1_730_000_000.0), received_at=525.0)
    d.add_task(task('urgent_new', (1.0, 0.0), priority=250, created=600.0), received_at=600.0)
    commands, _ = d.assign(600.0, aging_now=600.0)
    # 75 s 기다린 회수 호출(200 + 75 × 100/60 = 325)이 막 들어온 250보다 앞선다
    assert commands[0].task_id == 'collect'


def test_add_task_ages_by_received_at_not_zero_created_time():
    # created_time을 비워(0)도, aging은 받은 시각(received_at)으로 잰다. created=0 그대로 쟀으면 대기가 수백 초로
    # 잡혀, 막 들어온 낮은 우선순위 작업(normal)이 역시 막 들어온 급한 회수 호출(collect)보다 무조건 앞섰을 것이다
    d = make(aging_rate=100.0 / 60.0)
    d.update_robot(robot('amr_01'), '', 600.0)
    d.add_task(task('normal', (1.0, 0.0), priority=100, created=0.0), received_at=600.0)
    d.add_task(task('collect', (2.0, 0.0), priority=200, created=600.0), received_at=600.0)
    commands, _ = d.assign(600.0, aging_now=600.0)
    # 둘 다 막 들어와 대기 0: 우선순위 그대로 회수 호출이 먼저
    assert commands[0].task_id == 'collect'


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


def test_decentral_robot_not_assigned_and_pending_withdrawn():
    """하이브리드: 분산 모드로 바뀐 로봇은 후보에서 빠지고, 확인 전 명령은 큐로 돌아가 다른 로봇에게 간다."""
    d = make()
    d.update_robot(robot('amr_01', (0.0, 0.0)), '', 0.0)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 0.0)
    d.add_task(task('t1', (1.0, 0.0)))
    [cmd], _ = d.assign(0.0)
    assert cmd.robot_id == 'amr_01'
    assert d.set_robot_mode('amr_01', True) == ['t1']
    assert d.pending_ids() == set()
    [again], _ = d.assign(0.5)
    assert again.robot_id == 'amr_02'
    assert d.set_robot_mode('amr_01', True) == []      # 이미 분산
    assert d.set_robot_mode('amr_09', True) == []      # 모르는 로봇
    # 중앙으로 돌아오면 다시 후보
    d.update_robot(robot('amr_02', (10.0, 0.0)), 't1', 0.6)
    d.add_task(task('t2', (1.0, 0.0)))
    d.set_robot_mode('amr_01', False)
    [third], _ = d.assign(0.7)
    assert third.robot_id == 'amr_01'


def test_station_reserved_until_task_finishes():
    # task()는 하역이 모두 (50, 50)이라 같은 도크로 가는 작업이다 (11/2 dock_a 교착)
    d = make(station_clearance_m=1.0)
    d.update_robot(robot('amr_01', (0.0, 0.0)), '', 0.0)
    d.update_robot(robot('amr_02', (10.0, 0.0)), '', 0.0)
    d.add_task(task('t1', (1.0, 0.0)))
    d.add_task(task('t2', (9.0, 0.0), created=1.0))
    commands, _ = d.assign(0.0)
    assert [c.task_id for c in commands] == ['t1']
    assert d.tasks['t2'].assignee == ''
    d.on_status('t1', commands[0].robot_id, STATE_IN_PROGRESS)
    assert d.assign(0.5) == ([], [])               # 진행 중이면 계속 잡혀 있다
    d.on_status('t1', commands[0].robot_id, STATE_COMPLETED)
    [cmd], _ = d.assign(1.0)
    assert cmd.task_id == 't2'


def test_station_conflict_in_queue_keeps_higher_priority():
    d = make(station_clearance_m=1.0)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.update_robot(robot('amr_02'), '', 0.0)
    d.add_task(task('low', (1.0, 0.0), priority=1))
    d.add_task(task('high', (2.0, 0.0), priority=9))
    commands, _ = d.assign(0.0)
    assert [c.task_id for c in commands] == ['high']


def test_different_stations_all_assigned():
    d = make(station_clearance_m=1.0)
    d.update_robot(robot('amr_01'), '', 0.0)
    d.update_robot(robot('amr_02'), '', 0.0)
    d.add_task(task('t1', (1.0, 0.0)))
    d.add_task(replace(task('t2', (6.0, 0.0)), dropoff_xy=(55.0, 50.0)))   # 픽업·하역 모두 5 m 떨어짐
    commands, _ = d.assign(0.0)
    assert sorted(c.task_id for c in commands) == ['t1', 't2']
    with pytest.raises(ValueError):
        make(station_clearance_m=-1.0)


# ---- 자동 충전 (배터리 인지 운영 1단계) ----


def charging(n=0, low=30.0, **kwargs):
    chargers = tuple(Station(f'charger_c{i}', (float(2 * i), -10.0)) for i in (1, 2, 3))
    waits = tuple(Station(f'wait_{i}', (20.0, float(i))) for i in (1, 2, 3, 4, 5))
    return make(charging_enabled=True, charge=ChargeConfig(low_percent=low, min_in_service=n),
                chargers=chargers, wait_stations=waits, **kwargs)


def _five(d, batteries, now=0.0):
    for i, b in enumerate(batteries, start=1):
        d.update_robot(robot(f'amr_0{i}', (float(i), 0.0), battery=b), '', now)


def test_charging_off_by_default_creates_nothing():
    d = make()
    _five(d, [10.0] * 5)
    assert d.plan_charging(0.0) == []
    assert d.assign(0.0) == ([], [])


def test_five_robots_n3_never_more_than_two_charging():
    # 명세 완료 기준: 가짜 로봇 5대, N=3에서 배터리가 동시에 떨어져도 충전하러 가는 로봇은 2대를 넘지 않는다
    d = charging(n=3)
    _five(d, [25.0, 12.0, 28.0, 18.0, 22.0])
    orders = d.plan_charging(0.0)
    assert [(o.robot_id, o.task_type) for o in orders] == [('amr_02', TYPE_CHARGE), ('amr_04', TYPE_CHARGE)]
    commands, rows = d.assign(0.0)
    assert sorted(c.robot_id for c in commands) == ['amr_02', 'amr_04'] and rows == []
    assert sorted(d.charging_robots()) == ['amr_02', 'amr_04']
    assert d.charge_waiting == ['amr_05', 'amr_01', 'amr_03']
    # 다음 틱에도 더 보내지 않는다 (한 대 충전이 끝나야 한 대가 간다)
    _five(d, [24.0, 12.0, 27.0, 18.0, 21.0], now=1.0)
    assert d.plan_charging(1.0) == [] and len(d.charging_robots()) == 2
    # amr_02 충전 완료 → 다음으로 낮은 amr_05가 간다. amr_02는 대기 위치로
    d.on_status(orders[0].task_id, 'amr_02', STATE_COMPLETED, now=2.0)
    _five(d, [24.0, 100.0, 27.0, 18.0, 21.0], now=2.0)
    nxt = d.plan_charging(2.0)
    assert [(o.robot_id, o.task_type) for o in nxt] == [('amr_05', TYPE_CHARGE), ('amr_02', TYPE_RETURN)]
    assert len(d.charging_robots()) == 2


def test_waiting_robot_keeps_serving_above_min_battery():
    # 충전 차례를 기다리는 로봇도 할당 하한(20) 위면 배송을 받는다
    d = charging(n=3)
    _five(d, [25.0, 12.0, 28.0, 18.0, 22.0])
    d.plan_charging(0.0)
    d.add_task(task('t1', (3.0, 0.0)))
    commands, _ = d.assign(0.0)
    assert [c.robot_id for c in commands if c.task_id == 't1'] == ['amr_03']


def test_charge_order_goes_before_delivery():
    # 충전 작업을 만든 로봇에게는 같은 틱에 배송을 주지 않는다
    d = charging()
    d.update_robot(robot('amr_01', battery=25.0), '', 0.0)
    d.add_task(task('t1'))
    d.plan_charging(0.0)
    commands, _ = d.assign(0.0)
    assert [c.task_id.split('_')[0] for c in commands] == ['charge']
    assert d.tasks['t1'].state == STATE_PENDING and not d.tasks['t1'].assignee


def test_busy_low_robot_charges_after_task():
    d = charging()
    d.update_robot(robot('amr_01', battery=15.0), '', 0.0)
    d.robots['amr_01'].task_id = 'x'
    assert d.plan_charging(0.0) == [] and d.charge_waiting == ['amr_01']
    d.robots['amr_01'].task_id = ''
    assert [o.robot_id for o in d.plan_charging(1.0)] == ['amr_01']


def _charged(d):
    """amr_01을 충전기 c1로 보내 충전을 끝낸다 (1 s)."""
    d.update_robot(robot('amr_01', (2.0, -10.0), battery=25.0), '', 0.0)
    charge = d.plan_charging(0.0)[0]
    d.assign(0.0)
    d.on_status(charge.task_id, 'amr_01', STATE_COMPLETED, now=1.0)
    d.update_robot(robot('amr_01', (2.0, -10.0), battery=100.0), '', 1.0)


def test_return_dropped_if_robot_takes_delivery():
    # 충전을 마친 로봇이 같은 틱에 배송을 받으면 대기 위치 복귀는 보내지 않는다 (충전기는 어차피 비워진다)
    d = charging()
    _charged(d)
    d.add_task(task('t1', (3.0, -10.0)))
    ret = d.plan_charging(1.0)
    assert [o.task_type for o in ret] == [TYPE_RETURN]
    commands, _ = d.assign(1.0)
    assert [c.task_id for c in commands] == ['t1']
    assert ret[0].task_id not in d.tasks


def test_return_goes_to_free_wait_station():
    d = charging()
    d.update_robot(robot('amr_02', (20.0, 1.0), battery=90.0), '', 0.0)   # wait_1에 서 있다
    _charged(d)
    d.update_robot(robot('amr_02', (20.0, 1.0), battery=90.0), '', 1.0)
    ret = d.plan_charging(1.0)
    assert [(o.robot_id, o.station) for o in ret] == [('amr_01', 'wait_2')]
    commands, _ = d.assign(1.0)
    assert [c.task_id for c in commands] == [ret[0].task_id]


def test_no_return_without_wait_stations():
    d = charging()
    d = make(charging_enabled=True, chargers=d.config.chargers)
    _charged(d)
    assert d.plan_charging(1.0) == []


def test_dead_robot_charge_order_dropped():
    d = charging()
    d.update_robot(robot('amr_01', battery=25.0), '', 0.0)
    d.robots['amr_01'].task_id = 'x'          # 일하는 중이라 아직 못 보낸다
    order = d.plan_charging(0.0)
    assert order == []
    d.robots['amr_01'].task_id = ''
    order = d.plan_charging(0.0)[0]
    d.robots['amr_01'].task_id = 'y'          # 보내기 전에 다른 출처의 작업을 받고 (분산 등)
    d.robots['amr_01'].last_seen = -10.0      # 죽었다
    assert d.plan_charging(5.0) == [] and order.task_id not in d.tasks
    assert d.charging_robots() == {}


def test_failed_charge_retried_to_same_robot_without_load():
    d = charging()
    d.update_robot(robot('amr_01', battery=25.0), '', 0.0)
    d.update_robot(robot('amr_02', battery=90.0), '', 0.0)
    order = d.plan_charging(0.0)[0]
    d.assign(0.0)
    d.on_status(order.task_id, 'amr_01', STATE_FAILED, error_code=502, now=1.0)
    assert d.tasks[order.task_id].state == STATE_PENDING
    commands, _ = d.assign(1.0)
    assert [(c.robot_id, c.task_id) for c in commands] == [('amr_01', order.task_id)]
    # 거부되면 큐로 돌아가도 load(부하균형 할당 수)는 건드리지 않는다
    d.on_ack('amr_01', commands[0].assignment_id, False)
    assert d.robots['amr_01'].load == 0 and d.robots['amr_02'].load == 0


def test_generator_charge_task_counts_toward_limit():
    # 작업 생성기가 낸 충전 작업(for_robot 없음)도 동시 충전 대수에 센다
    d = charging(n=1)
    d.update_robot(robot('amr_01', battery=90.0), '', 0.0)
    d.update_robot(robot('amr_02', battery=10.0), '', 0.0)
    d.add_task(TaskInfo(task_id='gen_c', task_type=TYPE_CHARGE, dropoff_xy=(6.0, -10.0), priority=0, created=0.0))
    d.assign(0.0)
    assert d.charging_robots() == {d.tasks['gen_c'].assignee: 'charger_c3'}
    assert d.plan_charging(0.0) == []


# ---- 에너지 예측 배정 (2단계) ----

UNIT = EnergyModel(move_per_m=1.0, idle_per_s=0.0, speed_mps=1.0, path_factor=1.0, reserve_percent=0.0)


def test_energy_check_keeps_low_robot_off_long_task():
    # #312 첫 Docker 실행: 충전 차례를 기다리던 22 % 로봇이 긴 배송을 받아 충전기 가는 길에 0 %가 됐다.
    # 켜면 그 로봇은 긴 작업에서만 빠지고, 짧은 작업은 받는다
    chargers = (Station('charger_c1', (0.0, -10.0)),)
    for check, expected in ((False, 'amr_01'), (True, 'amr_02')):
        d = make(energy_check=check, energy=UNIT, chargers=chargers)
        d.update_robot(robot('amr_01', (0.0, 0.0), battery=22.0), '', 0.0)
        d.update_robot(robot('amr_02', (30.0, 0.0), battery=90.0), '', 0.0)
        d.add_task(TaskInfo(task_id='long', task_type=0, pickup_xy=(0.0, 10.0), dropoff_xy=(20.0, 10.0),
                            priority=5, created=0.0))
        d.add_task(TaskInfo(task_id='short', task_type=0, pickup_xy=(0.0, -2.0), dropoff_xy=(0.0, -5.0),
                            priority=0, created=0.0))
        commands, rows = d.assign(0.0)
        assert {c.task_id: c.robot_id for c in commands}['long'] == expected
    assert {c.task_id: c.robot_id for c in commands} == {'long': 'amr_02', 'short': 'amr_01'}


def test_energy_check_leaves_task_queued_when_nobody_can():
    chargers = (Station('charger_c1', (0.0, -10.0)),)
    d = make(energy_check=True, energy=UNIT, chargers=chargers)
    d.update_robot(robot('amr_01', battery=22.0), '', 0.0)
    d.add_task(TaskInfo(task_id='long', task_type=0, pickup_xy=(0.0, 10.0), dropoff_xy=(20.0, 10.0),
                        priority=0, created=0.0))
    commands, rows = d.assign(0.0)
    assert commands == [] and [(r.task_id, r.robot_id) for r in rows] == [('long', '')]
    assert d.tasks['long'].state == STATE_PENDING


def test_battery_weight_validation():
    with pytest.raises(ValueError):
        make(battery_weight=-1.0)


def test_energy_check_counts_charge_queue_wait():
    # 2대, N=1: amr_02가 충전 중이라 amr_01은 작업을 마치고 충전 차례를 기다려야 한다.
    # 비교 실행에서 21 %로 끝낸 로봇이 90초 서서 닳아 3.6 %로 충전기에 닿았다 → 대기 소모까지 넣으면 그 작업을 안 준다
    model = EnergyModel(move_per_m=1.0, idle_per_s=0.1, speed_mps=1.0, path_factor=1.0, reserve_percent=0.0)
    chargers = (Station('charger_c1', (0.0, -10.0)),)
    for queue_on, expected in ((False, ['short']), (True, [])):
        d = make(energy_check=True, energy=model, chargers=chargers, charging_enabled=queue_on,
                 charge=ChargeConfig(low_percent=30.0, min_in_service=1), min_battery_percent=10.0)
        d.update_robot(robot('amr_01', (0.0, 0.0), battery=15.0, mode=IDLE), '', 0.0)
        # amr_02: 충전기 위에서 충전 중 (30 → 100 %, 140 s 남음)
        d.update_robot(robot('amr_02', (0.0, -10.0), battery=30.0, mode=BUSY), 'gen_c', 0.0)
        d.add_task(TaskInfo(task_id='gen_c', task_type=TYPE_CHARGE, dropoff_xy=(0.0, -10.0), priority=0, created=0.0))
        d.tasks['gen_c'].assignee = 'amr_02'
        d.tasks['gen_c'].state = STATE_IN_PROGRESS
        d.robots['amr_02'].task_id = 'gen_c'
        d.add_task(TaskInfo(task_id='short', task_type=0, pickup_xy=(0.0, -2.0), dropoff_xy=(0.0, -5.0),
                            priority=0, created=0.0))
        commands, _ = d.assign(0.0)
        assert [c.task_id for c in commands] == expected
    assert d.charge_queue(0.0).remaining_s == pytest.approx((140.0,))


# ---- 피크 대비 (4단계) ----

def _peak(**kwargs):
    charge = ChargeConfig(low_percent=30.0, min_in_service=0, peaks=((300.0, 600.0),), peak_target_percent=60.0)
    return Dispatcher(replace(charging(**kwargs).config, charge=charge))


def _charging_robot(d, battery, now=0.0):
    order = d.plan_charging(now, t=now)
    if order:
        d.assign(now)
        d.on_status(order[0].task_id, 'amr_01', STATE_IN_PROGRESS, now=now)
    d.update_robot(robot('amr_01', (2.0, -10.0), mode=2, battery=battery), d.robots['amr_01'].task_id, now)
    return d.robots['amr_01'].task_id


def test_peak_ends_charge_at_target_and_returns():
    d = _peak()
    d.update_robot(robot('amr_01', (2.0, -10.0), battery=25.0), '', 290.0)
    task_id = _charging_robot(d, 25.0, now=290.0)
    assert d.stop_charging(290.0) == []                 # 피크 전이면 끝까지 채운다
    _charging_robot(d, 55.0, now=310.0)
    assert d.stop_charging(310.0) == []                 # 피크지만 아직 60 % 밑
    _charging_robot(d, 61.0, now=320.0)
    first = d.stop_charging(320.0)
    assert [(c.robot_id, c.task_id, c.first) for c in first] == [('amr_01', task_id, True)]
    assert d.stop_charging(320.5) == []                 # 재전송 간격(1 s) 전
    again = d.stop_charging(321.5)
    assert [(c.assignment_id, c.first) for c in again] == [(first[0].assignment_id, False)]   # 같은 번호로
    # 로봇이 취소 완료를 보고하면 다시 할당하지 않고 대기 위치로 보낸다
    d.on_status(task_id, 'amr_01', STATE_FAILED, error_code=ERROR_CANCELED, now=322.0)
    assert d.tasks[task_id].state == STATE_FAILED
    d.update_robot(robot('amr_01', (2.0, -10.0), battery=61.0), '', 322.0)
    assert [o.task_type for o in d.plan_charging(322.0, t=322.0)] == [TYPE_RETURN]
    assert d.stop_charging(323.0) == []


def test_no_stop_without_peak_target():
    d = charging()
    d.update_robot(robot('amr_01', (2.0, -10.0), battery=25.0), '', 0.0)
    _charging_robot(d, 99.0, now=0.0)
    assert d.stop_charging(0.0) == []


# ---- 작업 중 배터리 부족 인터럽트 (3단계) ----

# 이동 1 %/m (경로 1배, 정지 소모 0)라 필요 배터리 = 남은 직선 거리 [m] + 여유 2 %
_MODEL = EnergyModel(move_per_m=1.0, idle_per_s=0.0, path_factor=1.0, reserve_percent=5.0)


def _interrupt(**kwargs):
    # 충전기 (0, -10), (2, -10). 자동 충전은 꺼 둬도 인터럽트한 로봇은 충전하러 보낸다
    chargers = (Station('charger_c1', (0.0, -10.0)), Station('charger_c2', (2.0, -10.0)))
    kwargs.setdefault('interrupt_enabled', True)
    # 할당 하한은 꺼 둔다 (인터럽트만 본다)
    kwargs.setdefault('min_battery_percent', 0.0)
    return make(chargers=chargers, energy=_MODEL, **kwargs)


def _delivery(task_id='t1'):
    # 픽업 (0, 10) → 배달 (0, 20) → 가장 가까운 충전기 (0, -10)까지 30 m
    return TaskInfo(task_id=task_id, task_type=0, pickup_xy=(0.0, 10.0), dropoff_xy=(0.0, 20.0), priority=0, created=0.0)


def _running(d, battery, phase='move_to_pickup', now=0.0):
    # 원점의 amr_01이 t1을 맡아 픽업으로 가는 중
    d.update_robot(robot('amr_01', (0.0, 0.0), battery=battery), '', now)
    d.add_task(_delivery())
    commands, _ = d.assign(now)
    assert [c.task_id for c in commands] == ['t1']
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, now=now, phase=phase)
    return commands[0]


def test_interrupt_off_by_default_sends_nothing():
    d = _interrupt(interrupt_enabled=False)
    _running(d, battery=10.0)
    assert d.interrupt_low_battery(0.0) == []


def test_interrupt_needs_chargers():
    d = make(interrupt_enabled=True, energy=_MODEL, min_battery_percent=0.0)
    _running(d, battery=10.0)
    assert d.interrupt_low_battery(0.0) == []


def test_enough_battery_not_interrupted():
    # 남은 경로 10 + 10 + 30 = 50 m → 52 %가 필요하다
    d = _interrupt()
    _running(d, battery=52.5)
    assert d.interrupt_low_battery(0.0) == []
    assert d.interrupt_need('t1') == pytest.approx(52.0)


def test_low_battery_before_pickup_cancels_requeues_and_charges():
    d = _interrupt()
    first = _running(d, battery=40.0)
    cancels = d.interrupt_low_battery(0.0)
    assert [(c.robot_id, c.task_id, c.first) for c in cancels] == [('amr_01', 't1', True)]
    assert cancels[0].assignment_id != first.assignment_id
    assert d.interrupting() == {'t1'}
    # 취소 완료 전에는 같은 번호로만 다시 보낸다
    assert d.interrupt_low_battery(0.5) == []
    again = d.interrupt_low_battery(1.0)
    assert [(c.assignment_id, c.first) for c in again] == [(cancels[0].assignment_id, False)]
    # 취소 완료: 작업은 재시도로 세지 않고 큐로, 로봇은 바로 가장 가까운 충전기로 (자동 충전이 꺼져 있어도)
    assert d.on_status('t1', 'amr_01', STATE_FAILED, ERROR_CANCELED, now=1.5)
    t1 = d.tasks['t1']
    assert (t1.state, t1.assignee, t1.failures, t1.interrupts, t1.phase) == (STATE_PENDING, '', 0, 1, '')
    assert d.interrupting() == set()
    orders = d.plan_charging(1.5)
    assert [(o.robot_id, o.task_type, o.station) for o in orders] == [('amr_01', TYPE_CHARGE, 'charger_c1')]
    commands, _ = d.assign(1.5)
    # 그 로봇은 배송이 아니라 충전을 받는다. t1은 큐에 남아 다른 로봇을 기다린다
    assert [(c.robot_id, c.task_id) for c in commands] == [('amr_01', orders[0].task_id)]
    assert d.tasks['t1'].state == STATE_PENDING and not d.tasks['t1'].assignee
    d.update_robot(robot('amr_02', (1.0, 1.0)), '', 2.0)
    commands, _ = d.assign(2.0)
    assert [(c.robot_id, c.task_id) for c in commands] == [('amr_02', 't1')]


def test_interrupted_robot_charges_regardless_of_min_in_service():
    # 최소 운영 5대(동시 충전 0대)여도 인터럽트한 로봇은 충전하러 간다
    d = _interrupt(charging_enabled=True, charge=ChargeConfig(low_percent=30.0, min_in_service=5))
    _running(d, battery=40.0)
    d.interrupt_low_battery(0.0)
    d.on_status('t1', 'amr_01', STATE_FAILED, ERROR_CANCELED, now=1.0)
    assert [o.robot_id for o in d.plan_charging(1.0)] == ['amr_01']


def test_interrupted_robot_waits_for_free_charger():
    d = _interrupt()
    _running(d, battery=40.0)
    d.interrupt_low_battery(0.0)
    d.on_status('t1', 'amr_01', STATE_FAILED, ERROR_CANCELED, now=1.0)
    # 충전기 두 곳 모두 다른 로봇이 쓰는 중
    for i, name in enumerate(('charger_c1', 'charger_c2'), start=2):
        d.update_robot(robot(f'amr_0{i}', battery=10.0), '', 1.0)
        d._add_directed(f'amr_0{i}', TYPE_CHARGE, Station(name, (0.0, -10.0)), 1.0)
    assert d.plan_charging(1.0) == []
    # 기다리는 동안 배송을 주지 않는다
    d.add_task(_delivery('t2'))
    commands, _ = d.assign(1.0)
    assert all(c.robot_id != 'amr_01' for c in commands)
    # 충전기 하나가 비면 간다
    busy = [t for t in d.tasks.values() if t.for_robot == 'amr_02'][0]
    d.on_status(busy.info.task_id, 'amr_02', STATE_COMPLETED, now=2.0)
    d.update_robot(robot('amr_01', (0.0, 0.0), battery=39.0), '', 2.0)
    assert [o.robot_id for o in d.plan_charging(2.0)] == ['amr_01']


@pytest.mark.parametrize('phase', ['loading', 'move_to_dropoff', 'docking', 'unloading'])
def test_loaded_task_not_interrupted(phase):
    d = _interrupt()
    _running(d, battery=5.0, phase=phase)
    assert d.interrupt_low_battery(0.0) == []


def test_interrupt_withdrawn_when_robot_loads_or_finishes():
    d = _interrupt()
    _running(d, battery=40.0)
    assert d.interrupt_low_battery(0.0)
    # 취소가 닿기 전에 짐을 실었다: 더 보내지 않고, 이후 보고는 평소대로
    d.on_status('t1', 'amr_01', STATE_IN_PROGRESS, now=0.5, phase='loading')
    assert d.interrupt_low_battery(2.0) == [] and d.interrupting() == set()
    d.on_status('t1', 'amr_01', STATE_COMPLETED, now=3.0)
    assert d.tasks['t1'].state == STATE_COMPLETED and d.tasks['t1'].interrupts == 0


def test_robot_failure_during_interrupt_counts_as_normal_failure():
    d = _interrupt()
    _running(d, battery=40.0)
    d.interrupt_low_battery(0.0)
    # 취소가 아닌 다른 실패(Nav 등)는 원래 규칙: 재시도 1회
    d.on_status('t1', 'amr_01', STATE_FAILED, 301, now=1.0)
    t1 = d.tasks['t1']
    assert (t1.state, t1.failures, t1.interrupts) == (STATE_PENDING, 1, 0)
    assert d.interrupting() == set() and d.plan_charging(1.0) == []


def test_server_charge_tasks_never_interrupted():
    d = _interrupt(charging_enabled=True, charge=ChargeConfig(low_percent=30.0))
    d.update_robot(robot('amr_01', battery=1.0), '', 0.0)
    orders = d.plan_charging(0.0)
    d.assign(0.0)
    d.on_status(orders[0].task_id, 'amr_01', STATE_IN_PROGRESS, now=0.0, phase='')
    assert d.interrupt_low_battery(0.0) == []


def test_negative_interrupt_reserve_rejected():
    with pytest.raises(ValueError):
        make(interrupt_reserve_percent=-1.0)
