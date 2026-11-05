"""collect_aging 지표 (rclpy 없이). python3 -m pytest experiments/fleet_collect_aging"""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from collect_aging import (TaskRow, load_run, run_condition, schedule_offset, starvation, summarize,  # noqa: E402
                           task_rows)

SCHEDULE = [
    {'task_id': 'n1', 'type': 'delivery', 'priority': 100},
    {'task_id': 'n2', 'type': 'delivery', 'priority': 100},
    {'task_id': 'c1', 'type': 'collect', 'priority': 200},
    {'task_id': 'late', 'type': 'delivery', 'priority': 100},
]


def outcome(task_id, request, assign='', state='', end=''):
    return {'task_id': task_id, 'request_time': str(request), 'first_assign_time': str(assign),
            'final_state': state, 'end_time': str(end)}


OUTCOMES = [
    outcome('n1', 100.0, 100.5, 'COMPLETED', 130.0),
    outcome('n2', 110.0, 170.0),                       # 할당은 됐지만 마감까지 못 끝냄
    outcome('c1', 120.0, 125.0, 'COMPLETED', 150.0),
    outcome('late', 250.0),                            # 마감(200) 뒤 요청 - 뺀다
]


def test_task_rows_kind_wait_and_cutoff():
    rows = {r.task_id: r for r in task_rows(OUTCOMES, SCHEDULE, cutoff=200.0)}
    assert set(rows) == {'n1', 'n2', 'c1'}
    assert (rows['c1'].kind, rows['c1'].priority, rows['c1'].wait) == ('collect', 200, 5.0)
    assert rows['n2'].wait == pytest.approx(60.0) and not rows['n2'].completed
    assert rows['n1'].completed


def test_assignment_after_cutoff_counts_as_unassigned():
    rows = task_rows([outcome('n1', 100.0, 210.0)], SCHEDULE, cutoff=200.0)
    assert rows[0].assign is None and rows[0].wait is None
    s = summarize(rows, cutoff=200.0)
    assert s['normal_unassigned'] == 1 and math.isnan(s['normal_wait_mean'])


def test_summarize_waits_unassigned_and_throughput():
    rows = task_rows(OUTCOMES + [outcome('extra', 190.0)],
                     SCHEDULE + [{'task_id': 'extra', 'type': 'delivery', 'priority': 100}], cutoff=200.0)
    s = summarize(rows, cutoff=200.0)
    assert s['collect_wait_mean'] == 5.0 and s['collect_wait_max'] == 5.0
    assert s['normal_wait_mean'] == pytest.approx((0.5 + 60.0) / 2) and s['normal_wait_max'] == 60.0
    assert (s['unassigned'], s['normal_unassigned'], s['collect_unassigned']) == (1, 1, 0)
    # 완료 2건 / (200 − 100) s × 60
    assert s['completed'] == 2 and s['throughput_per_min'] == pytest.approx(1.2)


def test_load_run_reads_harness_files(tmp_path):
    (tmp_path / 'meta.json').write_text(json.dumps({'seed': 3, 'observation_cutoff': {'cutoff': 200.0}}))
    (tmp_path / 'tasks.json').write_text(json.dumps({'meta': {}, 'tasks': SCHEDULE}))
    header = 'task_id,request_time,first_assign_time,robots,final_state,end_time,task_time_s,distance_m,error_code,runners\n'
    (tmp_path / 'task_outcomes.csv').write_text(header + 'c1,120.0,125.0,amr_01,COMPLETED,150.0,30,40,,amr_01\n')
    rows, cutoff, seed = load_run(tmp_path)
    assert (cutoff, seed, [r.task_id for r in rows]) == (200.0, 3, ['c1'])


def normal(task_id, request, assign):
    return TaskRow(task_id, 'normal', 100, request, assign, False)


def test_starvation_is_the_longest_gap_while_a_normal_task_waits():
    # 일반 작업 할당: 10, 20 (그 뒤 30에 요청한 n3은 100에 할당). 30~100이 가장 긴 기아(70 s)
    rows = [normal('n1', 0.0, 10.0), normal('n2', 5.0, 20.0), normal('n3', 30.0, 100.0),
            TaskRow('c1', 'collect', 200, 40.0, 41.0, False)]
    assert starvation(rows, (50.0, 60.0), cutoff=200.0) == pytest.approx(70.0)
    # 아무도 안 기다리는 빈 시간(20~30)은 기아가 아니다. 몰림 구간과 안 겹치는 기아는 뺀다
    assert starvation(rows, (21.0, 29.0), cutoff=200.0) == 0.0
    # 기아 구간 끝은 몰림 구간에서 자르지 않는다
    assert starvation(rows, (95.0, 96.0), cutoff=200.0) == pytest.approx(70.0)


def test_starvation_runs_to_cutoff_when_a_normal_task_is_never_assigned():
    rows = [normal('n1', 0.0, 10.0), normal('n2', 50.0, None)]
    assert starvation(rows, (60.0, 180.0), cutoff=300.0) == pytest.approx(250.0)
    assert starvation([normal('n1', 0.0, 10.0)], (60.0, 180.0), cutoff=300.0) == 0.0


def test_schedule_offset_maps_schedule_time_to_request_time():
    schedule = [{'task_id': 'a', 'at_s': 5.0}, {'task_id': 'b', 'at_s': 17.0}, {'task_id': 'c', 'at_s': 30.0}]
    outcomes = [outcome('a', 1005.0), outcome('b', 1017.1), outcome('c', 1029.9)]
    assert schedule_offset(outcomes, schedule) == pytest.approx(1000.0)


def test_run_condition_from_meta_params():
    # 첫 측정: load 없음, aging off/on
    assert run_condition({'aging': 'off'}, 1.667) == ('overload', 0.0)
    assert run_condition({'aging': 'on', 'aging_rate': ''}, 1.667) == ('overload', 1.667)
    # aging_rate를 직접 주면 그 값
    assert run_condition({'aging': 'off', 'aging_rate': '0.833', 'load': 'collect_burst'}, 1.667) ==         ('collect_burst', 0.833)
