"""collect_aging 지표 (rclpy 없이). python3 -m pytest experiments/fleet_collect_aging"""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from collect_aging import load_run, summarize, task_rows  # noqa: E402

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
