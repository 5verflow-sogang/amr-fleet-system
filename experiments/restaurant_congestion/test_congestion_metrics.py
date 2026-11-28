"""식당 시연 1-1 켬/끔 지표. 가짜 결과 폴더로 시험한다.

    python3 -m pytest experiments/restaurant_congestion
"""

import csv
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze  # noqa: E402
from congestion_metrics import State, busy_times, other_robot_choices, run_metrics  # noqa: E402

STATIONS = {'buffet_1': (5.0, 0.0), 't01': (20.0, 0.0)}
CROWD = [(2.0, 4.0, -1.0, 1.0)]


def _csv(path, header, rows):
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def test_busy_times_counts_only_busy_and_converts_to_sim():
    # amr_01: BUSY로 0~4 s 1 m/s(혼잡 구역 x 2~4를 지남), 4~8 s 멈춤. RTF 0.5라 시뮬레이션 길이는 절반
    series = [State(float(t), min(t, 4) * 1.0, 0.0, 1) for t in range(0, 9)]
    # amr_02: IDLE로 내내 멈춤 → 세지 않는다
    idle = [State(float(t), 10.0, 10.0, 0) for t in range(0, 9)]
    stopped, in_crowd = busy_times({'amr_01': series, 'amr_02': idle}, CROWD, rtf=0.5)
    assert stopped == pytest.approx(4 * 0.5)
    # x 2, 3, 4에서 시작하는 구간(2~3, 3~4, 4~5 s)과 x 4에 서 있는 동안(5~8 s)이 구역 안
    assert in_crowd == pytest.approx(6 * 0.5)


def test_other_robot_choice_needs_two_idle_robots():
    # t=10에 task1(buffet_1, x 5)이 amr_02(x 10, 5 m)에 할당. 가장 가까운 건 amr_01(x 0, 5 m)… 동률이면 다른 로봇 아님
    states = {'amr_01': [State(9.5, 0.0, 0.0, 0)], 'amr_02': [State(9.5, 10.0, 0.0, 0)],
              'amr_03': [State(9.5, 6.0, 0.0, 1)]}
    outcomes = [{'task_id': 'tie', 'first_assign_time': '10.0', 'robots': 'amr_02'},
                {'task_id': 'far', 'first_assign_time': '10.0', 'robots': 'amr_02'},
                {'task_id': 'near', 'first_assign_time': '10.0', 'robots': 'amr_01'},
                {'task_id': 'stale', 'first_assign_time': '20.0', 'robots': 'amr_01'}]
    schedule = {'tie': {'pickup': 'buffet_1'}, 'far': {'pickup': 'near_0'}, 'near': {'pickup': 'near_0'},
                'stale': {'pickup': 'buffet_1'}}
    stations = {**STATIONS, 'near_0': (1.0, 0.0)}
    # tie: 동률 → 다른 로봇 아님 / far: amr_02(9 m)는 amr_01(1 m)보다 멀다 → 다른 로봇 / near: 가장 가까움
    # stale: 상태가 10 s 넘게 지나 놀고 있는 로봇을 모른다 → 분모에서 빠진다. amr_03은 BUSY라 후보가 아니다
    assert other_robot_choices(outcomes, schedule, states, stations) == (3, 1)


def test_run_metrics_and_analyze_table(tmp_path):
    raw = tmp_path / 'raw'
    for cond, chosen, wait in (('off', 'amr_01', 1.0), ('on', 'amr_02', 2.0)):
        run = raw / cond / 'run_00'
        run.mkdir(parents=True)
        (run / 'meta.json').write_text(json.dumps({'fleet_clock': {'durations': 'sim', 'rtf': 0.5}}))
        (run / 'tasks.json').write_text(json.dumps({'tasks': [{'task_id': 't1', 'pickup': 'buffet_1'}]}))
        _csv(run / 'task_outcomes.csv',
             ['task_id', 'request_time', 'first_assign_time', 'robots', 'final_state', 'end_time', 'task_time_s',
              'distance_m', 'error_code', 'runners', 'request_sim', 'end_sim'],
             [['t1', '10.0', f'{10.0 + wait}', chosen, 'COMPLETED', '70.0', '30.0', '', '', chosen, '5.0', '35.0']])
        _csv(run / 'robot_states.csv', ['time', 'robot', 'x', 'y', 'mode', 'task_id'],
             [['9.0', 'amr_01', '0.0', '0.0', '0', ''], ['9.0', 'amr_02', '5.0', '6.0', '0', '']])
    m = run_metrics(raw / 'on' / 'run_00', STATIONS, CROWD)
    assert (m['tasks'], m['completed'], m['wait_mean'], m['task_time_mean']) == (1, 1, 1.0, 30.0)
    # amr_01 5 m, amr_02 6 m → amr_02는 가까운 로봇이 아니다
    assert (m['choice_eligible'], m['choice_other'], m['choice_other_rate']) == (1, 1, 1.0)
    off = run_metrics(raw / 'off' / 'run_00', STATIONS, CROWD)
    assert off['choice_other_rate'] == 0.0 and math.isnan(off['task_time_p95']) is False

    stations = tmp_path / 'stations.yaml'
    stations.write_text('stations:\n  buffet_1: [5.0, 0.0, 0.0]\n')
    zones = tmp_path / 'zones.yaml'
    zones.write_text('zones:\n  - {id: crowd, area: [2.0, 4.0, -1.0, 1.0], capacity: 2}\n')
    assert analyze.main([str(raw), '--out', str(tmp_path / 'out'), '--stations', str(stations),
                         '--zones', str(zones), '--crowd', 'crowd']) == 0
    table = (tmp_path / 'out' / 'results.md').read_text(encoding='utf-8')
    assert '| off | 1 | 1 / 1 |' in table and '| on | 1 | 1 / 1 |' in table
    assert 'off 1건, on 1건' in table
