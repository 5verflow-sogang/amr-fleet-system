"""실행 폴더 요약 (ROS 없이). 가짜 실행 폴더를 만들어 묶기·실패 제외·KPI 판정을 본다."""

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.runner import fault_meta, render  # noqa: E402
from harness.summary import fleet_metrics, kpi_verdict, nav_metrics, summarize, to_markdown  # noqa: E402


def make_run(root: Path, name: str, scenario='loc_basic', label='', outcome='completed', rmse=0.02):
    run = root / name
    (run / 'eval_ekf').mkdir(parents=True)
    (run / 'meta.json').write_text(json.dumps(
        {'scenario': scenario, 'label': label, 'outcome': outcome, 'real_time_factor': 1.0}))
    seg = {'n': 500, 'rmse': rmse, 'mean': rmse, 'max': rmse * 2}
    (run / 'eval_ekf' / 'localization_summary.json').write_text(json.dumps(
        {'robot': 'amr_01', 'segments': {'stationary': seg, 'straight': seg, 'rotation': None}}))
    return run


def test_groups_by_label_and_excludes_failed(tmp_path):
    runs = [make_run(tmp_path, f'a{i}', rmse=0.02 + 0.001 * i) for i in range(5)]
    runs += [make_run(tmp_path, f'b{i}', label='q01', rmse=0.04) for i in range(3)]
    runs.append(make_run(tmp_path, 'a_fail', outcome='timeout', rmse=9.0))
    s = summarize(runs)
    assert set(s) == {'loc_basic', 'loc_basic/q01'}
    a = s['loc_basic']
    assert len(a['runs']) == 5 and a['failed'] == [{'run_id': 'a_fail', 'outcome': 'timeout'}]
    m = a['metrics']['loc.amr_01.eval_ekf.stationary.rmse']
    assert m['n'] == 5 and m['mean'] == pytest.approx(0.022)
    assert m['kpi'] == 'PASS'
    # 0.04 m는 정지 KPI 3 cm를 넘고 직진 5 cm 안이다
    b = s['loc_basic/q01']['metrics']
    assert b['loc.amr_01.eval_ekf.stationary.rmse']['kpi'] == 'FAIL'
    assert b['loc.amr_01.eval_ekf.straight.rmse']['kpi'] == 'PASS'
    md = to_markdown(s)
    assert '## loc_basic/q01' in md and '실패 1회' in md


def test_kpi_verdict_marginal():
    # 평균은 기준 안이지만 CI 상한이 넘으면 MARGINAL
    assert kpi_verdict('loc.amr_01.est.stationary.rmse', 0.029, 0.033) == 'MARGINAL'
    assert kpi_verdict('loc.amr_01.est.stationary.max', 0.5, 0.6) is None


def test_fleet_metrics(tmp_path):
    with (tmp_path / 'fleet_latency.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['topic', 'robot', 'key', 'send_time', 'recv_time', 'latency_ms', 'delivered'])
        w.writerow(['heartbeat', 'amr_01', '1:0', '1.0', '1.2', '200.0', '1'])
        w.writerow(['heartbeat', 'amr_02', '1:0', '1.0', '', '', '0'])
        w.writerow(['heartbeat', 'amr_01', '1:9', '99.0', '', '', ''])   # 관측 마감 뒤: 판정 안 함
    with (tmp_path / 'response_time.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['cmd_time', 'response_time', 'latency_ms', 'robot', 'task_id', 'latency_sim_ms',
                    'assignment_id', 'ack_time', 'ack_rtt_ms', 'resends'])
        w.writerow(['1.0', '', '', 'amr_01', 't1', '', '1', '1.25', '250.0', '0'])
        w.writerow(['2.0', '', '', 'amr_02', 't2', '', '2', '', '', '5'])
    m = fleet_metrics(tmp_path)
    assert m['fleet.loss_rate'] == pytest.approx(0.5)
    assert m['fleet.latency_ms.mean'] == pytest.approx(200.0)
    assert m['fleet.ack_rate'] == pytest.approx(0.5)
    assert m['fleet.resends.mean'] == pytest.approx(2.5)
    assert 'fleet.response_ms.mean' not in m   # 가짜 로봇: odom 없음


def test_render_and_fault_meta():
    params = {'run_dir': '/r/x', 'seed': 3}
    assert render({'cmd': 'a {seed} {run_dir}/f', 'l': ['{seed}']}, params) == {'cmd': 'a 3 /r/x/f', 'l': ['3']}
    assert fault_meta({}) is None
    f = fault_meta({'delay_ms': '200', 'loss_pct': '5'})
    assert f == {'delay_ms': 200.0, 'loss_pct': 5.0, 'direction': 'server_to_robot',
                 'kill_at_s': -1.0, 'restart_after_s': -1.0, 'kill_robot': 'none', 'kill_robot_at_s': -1.0}
    f = fault_meta({'delay_ms': '0', 'kill_robot': 'amr_03', 'kill_robot_at_s': '120'})
    assert (f['kill_robot'], f['kill_robot_at_s']) == ('amr_03', 120.0)


def test_fleet_task_and_recovery_metrics(tmp_path):
    run = tmp_path / 'r'
    run.mkdir()
    (run / 'task_outcomes.csv').write_text(
        'task_id,request_time,first_assign_time,robots,final_state,end_time\n'
        't1,0,1,amr_01,COMPLETED,8\nt2,10,11,amr_01 amr_02,COMPLETED,40\nt3,50,,,,\nt4,51,52,amr_03,,\n')
    m = fleet_metrics(run)
    assert (m['fleet.tasks_requested'], m['fleet.tasks_completed'], m['fleet.task_completion_rate']) == (4, 2, 0.5)
    assert 'fleet.reclaimed_tasks' not in m      # 로봇 장애 없는 실행

    (run / 'recovery.csv').write_text(
        'robot,kill_time,task_id,reassign_time,reassigned_robot,recovery_s,completed\n'
        'amr_01,15,t2,21,amr_02,6.000,1\namr_01,15,t9,,,,0\n')
    m = fleet_metrics(run)
    assert m['fleet.reclaimed_tasks'] == 2
    assert (m['fleet.recovery_s.mean'], m['fleet.recovery_s.max']) == (6.0, 6.0)
    assert m['fleet.reclaimed_completed_rate'] == 0.5

    (run / 'recovery.csv').write_text('robot,kill_time,task_id,reassign_time,reassigned_robot,recovery_s,completed\n'
                                      'amr_03,15,,,,,\n')       # 놀던 로봇이 죽음
    m = fleet_metrics(run)
    assert m['fleet.reclaimed_tasks'] == 0 and 'fleet.recovery_s.mean' not in m



def test_nav_metrics(tmp_path):
    # 목표 주행 1회 결과(nav_result.json): 성공은 1/0, 값이 없는 지표(장애물 없는 시나리오)는 뺀다
    ok = tmp_path / 'ok'
    ok.mkdir()
    (ok / 'nav_result.json').write_text(json.dumps({
        'success': True, 'arrival_s': 12.5, 'min_obstacle_m': None, 'path_length_m': 3.4,
        'cte_mean_m': 0.02, 'cte_max_m': 0.08}))
    m = nav_metrics(ok)
    assert m == {'nav.success': 1.0, 'nav.arrival_s': 12.5, 'nav.path_length_m': 3.4,
                 'nav.cte_mean_m': 0.02, 'nav.cte_max_m': 0.08}
    bad = tmp_path / 'bad'
    bad.mkdir()
    (bad / 'nav_result.json').write_text(json.dumps({
        'success': False, 'arrival_s': None, 'min_obstacle_m': 0.12, 'path_length_m': 1.0}))
    assert nav_metrics(bad) == {'nav.success': 0.0, 'nav.min_obstacle_m': 0.12, 'nav.path_length_m': 1.0}
    assert nav_metrics(tmp_path / 'none') == {}
