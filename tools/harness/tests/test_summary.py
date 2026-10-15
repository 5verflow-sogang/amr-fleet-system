"""실행 폴더 요약 (ROS 없이). 가짜 실행 폴더를 만들어 묶기·실패 제외·KPI 판정을 본다."""

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.runner import fault_meta, render  # noqa: E402
from harness.summary import (avoidance_metrics, fleet_metrics, kidnap_metrics, kpi_verdict,  # noqa: E402
                             nav_kpi_metrics, nav_metrics, summarize, to_markdown)


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


def write_nav(root: Path, name: str, **result) -> Path:
    run = root / name
    run.mkdir()
    (run / 'nav_result.json').write_text(json.dumps(result))
    return run


def test_nav_metrics(tmp_path):
    # 성공: 도착 시간이 arrival_s(성공만)·reached_s(Nav2 도착 모두) 둘 다. 값이 없는 지표는 뺀다
    ok = write_nav(tmp_path, 'ok', result='succeeded', success=True, collision=False, reached_s=12.5,
                   arrival_s=12.5, min_obstacle_m=None, path_length_m=3.4, cte_mean_m=0.02, cte_max_m=0.08)
    assert nav_metrics(ok) == {'nav.success': 1.0, 'nav.arrival_s': 12.5, 'nav.reached_s': 12.5,
                               'nav.path_length_m': 3.4, 'nav.cte_mean_m': 0.02, 'nav.cte_max_m': 0.08}
    # 충돌했지만 Nav2는 도착: reached_s만, arrival_s 평균에는 안 섞인다 (#145 민지 리뷰)
    hit = write_nav(tmp_path, 'hit', result='succeeded', success=False, collision=True, reached_s=39.5,
                    arrival_s=39.5, min_obstacle_m=0.0, path_length_m=4.4)
    m = nav_metrics(hit)
    assert m['nav.success'] == 0.0 and m['nav.reached_s'] == 39.5 and 'nav.arrival_s' not in m
    # reached_s가 없는 옛 결과: result가 succeeded면 arrival_s를 Nav2 도착 시간으로 본다
    old = write_nav(tmp_path, 'old', result='succeeded', success=False, arrival_s=7.2, min_obstacle_m=0.0)
    m = nav_metrics(old)
    assert m['nav.reached_s'] == 7.2 and 'nav.arrival_s' not in m
    # 도착 못 함
    bad = write_nav(tmp_path, 'bad', result='aborted', success=False, arrival_s=None, min_obstacle_m=0.12,
                    path_length_m=1.0)
    assert nav_metrics(bad) == {'nav.success': 0.0, 'nav.min_obstacle_m': 0.12, 'nav.path_length_m': 1.0}
    assert nav_metrics(tmp_path / 'none') == {}


def test_kidnap_metrics_success_rate(tmp_path):
    def trial(name, **k):
        run = tmp_path / name
        (run / 'kidnap').mkdir(parents=True)
        (run / 'meta.json').write_text(json.dumps({'scenario': 'kidnap', 'label': 'near', 'outcome': 'completed'}))
        (run / 'kidnap' / 'kidnap.json').write_text(json.dumps(k))
        return run

    ok = dict(error=None, success=True, detected=True, recovered=True, detect_s=1.5, recover_s=9.5, false_alarms=0)
    lost = dict(error=None, success=False, detected=True, recovered=False, detect_s=2.0, recover_s=None, false_alarms=0)
    runs = [trial('r0', **ok), trial('r1', **ok), trial('r2', **lost), trial('r3', error='set_pose 실패'),
            trial('r4', **{**ok, 'valid': False})]   # 순간 이동 때 이미 복구 중 → 분모에서 뺌
    assert kidnap_metrics(runs[0]) == {'kidnap.valid': 1.0, 'kidnap.success': 1.0, 'kidnap.detected': 1.0,
                                       'kidnap.recovered': 1.0, 'kidnap.late_recovered': 0.0,
                                       'kidnap.false_alarms': 0.0, 'kidnap.detect_s': 1.5, 'kidnap.recover_s': 9.5}
    assert 'kidnap.recover_s' not in kidnap_metrics(runs[2])
    assert kidnap_metrics(runs[3]) == {}   # 시험이 안 된 실행은 성공률 분모에서 뺀다
    assert kidnap_metrics(runs[4]) == {'kidnap.valid': 0.0}
    m = summarize(runs)['kidnap/near']['metrics']
    assert m['kidnap.success']['n'] == 3
    assert m['kidnap.success']['mean'] == pytest.approx(2 / 3)
    assert m['kidnap.recover_s']['n'] == 2


def _csv(path, header, rows):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def test_fleet_stack_metrics(tmp_path):
    # 응답 시간 두 기준, 작업 시간·이동 거리, 구역 대기·교착, 정체 (fleet_stack 시나리오의 지표 파일)
    _csv(tmp_path / 'response_time.csv',
         ['cmd_time', 'response_time', 'latency_ms', 'robot', 'task_id', 'latency_sim_ms', 'assignment_id',
          'ack_time', 'ack_rtt_ms', 'resends', 'recv_time', 'latency_recv_ms'],
         [['1.0', '1.3', '300.0', 'amr_01', 't1', '', '1', '1.2', '200.0', '0', '1.1', '200.0'],
          ['2.0', '2.25', '250.0', 'amr_02', 't2', '', '2', '2.2', '200.0', '0', '2.1', '150.0']])
    _csv(tmp_path / 'task_outcomes.csv',
         ['task_id', 'request_time', 'first_assign_time', 'robots', 'final_state', 'end_time', 'task_time_s',
          'distance_m'],
         [['t1', '0', '1', 'amr_01', 'COMPLETED', '100', '100.0', '40.0'],
          ['t2', '0', '2', 'amr_02', 'COMPLETED', '60', '60.0', '20.0'],
          ['t3', '0', '', '', '', '', '', '']])
    _csv(tmp_path / 'zone_waits.csv',
         ['robot', 'zone', 'request_id', 'request_time', 'grant_time', 'wait_s', 'outcome', 'release_time'],
         [['amr_01', 'narrow_01', '1', '0', '5', '5.0', 'granted', '20'],
          ['amr_02', 'narrow_01', '1', '0', '', '45.0', 'waiting', ''],
          ['amr_03', 'narrow_02', '1', '0', '', '0.0', 'rejected', ''],
          ['amr_04', 'narrow_02', '2', '0', '1', '0.0', 'revoked', '9']])
    _csv(tmp_path / 'stalls.csv', ['robot', 'task_id', 'phase', 'start_time', 'end_time', 'duration_s'], [])
    m = fleet_metrics(tmp_path)
    assert m['fleet.response_ms.mean'] == pytest.approx(275.0)
    assert m['fleet.response_recv_ms.mean'] == pytest.approx(175.0)
    assert (m['fleet.task_time_s.mean'], m['fleet.distance_m.total']) == (pytest.approx(80.0), pytest.approx(60.0))
    assert (m['fleet.zone_requests'], m['fleet.zone_long_waits']) == (4, 1)
    assert (m['fleet.deadlock_rejects'], m['fleet.deadlock_revokes']) == (1, 1)
    assert m['fleet.stalls'] == 0 and 'fleet.stall_s.max' not in m
    # KPI: 수신 기준 200 ms 안, 장기 대기 1건은 FAIL, 정체 0건은 PASS
    assert kpi_verdict('fleet.response_recv_ms.mean', 175.0, 190.0) == 'PASS'
    assert kpi_verdict('fleet.zone_long_waits', 1.0, 1.0) == 'FAIL'
    assert kpi_verdict('fleet.stalls', 0.0, 0.0) == 'PASS'
    # 교착을 일부러 만드는 시나리오에서는 판정하지 않는다
    assert kpi_verdict('fleet.zone_long_waits', 1.0, 1.0, scenario='fleet_deadlock') is None
    assert kpi_verdict('fleet.response_recv_ms.mean', 175.0, 190.0, scenario='fleet_deadlock') == 'PASS'


def _stack_run(root, name, n, latency_ms=150.0, scenario='fleet_stack'):
    """응답 시간 n건 (그중 1건은 명령 순간 움직이던 로봇) + 움직임을 못 찾은 명령 1건."""
    run = root / name
    run.mkdir()
    (run / 'meta.json').write_text(json.dumps({'scenario': scenario, 'label': '', 'outcome': 'completed'}))
    rows = [['1.0', '1.2', f'{latency_ms}', 'amr_01', f't{i}', '120.0', str(i), '', '', '0', '1.1',
             f'{latency_ms - 50}', '1' if i == 0 else '0'] for i in range(n)]
    rows.append(['9.0', '', '', 'amr_02', 'tx', '', '99', '', '', '0', '', '', ''])
    _csv(run / 'response_time.csv',
         ['cmd_time', 'response_time', 'latency_ms', 'robot', 'task_id', 'latency_sim_ms', 'assignment_id',
          'ack_time', 'ack_rtt_ms', 'resends', 'recv_time', 'latency_recv_ms', 'moving_at_cmd'], rows)
    return run


def test_response_extras_and_min_samples(tmp_path):
    m = fleet_metrics(_stack_run(tmp_path, 'one', 4))
    assert m['fleet.response_sim_ms.mean'] == pytest.approx(120.0)
    assert m['fleet.response_coverage'] == pytest.approx(4 / 5)
    assert m['fleet.response_moving_at_cmd'] == 1
    # 실행 3회 × 움직임 20건 = 60건이면 판정, 2회 × 20건 = 40건이면 보류 (명세서 "50회 이상")
    enough = summarize([_stack_run(tmp_path, f'a{i}', 20) for i in range(3)])['fleet_stack']['metrics']
    assert enough['fleet.response_recv_ms.mean']['kpi'] == 'PASS'
    short = summarize([_stack_run(tmp_path, f'b{i}', 20, scenario='fleet_short') for i in range(2)])
    d = short['fleet_short']['metrics']['fleet.response_ms.mean']
    assert 'kpi' not in d and d['kpi_note'] == '보류 (n=40<50)'
    assert '보류 (n=40<50)' in to_markdown(short)


def test_nav_kpi_metrics_and_verdict(tmp_path):
    def run(name, cte_kpi, plan):
        d = tmp_path / name / 'nav_kpi'
        d.mkdir(parents=True)
        (tmp_path / name / 'meta.json').write_text(json.dumps({'scenario': 'nav_kpi', 'label': name[:-1],
                                                               'outcome': 'completed'}))
        (d / 'nav_kpi.json').write_text(json.dumps({
            'plan_success_rate': plan, 'nav_success_rate': 0.9, 'replan_p95_ms': 13.0, 'replan_max_ms': 39.0,
            'pred_err_mean': 0.055, 'pred_err_median': 0.05, 'pred_err_no_recovery_mean': float('nan'),
            'cte_straight_mean_m': 0.063, 'cte_curve_mean_m': 0.067, 'definition': {'cte_kpi': cte_kpi}}))
        return tmp_path / name
    pp = nav_kpi_metrics(run('pp0', True, 1.0))
    assert pp['navkpi.cte_straight_mean_m'] == 0.063 and 'navkpi.pred_err_no_recovery_mean' not in pp
    dwa = nav_kpi_metrics(run('dwa0', False, 1.0))
    assert 'navkpi.cte_straight_mean_m' not in dwa and dwa['navkpi.ref_cte_straight_mean_m'] == 0.063
    # 클수록 좋은 지표는 CI 하한으로
    assert kpi_verdict('navkpi.plan_success_rate', 1.0, 1.0, 0.99) == 'PASS'
    assert kpi_verdict('navkpi.plan_success_rate', 0.99, 1.0, 0.96) == 'MARGINAL'
    assert kpi_verdict('navkpi.plan_success_rate', 0.95, 0.97, 0.93) == 'FAIL'
    assert kpi_verdict('navkpi.cte_straight_mean_m', 0.063, 0.07, 0.05) == 'FAIL'
    assert kpi_verdict('navkpi.ref_cte_straight_mean_m', 0.063, 0.07, 0.05) is None
    m = summarize([run('pp1', True, 1.0), run('pp2', True, 1.0)])['nav_kpi/pp']['metrics']
    assert m['navkpi.replan_p95_ms']['kpi'] == 'PASS' and m['navkpi.cte_straight_mean_m']['kpi'] == 'FAIL'


def test_failure_causes_by_error_code(tmp_path):
    run = tmp_path / 'r'
    run.mkdir()
    (run / 'task_outcomes.csv').write_text(
        'task_id,request_time,first_assign_time,robots,final_state,end_time,error_code\n'
        't1,0,1,amr_01,COMPLETED,10,0\n'
        't2,0,1,amr_01,FAILED,10,203\n'
        't3,0,1,amr_02,FAILED,10,202\n'
        't4,0,1,amr_03,FAILED,10,904\n')
    m = fleet_metrics(run)
    assert m['fleet.tasks_failed_localization'] == 1 and m['fleet.tasks_failed_nav'] == 1
    assert m['fleet.tasks_failed_docking'] == 1 and m['fleet.tasks_failed_traffic'] == 0
    # 열이 없던 예전 결과는 원인 지표를 내지 않는다
    (run / 'task_outcomes.csv').write_text('task_id,request_time,first_assign_time,robots,final_state,end_time\n'
                                           't1,0,1,amr_01,FAILED,10\n')
    assert not [k for k in fleet_metrics(run) if k.startswith('fleet.tasks_failed_')]


def test_avoidance_metrics(tmp_path):
    run = tmp_path / 'r'
    (run / 'avoidance').mkdir(parents=True)
    assert avoidance_metrics(run) == {}
    summary = run / 'avoidance' / 'avoidance_summary.json'
    # 유효 실행, 복귀 못 한 회피 1건: max_return_s는 비지만 unreturned로 최악 실행이 남는다
    summary.write_text(json.dumps({
        'robot': 'amr_01', 'collision': {'count': 0, 'min_clearance': 0.15},
        'deviation': {'max': 0.6}, 'avoidance': {'max_return_s': None, 'unreturned': 1},
        'kpi': {'valid': True, 'pass': False}}))
    m = avoidance_metrics(run)
    assert m == {'avoid.amr_01.collisions': 0, 'avoid.amr_01.unreturned': 1, 'avoid.amr_01.min_clearance': 0.15,
                 'avoid.amr_01.max_deviation': 0.6, 'avoid.amr_01.valid': 1.0, 'avoid.amr_01.pass': 0.0}
    assert kpi_verdict('avoid.amr_01.max_deviation', 0.6, 0.7) is None     # 판정은 avoidance_report
    # 무효 실행은 pass를 넣지 않는다 (합격률이 avoidance_report와 같아지게)
    summary.write_text(json.dumps({
        'robot': 'amr_01', 'collision': {'count': 1, 'min_clearance': 0.0},
        'deviation': {'max': None}, 'avoidance': {'max_return_s': None, 'unreturned': 0},
        'kpi': {'valid': False, 'pass': False}}))
    m = avoidance_metrics(run)
    assert m['avoid.amr_01.valid'] == 0.0 and 'avoid.amr_01.pass' not in m
    assert m['avoid.amr_01.collisions'] == 1
