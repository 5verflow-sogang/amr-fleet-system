"""실행 폴더 요약 (ROS 없이). 가짜 실행 폴더를 만들어 묶기·실패 제외·KPI 판정을 본다."""

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.runner import fault_meta, render  # noqa: E402
from harness.summary import (avoidance_metrics, fleet_metrics, kidnap_metrics, kpi_verdict,  # noqa: E402
                             nav_kpi_metrics, nav_metrics, summarize, to_markdown, tracking_metrics)


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
    # 연구 1 비교군: fleet_fake fleet_mode·relay_auction (index.csv relay_auction 열). 'false' 문자열은 False
    f = fault_meta({'delay_ms': '0', 'fleet_mode': 'decentral', 'relay_auction': 'true'})
    assert (f['fleet_mode'], f['relay_auction']) == ('decentral', True)
    assert fault_meta({'delay_ms': '0', 'relay_auction': 'false'})['relay_auction'] is False


def test_fault_meta_extended_conditions():
    """확장 조건 params가 있는 시나리오(fleet_fake)만 allow_out_of_spec·blackout을 남긴다."""
    base = {'delay_ms': '100', 'loss_pct': '30', 'allow_out_of_spec': 'true', 'blackout_s': '0'}
    f = fault_meta(base)
    assert (f['loss_pct'], f['allow_out_of_spec']) == (30.0, True) and 'blackout' not in f
    f = fault_meta({**base, 'blackout_s': '3', 'blackout_at_s': '60', 'blackout_period_s': '120',
                    'blackout_robot': 'amr_03'})
    assert f['blackout'] == {'at_s': 60.0, 'duration_s': 3.0, 'period_s': 120.0, 'robot': 'amr_03', 'mode': 'delay'}
    # YAML false가 문자열 'False'로 렌더링돼도 거짓
    assert fault_meta({'delay_ms': '0', 'allow_out_of_spec': 'False'})['allow_out_of_spec'] is False


def test_fleet_task_and_recovery_metrics(tmp_path):
    run = tmp_path / 'r'
    run.mkdir()
    (run / 'task_outcomes.csv').write_text(
        'task_id,request_time,first_assign_time,robots,final_state,end_time\n'
        't1,0,1,amr_01,COMPLETED,8\nt2,10,11,amr_01 amr_02,COMPLETED,40\nt3,50,,,,\nt4,51,52,amr_03,,\n')
    m = fleet_metrics(run)
    assert (m['fleet.tasks_requested'], m['fleet.tasks_completed'], m['fleet.task_completion_rate']) == (4, 2, 0.5)
    assert m['fleet.throughput_per_min'] == pytest.approx(3.0)     # 완료 2건 / 첫 요청 0초~마지막 완료 40초
    assert 'fleet.reclaimed_tasks' not in m      # 로봇 장애 없는 실행
    assert 'fleet.duplicate_tasks' not in m      # runners 열이 없는 옛 결과

    (run / 'task_outcomes.csv').write_text(
        'task_id,request_time,first_assign_time,robots,final_state,end_time,runners\n'
        't1,0,1,amr_01,COMPLETED,8,amr_01\nt2,10,11,amr_02,COMPLETED,40,amr_03 amr_02\nt3,50,,,,,\n')
    assert fleet_metrics(run)['fleet.duplicate_tasks'] == 1
    assert 'fleet.fallbacks' not in fleet_metrics(run)

    (run / 'fleet_modes.csv').write_text(
        'robot,mode,start_time,end_time,duration_s\n'
        'amr_01,central,1,100,99.000\namr_03,central,1,40.5,39.500\namr_03,decentral,40.5,75,34.500\n'
        'amr_03,central,75,100,25.000\n')
    m = fleet_metrics(run)
    assert (m['fleet.fallbacks'], m['fleet.decentral_s']) == (1, 34.5)

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

    (run / 'task_outcomes.csv').write_text('task_id,request_time,first_assign_time,robots,final_state,end_time\n'
                                           't1,0,1,amr_01,,\n')            # 끝난 작업이 없으면 처리량 없음
    assert 'fleet.throughput_per_min' not in fleet_metrics(run)


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


def track_summary(pred=0.2, hold=0.6, vel=0.1, approach=None, false_safety=None):
    """tracking_eval summary.json에서 tracking_metrics가 읽는 부분만 (amr_perception_core.tracking_eval 형식)."""
    def st(rmse=None, mean=None, n=100, mx=None):
        return {'n': n if rmse is not None or mean is not None else 0, 'rmse': rmse, 'mean': mean,
                'median': mean, 'p95': mx, 'max': mx}
    overall = {'pos': st(0.15), 'vel': st(vel), 'speed': st(mean=-0.05), 'radial': st(mean=-0.1),
               'heading': st(mean=8.0), 'nees_pos': dict(st(mean=4.0), within_chi2_95=0.8),
               'nees_vel': dict(st(mean=3.0), within_chi2_95=0.9),
               'pred_1': st(pred), 'hold_1': st(hold), 'pred_2': st(0.5), 'hold_2': st(1.2),
               'match_rate_in_range': 0.95, 'id_switches': 2}
    by_speed = {b: {'seen': 1, 'classified': 1, 'from_seen': st(mean=1.2)} for b in ('<0.6', '0.6-1.0', '>=1.0')}
    return {'overall': overall, 'false_dynamic_per_frame': 0.01, 'duplicate_dynamic_per_frame': 0.0,
            'latency': {'from_seen': st(mean=1.2, mx=2.0), 'not_classified': 0, 'by_speed': by_speed},
            'approach': approach, 'false_dynamic_safety': false_safety}


def make_track_run(root: Path, name: str, scenario: str, outcome='completed', **kw):
    run = root / name
    (run / 'tracking').mkdir(parents=True)
    (run / 'meta.json').write_text(json.dumps({'scenario': scenario, 'label': 'main', 'outcome': outcome}))
    (run / 'tracking' / 'summary.json').write_text(json.dumps(track_summary(**kw)))
    return run


def approach(ttc, dist=7.0, latency=1.7, late=False):
    """summary.json approach. ttc 0이면 판정 없음(late면 최근접 뒤 판정), None이면 잴 수 없음 (기록 시작 전부터)."""
    judged = bool(ttc) or late
    return {'gt_id': 'worker_approach', 'ttc_s': ttc, 'judged_before_closest': int(bool(ttc)),
            'dynamic_dist': dist if judged else None, 'latency_from_seen': latency if judged else None,
            'note': '' if ttc else '최근접 뒤 판정' if late else '판정 없음' if ttc == 0.0 else '첫 구간이 기록 시작 전부터'}


def test_tracking_metrics(tmp_path):
    run = tmp_path / 'r'
    run.mkdir()
    assert tracking_metrics(run) == {}
    run = make_track_run(tmp_path, 'a', 'tracking_kpi_a', pred=0.25, hold=0.65, vel=0.12)
    m = tracking_metrics(run)
    assert m['track.pred_gain_1s'] == pytest.approx(0.40) and m['track.pred_1s.n'] == 100
    assert m['track.vel.rmse'] == 0.12 and m['track.vel.n'] == 100
    assert m['track.radial.mean'] == -0.1 and m['track.nees_pos.within95'] == 0.8
    assert m['track.latency.ge1_0.mean'] == 1.2 and m['track.id_switches'] == 2
    assert not [k for k in m if k.startswith('track.d1_')]          # 물체가 여럿인 기록 (approach 없음)
    # 로봇 자세·속도가 없던 기록: 가짜 때문에 늘어난 감속·정지는 없다
    assert m['track.false_dynamic_per_frame'] == 0.01
    assert not [k for k in m if k.startswith(('track.false_limited', 'track.false_stop', 'track.false_min'))]
    fs = {'frames': 3000, 'duration_s': 299.9, 'limited_s': 6.0, 'limited_frac': 0.02, 'limited_events': 4,
          'stop_s': 1.2, 'stop_frac': 0.004, 'stop_events': 2, 'false_min_clearance': 0.35,
          'false_min_ttc': None}
    m = tracking_metrics(make_track_run(tmp_path, 'fs', 'tracking_kpi_e', false_safety=fs))
    assert m['track.false_limited_frac'] == 0.02 and m['track.false_stop_frac'] == 0.004
    assert m['track.false_stop_events'] == 2 and m['track.false_min_clearance'] == 0.35
    # 가짜가 없던 기록은 최소 거리가 비어 있어 넣지 않는다 (비율·횟수 0은 넣는다)
    m = tracking_metrics(make_track_run(tmp_path, 'fs0', 'tracking_kpi_e', false_safety=dict(
        fs, limited_s=0.0, limited_frac=0.0, limited_events=0, stop_s=0.0, stop_frac=0.0, stop_events=0,
        false_min_clearance=None)))
    assert m['track.false_stop_events'] == 0.0 and 'track.false_min_clearance' not in m
    # 움직이는 GT와 짝이 없으면 P1 값은 없고 표본 수 0이 남는다
    run = make_track_run(tmp_path, 'b', 'tracking_kpi_a', pred=None, hold=None)
    m = tracking_metrics(run)
    assert 'track.pred_gain_1s' not in m and m['track.pred_1s.n'] == 0
    # 장면 C: 판정 없이 최근접까지 오면 0, 잴 수 없으면 넣지 않는다
    m = tracking_metrics(make_track_run(tmp_path, 'c', 'tracking_kpi_c10', approach=approach(4.3)))
    assert m['track.d1_ttc_s'] == 4.3 and m['track.d1_dist_m'] == 7.0 and m['track.d1_latency_s'] == 1.7
    m = tracking_metrics(make_track_run(tmp_path, 'd', 'tracking_kpi_c10', approach=approach(0.0)))
    assert m['track.d1_ttc_s'] == 0.0 and 'track.d1_dist_m' not in m
    # 최근접 뒤 판정: 거리·지연은 있지만 인정된 판정이 아니라 참고 분포에 넣지 않는다
    m = tracking_metrics(make_track_run(tmp_path, 'late', 'tracking_kpi_c10',
                                        approach=approach(0.0, dist=6.1, latency=8.2, late=True)))
    assert m['track.d1_ttc_s'] == 0.0 and 'track.d1_dist_m' not in m and 'track.d1_latency_s' not in m
    m = tracking_metrics(make_track_run(tmp_path, 'e', 'tracking_kpi_c10',
                                        approach=dict(approach(None), dynamic_dist=3.9, latency_from_seen=1.7)))
    assert not [k for k in m if k.startswith('track.d1_')]


def test_tracking_p1_p2_verdict_only_in_scenes_abde(tmp_path):
    a = [make_track_run(tmp_path, f'a{i}', 'tracking_kpi_a', pred=0.2, hold=0.6 + 0.01 * i, vel=0.1 + 0.01 * i)
         for i in range(10)]
    c = [make_track_run(tmp_path, f'c{i}', 'tracking_kpi_c10', approach=approach(4.0)) for i in range(10)]
    s = summarize(a + c)
    ma, mc = s['tracking_kpi_a/main']['metrics'], s['tracking_kpi_c10/main']['metrics']
    assert ma['track.pred_gain_1s']['kpi'] == 'PASS' and ma['track.vel.rmse']['kpi'] == 'PASS'
    assert 'kpi' not in ma['track.pos.rmse']                         # 참고 지표
    assert 'kpi' not in mc['track.pred_gain_1s'] and 'kpi' not in mc['track.vel.rmse']
    # P1은 CI 하한이 0보다 커야 한다. 추정 속도가 0이면 예측 = 정지 가정이라 차가 정확히 0이다
    assert kpi_verdict('track.pred_gain_1s', 0.0, 0.0, 0.0, 'tracking_kpi_b') == 'FAIL'
    assert kpi_verdict('track.pred_gain_1s', 0.05, 0.1, -0.01, 'tracking_kpi_d') == 'MARGINAL'
    assert kpi_verdict('track.pred_gain_1s', -0.1, 0.0, -0.2, 'tracking_kpi_e') == 'FAIL'
    assert kpi_verdict('track.vel.rmse', 0.18, 0.22, 0.15, 'tracking_kpi_e') == 'MARGINAL'
    assert kpi_verdict('track.vel.rmse', 0.25, 0.3, 0.21, 'tracking_kpi_a') == 'FAIL'
    assert kpi_verdict('track.vel.rmse', 0.1, 0.12, 0.08, 'tracking_kpi') is None   # 공통 틀


def test_tracking_d1_worst_case_verdict(tmp_path):
    def d1(name, ttcs, scenario='tracking_kpi_c10', failed=0):
        root = tmp_path / name
        runs = [make_track_run(root, f'r{i}', scenario, approach=approach(t)) for i, t in enumerate(ttcs)]
        runs += [make_track_run(root, f'f{i}', scenario, outcome='timeout') for i in range(failed)]
        g = summarize(runs)[f'{scenario}/main']
        return g['metrics'].get('track.d1_ttc_s', {}), to_markdown({'g': g})

    ok = [3.0 + 0.1 * i for i in range(10)]
    m, md = d1('pass', ok)
    assert m['kpi'] == 'PASS' and m['min'] == 3.0 and '최악값으로 판정' in md
    assert d1('pass15', ok, 'tracking_kpi_c15')[0]['kpi'] == 'PASS'
    # 한 번이라도 3 s 밑이면 평균이 높아도 FAIL. 판정 없이 최근접까지 온 실행(0)도 FAIL
    assert d1('one_low', ok[:9] + [2.9])[0]['kpi'] == 'FAIL'
    assert d1('unjudged', ok[:9] + [0.0])[0]['kpi'] == 'FAIL'
    # 재지 못한 실행이 있으면 보류 (빼고 PASS를 주지 않는다)
    m = d1('short', ok[:9])[0]
    assert 'kpi' not in m and m['kpi_note'] == '보류 (n=9<10)'
    m = d1('harness_fail', ok, failed=1)[0]
    assert 'kpi' not in m and m['kpi_note'] == '보류 (하네스 실패 1회)'
    m = d1('missing', ok + [None])[0]
    assert 'kpi' not in m and m['kpi_note'] == '보류 (값 없는 실행 1회)'
    assert d1('fail_first', ok[:9] + [2.0], failed=1)[0]['kpi'] == 'FAIL'   # 실패 실행이 있어도 FAIL은 FAIL
    # 0.5 m/s(c05)는 참고 속력이라 판정하지 않는다
    m = d1('c05', ok, 'tracking_kpi_c05')[0]
    assert 'kpi' not in m and 'kpi_note' not in m
