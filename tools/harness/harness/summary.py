"""실행 폴더(results/<run_id>/)들을 모아 지표별 평균·95% 신뢰구간을 낸다. ROS 없이 돈다.

실행 하나 = 표본 하나다. 실행마다 지표를 먼저 계산하고(예: 그 실행의 정지 구간 RMSE),
그 값들로 신뢰구간을 낸다. `outcome`이 `completed`가 아닌 실행은 빼고 실패 수로 따로 보고한다(README "요약 통계").
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional

from .stats import Interval, percentile, summarize_runs

# 원 명세서 위치 오차 KPI [m] (amr_localization/eval/metrics.py KPI_THRESHOLDS와 같은 값)
LOCALIZATION_KPI = {'stationary': 0.03, 'straight': 0.05, 'rotation': 0.08}
# 원 명세서 응답 시간 KPI [ms]. "50회 이상" 측정이라 움직임을 찾은 명령이 그보다 적으면 판정을 비운다
RESPONSE_KPI_MS = 200.0
RESPONSE_MIN_N = 50
# 판정할 응답 시간 지표 → 그 표본 수 지표
RESPONSE_KPI = {'fleet.response_ms.mean': 'fleet.response_ms.n',
                'fleet.response_recv_ms.mean': 'fleet.response_recv_ms.n'}
# 원 명세서 CPU 사용률 KPI [%]. 실행한 컨테이너가 쓸 수 있는 코어 기준 (기준 기기는 docker --cpus로 맞춘다)
CPU_KPI_PCT = 80.0
# 교착 판정 (4주차 10/13 결정): 구역 허가 장기 대기·이동 중 정체가 0건이어야 "5대 교착 없이 수행"
# 장기 대기 기준은 amr_fleet_manager fleet_metrics.ZONE_LONG_WAIT_S와 같다 (서버 release_timeout_s)
ZONE_LONG_WAIT_S = 30.0
COUNT_ZERO_KPI = ('fleet.zone_long_waits', 'fleet.stalls')
# 시나리오가 일부러 만드는 상황이라 판정하면 오해되는 지표 (fleet_deadlock은 교착을 유도한다)
NO_VERDICT = {'fleet_deadlock': set(COUNT_ZERO_KPI)}
# 작업 실패 원인 묶음 (amr_msgs ErrorInfo.code, docs/api.md). 위치(203)를 주행(201·202)과 나눠 센다 (#148 지수 합의).
# assign = 배터리 부족 거부(101, BT CheckBattery #253)·이미 수행 중(102), canceled = 서버 취소(103)·다른 로봇이 먼저
# 수락(104). 어느 묶음에도 없는 코드(새 코드, 0 등)는 tasks_failed_other로 세어 실패가 조용히 빠지지 않게 한다
FAILURE_CAUSES = {'assign': {'101', '102'}, 'canceled': {'103', '104'}, 'localization': {'203'}, 'nav': {'201', '202'}, 'traffic': {'301', '302', '303'},
                  'comm': {'401', '402'}, 'task_timeout': {'501'}, 'charge': {'502'},
                  'docking': {str(c) for c in range(900, 1000)}}
# 원 명세 경로 계획(4장)·경로 추종(5장) KPI: (기준, 방향). min = 클수록 좋음 (CI 하한으로 판정), max = 작을수록 좋음
NAV_KPI = {'navkpi.plan_success_rate': (0.98, 'min'), 'navkpi.replan_p95_ms': (500.0, 'max'),
           'navkpi.pred_err_mean': (0.15, 'max'), 'navkpi.cte_straight_mean_m': (0.05, 'max'),
           'navkpi.cte_curve_mean_m': (0.10, 'max')}
# 추적 정확도 KPI (docs/algorithms/perception.md 1-7): (기준, 방향). gt = 기준보다 커야 함 (CI 하한으로 판정).
# P1은 0과 같아도 불합격이다 (추정 속도가 0이면 예측과 정지 가정이 같아진다). 장면 A·B·D·E에서만 판정한다
# P2는 센서 범위(12 m) 안 짝의 속도 RMSE다 (3막 재측정부터. 1차 결과 후·재측정 전 등록, perception.md 1-7 P2 보완).
# 전체 거리 track.vel.rmse는 참고 지표 (1차 본 측정 판정은 이 값)
TRACK_KPI = {'track.pred_gain_1s': (0.0, 'gt'), 'track.vel_in_range.rmse': (0.2, 'max')}
TRACK_KPI_SCENARIOS = {'tracking_kpi_a', 'tracking_kpi_b', 'tracking_kpi_d', 'tracking_kpi_e'}
# 평균·CI로 판정하는 KPI의 최소 실행 수. 모자라면 보류다 (P1·P2는 장면마다 10회로 미리 정했다, perception.md 1-7)
# TRACK_KPI에서 만든다: P2 지표 이름이 바뀌어도(전체 → 센서 범위 안) 보류 규칙이 같이 따라가게
KPI_MIN_RUNS = {metric: 10 for metric in TRACK_KPI}
# 최악값으로 판정하는 KPI: 지표 → (기준 하한, 최소 실행 수, 판정할 시나리오). 안전 기준이라 평균·CI가 아니라
# 모든 실행이 기준 이상이어야 한다 (D1, perception.md 1-7)
WORST_KPI = {'track.d1_ttc_s': (3.0, 10, {'tracking_kpi_c10', 'tracking_kpi_c15'})}
# tracking_eval 판정 지연 속력 구간 → 지표 이름
TRACK_SPEED_BINS = {'<0.6': 'lt0_6', '0.6-1.0': '0_6to1_0', '>=1.0': 'ge1_0'}


def load_meta(run_dir: Path) -> dict:
    path = run_dir / 'meta.json'
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}


def localization_metrics(run_dir: Path) -> dict[str, float]:
    """localization_summary.json(로봇마다 하위 폴더 또는 실행 폴더 바로 아래) → {지표 이름: 값}."""
    out = {}
    for path in sorted(run_dir.glob('**/localization_summary.json')):
        s = json.loads(path.read_text(encoding='utf-8'))
        # 같은 로봇을 여러 추정기로 재면 폴더 이름으로 구분한다 (예: eval_ekf/, eval_amcl/)
        source = path.parent.name if path.parent != run_dir else 'est'
        for seg, v in (s.get('segments') or {}).items():
            if v is None:
                continue
            prefix = f"loc.{s.get('robot', 'robot')}.{source}.{seg}"
            out[f'{prefix}.rmse'] = v['rmse']
            out[f'{prefix}.max'] = v['max']
            out[f'{prefix}.n'] = v['n']
    return out


def _read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _floats(rows: Iterable[dict], key: str) -> list[float]:
    return [float(r[key]) for r in rows if r.get(key) not in (None, '')]


def _put_dist(out: dict, prefix: str, values: list[float], *stats: str) -> None:
    """values의 통계(mean·p95·p99·max·n)를 out[f'{prefix}.{통계}']에 넣는다. 비어 있으면 아무것도 넣지 않는다.

    평균은 받은 순서대로 더한다 (부동소수점 합은 순서에 따라 끝자리가 달라져, 예전 요약 값과 같게 둔다).
    """
    if not values:
        return
    ordered = sorted(values)
    calc = {'mean': lambda: sum(values) / len(values), 'p95': lambda: percentile(ordered, 0.95),
            'p99': lambda: percentile(ordered, 0.99), 'max': lambda: ordered[-1], 'n': lambda: len(values)}
    for stat in stats:
        out[f'{prefix}.{stat}'] = calc[stat]()


def fleet_metrics(run_dir: Path) -> dict[str, float]:
    """fleet_latency.csv, response_time.csv, task_outcomes.csv, recovery.csv → 실행 하나의 요약 값."""
    out = {}
    lat_rows = _read_csv(run_dir / 'fleet_latency.csv')
    judged = [r for r in lat_rows if r.get('delivered') in ('0', '1')]
    if judged:
        out['fleet.loss_rate'] = sum(r['delivered'] == '0' for r in judged) / len(judged)
        lat = sorted(_floats((r for r in judged if r['delivered'] == '1'), 'latency_ms'))
        _put_dist(out, 'fleet.latency_ms', lat, 'mean', 'p95', 'p99', 'max')
    resp_rows = _read_csv(run_dir / 'response_time.csv')
    if resp_rows:
        out['fleet.commands'] = len(resp_rows)
        acked = sorted(_floats(resp_rows, 'ack_rtt_ms'))
        out['fleet.ack_rate'] = len(acked) / len(resp_rows)
        _put_dist(out, 'fleet.ack_rtt_ms', acked, 'mean', 'p95', 'max')
        out['fleet.resends.mean'] = sum(_floats(resp_rows, 'resends')) / len(resp_rows)
        resp = sorted(_floats(resp_rows, 'latency_ms'))
        _put_dist(out, 'fleet.response_ms', resp, 'mean', 'max', 'n')
        # 수신 기준 (로봇이 받은 시각 → 첫 움직임). 원 명세 성능 요구사항 "명령 수신 ~ 로봇 반응"
        _put_dist(out, 'fleet.response_recv_ms', sorted(_floats(resp_rows, 'latency_recv_ms')), 'mean', 'max', 'n')
        # 시뮬레이션 시간 기준 (발행 → 움직임). RTF < 1이면 실제 시간 값이 부푼다. KPI 시계는 회의에서 정한다 (#171 리뷰 2)
        _put_dist(out, 'fleet.response_sim_ms', sorted(_floats(resp_rows, 'latency_sim_ms')), 'mean', 'max')
        if resp:
            # 움직임을 찾은 명령의 비율. 못 찾은 명령은 평균에서 조용히 빠지므로 같이 본다 (#171 리뷰 3)
            out['fleet.response_coverage'] = len(resp) / len(resp_rows)
            # 명령 순간 이미 움직이던 경우 (지연이 0에 가깝게 나온다, #171 리뷰 4)
            out['fleet.response_moving_at_cmd'] = sum(r.get('moving_at_cmd') == '1' for r in resp_rows
                                                      if r.get('latency_ms'))
    # 작업 결과. 끝 무렵에 요청된 작업은 실행 안에 못 끝나므로, 완료율은 같은 seed(같은 작업 일정)끼리 비교한다
    tasks = _read_csv(run_dir / 'task_outcomes.csv')
    if tasks:
        done = sum(r['final_state'] == 'COMPLETED' for r in tasks)
        out['fleet.tasks_requested'] = len(tasks)
        out['fleet.tasks_completed'] = done
        out['fleet.task_completion_rate'] = done / len(tasks)
        # 할당 최적성 (원 명세 9장: 총 이동 거리, 작업 완료 시간). 완료한 작업만
        completed = [r for r in tasks if r['final_state'] == 'COMPLETED']
        times = _floats(completed, 'task_time_s')
        if times:
            out['fleet.task_time_s.mean'] = sum(times) / len(times)
        # 처리량(완료 작업/분): 첫 요청부터 마지막 완료까지. 투입이 로봇 처리 능력보다 느리면 투입 속도에 묶인다
        # (통계 설계 5절). 실행 끝까지가 아니라 마지막 완료까지라 꼬리의 미완료 작업 시간은 빠진다
        requested = _floats(tasks, 'request_time')
        ends = _floats(completed, 'end_time')
        if requested and ends and max(ends) > min(requested):
            out['fleet.throughput_per_min'] = len(ends) / ((max(ends) - min(requested)) / 60.0)
        dist = _floats(completed, 'distance_m')
        if dist:
            out['fleet.distance_m.total'] = sum(dist)
            out['fleet.distance_m.mean'] = sum(dist) / len(dist)
        # 실패 원인 (amr_msgs ErrorInfo 코드, #173). task_outcomes.csv에 error_code 열이 있을 때만 (fleet_metrics, 하연)
        if 'error_code' in tasks[0]:
            failed = [r for r in tasks if r['final_state'] == 'FAILED']
            for name, codes in FAILURE_CAUSES.items():
                out[f'fleet.tasks_failed_{name}'] = sum(r['error_code'] in codes for r in failed)
            known = set().union(*FAILURE_CAUSES.values())
            out['fleet.tasks_failed_other'] = sum(r['error_code'] not in known for r in failed)
        # 같은 작업을 둘 이상의 로봇이 실제로 움직이며 한 수 (하이브리드 중복 실행, architecture.md 5-2절).
        # task_outcomes.csv에 runners 열이 있을 때만
        if 'runners' in tasks[0]:
            out['fleet.duplicate_tasks'] = sum(len(r['runners'].split()) > 1 for r in tasks)
    # 하이브리드 폴백 (architecture.md 5-2절): 로봇이 분산으로 넘어간 횟수와 분산 모드로 돈 시간(로봇별 합)
    if (run_dir / 'fleet_modes.csv').exists():
        decentral = [r for r in _read_csv(run_dir / 'fleet_modes.csv') if r['mode'] == 'decentral']
        out['fleet.fallbacks'] = len(decentral)
        out['fleet.decentral_s'] = sum(_floats(decentral, 'duration_s'))
    # 구역 허가 대기와 교착 해소 (구역을 쓰는 시나리오만 파일이 있다)
    if (run_dir / 'zone_waits.csv').exists():
        zones = _read_csv(run_dir / 'zone_waits.csv')
        waits = _floats(zones, 'wait_s')
        out['fleet.zone_requests'] = len(zones)
        out['fleet.zone_long_waits'] = sum(
            float(r['wait_s']) > ZONE_LONG_WAIT_S for r in zones if r['outcome'] in ('granted', 'waiting'))
        out['fleet.deadlock_revokes'] = sum(r['outcome'] == 'revoked' for r in zones)
        out['fleet.deadlock_rejects'] = sum(r['outcome'] == 'rejected' for r in zones)
        _put_dist(out, 'fleet.zone_wait_s', waits, 'mean', 'max')
    # 이동 중 정체 (로봇 odom을 녹화한 시나리오만 파일이 있다)
    if (run_dir / 'stalls.csv').exists():
        stalled = _read_csv(run_dir / 'stalls.csv')
        out['fleet.stalls'] = len(stalled)
        if stalled:
            out['fleet.stall_s.max'] = max(_floats(stalled, 'duration_s'))
    # 로봇 장애 회수 (fault_events.csv에 robot_kill이 있을 때만 파일이 있다)
    reclaimed = [r for r in _read_csv(run_dir / 'recovery.csv') if r.get('task_id')]
    if (run_dir / 'recovery.csv').exists():
        out['fleet.reclaimed_tasks'] = len(reclaimed)
        _put_dist(out, 'fleet.recovery_s', _floats(reclaimed, 'recovery_s'), 'mean', 'max')
        if reclaimed:
            out['fleet.reclaimed_completed_rate'] = sum(r['completed'] == '1' for r in reclaimed) / len(reclaimed)
    return out


# 서버·로봇 1대의 기준 컴퓨터 코어 수. 원 명세에 기준 기기가 없어 팀이 정한 가정 (10/28 제안, 로봇 탑재 컴퓨터 4코어)
REF_CORES = 4.0


def cpu_metrics(run_dir: Path) -> dict[str, float]:
    """cpu.csv(cpu_monitor.py) → sys.* 지표. 사용 코어 수는 기기와 덜 상관있어 기준 기기를 바꿔 판정할 때 쓴다."""
    rows = _read_csv(run_dir / 'cpu.csv')
    out: dict[str, float] = {}
    cores = sorted(_floats(rows, 'cores'))
    if not cores:
        return out
    _put_dist(out, 'sys.cpu_cores', cores, 'mean', 'p95', 'max')
    # 작업이 끝난 뒤 쉬는 구간도 평균에 들어가 낮아지므로 P95도 같이 본다 (#171 리뷰 5)
    _put_dist(out, 'sys.cpu_pct', sorted(_floats(rows, 'cpu_pct')), 'mean', 'p95', 'max')
    allowed = _floats(rows, 'allowed_cores')
    out['sys.allowed_cores'] = allowed[-1]
    host = _floats(rows, 'host_cpu_pct')
    if host:
        out['sys.host_cpu_pct.mean'] = sum(host) / len(host)
    sim = _floats(rows, 'cores_sim')
    if len(sim) == len(cores):
        # 시뮬레이터를 뺀 코어 수 (실제 로봇에서 도는 소프트웨어 몫의 근사)
        no_sim = [float(r['cores']) - float(r['cores_sim']) for r in rows if r.get('cores') and r.get('cores_sim')]
        out['sys.cpu_cores_no_sim.mean'] = sum(no_sim) / len(no_sim)
    for group in ('sim', 'nav2', 'localization', 'fleet', 'perception', 'other'):
        values = _floats(rows, f'cores_{group}')
        if values:
            out[f'sys.cpu_cores.{group}.mean'] = sum(values) / len(values)
            # 실행한 컴퓨터(컨테이너가 쓸 수 있는 코어) 대비 비율
            out[f'sys.cpu_pct.{group}.mean'] = 100.0 * out[f'sys.cpu_cores.{group}.mean'] / allowed[-1]
    # 역할별 (harness/cpu.py role): 서버, 로봇 1대(구간마다 가장 많이 쓴 로봇), 실험 도구
    for key in ('server', 'tool', 'robot_max', 'robot_mean'):
        values = sorted(_floats(rows, f'cores_{key}'))
        if values:
            _put_dist(out, f'sys.cpu_cores.{key}', values, 'mean', 'p95')
            out[f'sys.cpu_pct.{key}.mean'] = 100.0 * out[f'sys.cpu_cores.{key}.mean'] / allowed[-1]
            if key in ('server', 'robot_max'):
                # 판정 기준: 서버·로봇 1대를 각각 REF_CORES 코어 컴퓨터로 본 사용률 (명세에 기준 기기가 없어 팀 가정)
                out[f'sys.cpu_ref_pct.{key}.mean'] = 100.0 * out[f'sys.cpu_cores.{key}.mean'] / REF_CORES
                out[f'sys.cpu_ref_pct.{key}.p95'] = 100.0 * out[f'sys.cpu_cores.{key}.p95'] / REF_CORES
    out['sys.cpu_samples'] = len(cores)
    return out


def nav_metrics(run_dir: Path) -> dict[str, float]:
    """nav_result.json(목표 1회 주행 결과, experiments/teb_compare/nav_measure.py) → nav.* 지표.

    nav.success는 성공(Nav2 도착이고 충돌 없음) 1·실패 0이라 평균이 성공률이다.
    도착 시간은 둘로 나눈다 (#145 민지 리뷰):
      nav.arrival_s  성공한 실행만
      nav.reached_s  Nav2가 목표에 닿은 실행 모두 (충돌 포함)
    reached_s가 없는 옛 결과는 result가 succeeded일 때의 arrival_s를 Nav2 도착 시간으로 본다.
    값이 없는 지표(장애물 없는 시나리오의 최소 장애물 거리 등)는 빼서 평균에 0이 섞이지 않게 한다.
    """
    out = {}
    for path in sorted(run_dir.glob('**/nav_result.json')):
        r = json.loads(path.read_text(encoding='utf-8'))
        out['nav.success'] = 1.0 if r.get('success') else 0.0
        reached = r.get('reached_s', r.get('arrival_s') if r.get('result') == 'succeeded' else None)
        if isinstance(reached, (int, float)):
            out['nav.reached_s'] = float(reached)
            if r.get('success'):
                out['nav.arrival_s'] = float(reached)
        for key in ('min_obstacle_m', 'path_length_m', 'cte_mean_m', 'cte_max_m'):
            if isinstance(r.get(key), (int, float)):
                out[f'nav.{key}'] = float(r[key])
    return out


def kidnap_metrics(run_dir: Path) -> dict[str, float]:
    """kidnap.json(amr_localization kidnap_trial) → 성공·감지·복구 (0/1 지표의 평균이 성공률이다).

    시험 자체가 안 된 실행(순간 이동 실패 등, error)은 지표를 내지 않는다. 복구 시간은 복구한 실행만.
    """
    out = {}
    for path in sorted(run_dir.glob('**/kidnap.json')):
        k = json.loads(path.read_text(encoding='utf-8'))
        if k.get('error') or 'success' not in k:
            continue
        # 순간 이동 순간에 이미 복구 중이던 실행은 감지를 잴 수 없어 분모에서 뺀다 (kidnap.valid로 몇 개인지 남김)
        out['kidnap.valid'] = float(k.get('valid', True))
        if not k.get('valid', True):
            continue
        out['kidnap.success'] = float(k['success'])
        out['kidnap.late_recovered'] = float(k.get('late_recover_s') is not None)
        out['kidnap.detected'] = float(k['detected'])
        out['kidnap.recovered'] = float(k['recovered'])
        out['kidnap.false_alarms'] = float(k.get('false_alarms', 0))
        if k.get('detect_s') is not None:
            out['kidnap.detect_s'] = float(k['detect_s'])
        if k.get('recover_s') is not None:
            out['kidnap.recover_s'] = float(k['recover_s'])
    return out


def nav_kpi_metrics(run_dir: Path) -> dict[str, float]:
    """nav_kpi.json(하네스 nav_kpi, experiments/nav_kpi/nav_kpi.py) → navkpi.* 지표.

    CTE는 path_tracker로 잰 실행(definition.cte_kpi)만 KPI 이름(navkpi.cte_*)으로 넣고, DWA(Nav2)로 잰 값은
    참고 지표(navkpi.ref_cte_*)로 넣어 판정하지 않는다 (CTE KPI는 5장 모션 제어, 이진 10/13).
    """
    out = {}
    for path in sorted(run_dir.glob('**/nav_kpi.json')):
        k = json.loads(path.read_text(encoding='utf-8'))
        cte_kpi = (k.get('definition') or {}).get('cte_kpi', False)
        for key in ('plan_success_rate', 'nav_success_rate', 'replan_p95_ms', 'replan_max_ms', 'pred_err_mean',
                    'pred_err_median', 'pred_err_no_recovery_mean', 'cte_straight_mean_m', 'cte_curve_mean_m',
                    'cte_est_straight_mean_m', 'cte_est_curve_mean_m', 'loc_err_straight_mean_m', 'loc_err_curve_mean_m'):
            v = k.get(key)
            if isinstance(v, (int, float)) and not math.isnan(v):
                # 추정 기준 CTE·위치 오차는 원인 나누기용 참고 (판정 없음)
                name = key if cte_kpi or not key.startswith('cte_') or key.startswith('cte_est') else 'ref_' + key
                out[f'navkpi.{name}'] = float(v)
    return out


def avoidance_metrics(run_dir: Path) -> dict[str, float]:
    """avoidance_summary.json(amr_safety avoidance_eval, test-plan I17) → 실행 하나의 회피 지표.

    I17 KPI는 "30회 모두"라 평균·CI가 아니라 최악값으로 판정한다. 판정은 `avoidance_report`가 하고,
    여기서는 실행별 값의 분포만 보여 준다 (KPI 열은 비운다).
    - pass는 유효 실행만 넣는다. 무효 실행을 0으로 넣으면 합격률이 avoidance_report와 달라진다 (valid로 따로 센다)
    - max_return_s는 복귀 못 한 회피가 있으면 비어 있다. 최악 실행이 분포에서 빠지지 않게 unreturned를 늘 넣는다
    """
    out = {}
    for path in sorted(run_dir.glob('**/avoidance_summary.json')):
        s = json.loads(path.read_text(encoding='utf-8'))
        prefix = f"avoid.{s.get('robot', 'robot')}"
        out[f'{prefix}.collisions'] = s['collision']['count']
        out[f'{prefix}.unreturned'] = s['avoidance']['unreturned']
        for key, value in (('min_clearance', s['collision']['min_clearance']),
                           ('max_deviation', s['deviation']['max']),
                           ('max_return_s', s['avoidance']['max_return_s'])):
            if value is not None:
                out[f'{prefix}.{key}'] = value
        out[f'{prefix}.valid'] = 1.0 if s['kpi']['valid'] else 0.0
        if s['kpi']['valid']:
            out[f'{prefix}.pass'] = 1.0 if s['kpi']['pass'] else 0.0
    return out


def tracking_metrics(run_dir: Path) -> dict[str, float]:
    """tracking/summary.json(amr_tracking tracking_eval, perception.md 1-7) → track.* 지표.

    판정하는 지표 (TRACK_KPI, WORST_KPI):
      track.pred_gain_1s  정지 가정 RMSE − 등속 예측 RMSE (+1 s, 움직이는 GT). P1
      track.vel_in_range.rmse  센서 범위(12 m) 안 짝의 속도 벡터 오차 RMSE. P2
      track.d1_ttc_s      접근 판정 TTC (GT 물체가 하나인 기록만, approach.ttc_s). D1. 판정 없이 최근접까지 오면 0이고,
                          잴 수 없는 실행(None)은 넣지 않는다. 그 실행이 빠진 것은 최악값 판정이 보류로 드러낸다
    나머지는 참고 지표다. 표본이 없어 비어 있는 값은 넣지 않고, P1·P2의 표본 수(.n)는 늘 넣는다.
    track.false_limited_frac·false_stop_frac·false_stop_events·false_min_clearance는 가짜 동적 트랙 때문에 늘어난
    감속·정지다 (false_dynamic_safety, 로봇 자세·속도가 있는 기록만).
    """
    out = {}
    for path in sorted(run_dir.glob('**/tracking/summary.json')):
        s = json.loads(path.read_text(encoding='utf-8'))
        o = s['overall']

        def put(name, value):
            if isinstance(value, (int, float)) and not math.isnan(value):
                out[f'track.{name}'] = float(value)

        if o['pred_1']['rmse'] is not None and o['hold_1']['rmse'] is not None:
            out['track.pred_gain_1s'] = o['hold_1']['rmse'] - o['pred_1']['rmse']
        out['track.pred_1s.n'] = o['pred_1']['n']
        put('vel.rmse', o['vel']['rmse'])
        out['track.vel.n'] = o['vel']['n']
        if 'vel_in_range' in o:          # 전 평가 결과에는 없다
            put('vel_in_range.rmse', o['vel_in_range']['rmse'])
            out['track.vel_in_range.n'] = o['vel_in_range']['n']
        put('match_rate', o['match_rate_in_range'])
        put('pos.rmse', o['pos']['rmse'])
        put('radial.mean', o['radial']['mean'])          # 센서 방향 편향 (음수 = 센서 쪽)
        put('speed.mean', o['speed']['mean'])            # 속력 편향
        put('heading.mean', o['heading']['mean'])        # [°]
        for h in ('1', '2'):
            put(f'pred_{h}s.rmse', (o.get(f'pred_{h}') or {}).get('rmse'))
            put(f'hold_{h}s.rmse', (o.get(f'hold_{h}') or {}).get('rmse'))
        for k in ('nees_pos', 'nees_vel'):
            put(f'{k}.mean', o[k]['mean'])
            put(f'{k}.within95', o[k]['within_chi2_95'])
        put('id_switches', o['id_switches'])
        lat = s['latency']
        put('latency.from_seen.mean', lat['from_seen']['mean'])
        put('latency.from_seen.max', lat['from_seen']['max'])
        put('latency.not_classified', lat['not_classified'])
        for b, name in TRACK_SPEED_BINS.items():
            put(f'latency.{name}.mean', lat['by_speed'][b]['from_seen']['mean'])
        put('false_dynamic_per_frame', s['false_dynamic_per_frame'])
        put('duplicate_dynamic_per_frame', s['duplicate_dynamic_per_frame'])
        fs = s.get('false_dynamic_safety')
        if fs:
            # 가짜 동적 트랙 때문에 늘어난 감속·정지 (안전 노드 규칙). limited는 감속 또는 정지, stop은 정지
            put('false_limited_frac', fs['limited_frac'])
            put('false_stop_frac', fs['stop_frac'])
            put('false_stop_events', fs['stop_events'])
            put('false_min_clearance', fs['false_min_clearance'])
        ap = s.get('approach')
        if ap:
            put('d1_ttc_s', ap['ttc_s'])
            if ap['ttc_s'] is not None and ap['judged_before_closest']:
                # 판정 거리·지연은 인정된 판정만 (최근접 뒤 판정·잴 수 없는 실행이 섞이지 않게)
                put('d1_dist_m', ap['dynamic_dist'])
                put('d1_latency_s', ap['latency_from_seen'])
    return out


def run_metrics(run_dir: Path) -> dict[str, float]:
    meta = load_meta(run_dir)
    out = {**localization_metrics(run_dir), **fleet_metrics(run_dir), **nav_metrics(run_dir),
           **kidnap_metrics(run_dir), **nav_kpi_metrics(run_dir), **cpu_metrics(run_dir),
           **avoidance_metrics(run_dir), **tracking_metrics(run_dir)}
    rtf = meta.get('real_time_factor')
    if isinstance(rtf, (int, float)):
        out['run.real_time_factor'] = float(rtf)
    return out


def group_key(meta: dict) -> str:
    """같은 조건끼리 묶는 이름: 시나리오 + 라벨 (라벨은 run.py --label)."""
    label = meta.get('label') or ''
    return f"{meta.get('scenario', '?')}{'/' + label if label else ''}"


def kpi_verdict(metric: str, mean: float, ci_high: Optional[float], ci_low: Optional[float] = None,
                scenario: str = '') -> Optional[str]:
    """KPI가 있는 지표만 판정한다. CI 상한까지 기준 안이면 PASS, 평균만 안이면 MARGINAL, 아니면 FAIL.

    클수록 좋은 지표(성공률)는 거꾸로 CI 하한까지 기준 이상이면 PASS다. 시나리오가 일부러 만드는 상황의
    지표(NO_VERDICT)는 판정하지 않는다. 추적 KPI(TRACK_KPI)는 판정 장면(TRACK_KPI_SCENARIOS)에서만 판정한다.
    """
    if metric in NO_VERDICT.get(scenario, ()):
        return None
    rule = kpi_rule(metric, scenario)
    if rule is None:
        return None
    threshold, kind = rule
    # 기준을 만족하는지와 그때 볼 CI 끝: max는 상한, min·gt는 하한. P1(gt)은 기준과 같아도 안 된다
    meets = {'max': lambda v: v <= threshold, 'min': lambda v: v >= threshold, 'gt': lambda v: v > threshold}[kind]
    bound = ci_high if kind == 'max' else ci_low
    if bound is not None and meets(bound):
        return 'PASS'
    return 'MARGINAL' if meets(mean) else 'FAIL'


def kpi_rule(metric: str, scenario: str = '') -> Optional[tuple[float, str]]:
    """지표의 KPI (기준, 방향). 방향: max = 작을수록 좋음, min = 클수록 좋음, gt = 기준보다 커야 함. 없으면 None."""
    parts = metric.split('.')
    if parts[0] == 'loc' and parts[-1] == 'rmse':
        threshold = LOCALIZATION_KPI.get(parts[-2])
        return None if threshold is None else (threshold, 'max')
    if metric in RESPONSE_KPI:
        return RESPONSE_KPI_MS, 'max'
    if metric in COUNT_ZERO_KPI:
        return 0.0, 'max'
    if metric == 'sys.cpu_pct.mean':
        return CPU_KPI_PCT, 'max'
    if metric in NAV_KPI:
        return NAV_KPI[metric]
    if metric in TRACK_KPI and scenario in TRACK_KPI_SCENARIOS:
        return TRACK_KPI[metric]
    return None


def worst_kpi_verdict(metric: str, iv: Interval, scenario: str, runs: int,
                      failed: int) -> tuple[Optional[str], Optional[str]]:
    """최악값 KPI(WORST_KPI) 판정 → (판정, 보류 사유). 판정하지 않는 지표·시나리오면 (None, None).

    가장 나쁜 실행이 기준 밑이면 FAIL. 아니면 하네스 실패 실행이 있거나, 완료했는데 값이 없는 실행이 있거나,
    실행이 최소 횟수보다 적으면 보류다. 재지 못한 실행을 빼고 PASS를 주면 성공한 실행만 고르는 셈이다
    (avoidance_report와 같은 원칙). runs = 완료한 실행 수, failed = 하네스 실패 실행 수.
    """
    rule = WORST_KPI.get(metric)
    if rule is None or scenario not in rule[2]:
        return None, None
    limit, min_runs, _ = rule
    if iv.min < limit:
        return 'FAIL', None
    if failed:
        return None, f'보류 (하네스 실패 {failed}회)'
    if iv.n < runs:
        return None, f'보류 (값 없는 실행 {runs - iv.n}회)'
    if iv.n < min_runs:
        return None, f'보류 (n={iv.n}<{min_runs})'
    return 'PASS', None


def summarize(run_dirs: Iterable[Path]) -> dict:
    """{그룹: {"runs": [...], "failed": [...], "metrics": {지표: Interval + verdict}}}."""
    groups: dict[str, dict] = defaultdict(lambda: {'runs': [], 'failed': [], 'values': defaultdict(list)})
    for run_dir in sorted(Path(d) for d in run_dirs):
        meta = load_meta(run_dir)
        if not meta:
            continue
        g = groups[group_key(meta)]
        g['scenario'] = meta.get('scenario', '')
        if meta.get('outcome') != 'completed':
            g['failed'].append({'run_id': run_dir.name, 'outcome': meta.get('outcome')})
            continue
        g['runs'].append(run_dir.name)
        for k, v in run_metrics(run_dir).items():
            g['values'][k].append(v)

    result = {}
    for name, g in sorted(groups.items()):
        metrics = {}
        for i, (metric, values) in enumerate(sorted(g['values'].items())):
            iv = summarize_runs(values, seed=i)
            if iv is None:
                continue
            d = iv.as_dict()
            verdict = kpi_verdict(metric, iv.mean, iv.ci_high, iv.ci_low, g.get('scenario', ''))
            n_key = RESPONSE_KPI.get(metric)
            samples = int(sum(g['values'].get(n_key, []))) if n_key else 0
            min_runs = KPI_MIN_RUNS.get(metric, 0)
            if verdict in ('PASS', 'MARGINAL') and iv.n < len(g['runs']):
                # 값이 없는 실행을 빼고 통과시키면 잰 실행만 고르는 셈이다. 잰 실행만으로도 FAIL이면 FAIL로 둔다
                # (worst_kpi_verdict와 같은 원칙)
                d['kpi_note'] = f"보류 (값 없는 실행 {len(g['runs']) - iv.n}회)"
            elif verdict and iv.n < min_runs:
                d['kpi_note'] = f'보류 (n={iv.n}<{min_runs})'
            elif verdict and n_key and samples < RESPONSE_MIN_N:
                # 명세서 "50회 이상": 실행을 모두 합쳐 움직임을 찾은 명령이 모자라면 판정하지 않는다
                d['kpi_note'] = f'보류 (n={samples}<{RESPONSE_MIN_N})'
            elif verdict:
                d['kpi'] = verdict
            worst, note = worst_kpi_verdict(metric, iv, g.get('scenario', ''), len(g['runs']), len(g['failed']))
            if worst:
                d['kpi'] = worst
            elif note:
                d['kpi_note'] = note
            metrics[metric] = d
        result[name] = {'runs': g['runs'], 'failed': g['failed'], 'metrics': metrics}
    return result


def _fmt(v: Optional[float], metric: str) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '—'
    # 위치 오차는 cm로 보여 준다 (원값은 m)
    if metric.startswith('loc.') and not metric.endswith('.n'):
        return f'{v * 100:.2f}'
    return f'{v:.3g}' if abs(v) < 1000 else f'{v:.0f}'


def to_markdown(summary: dict) -> str:
    """사람이 읽는 표. 위치 오차는 cm, 지연은 ms."""
    lines = []
    for name, g in summary.items():
        lines.append(f'## {name}')
        lines.append('')
        lines.append(f"완료 {len(g['runs'])}회, 실패 {len(g['failed'])}회"
                     + (f" ({', '.join(f['run_id'] + ':' + str(f['outcome']) for f in g['failed'])})" if g['failed'] else ''))
        lines.append('')
        lines.append('| 지표 | N | 평균 | 95% CI | 최소 | 최대 | KPI |')
        lines.append('| --- | --- | --- | --- | --- | --- | --- |')
        for metric, d in g['metrics'].items():
            if metric.endswith('.n'):
                continue
            ci = '—' if d['ci_low'] is None else f"{_fmt(d['ci_low'], metric)} ~ {_fmt(d['ci_high'], metric)}"
            lines.append(f"| `{metric}` | {d['n']} | {_fmt(d['mean'], metric)} | {ci} | "
                         f"{_fmt(d['min'], metric)} | {_fmt(d['max'], metric)} | "
                         f"{d.get('kpi') or d.get('kpi_note', '')} |")
        lines.append('')
    lines.append('위치 오차(`loc.*`)는 cm. KPI: CI 상한까지 기준 안이면 PASS, 평균만 안이면 MARGINAL.')
    worst = sorted({m for g in summary.values() for m in g['metrics'] if m in WORST_KPI})
    if worst:
        lines.append(f"{', '.join(f'`{m}`' for m in worst)}는 최악값으로 판정한다: "
                     '모든 실행이 기준 이상이어야 PASS (최소 열).')
    return '\n'.join(lines) + '\n'
