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

from .stats import percentile, summarize_runs

# 원 명세서 위치 오차 KPI [m] (amr_localization/eval/metrics.py KPI_THRESHOLDS와 같은 값)
LOCALIZATION_KPI = {'stationary': 0.03, 'straight': 0.05, 'rotation': 0.08}
# 원 명세서 응답 시간 KPI [ms]
RESPONSE_KPI_MS = 200.0
# 작업 실패 원인 묶음 (ErrorInfo.code 대역, docs/api.md). 위치(203)를 주행(201·202)과 나눠 센다 (#148 지수 합의)
FAILURE_CAUSES = {'localization': {'203'}, 'nav': {'201', '202'}, 'traffic': {'301', '302', '303'},
                  'comm': {'401', '402'}, 'task_timeout': {'501'}, 'charge': {'502'},
                  'docking': {str(c) for c in range(900, 1000)}}
# 원 명세 경로 계획(4장)·경로 추종(5장) KPI: (기준, 방향). min = 클수록 좋음 (CI 하한으로 판정), max = 작을수록 좋음
NAV_KPI = {'navkpi.plan_success_rate': (0.98, 'min'), 'navkpi.replan_p95_ms': (500.0, 'max'),
           'navkpi.pred_err_mean': (0.15, 'max'), 'navkpi.cte_straight_mean_m': (0.05, 'max'),
           'navkpi.cte_curve_mean_m': (0.10, 'max')}


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


def fleet_metrics(run_dir: Path) -> dict[str, float]:
    """fleet_latency.csv, response_time.csv, task_outcomes.csv, recovery.csv → 실행 하나의 요약 값."""
    out = {}
    lat_rows = _read_csv(run_dir / 'fleet_latency.csv')
    judged = [r for r in lat_rows if r.get('delivered') in ('0', '1')]
    if judged:
        out['fleet.loss_rate'] = sum(r['delivered'] == '0' for r in judged) / len(judged)
        lat = sorted(_floats((r for r in judged if r['delivered'] == '1'), 'latency_ms'))
        if lat:
            out['fleet.latency_ms.mean'] = sum(lat) / len(lat)
            out['fleet.latency_ms.p95'] = percentile(lat, 0.95)
            out['fleet.latency_ms.p99'] = percentile(lat, 0.99)
            out['fleet.latency_ms.max'] = lat[-1]
    resp_rows = _read_csv(run_dir / 'response_time.csv')
    if resp_rows:
        out['fleet.commands'] = len(resp_rows)
        acked = sorted(_floats(resp_rows, 'ack_rtt_ms'))
        out['fleet.ack_rate'] = len(acked) / len(resp_rows)
        if acked:
            out['fleet.ack_rtt_ms.mean'] = sum(acked) / len(acked)
            out['fleet.ack_rtt_ms.p95'] = percentile(acked, 0.95)
            out['fleet.ack_rtt_ms.max'] = acked[-1]
        out['fleet.resends.mean'] = sum(_floats(resp_rows, 'resends')) / len(resp_rows)
        resp = sorted(_floats(resp_rows, 'latency_ms'))
        if resp:
            out['fleet.response_ms.mean'] = sum(resp) / len(resp)
            out['fleet.response_ms.max'] = resp[-1]
            out['fleet.response_ms.n'] = len(resp)
    # 작업 결과. 끝 무렵에 요청된 작업은 실행 안에 못 끝나므로, 완료율은 같은 seed(같은 작업 일정)끼리 비교한다
    tasks = _read_csv(run_dir / 'task_outcomes.csv')
    if tasks:
        done = sum(r['final_state'] == 'COMPLETED' for r in tasks)
        out['fleet.tasks_requested'] = len(tasks)
        out['fleet.tasks_completed'] = done
        out['fleet.task_completion_rate'] = done / len(tasks)
        # 실패 원인 (amr_msgs ErrorInfo 코드, #173). task_outcomes.csv에 error_code 열이 있을 때만 (fleet_metrics, 하연)
        if 'error_code' in tasks[0]:
            failed = [r for r in tasks if r['final_state'] == 'FAILED']
            for name, codes in FAILURE_CAUSES.items():
                out[f'fleet.tasks_failed_{name}'] = sum(r['error_code'] in codes for r in failed)
    # 로봇 장애 회수 (fault_events.csv에 robot_kill이 있을 때만 파일이 있다)
    reclaimed = [r for r in _read_csv(run_dir / 'recovery.csv') if r.get('task_id')]
    if (run_dir / 'recovery.csv').exists():
        out['fleet.reclaimed_tasks'] = len(reclaimed)
        recov = _floats(reclaimed, 'recovery_s')
        if recov:
            out['fleet.recovery_s.mean'] = sum(recov) / len(recov)
            out['fleet.recovery_s.max'] = max(recov)
        if reclaimed:
            out['fleet.reclaimed_completed_rate'] = sum(r['completed'] == '1' for r in reclaimed) / len(reclaimed)
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
                    'pred_err_median', 'pred_err_no_recovery_mean', 'cte_straight_mean_m', 'cte_curve_mean_m'):
            v = k.get(key)
            if isinstance(v, (int, float)) and not math.isnan(v):
                name = key if cte_kpi or not key.startswith('cte_') else 'ref_' + key
                out[f'navkpi.{name}'] = float(v)
    return out


def run_metrics(run_dir: Path) -> dict[str, float]:
    meta = load_meta(run_dir)
    out = {**localization_metrics(run_dir), **fleet_metrics(run_dir), **nav_metrics(run_dir),
           **kidnap_metrics(run_dir), **nav_kpi_metrics(run_dir)}
    rtf = meta.get('real_time_factor')
    if isinstance(rtf, (int, float)):
        out['run.real_time_factor'] = float(rtf)
    return out


def group_key(meta: dict) -> str:
    """같은 조건끼리 묶는 이름: 시나리오 + 라벨 (라벨은 run.py --label)."""
    label = meta.get('label') or ''
    return f"{meta.get('scenario', '?')}{'/' + label if label else ''}"


def kpi_verdict(metric: str, mean: float, ci_high: Optional[float], ci_low: Optional[float] = None) -> Optional[str]:
    """KPI가 있는 지표만 판정한다. CI 상한까지 기준 안이면 PASS, 평균만 안이면 MARGINAL, 아니면 FAIL.

    클수록 좋은 지표(성공률)는 거꾸로 CI 하한까지 기준 이상이면 PASS다.
    """
    threshold = None
    parts = metric.split('.')
    if parts[0] == 'loc' and parts[-1] == 'rmse':
        threshold = LOCALIZATION_KPI.get(parts[-2])
    elif metric == 'fleet.response_ms.mean':
        threshold = RESPONSE_KPI_MS
    elif metric in NAV_KPI:
        threshold, kind = NAV_KPI[metric]
        if kind == 'min':
            if ci_low is not None and ci_low >= threshold:
                return 'PASS'
            return 'MARGINAL' if mean >= threshold else 'FAIL'
    if threshold is None:
        return None
    if ci_high is not None and ci_high <= threshold:
        return 'PASS'
    return 'MARGINAL' if mean <= threshold else 'FAIL'


def summarize(run_dirs: Iterable[Path]) -> dict:
    """{그룹: {"runs": [...], "failed": [...], "metrics": {지표: Interval + verdict}}}."""
    groups: dict[str, dict] = defaultdict(lambda: {'runs': [], 'failed': [], 'values': defaultdict(list)})
    for run_dir in sorted(Path(d) for d in run_dirs):
        meta = load_meta(run_dir)
        if not meta:
            continue
        g = groups[group_key(meta)]
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
            verdict = kpi_verdict(metric, iv.mean, iv.ci_high, iv.ci_low)
            if verdict:
                d['kpi'] = verdict
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
                         f"{_fmt(d['min'], metric)} | {_fmt(d['max'], metric)} | {d.get('kpi', '')} |")
        lines.append('')
    lines.append('위치 오차(`loc.*`)는 cm. KPI: CI 상한까지 기준 안이면 PASS, 평균만 안이면 MARGINAL.')
    return '\n'.join(lines) + '\n'
