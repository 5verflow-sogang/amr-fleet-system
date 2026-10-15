"""Nav KPI 계산 (ROS 불필요). nav_kpi.py가 부르고 tools/harness/tests/test_nav_kpi_core.py가 시험한다.

원 명세 4장 경로 계획 성능과 5장 경로 추종:
- 경로 계획 성공률 ≥ 98 % (출발·목표 50쌍)
- 경로 재계획 ≤ 500 ms
- 실제 주행 시간 대비 예측 시간 오차 ≤ 15 %
- CTE 직선 ≤ 5 cm, 곡선 ≤ 10 cm (계획 경로와 실제 궤적 사이 수직 거리 평균, 직선·곡선 분리)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

Point = tuple[float, float]

KPI = {
    'plan_success_rate': (0.98, 'min'),
    'replan_p95_ms': (500.0, 'max'),
    'pred_err_mean': (0.15, 'max'),
    'cte_straight_mean_m': (0.05, 'max'),
    'cte_curve_mean_m': (0.10, 'max'),
}


def path_length(path: Sequence[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def predicted_time(length_m: float, cruise_mps: float, accel_mps2: float) -> float:
    """사다리꼴 속도 프로파일로 간 시간. 순항 속도까지 못 올라가는 짧은 경로는 삼각형."""
    if length_m <= 0:
        return 0.0
    ramp = cruise_mps ** 2 / accel_mps2          # 가속 + 감속 구간 길이
    if length_m >= ramp:
        return length_m / cruise_mps + cruise_mps / accel_mps2
    return 2.0 * math.sqrt(length_m / accel_mps2)


def prediction_error(actual_s: float, predicted_s: float) -> float:
    """|실제 − 예측| / 실제 (원 명세: 실제 주행 시간 대비)."""
    return abs(actual_s - predicted_s) / actual_s if actual_s > 0 else math.nan


def curvature(path: Sequence[Point], window_m: float = 0.3) -> list[float]:
    """경로 점마다 곡률 [1/m]: 앞뒤 window_m 지점의 진행 방향 차이 ÷ 그 사이 호 길이."""
    n = len(path)
    s = [0.0]
    for a, b in zip(path, path[1:]):
        s.append(s[-1] + math.dist(a, b))
    heading = [math.atan2(path[min(i + 1, n - 1)][1] - path[max(i - 1, 0)][1],
                          path[min(i + 1, n - 1)][0] - path[max(i - 1, 0)][0]) for i in range(n)]
    out = []
    j0 = j1 = 0
    for i in range(n):
        while j0 < i and s[i] - s[j0] > window_m:
            j0 += 1
        while j1 < n - 1 and s[j1] - s[i] < window_m:
            j1 += 1
        arc = s[j1] - s[j0]
        d = math.atan2(math.sin(heading[j1] - heading[j0]), math.cos(heading[j1] - heading[j0]))
        out.append(abs(d) / arc if arc > 1e-6 else 0.0)
    return out


@dataclass(frozen=True)
class Projection:
    distance: float      # 계획 경로까지 수직 거리 (CTE) [m]
    index: int           # 가장 가까운 선분의 시작 점 번호
    s: float             # 경로 시작부터 투영점까지 호 길이 [m]


def cumulative(path: Sequence[Point]) -> list[float]:
    """경로 점마다 시작부터의 호 길이."""
    out = [0.0]
    for a, b in zip(path, path[1:]):
        out.append(out[-1] + math.dist(a, b))
    return out


def project(p: Point, path: Sequence[Point], cum: Sequence[float], hint: int = 0,
            search: int = 200) -> Optional[Projection]:
    """점을 경로(꺾은선)에 투영한다. hint(지난번 선분) 근처만 찾아, 경로가 되돌아오는 곳에 잘못 붙지 않게 한다."""
    best = None
    for i in range(max(hint - 5, 0), min(hint + search, len(path) - 1)):
        a, b = path[i], path[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
        d = math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))
        if best is None or d < best.distance:
            best = Projection(d, i, cum[i] + t * math.sqrt(l2))
    return best


def p95(values: Sequence[float]) -> float:
    """보간 없는 95번째 백분위 (최근접 순위). 값이 적으면 최댓값에 가깝다."""
    if not values:
        return math.nan
    v = sorted(values)
    return v[min(len(v) - 1, max(0, math.ceil(0.95 * len(v)) - 1))]


def median(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def summarize(pairs: Sequence[dict], replans_ms: Sequence[float], cte: Sequence[tuple[float, str]],
              cte_kpi: bool = True) -> dict:
    """쌍 결과·재계획 시간·CTE 샘플 → 실행 하나의 KPI 값과 판정.

    pairs: plan_ok(bool), nav_success(bool), pred_err(float 또는 None; 도착한 쌍만), recoveries(int, 없으면 0)
    cte: (거리, 'straight' | 'curve')
    cte_kpi: False면 CTE는 판정하지 않는다 (DWA로 잰 값은 참고 지표, CTE KPI는 path_tracker)
    """
    n = len(pairs)
    arrived = [p for p in pairs if p.get('pred_err') is not None and not math.isnan(p['pred_err'])]
    errs = [p['pred_err'] for p in arrived]
    clean = [p['pred_err'] for p in arrived if not p.get('recoveries')]
    out = {
        'pairs': n,
        'plan_success_rate': sum(bool(p['plan_ok']) for p in pairs) / n if n else math.nan,
        'nav_success_rate': sum(bool(p['nav_success']) for p in pairs) / n if n else math.nan,
        'replan_n': len(replans_ms),
        'replan_p95_ms': p95(replans_ms),
        'replan_max_ms': max(replans_ms) if replans_ms else math.nan,
        'pred_err_mean': mean(errs),
        'pred_err_within_15pct': sum(e <= 0.15 for e in errs) / len(errs) if errs else math.nan,
        'pred_err_median': median(errs),
        'pred_err_no_recovery_mean': mean(clean),
        'pred_err_no_recovery_n': len(clean),
        'cte_straight_mean_m': mean([d for d, seg in cte if seg == 'straight']),
        'cte_curve_mean_m': mean([d for d, seg in cte if seg == 'curve']),
        'cte_straight_max_m': max((d for d, seg in cte if seg == 'straight'), default=math.nan),
        'cte_curve_max_m': max((d for d, seg in cte if seg == 'curve'), default=math.nan),
    }
    verdict = {}
    for key, (threshold, kind) in KPI.items():
        if key.startswith('cte_') and not cte_kpi:
            continue
        v = out[key]
        if v is None or math.isnan(v):
            continue
        verdict[key] = 'PASS' if (v >= threshold if kind == 'min' else v <= threshold) else 'FAIL'
    out['verdict'] = verdict
    return out
