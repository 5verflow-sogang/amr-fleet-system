"""BPR 식으로 구역 정원 구하기 (ROS 없이 계산만).

통과 시간 T(n) = T0 × (1 + (n / C)^β)
  T0: 손님 0명일 때 통과 시간, n: 구역 안 손님 수, C: 정원, β: 사람이 늘수록 얼마나 급하게 막히는지
교통공학 BPR(Bureau of Public Roads) 함수에서 α를 1로 둔 꼴이다. α = 1이면 n = C일 때 T = 2 × T0라서
"통과 시간이 2배가 되는 인원 = 정원"이라는 우리 정의와 같다.
"""

from __future__ import annotations

import math
from statistics import median
from typing import Iterable, Mapping, Optional, Sequence


def bpr(n: float, t0: float, c: float, beta: float) -> float:
    return t0 * (1.0 + (n / c) ** beta)


def medians(samples: Mapping[int, Sequence[float]]) -> dict[int, float]:
    """손님 수마다 통과 시간 중앙값. 시간 초과로 못 지난 통과는 제한 시간 값으로 넣는다 (중앙값은 그 값이 정확하지 않아도 덜 흔들린다)."""
    return {n: median(v) for n, v in sorted(samples.items()) if v}


def doubling_point(meds: Mapping[int, float]) -> Optional[float]:
    """중앙값이 처음 2 × T0를 넘는 두 인원 사이를 직선으로 이어 2 × T0가 되는 인원. 끝까지 안 넘으면 None (정원 > 잰 최대 인원)."""
    if 0 not in meds:
        return None
    target = 2.0 * meds[0]
    pts = sorted(meds.items())
    for (n0, t0), (n1, t1) in zip(pts, pts[1:]):
        if t0 < target <= t1:
            return n0 + (n1 - n0) * (target - t0) / (t1 - t0)
    return None


def fit(meds: Mapping[int, float], c_grid: Optional[Iterable[float]] = None,
        beta_grid: Optional[Iterable[float]] = None) -> dict:
    """T0는 0명 중앙값으로 고정하고, C·β를 격자로 찾아 손님 1명 이상인 중앙값의 상대 오차 제곱합을 최소로 한다."""
    if 0 not in meds:
        raise ValueError('손님 0명 측정이 있어야 T0를 정한다')
    t0 = meds[0]
    pts = [(n, t) for n, t in meds.items() if n > 0]
    if len(pts) < 2:
        raise ValueError('손님 1명 이상인 조건이 둘 이상 있어야 C와 β를 함께 맞춘다')
    c_grid = list(c_grid) if c_grid is not None else [round(0.5 + 0.05 * i, 2) for i in range(391)]   # 0.5 ~ 20
    beta_grid = list(beta_grid) if beta_grid is not None else [round(0.3 + 0.05 * i, 2) for i in range(75)]  # 0.3 ~ 4
    best = None
    for c in c_grid:
        for b in beta_grid:
            err = sum(((bpr(n, t0, c, b) - t) / t) ** 2 for n, t in pts)
            if best is None or err < best[0]:
                best = (err, c, b)
    err, c, b = best
    return {'t0': t0, 'capacity': c, 'beta': b, 'rmse_rel': math.sqrt(err / len(pts)),
            'at_edge': c in (c_grid[0], c_grid[-1]) or b in (beta_grid[0], beta_grid[-1])}


def congestion_weight(beta: float, n_ref: float, c: float) -> float:
    """배정 비용 weight 근거: 혼잡도 v인 구역 1 m를 지나면 (1 + weight × v) m를 간 셈 = T(n)/T0.
    혼잡도를 지금처럼 v = n / C로 두면 weight = (n/C)^β / (n/C). β = 1이면 1이다."""
    v = n_ref / c
    return (v ** beta) / v if v > 0 else float('nan')
