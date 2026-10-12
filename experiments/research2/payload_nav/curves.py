"""학습 곡선 요약 지표 (scripts/plot_curves.py가 쓴다). numpy만 쓴다.

튜닝 비교는 마지막 값 하나가 아니라 곡선 전체로 한다. 마지막 값만 보면 빨리 배우는지, 중간에 무너지는지,
시드마다 흔들리는지가 사라진다 (README "학습 곡선 비교").
"""

from typing import Optional, Sequence

import numpy as np


def common_horizon(curves: Sequence[tuple]) -> float:
    """여러 곡선이 모두 기록을 가진 마지막 스텝 (각 곡선 마지막 스텝의 최솟값).

    실행마다 학습량이 다르면(예: 272만 스텝과 103만 스텝) "최종"이 서로 다른 구간의 평균이 되어 비교할 수 없다.
    요약은 이 스텝까지 잘라서 낸다.
    """
    return min(float(np.max(np.asarray(s, dtype=float))) for s, _ in curves)


def curve_summary(steps: Sequence[float], values: Sequence[float], final_frac: float = 0.25,
                  threshold: Optional[float] = None, higher_is_better: bool = True,
                  until: Optional[float] = None) -> dict:
    """검증 곡선 하나(스텝, 값)의 요약.

    - until: 이 스텝까지만 쓴다 (common_horizon). 없으면 곡선 전체
    - auc: 쓴 구간 모든 검증 점의 평균. 얼마나 빨리, 꾸준히 배우는지 (점 간격이 같으므로 곡선 아래 면적에 비례)
    - final: 쓴 구간의 마지막 final_frac(스텝 기준) 점들의 평균. 끝에서의 성능
    - best: 가장 좋은 값. 높을수록 좋은 지표(성공률)는 최고, 낮을수록 좋은 지표(충돌률·시간 초과율)는 최저.
      여러 번 잰 것 중 극값이라 좋은 쪽으로 치우치므로 대표값으로 쓰지 않는다
    - worsen: best에서 final까지 나빠진 양 (0 이상이면 학습 중 나빠졌다)
    - steps_to: 처음 threshold에 닿은 스텝 (높을수록 좋으면 이상, 낮을수록 좋으면 이하). threshold가 없거나
      닿지 못하면 None. 학습 속도
    """
    s = np.asarray(steps, dtype=float)
    v = np.asarray(values, dtype=float)
    order = np.argsort(s)
    s, v = s[order], v[order]
    if until is not None:
        keep = s <= until + 1e-9
        s, v = s[keep], v[keep]
    if s.size == 0:
        raise ValueError('빈 곡선')
    final_mask = s >= s[-1] - final_frac * (s[-1] - s[0])
    final = float(np.mean(v[final_mask]))
    best = float(np.max(v)) if higher_is_better else float(np.min(v))
    steps_to: Optional[float] = None
    if threshold is not None:
        hit = np.nonzero(v >= threshold if higher_is_better else v <= threshold)[0]
        steps_to = float(s[hit[0]]) if hit.size else None
    return {'points': int(s.size), 'last_step': float(s[-1]), 'auc': float(np.mean(v)),
            'final': final, 'best': best, 'worsen': (best - final) if higher_is_better else (final - best),
            'steps_to': steps_to}


def mean_band(curves: Sequence[tuple], num: int = 50):
    """여러 시드 곡선을 공통 스텝 격자로 맞춘 평균·최소·최대. 모든 시드가 있는 구간까지만 쓴다.

    curves: [(steps, values), ...]. 반환: (grid, mean, low, high)
    """
    start = max(float(np.min(s)) for s, _ in curves)
    end = min(float(np.max(s)) for s, _ in curves)
    if end <= start:
        grid = np.array([end])
    else:
        grid = np.linspace(start, end, num)
    stack = np.stack([np.interp(grid, np.asarray(s, float), np.asarray(v, float)) for s, v in curves])
    return grid, stack.mean(axis=0), stack.min(axis=0), stack.max(axis=0)
