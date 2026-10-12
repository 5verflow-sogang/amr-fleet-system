"""학습 곡선 요약 지표 (scripts/plot_curves.py가 쓴다). numpy만 쓴다.

튜닝 비교는 마지막 값 하나가 아니라 곡선 전체로 한다. 마지막 값만 보면 빨리 배우는지, 중간에 무너지는지,
시드마다 흔들리는지가 사라진다 (README "학습 곡선 비교").
"""

from typing import Optional, Sequence

import numpy as np


def curve_summary(steps: Sequence[float], values: Sequence[float], final_frac: float = 0.25,
                  threshold: float = 0.8) -> dict:
    """검증 곡선 하나(스텝, 값)의 요약.

    - auc: 모든 검증 점의 평균. 얼마나 빨리, 꾸준히 배우는지 (점 간격이 같으므로 곡선 아래 면적에 비례)
    - final: 마지막 final_frac 구간(스텝 기준) 점들의 평균. 끝에서의 성능
    - peak: 최고값. 여러 번 잰 것 중 최댓값이라 위로 치우치므로 대표값으로 쓰지 않는다
    - drop: peak − final. 학습 중에 무너졌는지
    - steps_to: 처음 threshold 이상이 된 스텝 (없으면 None). 학습 속도
    """
    s = np.asarray(steps, dtype=float)
    v = np.asarray(values, dtype=float)
    if s.size == 0:
        raise ValueError('빈 곡선')
    order = np.argsort(s)
    s, v = s[order], v[order]
    final_mask = s >= s[-1] - final_frac * (s[-1] - s[0])
    final = float(np.mean(v[final_mask]))
    above = np.nonzero(v >= threshold)[0]
    steps_to: Optional[float] = float(s[above[0]]) if above.size else None
    peak = float(np.max(v))
    return {'points': int(s.size), 'last_step': float(s[-1]), 'auc': float(np.mean(v)),
            'final': final, 'peak': peak, 'drop': peak - final, 'steps_to': steps_to}


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
