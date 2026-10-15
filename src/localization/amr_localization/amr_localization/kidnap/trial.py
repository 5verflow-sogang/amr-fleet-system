"""순간 이동(Kidnapped) 시험 한 번의 목표 자세 고르기와 성공 판정. ROS에 의존하지 않는다.

판정 기준은 docs/algorithms/localization.md 4-4절이다.
- 감지: 순간 이동 뒤 detect_s(5초) 안에 복구 노드 상태가 tracking에서 벗어나는 전환.
  순간 이동 순간에 이미 tracking이 아니면(앞서 복구 중) 감지를 잴 수 없으니 무효(valid=False)로 뺀다 (#148 하연 리뷰)
- 복구: 기준 시각(감지, 못 했으면 순간 이동) 뒤 window_s(60초) 안에, 로봇 진행 방향 오차 ≤ along_m(0.3 m),
  옆 방향 오차 ≤ lateral_m(0.1 m), 방향 오차 ≤ yaw_rad(0.1 rad)를 hold_s(3초) 이상 유지
- 성공: 감지와 복구 둘 다
"""

from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass
from typing import Optional, Sequence

from ..eval.trajectory import Pose2D

NEAR, FAR = 'near', 'far'

# 창고 월드(amr_gazebo warehouse.sdf) 통로. 랙 열 A(y=5)·B(y=−2)·C(y=−9), 깊이 1.2 m라 열 사이 통로 폭은 5.8 m.
# 로봇(0.6 × 0.4 m)이 랙에 닿지 않게 통로 가운데 ±1 m 안에만 놓는다
AISLE_AB_Y = 1.5
AISLE_BC_Y = -5.5
AISLE_HALF_M = 1.0
AISLE_X = (-20.0, 20.0)
# B–C 통로 가운데 기둥 (x, y). 1.5 m 안에는 놓지 않는다
PILLARS_BC = ((-7.5, -5.5), (7.5, -5.5))
PILLAR_CLEAR_M = 1.5


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pick_target(mode: str, seed: int, start: Pose2D) -> Pose2D:
    """순간 이동할 world 자세. 같은 seed면 같은 곳으로 옮긴다.

    near: 지금 통로(A–B) 안에서 진행 방향 1~2.5 m, 옆 ±1 m 안, 방향 ±0.2~0.6 rad. 국소 재초기화(σ 2.5 m)가 덮는 범위
    far:  다른 통로(B–C)의 무작위 위치·방향. 랙이 7.5 m 간격으로 반복돼 스캔만으로는 어느 통로인지 헷갈리는 경우
    """
    rng = random.Random(seed)
    if mode == NEAR:
        x = start.x + rng.choice((-1.0, 1.0)) * rng.uniform(1.0, 2.5)
        y = AISLE_AB_Y + rng.uniform(-AISLE_HALF_M, AISLE_HALF_M)
        yaw = wrap(start.yaw + rng.choice((-1.0, 1.0)) * rng.uniform(0.2, 0.6))
        return Pose2D(start.t, x, y, yaw)
    if mode == FAR:
        while True:
            x = rng.uniform(*AISLE_X)
            if all(math.hypot(x - px, AISLE_BC_Y - py) > PILLAR_CLEAR_M for px, py in PILLARS_BC):
                break
        y = AISLE_BC_Y + rng.uniform(-AISLE_HALF_M, AISLE_HALF_M)
        return Pose2D(start.t, x, y, rng.uniform(-math.pi, math.pi))
    raise ValueError(f'mode는 {NEAR} 또는 {FAR} (지금 {mode!r})')


def robot_frame_error(est: Pose2D, gt: Pose2D) -> tuple[float, float, float]:
    """추정 − 정답을 정답 로봇 기준으로 나눈다: (진행 방향 m, 옆 방향 m, 방향 rad)."""
    dx, dy = est.x - gt.x, est.y - gt.y
    c, s = math.cos(gt.yaw), math.sin(gt.yaw)
    return c * dx + s * dy, -s * dx + c * dy, wrap(est.yaw - gt.yaw)


@dataclass(frozen=True)
class Criteria:
    detect_s: float = 5.0
    window_s: float = 60.0
    along_m: float = 0.3
    lateral_m: float = 0.1
    yaw_rad: float = 0.1
    hold_s: float = 3.0


@dataclass(frozen=True)
class Sample:
    """한 시점의 오차 (순간 이동 뒤). t는 시뮬레이션 시각."""

    t: float
    along: float
    lateral: float
    yaw: float

    def ok(self, c: Criteria) -> bool:
        return abs(self.along) <= c.along_m and abs(self.lateral) <= c.lateral_m and abs(self.yaw) <= c.yaw_rad


def first_hold(samples: Sequence[Sample], t_from: float, t_until: float, c: Criteria) -> Optional[float]:
    """t_from 뒤 기준 안 오차가 hold_s 이상 이어지기 시작한 가장 이른 시각 (t_until을 넘으면 None)."""
    start = None
    for s in samples:
        if s.t < t_from:
            continue
        if not s.ok(c):
            start = None
            continue
        if start is None:
            if s.t > t_until:
                return None
            start = s.t
        if s.t - start >= c.hold_s:
            return start
    return None


def evaluate(t_kidnap: float, states: Sequence[tuple[float, str]], samples: Sequence[Sample],
             c: Criteria = Criteria()) -> dict:
    """시험 한 번 판정. states는 (시각, 복구 노드 상태) 목록, samples는 순간 이동 뒤 오차."""
    false_alarms = sum(1 for (t, s), (_, prev) in zip(states[1:], states)
                       if t < t_kidnap and prev == 'tracking' and s != 'tracking')
    before = [s for t, s in states if t < t_kidnap]
    valid = not before or before[-1] == 'tracking'
    # tracking → 다른 상태로 바뀐 첫 전환만 감지로 센다
    t_detect = next((t for (t, s), (_, prev) in zip(states[1:], states) if t >= t_kidnap and prev == 'tracking'
                     and s != 'tracking'), None)
    detect_s = None if t_detect is None else t_detect - t_kidnap
    detected = detect_s is not None and detect_s <= c.detect_s
    t_ref = t_detect if detected else t_kidnap
    t_hold = first_hold(samples, t_ref, t_ref + c.window_s, c)
    last = samples[-1] if samples else None
    # 판정 창이 끝난 뒤의 복구(늦은 복구)도 따로 센다. failed에서 tracking으로 돌아오는 경우가 있다 (recovery.py)
    t_late = first_hold(samples, t_ref + c.window_s, math.inf, c) if t_hold is None else None
    return {
        'valid': valid,
        'detected': detected,
        'detect_s': None if detect_s is None else round(detect_s, 2),
        'recovered': t_hold is not None,
        'recover_s': None if t_hold is None else round(t_hold - t_ref, 2),
        'success': valid and detected and t_hold is not None,
        'late_recover_s': None if t_late is None else round(t_late - t_ref, 2),
        'false_alarms': false_alarms,
        'final_state': states[-1][1] if states else None,
        'final_error': None if last is None else {
            'along_m': round(last.along, 3), 'lateral_m': round(last.lateral, 3), 'yaw_rad': round(last.yaw, 3)},
        'criteria': asdict(c),
    }
