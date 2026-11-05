"""할당 비용 함수."""

from __future__ import annotations

import math
from typing import Sequence

from .types import CostFn, Point, RobotInfo, TaskInfo

# 혼잡 구역 1개: ([x_min, x_max, y_min, y_max], 혼잡도 0~1). amr_msgs/ZoneCongestion의 area·value
CongestedArea = tuple[tuple[float, float, float, float], float]


def euclidean_cost(robot: RobotInfo, task: TaskInfo) -> float:
    """로봇 → 작업 시작 지점 직선 거리 (m).

    픽업 → 목적지 구간은 어느 로봇이 맡아도 같으므로 넣지 않는다.
    """
    return math.dist(robot.xy, task.start_xy)


def segment_length_in_rect(a: Point, b: Point, area: Sequence[float]) -> float:
    """선분 a→b 중 사각형 area([x_min, x_max, y_min, y_max]) 안에 든 길이 (Liang-Barsky 자르기)."""
    x0, x1, y0, y1 = area
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p == 0.0:
            if q < 0.0:
                return 0.0   # 축에 평행하고 사각형 밖
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return 0.0
    return (t1 - t0) * math.hypot(dx, dy)


def make_congestion_cost(areas: Sequence[CongestedArea], weight: float) -> CostFn:
    """직선 거리 + weight × Σ(구역 혼잡도 × 로봇 → 작업 시작 지점 선분이 그 구역 안을 지나는 길이) (m).

    혼잡도 1.0인 구역 1 m를 지나면 (1 + weight) m를 간 것으로 본다. 실제 경로가 아니라 직선이라, 벽 너머 구역이
    잡히거나 경로가 돌아가는 구역을 놓칠 수 있다 (식당처럼 트인 홀을 가정한다). areas가 비거나 weight가 0이면
    euclidean_cost와 같다.
    """
    if weight < 0.0:
        raise ValueError(f'혼잡도 가중치는 0 이상이어야 한다: {weight}')
    areas = [(tuple(area), value) for area, value in areas if value > 0.0]

    def congestion_cost(robot: RobotInfo, task: TaskInfo) -> float:
        a, b = robot.xy, task.start_xy
        extra = sum(value * segment_length_in_rect(a, b, area) for area, value in areas)
        return math.dist(a, b) + weight * extra

    return congestion_cost
