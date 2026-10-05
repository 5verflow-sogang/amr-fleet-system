"""동적 장애물 이동 패턴. rclpy를 import하지 않는다 (단위 테스트용).

config/dynamic_obstacles.yaml의 장애물 1개를 받아, 현재 위치에서 낼 월드 기준 속도를 정한다.

- straight: waypoints 첫 점과 끝 점 사이 직선 왕복
- curve: waypoints를 지나는 Catmull-Rom 곡선을 따라 왕복
- random: waypoints가 이루는 사각 영역 안에서 무작위 목표점으로 직선 이동 (random waypoint 모델)

속도는 구간(왕복 한 번, 무작위 목표점 하나)마다 speed_range_mps 안에서 무작위로 정한다.
"""

from __future__ import annotations

import math
import random
from typing import List, Sequence, Tuple

Point = Tuple[float, float]

PATTERNS = ('straight', 'curve', 'random')
# 목표점에 이만큼 가까워지면 다음 점으로 넘어간다 [m]
REACH_TOLERANCE = 0.15
# 곡선 표본 간격 [m]
CURVE_SPACING = 0.5


def catmull_rom(points: Sequence[Point], spacing: float = CURVE_SPACING) -> List[Point]:
    """points를 모두 지나는 Catmull-Rom 곡선을 spacing 간격 정도로 표본화한다 (양 끝점 포함)."""
    if len(points) < 3:
        return [tuple(p) for p in points]
    # 양 끝은 끝점을 한 번 더 써서 곡선이 끝점에서 시작·종료하게 한다
    ext = [points[0], *points, points[-1]]
    out: List[Point] = [tuple(points[0])]
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        n = max(1, math.ceil(math.dist(p1, p2) / spacing))
        for k in range(1, n + 1):
            t = k / n
            t2, t3 = t * t, t * t * t
            out.append(tuple(
                0.5 * (2 * p1[j] + (-p0[j] + p2[j]) * t
                       + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t2
                       + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t3)
                for j in range(2)))
    return out


class ObstacleMotion:
    """장애물 1개의 이동 상태. step()에 현재 위치를 주면 낼 속도 (vx, vy) [m/s, 월드 기준]를 돌려준다."""

    def __init__(self, spec: dict, rng: random.Random):
        self.id = spec['id']
        self.pattern = spec['pattern']
        if self.pattern not in PATTERNS:
            raise ValueError(f'{self.id}: pattern은 {PATTERNS} 중 하나여야 한다: {self.pattern}')
        lo, hi = spec['speed_range_mps']
        if not 0 < lo <= hi:
            raise ValueError(f'{self.id}: speed_range_mps가 잘못됐다: {spec["speed_range_mps"]}')
        self.speed_range = (float(lo), float(hi))
        waypoints = [(float(w['x']), float(w['y'])) for w in spec['waypoints']]
        if len(waypoints) < 2:
            raise ValueError(f'{self.id}: waypoints가 2개 이상 필요하다')
        self._rng = rng

        if self.pattern == 'random':
            xs, ys = zip(*waypoints)
            self.area = (min(xs), max(xs), min(ys), max(ys))
            self.path: List[Point] = []
            self._goal = self._random_goal()
        else:
            self.path = waypoints[:1] + waypoints[-1:] if self.pattern == 'straight' \
                else catmull_rom(waypoints)
            self._index = 1      # 지금 향하는 path 점
            self._dir = 1        # 1: 정방향, -1: 되돌아오는 중
            self._goal = self.path[1]
        self.speed = self._new_speed()

    @property
    def goal(self) -> Point:
        return self._goal

    def step(self, xy: Point) -> Tuple[float, float]:
        """현재 위치 xy에서 낼 월드 기준 속도. 목표점에 닿으면 다음 목표점으로 넘긴다."""
        while math.dist(xy, self._goal) < REACH_TOLERANCE:
            self._advance()
        dx, dy = self._goal[0] - xy[0], self._goal[1] - xy[1]
        d = math.hypot(dx, dy)
        return self.speed * dx / d, self.speed * dy / d

    def _advance(self) -> None:
        if self.pattern == 'random':
            self._goal = self._random_goal()
            self.speed = self._new_speed()
            return
        nxt = self._index + self._dir
        if not 0 <= nxt < len(self.path):
            # 끝에 닿았다: 방향을 바꾸고 이번 왕복 구간의 속도를 새로 뽑는다
            self._dir = -self._dir
            nxt = self._index + self._dir
            self.speed = self._new_speed()
        self._index = nxt
        self._goal = self.path[nxt]

    def _random_goal(self) -> Point:
        x0, x1, y0, y1 = self.area
        return self._rng.uniform(x0, x1), self._rng.uniform(y0, y1)

    def _new_speed(self) -> float:
        return self._rng.uniform(*self.speed_range)


def body_twist(vx: float, vy: float, yaw: float, yaw_gain: float = 2.0,
               max_yaw_rate: float = 1.5) -> Tuple[float, float, float]:
    """월드 기준 속도를 몸체 기준 (vx, vy, wz)로 바꾼다. 몸체는 진행 방향을 보도록 돌린다.

    이동은 몸체 방향과 상관없이 정확히 따라가고 (전방향 이동), 방향 회전은 겉모습과
    LiDAR에 보이는 모양을 위한 것이다.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = c * vx + s * vy, -s * vx + c * vy
    if math.hypot(vx, vy) < 1e-6:
        return bx, by, 0.0
    err = math.atan2(vy, vx) - yaw
    err = math.atan2(math.sin(err), math.cos(err))
    wz = max(-max_yaw_rate, min(max_yaw_rate, yaw_gain * err))
    return bx, by, wz
