"""동적 장애물 이동 패턴. rclpy를 import하지 않는다 (단위 테스트용).

config/dynamic_obstacles.yaml의 장애물 1개를 받아, 현재 위치에서 낼 월드 기준 속도를 정한다.

- straight: waypoints 첫 점과 끝 점 사이 직선 왕복
- curve: waypoints를 지나는 Catmull-Rom 곡선을 따라 왕복
- random: waypoints가 이루는 사각 영역 안에서 무작위 목표점으로 직선 이동 (random waypoint 모델)

속도는 구간(왕복 한 번, 무작위 목표점 하나)마다 speed_range_mps 안에서 무작위로 정한다.

yield_to_robots(선택, 로봇 네임스페이스 목록)를 주면 그 로봇이 멈춘 채 앞길을 막을 때 그 앞에서 선다.
사람은 멈춘 로봇에 걸어 들어가지 않는다. 움직이는 로봇에는 비키지 않는다 (로봇이 피하는지 보는 시험이므로).
한번 서면 로봇이 다시 움직여도 이번 구간(다음 경유점까지) 길에서 비킬 때까지 기다린다
(막 출발한 로봇에 걸어 들어가지 않게). 로봇이 YIELD_PATIENCE_S 동안 계속 멈춰 있으면 온 쪽으로 돌아선다.
straight·random 패턴만 쓸 수 있다 (curve는 구간이 표본 간격 0.5 m로 짧아 앞을 충분히 못 본다).
yield_robot_size(선택, [길이, 폭] m)는 비킬 때 보는 로봇 차체 크기다. 없으면 물류 AMR(ROBOT_LENGTH × ROBOT_WIDTH).
식당 서빙 로봇(body:=serving)은 0.565 × 0.537 m를 준다.
yield_patience_s·yield_progress_m(선택)은 언제 돌아설지 정한다. 기본(없음)은 막은 로봇의 속력이 STOPPED_SPEED 아래로
YIELD_PATIENCE_S 동안 이어질 때다. yield_progress_m을 주면 속력 대신 위치를 본다: 막은 로봇이 그만큼도 못 움직인 채
yield_patience_s가 지나면 돌아선다. 제자리에서 돌거나 조금씩 들썩이는 로봇(속력이 기준을 넘나든다) 앞에서 손님이 한없이
서 있지 않게 한다 (식당 리허설: 손님과 로봇이 서로 기다림). random 패턴은 돌아설 때 로봇이 막지 않는 목표점을 고른다.
"""

from __future__ import annotations

import math
import random
from typing import List, Mapping, Optional, Sequence, Tuple

Point = Tuple[float, float]
# 로봇 상태: world x, y [m], yaw [rad], 속력 [m/s]
RobotState = Tuple[float, float, float, float]

PATTERNS = ('straight', 'curve', 'random')
# 목표점에 이만큼 가까워지면 다음 점으로 넘어간다 [m]
REACH_TOLERANCE = 0.15
# 곡선 표본 간격 [m]
CURVE_SPACING = 0.5

# yield_to_robots: 로봇 GT 속력이 이 값 [m/s] 미만이면 멈춘 것으로 본다
STOPPED_SPEED = 0.05
# 몸이 진행 방향으로 이만큼 [m] 더 가면 멈춘 로봇 차체에 닿을 때 선다. 사람이 두는 간격이고, 멀리서 다가와
# 서면 안전 노드 긴급정지 띠(차체 앞 0.3 m) 밖이라 선 작업자 때문에 로봇이 긴급정지에 묶이지 않는다
YIELD_DISTANCE = 0.5
# 몸 둘레에서 이만큼 [m] 안으로 스쳐도 막힌 것으로 본다 (avoidance_eval 충돌 기준 0.01 m + 몸을 돌리며 걸어
# 0.5 m 앞 예측 직선에서 옆으로 벗어나는 정도: I17 시험 녹화 30개에서 중앙값 0.022 m, p99 0.029 m)
YIELD_SIDE_MARGIN = 0.05
# 서 있는 동안 막은 로봇이 이만큼 [s] 계속 멈춰 있으면 돌아선다. 로봇이 작업자가 이미 가까이 있을 때 서면
# 작업자는 긴급정지 띠 안이고 로봇은 후진하지 않아(min_vel_x 0) 둘 다 기다리기만 한다 (로컬 시험에서 50 s)
YIELD_PATIENCE_S = 5.0
# 로봇 차체 [m] (amr_description, avoidance_eval robot_length·width). yield_robot_size가 없을 때 쓴다
ROBOT_LENGTH = 0.6
ROBOT_WIDTH = 0.4
# 장애물 몸을 덮는 원 반지름 [m] (models/dynamic_*/model.sdf 충돌체: 원기둥 r 0.25, 1.2×0.9·0.6×0.4 상자의 외접원)
# customer: 식당 손님(models/dynamic_customer 충돌체 원기둥 r 0.25)
BODY_RADIUS = {'worker': 0.25, 'forklift': 0.75, 'robot': 0.36, 'customer': 0.25}


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
        yield_to = spec.get('yield_to_robots') or []
        if not isinstance(yield_to, list):
            raise ValueError(f'{self.id}: yield_to_robots는 로봇 이름 목록이어야 한다: {yield_to!r}')
        self.yield_to = tuple(yield_to)
        if self.yield_to and spec.get('type') not in BODY_RADIUS:
            raise ValueError(f'{self.id}: yield_to_robots는 type이 {sorted(BODY_RADIUS)} 중 하나여야 한다')
        if self.yield_to and self.pattern == 'curve':
            raise ValueError(f'{self.id}: yield_to_robots는 straight·random 패턴만 쓸 수 있다')
        self.radius = BODY_RADIUS.get(spec.get('type'), 0.0)
        size = spec.get('yield_robot_size') or [ROBOT_LENGTH, ROBOT_WIDTH]
        if len(size) != 2 or not all(float(v) > 0 for v in size):
            raise ValueError(f'{self.id}: yield_robot_size는 [길이, 폭] (m, 양수): {size!r}')
        self.robot_half = (0.5 * float(size[0]), 0.5 * float(size[1]))
        self.patience_s = float(spec.get('yield_patience_s', YIELD_PATIENCE_S))
        progress = spec.get('yield_progress_m')
        self.progress_m = None if progress is None else float(progress)
        if self.patience_s <= 0 or (self.progress_m is not None and self.progress_m <= 0):
            raise ValueError(f'{self.id}: yield_patience_s·yield_progress_m은 양수여야 한다')
        self._anchor: Optional[List[Point]] = None   # yield_progress_m: 기다리기 시작한 때 막은 로봇들 위치
        self.waiting = False     # 멈춘 로봇 앞에서 서 있는 중
        self.turn_backs = 0      # 오래 멈춘 로봇 앞에서 돌아선 횟수
        self._stuck_since: Optional[float] = None   # 막은 로봇이 계속 멈춰 있기 시작한 시각 [s]
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

    def step(self, xy: Point, robots: Optional[Mapping[str, RobotState]] = None,
             t: Optional[float] = None) -> Tuple[float, float]:
        """현재 위치 xy에서 낼 월드 기준 속도. 목표점에 닿으면 다음 목표점으로 넘긴다.

        robots: 로봇 네임스페이스 → RobotState (world 기준). yield_to_robots의 로봇이 앞길을 막으면 (0, 0).
        t: 지금 시각 [s]. 없으면 돌아서지 않고 로봇이 비킬 때까지 기다린다.
        """
        while math.dist(xy, self._goal) < REACH_TOLERANCE:
            self._advance()
        if self.yield_to and self._must_wait(xy, robots or {}, t):
            return 0.0, 0.0
        dx, dy = self._goal[0] - xy[0], self._goal[1] - xy[1]
        d = math.hypot(dx, dy)
        return self.speed * dx / d, self.speed * dy / d

    def _must_wait(self, xy: Point, robots: Mapping[str, RobotState], t: Optional[float]) -> bool:
        """길을 막은 로봇 때문에 서야 하면 True (self.waiting도 같은 값).

        새로 설지는 YIELD_DISTANCE 앞까지, 그 로봇이 멈춰 있을 때만 정한다. 서 있을 때는 이번 구간 끝까지 보고
        로봇이 움직여도 길에서 비킬 때까지 기다린다. 0.5 m만 보면 옆으로 조금 비킨 로봇을 비켰다고 보고 걷기
        시작해, 아직 길 위에서 움직이는 로봇에 걸어 들어간다. 막은 로봇이 YIELD_PATIENCE_S 동안 계속 멈춰
        있으면 돌아서고 False.
        """
        dx, dy = self._goal[0] - xy[0], self._goal[1] - xy[1]
        d = math.hypot(dx, dy)
        reach = d if self.waiting else min(YIELD_DISTANCE, d)
        blocking = [state for name, state in robots.items()
                    if name in self.yield_to
                    and blocks_path(xy, (dx / d, dy / d), reach, state, self.radius + YIELD_SIDE_MARGIN,
                                    self.robot_half)]
        if self.progress_m is not None:
            return self._must_wait_progress(xy, blocking, robots, t)
        all_stopped = all(speed < STOPPED_SPEED for *_, speed in blocking)
        if not self.waiting:
            self.waiting = bool(blocking) and any(speed < STOPPED_SPEED for *_, speed in blocking)
            self._stuck_since = t if self.waiting and all_stopped else None
        elif not blocking:
            self.waiting = False
        elif not all_stopped:
            self._stuck_since = None
        elif self._stuck_since is None:
            self._stuck_since = t
        elif t is not None and t - self._stuck_since >= self.patience_s:
            self._turn_back(xy, robots)
            self.waiting = False
        return self.waiting

    def _must_wait_progress(self, xy: Point, blocking, robots: Mapping[str, RobotState], t: Optional[float]) -> bool:
        """yield_progress_m을 줬을 때: 막은 로봇이 progress_m도 못 움직인 채 patience_s가 지나면 돌아선다."""
        here = [(x, y) for x, y, *_ in blocking]
        if not self.waiting:
            self.waiting = bool(blocking) and any(speed < STOPPED_SPEED for *_, speed in blocking)
            self._stuck_since, self._anchor = (t, here) if self.waiting else (None, None)
            return self.waiting
        if not blocking:
            self.waiting, self._stuck_since, self._anchor = False, None, None
            return False
        moved = self._anchor is None or len(here) != len(self._anchor) or any(
            math.dist(a, b) >= self.progress_m for a, b in zip(sorted(here), sorted(self._anchor)))
        if moved or self._stuck_since is None:
            self._stuck_since, self._anchor = t, here
        elif t is not None and t - self._stuck_since >= self.patience_s:
            self._turn_back(xy, robots)
            self.waiting, self._anchor = False, None
        return self.waiting

    def _turn_back(self, xy: Optional[Point] = None, robots: Optional[Mapping[str, RobotState]] = None) -> None:
        """온 쪽으로 돌아선다 (random은 새 목표점, yield_progress_m이면 로봇이 막지 않는 쪽으로)."""
        self.turn_backs += 1
        self._stuck_since = None
        if self.pattern == 'random':
            self._goal = self._random_goal()
            if self.progress_m is not None and xy is not None and robots:
                for _ in range(20):
                    if not self._goal_blocked(xy, self._goal, robots):
                        break
                    self._goal = self._random_goal()
            return
        self._dir = -self._dir
        self._index += self._dir
        self._goal = self.path[self._index]

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

    def _goal_blocked(self, xy: Point, goal: Point, robots: Mapping[str, RobotState]) -> bool:
        dx, dy = goal[0] - xy[0], goal[1] - xy[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return False
        return any(blocks_path(xy, (dx / d, dy / d), d, state, self.radius + YIELD_SIDE_MARGIN, self.robot_half)
                   for name, state in robots.items() if name in self.yield_to)

    def _random_goal(self) -> Point:
        x0, x1, y0, y1 = self.area
        return self._rng.uniform(x0, x1), self._rng.uniform(y0, y1)

    def _new_speed(self) -> float:
        return self._rng.uniform(*self.speed_range)


def blocks_path(xy: Point, u: Point, reach: float, robot: RobotState, clearance: float,
                half: Tuple[float, float] = (0.5 * ROBOT_LENGTH, 0.5 * ROBOT_WIDTH)) -> bool:
    """xy에서 단위 방향 u로 reach [m] 가는 동안 로봇 차체(반길이·반폭 half)에 clearance [m]보다 가까워지는지.

    지금 차체에서 가장 가까운 점 쪽으로 가지 않으면 막히지 않은 것으로 본다. 점과 볼록 도형 사이 거리는
    직선을 따라 볼록 함수라, 처음에 줄지 않으면 그 방향으로는 계속 줄지 않는다 (이미 붙어 있어도 떨어지는 쪽은 간다).
    """
    x, y, yaw, _ = robot
    c, s = math.cos(yaw), math.sin(yaw)
    rx, ry = xy[0] - x, xy[1] - y
    px, py = c * rx + s * ry, -s * rx + c * ry          # 차체 기준 좌표 (x 앞, y 왼쪽)
    ux, uy = c * u[0] + s * u[1], -s * u[0] + c * u[1]
    hl, hw = half
    qx, qy = min(max(px, -hl), hl), min(max(py, -hw), hw)
    if (qx - px) * ux + (qy - py) * uy <= 0.0:
        return False
    return segment_box_distance(px, py, px + reach * ux, py + reach * uy, hl, hw) < clearance


def segment_box_distance(ax: float, ay: float, bx: float, by: float, hl: float, hw: float) -> float:
    """선분 a–b와 축 정렬 사각형 [-hl, hl]×[-hw, hw] 사이 최소 거리. 만나면 0."""
    if _segment_meets_box(ax, ay, bx, by, hl, hw):
        return 0.0
    # 만나지 않으면 최소 거리는 선분 끝점과 사각형, 또는 사각형 꼭짓점과 선분 사이에서 나온다
    best = min(math.hypot(max(abs(px) - hl, 0.0), max(abs(py) - hw, 0.0)) for px, py in ((ax, ay), (bx, by)))
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    for cx, cy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
        t = 0.0 if den <= 0.0 else min(max(((cx - ax) * dx + (cy - ay) * dy) / den, 0.0), 1.0)
        best = min(best, math.hypot(cx - (ax + t * dx), cy - (ay + t * dy)))
    return best


def _segment_meets_box(ax: float, ay: float, bx: float, by: float, hl: float, hw: float) -> bool:
    """Liang–Barsky: 선분 a + t(b − a), t ∈ [0, 1] 중 사각형 안(경계 포함)인 구간이 있는지."""
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax + hl), (dx, hl - ax), (-dy, ay + hw), (dy, hw - ay)):
        if p == 0.0:
            if q < 0.0:          # 이 축과 나란하고 바깥
                return False
        elif p < 0.0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)
    return t0 <= t1


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
