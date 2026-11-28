"""동적 장애물 이동 패턴. rclpy를 import하지 않는다 (단위 테스트용).

config/dynamic_obstacles.yaml의 장애물 1개를 받아, 현재 위치에서 낼 월드 기준 속도를 정한다.

- straight: waypoints 첫 점과 끝 점 사이 직선 왕복
- curve: waypoints를 지나는 Catmull-Rom 곡선을 따라 왕복
- random: waypoints가 이루는 사각 영역 안에서 무작위 목표점으로 직선 이동 (random waypoint 모델)
- stand: 제자리에서 조금씩 흔들리며 서 있다가 로봇이 오면 비켜 선다 (아래 stand 상수)
- routine: steps를 차례로 한다. 단계마다 path(경유점)를 따라 걸어 마지막 점에서 wait_s 동안 머문다 (키오스크 결제 → 자리에
  앉기 → 정수대 → 자리처럼). 마지막 단계 뒤에는 loop_from 단계부터 되풀이한다. 경로는 지도에서 미리 뽑아 둔다
  (scripts/gen_guest_routines.py). 걷는 동안은 straight와 같은 규칙(keep_away·yield·peers)으로 서고, 오래 막히면 직전
  경유점(첫 경유점이면 떠나온 자리)으로 물러났다 다시 간다. 같은 경유점으로 가다 다른 손님에 두 번째 막히면 그 손님이 비킬 때까지
  기다리지 않고 지나간다 (좁은 통로에서 마주친 두 손님이 번갈아 물러서기만 하지 않게. 몸 충돌체는 마찰 0 구라 미끄러져 지나간다).
  머무는 동안 로봇이 오면 stand처럼 그 단계의 evade_dirs 쪽으로 비켰다가 돌아온다.

속도는 구간(왕복 한 번, 무작위 목표점 하나)마다 speed_range_mps 안에서 무작위로 정한다.

yield_to_robots(선택, 로봇 네임스페이스 목록)를 주면 그 로봇이 멈춘 채 앞길을 막을 때 그 앞에서 선다.
사람은 멈춘 로봇에 걸어 들어가지 않는다. 움직이는 로봇에는 비키지 않는다 (로봇이 피하는지 보는 시험이므로).
한번 서면 로봇이 다시 움직여도 이번 구간(다음 경유점까지) 길에서 비킬 때까지 기다린다
(막 출발한 로봇에 걸어 들어가지 않게). 로봇이 YIELD_PATIENCE_S 동안 계속 멈춰 있으면 온 쪽으로 돌아선다.
straight·random 패턴만 쓸 수 있다 (curve는 구간이 표본 간격 0.5 m로 짧아 앞을 충분히 못 본다).

keep_away_m(생성자 인자, 0이면 끔)을 주면 패턴·로봇 이름과 상관없이, 로봇 중심이 keep_away_m 안에 있고 그쪽으로
걷는 중이면(로봇 쪽 성분이 양수) 그 자리에 선다. 움직이는 로봇에도 선다 (식당 시연: 손님이 로봇에 부딪히지 않게).
로봇이 멀어지거나 걷는 방향이 로봇에서 멀어지면 다시 걷는다. KEEP_AWAY_PATIENCE_S 넘게 서 있으면 온 쪽으로 돌아선다.
로봇 쪽으로 걷는 게 아니어도, 움직이는 로봇이 keep_away_m 안이고 손님이 그 로봇 앞쪽이나 옆이면(나란히 걷기, 로봇 앞에서
같은 쪽으로 걷기) 로봇 진행선에서 옆으로 비킨다 (EVADE_SPEED). 로봇이 멀어지면 원래 가던 길로 간다.
yield_robot_size(선택, [길이, 폭] m)는 비킬 때 보는 로봇 차체 크기다. 없으면 물류 AMR(ROBOT_LENGTH × ROBOT_WIDTH).
식당 서빙 로봇(body:=serving)은 0.565 × 0.537 m를 준다.
yield_patience_s·yield_progress_m(선택)은 언제 돌아설지 정한다. 기본(없음)은 막은 로봇의 속력이 STOPPED_SPEED 아래로
YIELD_PATIENCE_S 동안 이어질 때다. yield_progress_m을 주면 속력 대신 위치를 본다: 막은 로봇이 그만큼도 못 움직인 채
yield_patience_s가 지나면 돌아선다. 제자리에서 돌거나 조금씩 들썩이는 로봇(속력이 기준을 넘나든다) 앞에서 손님이 한없이
서 있지 않게 한다 (식당 리허설: 손님과 로봇이 서로 기다림). random 패턴은 돌아설 때 로봇이 막지 않는 목표점을 고른다.

peers(step 인자, 다른 손님 위치 목록)를 주면 손님끼리도 본다 (식당 시연: 손님이 서로 겹쳐 한 점에 뭉치지 않게).
걷는 손님은 진행 방향 PEER_AHEAD_M 앞, 진행선 옆 PEER_GAP_M 안에 다른 손님이 있으면 서고, PEER_PATIENCE_S
(0.5 ~ 1.5배 무작위: 마주 선 두 손님이 같이 돌아서지 않게) 넘게 막히면 돌아선다. stand 손님은 다른 손님(또는 그 손님이
비켜 가는 자리) PEER_GAP_M 안인 자리로는 비키지 않고, 제자리로 돌아갈 때 앞에 손님이 있으면 기다린다.
peers를 안 주면 예전과 같다.
"""

from __future__ import annotations

import math
import random
from typing import List, Mapping, Optional, Sequence, Tuple

Point = Tuple[float, float]
# 로봇 상태: world x, y [m], yaw [rad], 속력 [m/s]
RobotState = Tuple[float, float, float, float]

PATTERNS = ('straight', 'curve', 'random', 'stand', 'routine')

# stand 패턴 (식당 시연: 서 있는 손님). 제자리(spawn)에서 waypoints 두 점 사이를 조금씩 왕복하다가, 로봇이 오면
# evade_dirs 중 로봇 진행선에서 가장 먼 쪽으로 비켜 서고, 로봇이 멀어지면 제자리로 돌아온다.
# 로봇 중심이 이 거리 [m] 안이고 손님이 로봇 진행선 옆 EVADE_LANE_M 안(로봇 뒤쪽 제외)이면 비킨다
EVADE_TRIGGER_M = 1.5
# 로봇이 멈춰 있거나 진행선 밖이어도 이만큼 [m] 안이면 비킨다
EVADE_CLOSE_M = 0.7
EVADE_LANE_M = 0.9
# 비킬 때 가는 거리 [m]와 속력 [m/s]
EVADE_M = 0.8
EVADE_SPEED = 0.6
# 비킨 자리가 로봇 진행선에서 이만큼 [m] 떨어지면 길을 튼 것으로 본다 (로봇 반폭 0.27 + 손님 0.25 + 여유)
EVADE_CLEAR_M = 0.6
# 모든 로봇이 이만큼 [m] 밖이면 제자리로 돌아간다
EVADE_RELEASE_M = 2.0
# 옆으로 틀 데가 없는 좁은 통로: 로봇(0.8 m/s)보다 빨리 이만큼 [m] 앞으로 가서 가로 통로(행 사이 1.8 m)로 빠진다
EVADE_FLEE_M = 2.0
EVADE_FLEE_SPEED = 1.0
# peers: 손님 둘 중심 사이를 이만큼 [m] 띄운다 (몸 반지름 0.25 × 2 + 여유 0.1)
PEER_GAP_M = 0.6
# 진행 방향 이만큼 [m] 앞까지 다른 손님이 있는지 본다
PEER_AHEAD_M = 0.7
# 다른 손님에 막혀 이만큼 [s] (0.5 ~ 1.5배 무작위) 서 있으면 돌아선다 (stand가 제자리로 갈 때는 그냥 간다)
PEER_PATIENCE_S = 2.0
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
# keep_away_m: 로봇 때문에 이만큼 [s] 넘게 서 있으면 온 쪽으로 돌아선다 (로봇이 손님 앞에 멈춰 둘 다 기다리지 않게)
KEEP_AWAY_PATIENCE_S = 5.0
# keep_away_m: 움직이는 로봇의 앞쪽·옆(로봇 진행 방향 성분이 거리의 -이 비율보다 큼)에 있으면 진행선에서 옆으로 비킨다
# (손님이 로봇과 나란히 걷거나 로봇 앞에서 같은 쪽으로 걸으면 로봇 안전 노드가 계속 감속한다: 11/17 1-2 켬, 약 8 s)
MAKE_WAY_BEHIND = 0.2
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

    def __init__(self, spec: dict, rng: random.Random, keep_away_m: float = 0.0):
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
        if not keep_away_m >= 0:
            raise ValueError(f'{spec["id"]}: keep_away_m은 0 이상이어야 한다: {keep_away_m}')
        self.keep_away_m = float(keep_away_m)
        self.keeping_away = False                     # 가까운 로봇 때문에 서 있는 중
        self._keep_since: Optional[float] = None      # 그렇게 서기 시작한 시각 [s]
        self._stuck_since: Optional[float] = None   # 막은 로봇이 계속 멈춰 있기 시작한 시각 [s]
        self._steps: List[dict] = []
        if self.pattern == 'routine':
            self._steps = _parse_steps(spec)
            self._loop_from = int(spec.get('loop_from', 0))
            if not 0 <= self._loop_from < len(self._steps):
                raise ValueError(f'{self.id}: loop_from은 0 ~ {len(self._steps) - 1}: {self._loop_from}')
            waypoints = [p for st in self._steps for p in st['path']]
            if len(waypoints) < 2:
                waypoints = waypoints * 2
        else:
            waypoints = [(float(w['x']), float(w['y'])) for w in spec['waypoints']]
        if len(waypoints) < 2:
            raise ValueError(f'{self.id}: waypoints가 2개 이상 필요하다')
        self._rng = rng

        self._home: Point = (float(spec['spawn']['x']), float(spec['spawn']['y'])) if 'spawn' in spec \
            else waypoints[0]
        self.evade_dirs: List[Point] = []
        for d in spec.get('evade_dirs') or []:
            n = math.hypot(float(d[0]), float(d[1]))
            if n <= 0:
                raise ValueError(f'{self.id}: evade_dirs에 0 벡터가 있다')
            self.evade_dirs.append((float(d[0]) / n, float(d[1]) / n))
        self.evade_m = float(spec.get('evade_m', EVADE_M))
        self.evading = False                      # stand: 로봇을 피해 비켜 서는 중 (또는 비킨 자리에 서 있는 중)
        self.returning = False                    # stand: 제자리로 돌아가는 중
        self._evade_goal: Optional[Point] = None
        self._evade_speed = EVADE_SPEED
        self.evades = 0                           # 비킨 횟수
        self.peer_waiting = False                 # 다른 손님에 막혀 서 있는 중
        self.peer_turn_backs = 0                  # 다른 손님에 막혀 돌아선 횟수
        self._peer_since: Optional[float] = None
        self._peer_patience = PEER_PATIENCE_S
        self._peer_pass = False                   # stand: 오래 막혀 그냥 지나가는 중

        # routine: 지금 단계·경유점, 머무는 중이면 끝나는 시각
        self.step_index = 0
        self._pi = 0
        self.dwelling = False
        self._dwell_until: Optional[float] = None
        self._leg_start: Optional[Point] = None    # 이번 단계를 떠나온 자리
        self._detour: Optional[Point] = None       # 물러서는 점 (닿으면 원래 경유점으로)
        self._peer_blocks = 0                      # 이번 경유점으로 가다 다른 손님에 막혀 물러선 횟수
        if self.pattern == 'random':
            xs, ys = zip(*waypoints)
            self.area = (min(xs), max(xs), min(ys), max(ys))
            self.path: List[Point] = []
            self._goal = self._random_goal()
        elif self.pattern == 'routine':
            self.path = []
            self._goal = self._steps[0]['path'][0]
        else:
            self.path = waypoints[:1] + waypoints[-1:] if self.pattern in ('straight', 'stand') \
                else catmull_rom(waypoints)
            self._index = 1      # 지금 향하는 path 점
            self._dir = 1        # 1: 정방향, -1: 되돌아오는 중
            self._goal = self.path[1]
        self.speed = self._new_speed()

    @property
    def goal(self) -> Point:
        return self._goal

    @property
    def claimed(self) -> Optional[Point]:
        """stand: 비켜 가는 자리 (다른 손님이 같은 자리로 비키지 않게 알린다). 비키는 중이 아니면 None."""
        return self._evade_goal if self.evading else None

    def step(self, xy: Point, robots: Optional[Mapping[str, RobotState]] = None,
             t: Optional[float] = None, peers: Optional[Sequence[Point]] = None) -> Tuple[float, float]:
        """현재 위치 xy에서 낼 월드 기준 속도. 목표점에 닿으면 다음 목표점으로 넘긴다.

        robots: 로봇 네임스페이스 → RobotState (world 기준). yield_to_robots의 로봇이 앞길을 막으면 (0, 0).
        t: 지금 시각 [s]. 없으면 돌아서지 않고 로봇이 비킬 때까지 기다린다.
        peers: 다른 손님 위치 (자기는 빼고). 앞길에 있으면 선다. 없으면 손님끼리 보지 않는다.
        """
        peers = peers or ()
        if self.pattern == 'stand':
            return self._stand_step(xy, robots or {}, peers, t)
        if self.pattern == 'routine':
            return self._routine_step(xy, robots or {}, peers, t)
        while math.dist(xy, self._goal) < REACH_TOLERANCE:
            self._advance()
        if self.keep_away_m > 0 and self._keep_away(xy, robots or {}, t):
            return 0.0, 0.0
        way = self._make_way(xy, robots or {}) if self.keep_away_m > 0 else None
        if way is not None:
            return way
        if self.yield_to and self._must_wait(xy, robots or {}, t):
            return 0.0, 0.0
        if self._peer_wait(xy, self._goal, peers, t):
            return 0.0, 0.0
        dx, dy = self._goal[0] - xy[0], self._goal[1] - xy[1]
        d = math.hypot(dx, dy)
        return self.speed * dx / d, self.speed * dy / d

    def _keep_away(self, xy: Point, robots: Mapping[str, RobotState], t: Optional[float]) -> bool:
        """keep_away_m 안의 로봇 쪽으로 걷는 중이면 True (self.keeping_away도 같은 값). 오래 서 있으면 돌아서고 False."""
        ux, uy = self._goal[0] - xy[0], self._goal[1] - xy[1]
        blocked = any(math.hypot(rx - xy[0], ry - xy[1]) < self.keep_away_m
                      and (rx - xy[0]) * ux + (ry - xy[1]) * uy > 0.0
                      for rx, ry, *_ in robots.values())
        if not blocked:
            self.keeping_away = False
            self._keep_since = None
            return False
        if not self.keeping_away:
            self.keeping_away = True
            self._keep_since = t
        elif t is not None and self._keep_since is not None and t - self._keep_since >= KEEP_AWAY_PATIENCE_S:
            # 온 쪽으로 돌아서면 로봇에서 멀어지는 방향이라 다음 걸음부터 걷는다
            self._turn_back()
            self.keeping_away = False
            self._keep_since = None
            return False
        return True

    def _make_way(self, xy: Point, robots: Mapping[str, RobotState]) -> Optional[Tuple[float, float]]:
        """keep_away_m 안에서 움직이는 로봇의 앞쪽·옆이면 그 로봇 진행선에서 옆으로 비키는 속도 (아니면 None).
        여럿이면 가장 가까운 로봇을 피한다. 로봇 뒤쪽(로봇이 멀어지는 중)은 비키지 않는다."""
        near = None
        for rx, ry, yaw, speed in robots.values():
            ox, oy = xy[0] - rx, xy[1] - ry
            d = math.hypot(ox, oy)
            if d >= self.keep_away_m or speed < STOPPED_SPEED:
                continue
            hx, hy = math.cos(yaw), math.sin(yaw)
            if ox * hx + oy * hy <= -MAKE_WAY_BEHIND * d:
                continue
            if near is None or d < near[0]:
                near = (d, ox, oy, hx, hy)
        if near is None:
            return None
        _, ox, oy, hx, hy = near
        nx, ny = -hy, hx                     # 진행 방향의 왼쪽
        if ox * nx + oy * ny < 0:            # 손님이 오른쪽이면 오른쪽으로
            nx, ny = -nx, -ny
        return EVADE_SPEED * nx, EVADE_SPEED * ny

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

    def _peer_wait(self, xy: Point, goal: Point, peers: Sequence[Point], t: Optional[float]) -> bool:
        """goal 쪽 앞길에 다른 손님이 있으면 True (self.peer_waiting도 같은 값).
        _peer_patience 넘게 막히면 돌아서고 False (stand는 돌아서지 않고 막힌 동안 그냥 지나간다)."""
        if not peer_ahead(xy, goal, peers):
            self.peer_waiting, self._peer_since, self._peer_pass = False, None, False
            return False
        if self._peer_pass:
            return False
        if not self.peer_waiting:
            self.peer_waiting, self._peer_since = True, t
            self._peer_patience = PEER_PATIENCE_S * self._rng.uniform(0.5, 1.5)
            return True
        if t is None or self._peer_since is None or t - self._peer_since < self._peer_patience:
            return True
        self.peer_waiting, self._peer_since = False, None
        if self.pattern == 'stand':
            self._peer_pass = True
        elif self.pattern == 'routine' and (self._peer_blocks >= 1 or self.dwelling):
            # 두 번째로 막혔다 (또는 머물던 자리로 돌아가는 중): 물러서지 않고 지나간다
            self._peer_pass = True
        elif self.pattern == 'routine':
            self._peer_blocks += 1
            self.peer_turn_backs += 1
            self._turn_back()
        else:
            self.peer_turn_backs += 1
            self._turn_back()
        return False

    # ---------------------------------------------------------------- routine: 정해진 일과
    @property
    def step_name(self) -> str:
        return self._steps[self.step_index]['name'] if self._steps else ''

    def _routine_step(self, xy: Point, robots: Mapping[str, RobotState], peers: Sequence[Point],
                      t: Optional[float]) -> Tuple[float, float]:
        if self.dwelling:
            v = self._dwell(xy, robots, peers, t)
            if v is not None:
                return v
        st = self._steps[self.step_index]
        path = st['path']
        if self._leg_start is None:
            self._leg_start = xy
        if self._detour is not None and math.dist(xy, self._detour) < REACH_TOLERANCE:
            self._detour = None
        while self._detour is None and math.dist(xy, path[self._pi]) < REACH_TOLERANCE:
            self._peer_blocks = 0
            if self._pi + 1 < len(path):
                self._pi += 1
                continue
            # 이번 단계 자리에 왔다: 머문다
            self.dwelling = True
            self._home = path[-1]
            self.evade_dirs = st['evade_dirs']
            self._dwell_until = None if t is None else t + self._rng.uniform(*st['wait_s'])
            self.speed = self._new_speed()
            return 0.0, 0.0
        self._goal = self._detour if self._detour is not None else path[self._pi]
        if self.keep_away_m > 0 and self._keep_away(xy, robots, t):
            return 0.0, 0.0
        way = self._make_way(xy, robots) if self.keep_away_m > 0 else None
        if way is not None:
            return way
        if self.yield_to and self._must_wait(xy, robots, t):
            return 0.0, 0.0
        if self._peer_wait(xy, self._goal, peers, t):
            return 0.0, 0.0
        return _toward(xy, self._goal, self.speed)

    def _dwell(self, xy: Point, robots: Mapping[str, RobotState], peers: Sequence[Point],
               t: Optional[float]) -> Optional[Tuple[float, float]]:
        """머무는 중의 속도. 다 머물렀으면 다음 단계로 넘기고 None (그 단계 첫 경유점으로 걷기 시작)."""
        threat = self._threat(xy, robots)
        if threat is not None:
            if self._evade_goal is None or (math.dist(xy, threat[:2]) < EVADE_CLOSE_M + 0.2
                                            and _lateral(threat, xy) < EVADE_CLEAR_M):
                self._evade_goal = self._pick_evade(xy, threat, peers)
                self.evading, self.returning = True, False
                self.evades += 1
        elif self.evading and all(math.dist(xy, r[:2]) > EVADE_RELEASE_M for r in robots.values()):
            self.evading, self.returning, self._evade_goal = False, True, None
        if self.evading:
            return _toward(xy, self._evade_goal, self._evade_speed)
        if self.returning:
            if math.dist(xy, self._home) > REACH_TOLERANCE:
                if self._peer_wait(xy, self._home, peers, t):
                    return 0.0, 0.0
                return _toward(xy, self._home, EVADE_SPEED)
            self.returning = False
        if t is not None and self._dwell_until is not None and t < self._dwell_until:
            return 0.0, 0.0
        # 다음 단계
        self.dwelling, self._dwell_until = False, None
        self.peer_waiting, self._peer_since, self._peer_pass = False, None, False
        nxt = self.step_index + 1
        self.step_index = nxt if nxt < len(self._steps) else self._loop_from
        self._pi, self._leg_start, self._detour, self._peer_blocks = 0, None, None, 0
        return None

    # ---------------------------------------------------------------- stand: 서 있다가 로봇이 오면 비킨다
    def _stand_step(self, xy: Point, robots: Mapping[str, RobotState], peers: Sequence[Point],
                    t: Optional[float]) -> Tuple[float, float]:
        threat = self._threat(xy, robots)
        if threat is not None:
            # 아직 안 비켰거나, 비킨 자리가 로봇 진행선 안이라 로봇이 다시 가까이 오면 지금 자리에서 한 번 더 비킨다
            if self._evade_goal is None or (math.dist(xy, threat[:2]) < EVADE_CLOSE_M + 0.2
                                            and _lateral(threat, xy) < EVADE_CLEAR_M):
                self._evade_goal = self._pick_evade(xy, threat, peers)
                self.evading, self.returning = True, False
                self.evades += 1
        elif self.evading and all(math.dist(xy, r[:2]) > EVADE_RELEASE_M for r in robots.values()):
            self.evading, self.returning, self._evade_goal = False, True, None
        if self.evading:
            return _toward(xy, self._evade_goal, self._evade_speed)
        if self.returning:
            if math.dist(xy, self._home) > 0.03:
                # 로봇을 피하는 중이 아니니 앞에 손님이 있으면 기다린다
                if self._peer_wait(xy, self._home, peers, t):
                    return 0.0, 0.0
                return _toward(xy, self._home, EVADE_SPEED)
            self.returning = False
            self.peer_waiting, self._peer_since, self._peer_pass = False, None, False
        # 제자리 왕복 (straight와 같은 두 점)
        while math.dist(xy, self._goal) < REACH_TOLERANCE:
            self._advance()
        return _toward(xy, self._goal, self.speed)

    def _threat(self, xy: Point, robots: Mapping[str, RobotState]) -> Optional[RobotState]:
        """비켜야 할 가장 가까운 로봇 (없으면 None)."""
        best = None
        for state in robots.values():
            d = math.dist(xy, state[:2])
            c, s = math.cos(state[2]), math.sin(state[2])
            ahead = c * (xy[0] - state[0]) + s * (xy[1] - state[1])
            on_lane = d < EVADE_TRIGGER_M and ahead > -0.3 and _lateral(state, xy) < EVADE_LANE_M
            if (on_lane or d < EVADE_CLOSE_M) and (best is None or d < math.dist(xy, best[:2])):
                best = state
        return best

    def _pick_evade(self, xy: Point, robot: RobotState, peers: Sequence[Point] = ()) -> Point:
        """evade_dirs(없으면 로봇 진행 방향의 양옆) 중 비킨 자리가 로봇 진행선에서 가장 먼 쪽.
        어느 쪽도 EVADE_CLEAR_M을 못 넘으면(좁은 통로) 로봇에서 멀어지는 쪽으로 간다.
        다른 손님 PEER_GAP_M 안인 자리는 빈 자리가 하나라도 있으면 고르지 않는다."""
        c, s = math.cos(robot[2]), math.sin(robot[2])
        perps = [(-s, c), (s, -c)]
        # 다시 비킬 때(좁은 통로에서 앞으로 밀려난 뒤)는 제자리 기준 evade_dirs에 더해 로봇 양옆도 본다 (통로 교차점)
        dirs = (self.evade_dirs or perps) + (perps if self.evading else [])
        base = self._home if not self.evading else xy
        self._evade_speed = EVADE_SPEED
        cands = [(base[0] + self.evade_m * dx, base[1] + self.evade_m * dy) for dx, dy in dirs]
        lateral = max(_free_of(cands, peers), key=lambda p: (_lateral(robot, p), math.dist(p, robot[:2])))
        if _lateral(robot, lateral) >= EVADE_CLEAR_M:
            return lateral
        # 옆으로 틀 데가 없다 (좁은 통로): 로봇보다 빨리 EVADE_FLEE_M 앞으로 가서 통로를 빠져나간다
        self._evade_speed = EVADE_FLEE_SPEED
        far = _free_of([(base[0] + EVADE_FLEE_M * dx, base[1] + EVADE_FLEE_M * dy) for dx, dy in dirs], peers)
        return max(far, key=lambda p: math.dist(p, robot[:2]))

    def _turn_back(self, xy: Optional[Point] = None, robots: Optional[Mapping[str, RobotState]] = None) -> None:
        """온 쪽으로 돌아선다 (random은 새 목표점, yield_progress_m이면 로봇이 막지 않는 쪽으로)."""
        self.turn_backs += 1
        self._stuck_since = None
        if self.pattern == 'routine':
            # 직전 경유점(첫 경유점이면 떠나온 자리)으로 물러난다. 거기 닿으면 다시 원래 경유점으로 간다
            back = self._steps[self.step_index]['path'][self._pi - 1] if self._pi > 0 else self._leg_start
            if back is not None:
                self._detour = back
                self._goal = back
            return
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


def _parse_steps(spec: dict) -> List[dict]:
    steps = spec.get('steps') or []
    if not steps:
        raise ValueError(f'{spec["id"]}: routine은 steps가 1개 이상 필요하다')
    out = []
    for i, st in enumerate(steps):
        path = [(float(p[0]), float(p[1])) for p in st.get('path') or []]
        lo, hi = st.get('wait_s', [0.0, 0.0])
        if not path or not 0 <= float(lo) <= float(hi):
            raise ValueError(f'{spec["id"]}: steps[{i}]에 path가 없거나 wait_s가 잘못됐다: {st!r}')
        dirs = []
        for d in st.get('evade_dirs') or []:
            n = math.hypot(float(d[0]), float(d[1]))
            if n <= 0:
                raise ValueError(f'{spec["id"]}: steps[{i}] evade_dirs에 0 벡터가 있다')
            dirs.append((float(d[0]) / n, float(d[1]) / n))
        out.append({'name': str(st.get('name', i)), 'path': path, 'wait_s': (float(lo), float(hi)),
                    'evade_dirs': dirs})
    return out


def peer_ahead(xy: Point, goal: Point, peers: Sequence[Point]) -> bool:
    """xy에서 goal 쪽으로 걸을 때 PEER_AHEAD_M 앞, 진행선 옆 PEER_GAP_M 안에 다른 손님이 있는지.
    그 손님이 goal보다 PEER_GAP_M 넘게 뒤면(그 앞에서 멈출 거면) 막히지 않은 것으로 본다."""
    dx, dy = goal[0] - xy[0], goal[1] - xy[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return False
    ux, uy = dx / d, dy / d
    for px, py in peers:
        rx, ry = px - xy[0], py - xy[1]
        ahead = rx * ux + ry * uy
        if 0.0 < ahead < min(PEER_AHEAD_M, d + PEER_GAP_M) and abs(rx * uy - ry * ux) < PEER_GAP_M:
            return True
    return False


def _free_of(cands: List[Point], peers: Sequence[Point]) -> List[Point]:
    """다른 손님 PEER_GAP_M 안인 후보를 뺀다. 다 빠지면 그대로 돌려준다."""
    free = [p for p in cands if all(math.dist(p, q) >= PEER_GAP_M for q in peers)]
    return free or cands


def _lateral(robot: RobotState, p: Point) -> float:
    """점 p가 로봇 진행선(로봇 중심을 지나 yaw 방향 직선)에서 떨어진 거리 [m]."""
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return abs(-s * (p[0] - robot[0]) + c * (p[1] - robot[1]))


def _toward(xy: Point, goal: Point, speed: float) -> Tuple[float, float]:
    dx, dy = goal[0] - xy[0], goal[1] - xy[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0, 0.0
    v = min(speed, d / 0.1)        # 마지막 한 걸음에서 넘치지 않게 (노드 주기 0.1 s 기준)
    return v * dx / d, v * dy / d


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
