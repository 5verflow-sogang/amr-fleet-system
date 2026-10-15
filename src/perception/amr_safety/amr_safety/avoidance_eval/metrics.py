"""동적 장애물 회피 KPI (test-plan I17) 계산. ROS에 의존하지 않는다.

원 명세 7장: 동적 장애물 회피 30회 충돌 0건, 경로 이탈 1 m 이내, 원래 경로 복귀 5초 이내.

입력은 모두 같은 고정 좌표계(Gazebo world)의 시계열이다.
- 로봇 정답 자세 (`/<ns>/ground_truth/odom`)
- 장애물 정답 자세·속력 (`/dynamic_obstacles/<id>/ground_truth/odom`)과 모양 (모델 충돌체)
- 기준 경로: 회피 전에 받은 전역 경로(첫 `plan`) 또는 고정 경로 (꺾은선)

지표 정의 (값은 AvoidanceParams, 근거는 README "회피 평가"):
- 이격 거리 clearance: 로봇 차체 사각형(0.6×0.4 m, 중심 = base_footprint)과 장애물 모양 사이 최소 거리.
  겹치면 0 이하. collision_margin 이하가 이어진 구간 하나를 충돌 1건으로 센다.
- 경로 이탈 deviation: 로봇 위치에서 기준 경로 꺾은선까지 최소 거리. 실행 최대값을 KPI로 본다.
- 회피 구간 episode: start_on 조건(이탈 > start_deviation, 또는 장애물 이격 < interaction_distance)이
  처음 참이 된 시각에 시작한다. 그 구간에서 이탈이 return_deviation을 마지막으로 넘은 뒤,
  이탈이 return_deviation 이하로 return_hold_s 동안 유지되기 시작한 시각을 복귀 시각으로 본다.
  복귀 시간 = 복귀 시각 − 시작 시각. 구간 앞뒤에 장애물이 interaction_distance 안에 있었으면
  회피 구간(obstacle_related), 아니면 추종 오차로 보고 복귀 KPI에서 뺀다.
"""

from __future__ import annotations

import bisect
from collections import Counter
import math
from dataclasses import asdict, dataclass, field
from typing import Iterable, Optional, Sequence

CIRCLE = 'circle'
BOX = 'box'

START_ON = ('deviation', 'proximity', 'either')


@dataclass(frozen=True)
class Pose:
    """한 시점의 2D 자세 (world 기준) [s, m, m, rad]. speed는 장애물 속력 [m/s] (모르면 None)."""

    t: float
    x: float
    y: float
    yaw: float = 0.0
    speed: Optional[float] = None


@dataclass(frozen=True)
class Shape:
    """장애물 모양. circle은 radius, box는 length(진행 방향)·width [m]."""

    kind: str
    radius: float = 0.0
    length: float = 0.0
    width: float = 0.0

    def __post_init__(self):
        if self.kind == CIRCLE and self.radius <= 0.0:
            raise ValueError(f'원 반지름은 양수여야 한다: {self.radius}')
        if self.kind == BOX and (self.length <= 0.0 or self.width <= 0.0):
            raise ValueError(f'사각형 크기는 양수여야 한다: {self.length}×{self.width}')
        if self.kind not in (CIRCLE, BOX):
            raise ValueError(f"kind는 '{CIRCLE}' 또는 '{BOX}': {self.kind}")


# amr_gazebo 동적 장애물 모델 충돌체 (models/dynamic_*/model.sdf). id의 앞부분(worker_1 → worker)으로 찾는다
SHAPES_BY_TYPE = {
    'worker': Shape(CIRCLE, radius=0.25),
    'forklift': Shape(BOX, length=1.2, width=0.9),
    'robot': Shape(BOX, length=0.6, width=0.4),
}


def shape_for_id(obstacle_id: str, default_radius: Optional[float] = None) -> Shape:
    """장애물 id(예: worker_1, forklift_2)의 모양. 모르는 종류는 default_radius 원, 없으면 KeyError."""
    kind = obstacle_id.split('_')[0]
    if kind in SHAPES_BY_TYPE:
        return SHAPES_BY_TYPE[kind]
    if default_radius is None:
        raise KeyError(f'모양을 모르는 장애물: {obstacle_id} (SHAPES_BY_TYPE: {sorted(SHAPES_BY_TYPE)})')
    return Shape(CIRCLE, radius=default_radius)


@dataclass(frozen=True)
class AvoidanceParams:
    """판정 기준값. 바꾸면 avoidance_summary.json의 params에 그대로 남는다."""

    # 로봇 차체 (nav2_params.yaml footprint, amr_description base 충돌체) [m]
    robot_length: float = 0.6
    robot_width: float = 0.4
    # 이격 거리가 이 값 이하면 충돌. Gazebo 충돌체는 서로 파고들지 않고 닿은 채 멈추므로 0보다 조금 크게 둔다
    # (GT 보간·샘플 간격 오차 포함) [m]
    collision_margin: float = 0.01
    # near-miss (명세서 연구 2 안전성 지표 "near-miss 0.3 m 미만") [m]
    near_miss_distance: float = 0.3
    # 회피 구간 시작: 이탈이 이 값을 넘으면 [m]
    start_deviation: float = 0.1
    # 복귀: 이탈이 이 값 이하로 return_hold_s 동안 유지 [m, s]
    return_deviation: float = 0.2
    return_hold_s: float = 1.0
    # 장애물과의 상호작용 거리 (차체-장애물 이격) [m]. 안전 노드 WARNING 근접 기준(1.5 m)과 같다
    interaction_distance: float = 1.5
    # 회피 구간 시작 조건: deviation(기본) / proximity / either
    start_on: str = 'deviation'
    # 이탈 시작 전 이만큼 앞까지 장애물이 가까웠으면 회피 구간으로 본다 [s]
    obstacle_lookback_s: float = 2.0
    # 장애물 GT가 이보다 오래 끊긴 시각은 그 장애물을 모르는 것으로 본다 [s]
    max_obstacle_gap_s: float = 0.2
    # 기준 경로 끝점에 이만큼 가까워지면 목표 도착 [m] (xy_goal_tolerance 0.25 + 여유)
    goal_tolerance: float = 0.3
    # 도착하지 못한 실행을 불합격으로 볼지 (멈춰 있기만 해도 충돌·이탈이 0이 되는 것을 막는다)
    require_goal: bool = True
    # 장애물을 못 만난 무효 실행 중 로봇 이동 거리가 이보다 짧으면 "거의 안 움직임"(Nav2 실패·정지 등)으로 본다 [m]
    stuck_travel_m: float = 0.5
    # 원 명세 KPI
    kpi_collisions: int = 0
    kpi_max_deviation: float = 1.0
    kpi_return_s: float = 5.0
    kpi_runs: int = 30

    def __post_init__(self):
        if self.start_on not in START_ON:
            raise ValueError(f'start_on은 {START_ON} 중 하나: {self.start_on}')
        if self.return_hold_s < 0.0:
            raise ValueError('return_hold_s는 0 이상')


# --- 기하 -------------------------------------------------------------------------------------------


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 0.0:
        return math.hypot(px - ax, py - ay)
    s = min(max(((px - ax) * dx + (py - ay) * dy) / den, 0.0), 1.0)
    return math.hypot(px - (ax + s * dx), py - (ay + s * dy))


def polyline_distance(x: float, y: float, path: Sequence[tuple[float, float]]) -> float:
    """점에서 꺾은선까지 최소 거리 (경로 이탈). 점이 하나면 그 점까지 거리."""
    if not path:
        raise ValueError('기준 경로가 비었다')
    if len(path) == 1:
        return math.hypot(x - path[0][0], y - path[0][1])
    return min(point_segment_distance(x, y, *path[i], *path[i + 1]) for i in range(len(path) - 1))


def polyline_length(path: Sequence[tuple[float, float]]) -> float:
    return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))


def box_corners(x: float, y: float, yaw: float, length: float, width: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = 0.5 * length, 0.5 * width
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _boxes_overlap(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> bool:
    """분리축 정리 (볼록 사각형 두 개)."""
    for poly in (a, b):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            nx, ny = -ey, ex
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def footprint_clearance(robot: Pose, length: float, width: float, obstacle: Pose, shape: Shape) -> float:
    """로봇 차체 사각형과 장애물 모양 사이 최소 거리 [m]. 겹치면 0 이하 (원은 파고든 정도만큼 음수)."""
    if shape.kind == CIRCLE:
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        rx, ry = obstacle.x - robot.x, obstacle.y - robot.y
        lx, ly = c * rx + s * ry, -s * rx + c * ry
        dx = max(abs(lx) - 0.5 * length, 0.0)
        dy = max(abs(ly) - 0.5 * width, 0.0)
        if dx == 0.0 and dy == 0.0:
            # 원 중심이 차체 안: 가장 가까운 변까지 거리만큼 더 파고든 것으로 본다
            return -shape.radius - min(0.5 * length - abs(lx), 0.5 * width - abs(ly))
        return math.hypot(dx, dy) - shape.radius
    a = box_corners(robot.x, robot.y, robot.yaw, length, width)
    b = box_corners(obstacle.x, obstacle.y, obstacle.yaw, shape.length, shape.width)
    if _boxes_overlap(a, b):
        return 0.0
    # 떨어진 볼록 다각형 사이 최소 거리는 한쪽 꼭짓점과 다른 쪽 변 사이에서 나온다
    best = math.inf
    for p, q in ((a, b), (b, a)):
        for px, py in p:
            for i in range(4):
                best = min(best, point_segment_distance(px, py, *q[i], *q[(i + 1) % 4]))
    return best


# --- 시계열 -----------------------------------------------------------------------------------------


def transform_path(path: Iterable[tuple[float, float]], origin: Sequence[float]) -> list[tuple[float, float]]:
    """map 좌표 경로를 world로. origin = world에서 본 map 원점 [x, y, yaw] (eval_node map_origin_in_world와 같은 뜻)."""
    ox, oy, oyaw = (float(v) for v in origin)
    c, s = math.cos(oyaw), math.sin(oyaw)
    return [(ox + c * x - s * y, oy + s * x + c * y) for x, y in path]


def interpolate_pose(samples: Sequence[Pose], times: Sequence[float], t: float, max_gap: float) -> Optional[Pose]:
    """시각 t의 자세를 앞뒤 샘플로 선형 보간한다. 앞뒤가 max_gap보다 멀면(끊김·범위 밖) None."""
    i = bisect.bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return samples[i]
    if i == 0 or i == len(times):
        j = 0 if i == 0 else len(times) - 1
        return samples[j] if abs(times[j] - t) <= max_gap else None
    a, b = samples[i - 1], samples[i]
    if b.t - a.t > max_gap:
        return None
    r = (t - a.t) / (b.t - a.t)
    speed = None
    if a.speed is not None and b.speed is not None:
        speed = a.speed + r * (b.speed - a.speed)
    return Pose(t, a.x + r * (b.x - a.x), a.y + r * (b.y - a.y),
                normalize_angle(a.yaw + r * normalize_angle(b.yaw - a.yaw)), speed)


@dataclass
class SampleResult:
    """로봇 GT 한 시점의 계산 결과 (avoidance.csv 한 줄)."""

    t: float
    x: float
    y: float
    yaw: float
    deviation: Optional[float]
    clearance: Optional[float]
    nearest_obstacle: str
    obstacle_speed: Optional[float]
    episode: Optional[int] = None
    collision: bool = False


@dataclass
class Episode:
    index: int
    start_t: float
    trigger: str
    max_deviation: float
    max_deviation_t: float
    return_t: Optional[float]
    return_s: Optional[float]
    min_clearance: Optional[float]
    nearest_obstacle: str
    obstacle_speed: Optional[float]
    obstacle_related: bool


@dataclass
class RunResult:
    samples: list[SampleResult]
    episodes: list[Episode]
    summary: dict = field(default_factory=dict)


def _robot_speed(robot: Sequence[Pose], i: int) -> float:
    if len(robot) < 2:
        return 0.0
    a, b = (robot[i - 1], robot[i]) if i > 0 else (robot[0], robot[1])
    dt = b.t - a.t
    return math.hypot(b.x - a.x, b.y - a.y) / dt if dt > 0 else 0.0


def evaluate_run(
    robot: Sequence[Pose],
    obstacles: dict[str, Sequence[Pose]],
    shapes: dict[str, Shape],
    reference: Sequence[tuple[float, float]],
    params: AvoidanceParams = AvoidanceParams(),
    reference_t: Optional[float] = None,
) -> RunResult:
    """실행 하나를 평가한다.

    robot·obstacles는 시각 순서가 아니어도 된다(정렬한다). reference는 world 좌표 꺾은선.
    reference_t가 있으면 그 시각 이후의 로봇 샘플만 본다 (기준 경로를 받기 전에는 이탈을 정의할 수 없다).
    """
    p = params
    robot = sorted(robot, key=lambda s: s.t)
    if reference_t is not None:
        robot = [s for s in robot if s.t >= reference_t]
    obs = {oid: sorted(v, key=lambda s: s.t) for oid, v in obstacles.items() if v}
    obs_times = {oid: [s.t for s in v] for oid, v in obs.items()}
    have_ref = len(reference) >= 1

    samples: list[SampleResult] = []
    per_obstacle_min: dict[str, float] = {}
    for r in robot:
        dev = polyline_distance(r.x, r.y, reference) if have_ref else None
        best, best_id, best_speed = None, '', None
        for oid, series in obs.items():
            o = interpolate_pose(series, obs_times[oid], r.t, p.max_obstacle_gap_s)
            if o is None:
                continue
            c = footprint_clearance(r, p.robot_length, p.robot_width, o, shapes[oid])
            per_obstacle_min[oid] = min(per_obstacle_min.get(oid, math.inf), c)
            if best is None or c < best:
                best, best_id, best_speed = c, oid, o.speed
        samples.append(SampleResult(r.t, r.x, r.y, r.yaw, dev, best, best_id, best_speed,
                                    collision=best is not None and best <= p.collision_margin))

    episodes = _detect_episodes(samples, p) if have_ref else []
    collisions = _collision_events(samples, robot)
    return RunResult(samples, episodes,
                     _summarize(samples, episodes, collisions, per_obstacle_min, reference, p, reference_t))


def _detect_episodes(samples: list[SampleResult], p: AvoidanceParams) -> list[Episode]:
    n = len(samples)
    times = [s.t for s in samples]
    dev = [s.deviation if s.deviation is not None else 0.0 for s in samples]
    near = [s.clearance is not None and s.clearance < p.interaction_distance for s in samples]

    def active(i: int) -> Optional[str]:
        by_dev = dev[i] > p.start_deviation
        if p.start_on in ('deviation', 'either') and by_dev:
            return 'deviation'
        if p.start_on in ('proximity', 'either') and near[i]:
            return 'proximity'
        return None

    # 각 시점 이후 이탈이 return_deviation을 처음 넘는 인덱스 (복귀 유지 판정용)
    next_over = [n] * (n + 1)
    for i in range(n - 1, -1, -1):
        next_over[i] = i if dev[i] > p.return_deviation else next_over[i + 1]

    def returned_at(k: int) -> Optional[int]:
        """k 이후 이탈이 return_deviation 이하로 return_hold_s 동안 유지되기 시작하는 첫 인덱스."""
        r = k
        while r < n:
            if dev[r] > p.return_deviation:
                r += 1
                continue
            j = next_over[r]
            # 끝까지 다시 넘지 않거나, 다시 넘는 시각이 유지 시간 뒤면 복귀
            if j >= n or times[j] - times[r] > p.return_hold_s:
                return r
            r = j + 1
        return None

    episodes: list[Episode] = []
    i = 0
    while i < n:
        trigger = active(i)
        if trigger is None:
            i += 1
            continue
        s = i
        # 활성 구간 끝 (return_hold_s보다 짧은 끊김은 이어 본다)
        e = s
        j = s + 1
        while j < n:
            if active(j) is not None:
                e = j
            elif times[j] - times[e] > p.return_hold_s:
                break
            j += 1
        over = [k for k in range(s, e + 1) if dev[k] > p.return_deviation]
        if over:
            r = returned_at(over[-1] + 1)
        else:
            r = s  # 경로 띠(return_deviation)를 벗어나지 않았다: 복귀 시간 0
        last = r if r is not None else n - 1
        span = range(s, max(e, last) + 1)
        k_max = max(span, key=lambda k: dev[k])
        # 장애물 관련 여부: 시작 obstacle_lookback_s 전부터 복귀(또는 끝)까지 가장 가까운 장애물
        t_from = times[s] - p.obstacle_lookback_s
        lo = bisect.bisect_left(times, t_from)
        window = [k for k in range(lo, max(e, last) + 1) if samples[k].clearance is not None]
        k_near = min(window, key=lambda k: samples[k].clearance) if window else None
        min_c = samples[k_near].clearance if k_near is not None else None
        ep = Episode(
            index=len(episodes), start_t=times[s], trigger=trigger,
            max_deviation=dev[k_max], max_deviation_t=times[k_max],
            return_t=times[r] if r is not None else None,
            return_s=(times[r] - times[s]) if r is not None else None,
            min_clearance=min_c,
            nearest_obstacle=samples[k_near].nearest_obstacle if k_near is not None else '',
            obstacle_speed=samples[k_near].obstacle_speed if k_near is not None else None,
            obstacle_related=min_c is not None and min_c < p.interaction_distance)
        episodes.append(ep)
        for k in span:
            samples[k].episode = ep.index
        if r is None:
            break
        i = max(e, r) + 1
    return episodes


def _collision_events(samples: list[SampleResult], robot: Sequence[Pose]) -> list[dict]:
    """충돌 판정이 이어진 구간 하나를 1건으로 센다 (장애물이 바뀌면 다른 건)."""
    events: list[dict] = []
    cur = None
    for i, s in enumerate(samples):
        if s.collision and cur is not None and cur['obstacle'] == s.nearest_obstacle:
            cur['end_t'] = s.t
            cur['min_clearance'] = min(cur['min_clearance'], s.clearance)
            continue
        if cur is not None:
            events.append(cur)
            cur = None
        if s.collision:
            cur = {'start_t': s.t, 'end_t': s.t, 'obstacle': s.nearest_obstacle,
                   'min_clearance': s.clearance, 'robot_speed': round(_robot_speed(robot, i), 3),
                   'obstacle_speed': s.obstacle_speed}
    if cur is not None:
        events.append(cur)
    return events


def _r(v: Optional[float], nd: int = 4) -> Optional[float]:
    return None if v is None else round(float(v), nd)


def _summarize(samples, episodes, collisions, per_obstacle_min, reference, p: AvoidanceParams,
               reference_t) -> dict:
    devs = [(s.deviation, s.t) for s in samples if s.deviation is not None]
    clears = [(s.clearance, s.t, s.nearest_obstacle) for s in samples if s.clearance is not None]
    max_dev = max(devs) if devs else None
    min_clear = min(clears) if clears else None
    related = [e for e in episodes if e.obstacle_related]
    unreturned = [e for e in related if e.return_s is None]
    returns = [e.return_s for e in related if e.return_s is not None]
    max_return = None if unreturned else (max(returns) if returns else 0.0 if related else None)

    goal_t = None
    if reference:
        gx, gy = reference[-1]
        goal_t = next((s.t for s in samples if math.hypot(s.x - gx, s.y - gy) <= p.goal_tolerance), None)
    encounter = min_clear is not None and min_clear[0] < p.interaction_distance
    travel = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(samples, samples[1:]))

    collision_free = len(collisions) <= p.kpi_collisions
    deviation_ok = max_dev is not None and max_dev[0] <= p.kpi_max_deviation
    return_ok = not unreturned and all(r <= p.kpi_return_s for r in returns)
    goal_ok = goal_t is not None or not p.require_goal
    reasons = []
    if not reference:
        reasons.append('기준 경로 없음')
    if not samples:
        reasons.append('로봇 GT 없음')
    if samples and not per_obstacle_min:
        reasons.append('장애물 GT 없음')     # 장애물 토픽을 못 받았거나 로봇 기록 시간과 겹치지 않음
    elif samples and not encounter:
        # 무효 원인을 나눈다: 시나리오가 장애물과 시간을 못 맞춘 것인지, 로봇이 못 간 것인지
        if goal_t is not None:
            cause = '목표 도착'
        elif travel < p.stuck_travel_m:
            cause = '로봇이 거의 안 움직임'
        else:
            cause = '목표 전에 멈춤'
        reasons.append(f'장애물이 {p.interaction_distance} m 안에 오지 않음: {cause}')
    valid = not reasons
    return {
        'reference': {'points': len(reference), 'length_m': _r(polyline_length(reference), 3) if reference else None,
                      'start': list(reference[0]) if reference else None,
                      'goal': list(reference[-1]) if reference else None, 'received_t': _r(reference_t, 3)},
        'samples': len(samples),
        'travel_m': _r(travel, 3),
        't_start': _r(samples[0].t, 3) if samples else None,
        't_end': _r(samples[-1].t, 3) if samples else None,
        'obstacles': {oid: {'min_clearance': _r(v)} for oid, v in sorted(per_obstacle_min.items())},
        'collision': {
            'count': len(collisions), 'events': collisions,
            'min_clearance': _r(min_clear[0]) if min_clear else None,
            'min_clearance_t': _r(min_clear[1], 3) if min_clear else None,
            'min_clearance_obstacle': min_clear[2] if min_clear else '',
            'near_miss': bool(min_clear and min_clear[0] < p.near_miss_distance),
        },
        'deviation': {'max': _r(max_dev[0]) if max_dev else None, 'max_t': _r(max_dev[1], 3) if max_dev else None},
        'episodes': [{k: (_r(v) if isinstance(v, float) else v) for k, v in asdict(e).items()} for e in episodes],
        'avoidance': {'episodes': len(related), 'tracking_episodes': len(episodes) - len(related),
                      'max_return_s': _r(max_return, 3), 'unreturned': len(unreturned)},
        'encounter': encounter,
        'reached_goal': goal_t is not None,
        'reached_goal_t': _r(goal_t, 3),
        'kpi': {'collision_free': collision_free, 'deviation_ok': deviation_ok, 'return_ok': return_ok,
                'goal_ok': goal_ok, 'valid': valid, 'invalid_reasons': reasons,
                'pass': valid and collision_free and deviation_ok and return_ok and goal_ok},
        'params': asdict(p),
    }


# --- 여러 실행 ---------------------------------------------------------------------------------------


def _binom_cdf(k: int, n: int, p: float) -> float:
    """P(X ≤ k), X ~ Binomial(n, p)."""
    return sum(math.comb(n, i) * p ** i * (1.0 - p) ** (n - i) for i in range(k + 1))


def clopper_pearson(k: int, n: int, confidence: float = 0.95) -> tuple[Optional[float], Optional[float]]:
    """성공 k / n회의 성공률 정확 신뢰구간 (Clopper-Pearson). scipy 없이 이분 탐색으로 구한다."""
    if n <= 0:
        return None, None
    alpha = 1.0 - confidence

    def solve(f):
        a, b = 0.0, 1.0      # f(a) < 0 < f(b)인 p를 찾는다
        for _ in range(60):
            m = 0.5 * (a + b)
            a, b = (m, b) if f(m) < 0 else (a, m)
        return 0.5 * (a + b)

    # 하한: P(X ≥ k | p) = α/2, 상한: P(X ≤ k | p) = α/2
    lo = 0.0 if k == 0 else solve(lambda p: (1.0 - _binom_cdf(k - 1, n, p)) - alpha / 2)
    hi = 1.0 if k == n else solve(lambda p: alpha / 2 - _binom_cdf(k, n, p))
    return lo, hi


def _run_row(name: str, s: dict) -> dict:
    k = s['kpi']
    row = {'run': name, 'valid': k['valid'], 'pass': k['pass'], 'collisions': s['collision']['count'],
           'min_clearance': s['collision']['min_clearance'], 'max_deviation': s['deviation']['max'],
           'max_return_s': s['avoidance']['max_return_s'], 'unreturned': s['avoidance']['unreturned'],
           'reached_goal': s['reached_goal'], 'invalid_reasons': list(k['invalid_reasons']),
           'reasons': list(k['invalid_reasons'])}
    for key, label in (('collision_free', '충돌'), ('deviation_ok', '이탈 초과'),
                       ('return_ok', '복귀 초과·미복귀'), ('goal_ok', '목표 미도착')):
        if not k[key]:
            row['reasons'].append(label)
    return row


def aggregate(runs: Sequence[tuple[str, Optional[dict]]], params: AvoidanceParams = AvoidanceParams(),
              failed: Sequence[tuple] = ()) -> dict:
    """실행별 요약(avoidance_summary.json) 여러 개 → I17 판정.

    runs: (실행 이름, 요약 또는 None(요약 파일 없음)).
    failed: 하네스 outcome이 completed가 아닌 (실행, outcome) 또는 (실행, outcome, 요약 또는 None).
    판정 (원 명세 KPI는 "30회 모두"라 평균·신뢰구간이 아니라 최악값으로 본다):
    1. 어느 실행에서든(무효·하네스 실패 포함) 충돌이 있으면 FAIL. 충돌은 기준 경로 없이 셀 수 있다
    2. valid(기준 경로 있음 + 장애물을 만남) 실행 중 KPI를 어긴 것이 있으면 FAIL
    3. 하네스 실패 실행, 요약이 없는 실행이 있거나 valid 실행이 kpi_runs보다 적으면 INSUFFICIENT.
       재지 못한 실행을 빼고 PASS를 주면 성공한 실행만 고르는 셈이다
    4. 그 외 PASS
    """
    per_run, valid, measured = [], [], []
    missing = 0
    for name, s in runs:
        if s is None:
            missing += 1
            per_run.append({'run': name, 'valid': False, 'pass': False, 'invalid_reasons': ['요약 없음'],
                            'reasons': ['요약 없음']})
            continue
        row = _run_row(name, s)
        per_run.append(row)
        measured.append(row)
        if row['valid']:
            valid.append(row)
    harness_failed = []
    for name, outcome, *rest in failed:
        s = rest[0] if rest else None
        harness_failed.append({'run': name, 'outcome': outcome})
        row = (_run_row(name, s) if s is not None
               else {'run': name, 'valid': False, 'pass': False, 'reasons': []})
        # 하네스 실패 실행은 유효 횟수에 넣지 않고, 충돌만 판정에 쓴다
        row.update({'valid': False, 'pass': False, 'harness_outcome': outcome})
        row['reasons'].insert(0, f'하네스 실패 ({outcome})')
        per_run.append(row)
        if s is not None:
            measured.append(row)

    def worst(key):
        vals = [r[key] for r in valid if r.get(key) is not None]
        return max(vals) if vals else None

    collided = [r['run'] for r in measured if r['collisions'] > params.kpi_collisions]
    failing = [r['run'] for r in valid if not r['pass']]
    insufficient = []
    if harness_failed:
        insufficient.append(f'하네스 실패 {len(harness_failed)}회')
    if missing:
        insufficient.append(f'요약 없음 {missing}회')
    if len(valid) < params.kpi_runs:
        insufficient.append(f'유효 실행 {len(valid)}/{params.kpi_runs}회')
    passed = sum(r['pass'] for r in valid)
    lo, hi = clopper_pearson(passed, len(valid))
    if collided or failing:
        verdict = 'FAIL'
    elif insufficient:
        verdict = 'INSUFFICIENT'
    else:
        verdict = 'PASS'
    return {
        'verdict': verdict,
        'runs': len(per_run), 'valid_runs': len(valid), 'required_runs': params.kpi_runs,
        'harness_failed': harness_failed,
        'insufficient_reasons': insufficient,
        # 무효 사유별 실행 수 (하네스 실패는 outcome별로 따로)
        'invalid_reason_counts': dict(Counter(reason for r in per_run if not r['valid'] and not r.get('harness_outcome')
                                              for reason in r.get('invalid_reasons', []))),
        'harness_outcome_counts': dict(Counter(h['outcome'] for h in harness_failed)),
        # 보고용. 판정은 최악값으로 하고 이 구간은 쓰지 않는다 (30/30이어도 실패율 95% 상한은 약 12%)
        'pass_rate': {'passed': passed, 'valid': len(valid),
                      'rate': _r(passed / len(valid), 4) if valid else None,
                      'ci95': [_r(lo, 4), _r(hi, 4)] if valid else None},
        'collisions': sum(r['collisions'] for r in measured),
        'runs_with_collision': sum(r['collisions'] > 0 for r in measured),
        'min_clearance': min((r['min_clearance'] for r in measured if r['min_clearance'] is not None), default=None),
        'max_deviation': worst('max_deviation'),
        'max_return_s': worst('max_return_s'),
        'unreturned': sum(r['unreturned'] for r in valid),
        'goal_missed': sum(not r['reached_goal'] for r in valid),
        'failing_runs': list(dict.fromkeys(collided + failing)),
        'kpi': {'collisions': params.kpi_collisions, 'max_deviation': params.kpi_max_deviation,
                'return_s': params.kpi_return_s, 'runs': params.kpi_runs},
        'per_run': per_run,
    }


def format_summary(s: dict) -> str:
    """실행 하나 요약 한 줄 (로그용)."""
    k = s['kpi']
    invalid = '' if k['valid'] else f" (무효: {', '.join(k['invalid_reasons'])})"
    return (f"충돌 {s['collision']['count']}건, 최소 이격 {s['collision']['min_clearance']} m, "
            f"최대 이탈 {s['deviation']['max']} m, 회피 {s['avoidance']['episodes']}회 "
            f"최대 복귀 {s['avoidance']['max_return_s']} s, 목표 도착 {s['reached_goal']} → "
            f"{'PASS' if k['pass'] else 'FAIL'}{invalid}")


def aggregate_markdown(a: dict) -> str:
    def f(v, nd=3):
        return '—' if v is None else f'{v:.{nd}f}' if isinstance(v, float) else str(v)

    k = a['kpi']
    pr = a['pass_rate']
    rate = (f"{pr['passed']}/{pr['valid']} ({100 * pr['ci95'][0]:.1f}~{100 * pr['ci95'][1]:.1f}%)"
            if pr['valid'] else '—')
    lines = [
        f"## I17 동적 장애물 회피: {a['verdict']}", '',
        f"유효 실행 {a['valid_runs']}/{a['required_runs']}회 (전체 {a['runs']}회, 하네스 실패 {len(a['harness_failed'])}회)", '',
        *([f"판정 보류 이유: {', '.join(a['insufficient_reasons'])}", '']
          if a['verdict'] == 'INSUFFICIENT' else []),
        '| 지표 | 값 | KPI |', '| --- | --- | --- |',
        f"| 충돌 (건, 모든 실행) | {a['collisions']} | {k['collisions']} |",
        f"| 최대 경로 이탈 [m] (최악 실행) | {f(a['max_deviation'])} | ≤ {k['max_deviation']} |",
        f"| 최대 복귀 시간 [s] (최악 실행) | {f(a['max_return_s'])} | ≤ {k['return_s']} |",
        f"| 복귀 못 한 회피 | {a['unreturned']} | 0 |",
        f"| 목표 미도착 실행 | {a['goal_missed']} | 0 |",
        f"| 최소 이격 [m] | {f(a['min_clearance'])} | — |",
        f"| 합격률 (유효 실행, 95% CI, 참고) | {rate} | — |", '',
        '| 실행 | 유효 | 판정 | 충돌 | 최소 이격 | 최대 이탈 | 최대 복귀 | 사유 |',
        '| --- | --- | --- | --- | --- | --- | --- | --- |',
    ]
    counts = [*(f'{n}회: {reason}' for reason, n in a['invalid_reason_counts'].items()),
              *(f'{n}회: 하네스 실패 ({o})' for o, n in a['harness_outcome_counts'].items())]
    if counts:
        lines[-2:-2] = ['무효·실패 사유: ' + ', '.join(counts), '']
    for r in a['per_run']:
        verdict = '하네스 실패' if r.get('harness_outcome') else 'PASS' if r['pass'] else 'FAIL'
        lines.append(f"| {r['run']} | {'O' if r['valid'] else 'X'} | {verdict} | "
                     f"{r.get('collisions', '—')} | {f(r.get('min_clearance'))} | {f(r.get('max_deviation'))} | "
                     f"{f(r.get('max_return_s'))} | {', '.join(r['reasons'])} |")
    return '\n'.join(lines) + '\n'
