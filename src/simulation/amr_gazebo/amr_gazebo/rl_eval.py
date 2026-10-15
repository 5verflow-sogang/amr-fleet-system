"""연구 2 RL 평가 환경 (Gazebo): 2D 학습 환경과 같은 시나리오를 재현하고 같은 지표를 잰다 (ROS 비의존).

2D 환경은 experiments/research2/payload_nav (연우). 시나리오 뽑기(scenarios.py)와 동역학 상수·충돌 판정
(dynamics.py)을 그 파일 그대로 불러 쓴다. 패키지 __init__은 gymnasium·tomllib(Python 3.11)를 불러서
ROS Humble(Python 3.10)에서 안 되므로 두 파일만 직접 읽는다 (load_research2).

같은 시드 → 같은 시나리오: 2D 환경은 reset(seed)에서 gymnasium np_random(seed)로 시나리오를 먼저 뽑는다.
gymnasium의 np_random은 Generator(PCG64(SeedSequence(seed)))라 여기서 같은 생성기를 만들면 같은 값이 나온다
(scenario_for). 평가 시드 규칙(목록 k, 에피소드 번호)은 seeds.py와 같다 (eval_seed).

EpisodeTracker는 2D 환경 step()의 종료 조건과 evaluation.run_episode()의 지표를 Gazebo GT로 다시 계산한다.
열 이름을 2D 평가 CSV(episodes.csv)와 같게 둬서 두 결과를 그대로 이어 붙여 비교한다.
"""

from dataclasses import dataclass, field
import importlib.util
import math
import os
import sys
import types
from typing import List, Optional, Sequence

import numpy as np

# experiments/research2/payload_nav/seeds.py와 같은 값
EVAL_SEED_BASE = 100_000
EVAL_LIST_STRIDE = 1_000

# 2D EnvConfig 기본값 (env.py). 종료 조건·지표 판정에 쓴다
DT = 0.1
MAX_TIME_S = 30.0
ARRIVE_DIST = 0.2
ARRIVE_SPEED = 0.1
OVERSHOOT_FAIL = 2.0
STOP_SPEED = 0.1
REACTION_S = 0.1
NEAR_MISS_M = 0.3          # evaluation.py
JERK_SKIP_STEPS = 2        # evaluation.py

# 충돌 비트마스크: 2D처럼 장애물은 벽을 지나간다(비켜서 통로 밖으로 나감). 로봇(기본 0xffff)은 둘 다와 부딪힌다
WALL_BITMASK = 0x04
OBSTACLE_BITMASK = 0x02

WALL_THICKNESS = 0.1
WALL_HEIGHT = 1.0
OBSTACLE_HEIGHT = 1.7


def default_research2_dir() -> Optional[str]:
    """experiments/research2를 찾는다: 환경 변수 AMR_RESEARCH2_DIR, compose 마운트, 작업 폴더 위쪽 순."""
    candidates = [os.environ.get('AMR_RESEARCH2_DIR', ''), '/ros2_ws/src/experiments/research2']
    d = os.getcwd()
    for _ in range(6):
        candidates.append(os.path.join(d, 'experiments', 'research2'))
        d = os.path.dirname(d)
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, 'payload_nav', 'scenarios.py')):
            return c
    return None


def load_research2(root: str) -> types.SimpleNamespace:
    """payload_nav의 dynamics·scenarios 모듈만 읽는다 (패키지 __init__을 실행하지 않는다)."""
    pkg_dir = os.path.join(root, 'payload_nav')
    name = 'payload_nav_gz'   # 원래 이름(payload_nav)과 겹치지 않게
    if name not in sys.modules:
        pkg = types.ModuleType(name)
        pkg.__path__ = [pkg_dir]
        sys.modules[name] = pkg
    mods = {}
    for mod in ('dynamics', 'scenarios'):
        full = f'{name}.{mod}'
        if full not in sys.modules:
            spec = importlib.util.spec_from_file_location(full, os.path.join(pkg_dir, f'{mod}.py'))
            module = importlib.util.module_from_spec(spec)
            sys.modules[full] = module
            spec.loader.exec_module(module)
        mods[mod] = sys.modules[full]
    return types.SimpleNamespace(**mods)


def eval_seed(eval_list: int, episode: int) -> int:
    """seeds.eval_seed와 같다: 평가 목록 k의 에피소드 번호 → 시나리오 시드."""
    if not 0 <= episode < EVAL_LIST_STRIDE:
        raise ValueError(f'에피소드 번호는 0~{EVAL_LIST_STRIDE - 1}: {episode}')
    return EVAL_SEED_BASE + eval_list * EVAL_LIST_STRIDE + episode


def scenario_for(r2, seed: int, popout: Optional[bool]):
    """2D 환경 reset(seed, options={'popout': popout})이 뽑는 것과 같은 시나리오."""
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence(seed)))
    return r2.scenarios.sample_scenario(rng, r2.scenarios.ScenarioConfig(), r2.dynamics.DEFAULT_PARAMS, popout)


@dataclass(frozen=True)
class Layout:
    """에피소드 하나의 Gazebo 배치 (world 좌표 = 2D 통로 좌표)."""
    start_x: float          # 가속 구간 시작 (로봇 스폰 위치)
    start_y: float
    heading: float
    runup_m: float          # 출발선(x = 0)까지 가속 거리
    wall_x0: float          # 벽 x 범위
    wall_x1: float
    half_width: float


def layout_for(r2, scenario, payload_kg: float, margin_m: float = 0.8) -> Layout:
    """출발선에서 initial_speed가 되도록 가속 구간을 둔다. 토크 한계 가속 a(m)로 v0²/(2a) + 여유."""
    a = r2.dynamics.max_linear_accel(payload_kg)
    runup = scenario.initial_speed ** 2 / (2.0 * a) + margin_m
    sx = -runup * math.cos(scenario.initial_heading)
    sy = scenario.initial_y - runup * math.sin(scenario.initial_heading)
    return Layout(sx, sy, scenario.initial_heading, runup, wall_x0=sx - 1.0,
                  wall_x1=scenario.goal_distance + OVERSHOOT_FAIL + 1.0,
                  half_width=0.5 * scenario.corridor_width)


def _material(rgb) -> str:
    c = ' '.join(f'{v:g}' for v in rgb)
    return f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>'


def _bitmask(mask: int) -> str:
    return f'<surface><contact><collide_bitmask>{mask:#06x}</collide_bitmask></contact></surface>'


def walls_sdf(layout: Layout, name: str = 'corridor') -> str:
    """통로 벽 두 개 (static). 안쪽 면이 y = ±half_width."""
    length = layout.wall_x1 - layout.wall_x0
    cx = 0.5 * (layout.wall_x0 + layout.wall_x1)
    links = []
    for side, sign in (('left', 1), ('right', -1)):
        cy = sign * (layout.half_width + WALL_THICKNESS / 2)
        size = f'{length:.4f} {WALL_THICKNESS} {WALL_HEIGHT}'
        geom = f'<geometry><box><size>{size}</size></box></geometry>'
        links.append(
            f'<link name="{side}"><pose>{cx:.4f} {cy:.4f} {WALL_HEIGHT / 2} 0 0 0</pose>'
            f'<collision name="c">{geom}{_bitmask(WALL_BITMASK)}</collision>'
            f'<visual name="v">{geom}{_material((0.85, 0.45, 0.10))}</visual></link>')
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>'
            f'{"".join(links)}</model></sdf>')


def obstacle_sdf(radius: float, name: str = 'popout') -> str:
    """돌발 장애물 (원기둥). 동적 장애물 모델(dynamic_worker)과 같이 중력을 끄고 VelocityControl로 움직인다.
    /model/<name>/cmd_vel (몸체 기준, yaw 0이라 world와 같다), /model/<name>/odometry (GT)."""
    geom = f'<geometry><cylinder><radius>{radius:.4f}</radius><length>{OBSTACLE_HEIGHT - 0.025}</length></cylinder></geometry>'
    return (
        f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}">'
        f'<link name="link"><gravity>false</gravity><pose>0 0 {OBSTACLE_HEIGHT / 2} 0 0 0</pose>'
        '<inertial><mass>70</mass><inertia><ixx>17.95</ixx><iyy>17.95</iyy><izz>2.19</izz></inertia></inertial>'
        f'<collision name="c"><pose>0 0 0.0125 0 0 0</pose>{geom}{_bitmask(OBSTACLE_BITMASK)}</collision>'
        f'<visual name="v">{geom}{_material((0.2, 0.3, 0.8))}</visual></link>'
        '<plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl"/>'
        '<plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">'
        '<odom_frame>world</odom_frame><odom_publish_frequency>100</odom_publish_frequency><dimensions>2</dimensions>'
        '</plugin></model></sdf>')


def obstacle_spot(r2, scenario, robot_x: float, robot_y: float, half_width: float):
    """env._spawn_popout과 같은 자리: 로봇 앞면 + gap + 반지름, 옆은 로봇 y + lateral_offset (통로 안으로 자름)."""
    po, p = scenario.popout, r2.dynamics.DEFAULT_PARAMS
    x = robot_x + 0.5 * p.length + po.gap + po.radius
    limit = max(0.0, half_width - po.radius)
    y = min(max(robot_y + po.lateral_offset, -limit), limit)
    return x, y


@dataclass
class Sample:
    t: float                 # 출발선을 지난 뒤 시간 [s]
    x: float
    y: float
    theta: float
    v: float
    w: float


@dataclass
class EpisodeTracker:
    """2D env.step()의 종료 조건과 evaluation.run_episode()의 지표를 GT 표본으로 계산한다.

    update(sample, obstacle_xy)는 GT가 올 때마다(100 Hz), control_tick(sample)은 정책 주기(10 Hz)마다 부른다.
    """
    r2: types.SimpleNamespace
    scenario: object
    payload_kg: float
    half_width: float
    outcome: Optional[str] = None
    collision_with: Optional[str] = None
    impact_speed: Optional[float] = None
    min_clearance: float = math.inf
    min_ttc: float = math.inf
    max_overshoot: float = 0.0
    speeds: List[float] = field(default_factory=list)
    x0: Optional[float] = None
    last: Optional[Sample] = None
    # 돌발 장애물
    popout_appeared: bool = False
    popout_gap_actual: Optional[float] = None
    popout_stoppable: Optional[bool] = None
    popout_blocked: Optional[bool] = None
    popout_result: Optional[str] = None
    popout_stop_distance: Optional[float] = None
    popout_stop_margin: Optional[float] = None
    _origin: Optional[tuple] = None
    _reach: float = 0.0

    @property
    def params(self):
        return self.r2.dynamics.DEFAULT_PARAMS

    def _state(self, s: Sample):
        return self.r2.dynamics.RobotState(s.x, s.y, s.theta, s.v, s.w)

    def popout_triggered(self, s: Sample) -> bool:
        """로봇 중심이 trigger_x를 지났는지 (env와 같은 위치 기준)."""
        return self.scenario.popout is not None and not self.popout_appeared and s.x >= self.scenario.popout.trigger_x

    def appear(self, s: Sample, obstacle_xy):
        """장애물이 나타난 순간. 실제 간격(로봇 앞면 → 장애물 표면, 경로 방향)으로 판정값을 남긴다."""
        po, p = self.scenario.popout, self.params
        ox, oy = obstacle_xy
        front_x = s.x + 0.5 * p.length
        self.popout_appeared = True
        self.popout_gap_actual = ox - po.radius - front_x
        self.popout_stoppable = bool(self.popout_gap_actual >= s.v * REACTION_S
                                     + self.r2.dynamics.braking_distance(s.v, self.payload_kg, p))
        need = p.width + 0.1
        left_space = self.half_width - (oy + po.radius)
        right_space = (oy - po.radius) + self.half_width
        self.popout_blocked = bool(max(left_space, right_space) < need)
        self._origin = (s.x, s.y)
        self._reach = ox - front_x

    def update(self, s: Sample, obstacle_xy=None):
        """GT 표본 하나: 충돌·제동 추적. 충돌하면 outcome = collision."""
        if self.outcome is not None:
            return
        if self.x0 is None:
            self.x0 = s.x
        self.last = s
        st, p = self._state(s), self.params
        if self.popout_appeared and obstacle_xy is not None:
            clear = self.r2.dynamics.footprint_clearance(st, obstacle_xy[0], obstacle_xy[1],
                                                         self.scenario.popout.radius, p)
            self.min_clearance = min(self.min_clearance, clear)
            if clear <= 0.0:
                self._collide('obstacle', s)
                return
        ys = [y for _, y in self.r2.dynamics.footprint_corners(st, p)]
        if min(self.half_width - max(ys), min(ys) + self.half_width) <= 0.0:
            self._collide('wall', s)
            return
        self._track_popout_stop(s)

    def _collide(self, what: str, s: Sample):
        self.outcome, self.collision_with, self.impact_speed = 'collision', what, s.v
        if self.popout_result is None and self.popout_appeared and what == 'obstacle':
            self.popout_result = 'collision'

    def _track_popout_stop(self, s: Sample):
        if self._origin is None or self.popout_result is not None:
            return
        traveled = math.hypot(s.x - self._origin[0], s.y - self._origin[1])
        if traveled > self._reach:
            self.popout_result = 'passed'
        elif s.v < STOP_SPEED:
            self.popout_result = 'stopped'
            self.popout_stop_distance = traveled
            self.popout_stop_margin = self.popout_gap_actual - traveled

    def control_tick(self, s: Sample, obstacle=None):
        """정책 주기(0.1 s)마다: 속도 기록, TTC, 도착·지나침·시간 초과. obstacle = (x, y, vx, vy) 또는 None."""
        if self.outcome is not None:
            return
        self.speeds.append(s.v)
        if obstacle is not None and self.popout_appeared:
            self.min_ttc = min(self.min_ttc, self._ttc(s, obstacle))
        goal = self.scenario.goal_distance
        dist = math.hypot(goal - s.x, s.y)
        overshoot = max(0.0, s.x - goal)
        self.max_overshoot = max(self.max_overshoot, overshoot)
        if dist < ARRIVE_DIST and abs(s.v) < ARRIVE_SPEED:
            self.outcome = 'success'
        elif overshoot > OVERSHOOT_FAIL:
            self.outcome = 'overshoot'
        elif s.t >= MAX_TIME_S:
            self.outcome = 'timeout'

    def _ttc(self, s: Sample, obstacle) -> float:
        """evaluation.time_to_collision과 같은 근사."""
        ox, oy, ovx, ovy = obstacle
        vx, vy = s.v * math.cos(s.theta), s.v * math.sin(s.theta)
        dx, dy = ox - s.x, oy - s.y
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return 0.0
        closing = ((vx - ovx) * dx + (vy - ovy) * dy) / dist
        if closing <= 1e-6:
            return math.inf
        clear = self.r2.dynamics.footprint_clearance(self._state(s), ox, oy, self.scenario.popout.radius, self.params)
        return max(0.0, clear) / closing

    def metrics(self) -> dict:
        """evaluation.run_episode()와 같은 열."""
        po = self.scenario.popout
        speeds = self.speeds
        accel = np.diff(speeds[JERK_SKIP_STEPS:]) / DT if len(speeds) > JERK_SKIP_STEPS + 1 else np.zeros(0)
        jerk = np.diff(accel) / DT if len(accel) > 1 else np.zeros(1)
        t = self.last.t if self.last else 0.0
        return {
            'outcome': self.outcome,
            'success': self.outcome == 'success',
            'collision': self.outcome == 'collision',
            'timeout': self.outcome == 'timeout',
            'collision_with': self.collision_with,
            'impact_speed': self.impact_speed,
            'time_s': t,
            'mean_speed': (self.last.x - self.x0) / t if self.last and t > 0 else 0.0,
            'min_clearance': self.min_clearance,
            'near_miss': self.min_clearance < NEAR_MISS_M,
            'min_ttc': self.min_ttc,
            'overshoot_m': self.max_overshoot,
            'jerk_rms': float(np.sqrt(np.mean(jerk ** 2))) if jerk.size else 0.0,
            'jerk_max': float(np.max(np.abs(jerk))) if jerk.size else 0.0,
            'payload_kg': self.payload_kg,
            'corridor_width': self.scenario.corridor_width,
            'popout_appeared': self.popout_appeared,
            'popout_kind': None if po is None else ('standing' if po.dwell_s > 0.0 else 'crossing'),
            'popout_stoppable': self.popout_stoppable,
            'popout_blocked': self.popout_blocked,
            'popout_result': self.popout_result,
            'popout_stop_distance': self.popout_stop_distance,
            'popout_stop_margin': self.popout_stop_margin,
        }


def brake_metrics(trace, t_cmd: float, x_cmd: float, v_cmd: float, accel: float, dv: float = 0.05) -> dict:
    """급정지 한 번의 GT 표본 [(stamp, x, y, theta, v, w)]으로 제동 거리·지연·감속을 잰다 (rl_eval_node brake_sweep).

    - 모델 제동 거리 = v²/(2a): 2D dynamics.step이 명령 순간부터 토크 한계 a로 감속할 때
    - 지연 = 속도가 dv 떨어진 시각 − 명령 시각 − dv/a (모델대로 감속했다면 걸렸을 시간을 뺀다)
    - 감속 = 속도가 80 %~20 %인 구간의 v(t) 기울기 (최소제곱)
    """
    last = trace[-1] if trace else (t_cmd, x_cmd, 0.0, 0.0, v_cmd, 0.0)
    model = v_cmd * v_cmd / (2.0 * accel)
    stop = last[1] - x_cmd
    t_drop = next((s[0] for s in trace if s[4] <= v_cmd - dv), None)
    t_stop = next((s[0] for s in trace if abs(s[4]) < 0.005), last[0])
    mid = [(s[0], s[4]) for s in trace if 0.2 * v_cmd < s[4] < 0.8 * v_cmd]
    decel = None
    if len(mid) >= 3:
        ts, vs = np.array([m[0] for m in mid]), np.array([m[1] for m in mid])
        decel = float(-np.polyfit(ts, vs, 1)[0])
    return {
        'v_cmd': round(v_cmd, 4),
        'stop_dist': round(stop, 4),
        'model_dist': round(model, 4),
        'excess': round(stop - model, 4),
        'dead_time_s': None if t_drop is None else round(t_drop - t_cmd - dv / accel, 4),
        'decel_fit': None if decel is None else round(decel, 3),
        'decel_model': round(accel, 3),
        'stop_time_s': round(t_stop - t_cmd, 3),
    }


def policy_obs(s: Sample, scenario, layout: Layout, payload_kg: float, obstacles: Sequence[dict]) -> dict:
    """정책에 넘기는 관측 (dict). 2D 관측 48차원은 이 값들로 만들 수 있다 (env._observe 참고).
    obstacles: 나타난 장애물만 [{x, y, vx, vy, radius}] (2D처럼 나타나기 전에는 안 보인다)."""
    return {
        't': s.t, 'x': s.x, 'y': s.y, 'theta': s.theta, 'v': s.v, 'w': s.w,
        'goal_x': scenario.goal_distance, 'goal_y': 0.0, 'half_width': layout.half_width,
        'initial_speed': scenario.initial_speed, 'payload_kg': payload_kg,
        'remaining_s': max(0.0, MAX_TIME_S - s.t), 'dt': DT, 'obstacles': list(obstacles),
    }
