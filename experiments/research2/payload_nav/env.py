"""적재 질량 RL 로컬 플래너 학습 환경 v0 (명세서 4-2절).

로봇은 직선 통로(중심선 y = 0, 진행 방향 +x)를 따라 goal_distance 앞의 목표로 가서, 정지 조건(0.2 m 이내,
0.1 m/s 미만)을 만족해야 도착으로 본다. 에이전트는 0.1 s마다 [v_cmd, ω_cmd]를 낸다.

관측 48차원 (로봇 기준 상대좌표, 질량과 무관한 고정 상수로 정규화):
    목표 3 | 자기 상태 2 | 직전 명령 2 | 장애물 3개 × 6 | 최근 4스텝 이력 × 4
    | 경로 2 (중심선에서 옆으로 벗어난 거리, 경로 대비 방향 오차) | 벽 2 (왼쪽·오른쪽 벽까지 거리)
    | 남은 시간 1 | 적재 질량 2 (값, 앎 여부)
질량 칸은 제안 방법(observe_mass=True)만 채운다. 나머지 비교군은 (0, 0) = "질량을 모름"이다.
비교군끼리 신경망 구조를 같게 두어 결과 차이를 질량 정보 하나로 돌리기 위해서다.
앎 여부 플래그가 있어야 "질량 정보 없음"(0, 0)과 "0 kg"(0, 1)이 구분된다.
제안 방법은 학습 중 mass_hide_prob 확률로 질량을 가려(0, 0) 정보가 없을 때의 행동도 배운다.

남은 시간을 관측에 넣고 시간 초과를 실패(terminated)로 끝낸다. 시간 초과 페널티를 주려면 정책이
남은 시간을 알아야 보상이 상태만으로 정해진다 (ABS, RSS 2024와 같은 방식).

돌발 장애물이 나타나면 그 순간부터 멈출 때까지 간 거리(실제 제동 거리)와 제동 거리 여유
(나타날 때 간격 − 실제 제동 거리), 충돌했다면 충돌 순간 속도를 info에 남긴다 (명세서 4-5절).
"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from .dynamics import (DEFAULT_PARAMS, RobotParams, RobotState, braking_distance,
                       footprint_clearance, footprint_corners, max_linear_accel, step as dynamics_step,
                       wrap_angle)
from .scenarios import Scenario, ScenarioConfig, sample_scenario

K_OBSTACLES = 3
HISTORY = 4
OBS_DIM = 3 + 2 + 2 + 6 * K_OBSTACLES + 4 * HISTORY + 2 + 2 + 1 + 2  # 48
OBS_CLIP = 5.0
MASS_OBS_MODES = ('mass', 'brake_ratio')


@dataclass(frozen=True)
class EnvConfig:
    dt: float = 0.1                     # s. 정책 주기 10 Hz
    substeps: int = 4                   # 적분·충돌 판정을 나눠 하는 횟수 (2 m/s에서 5 cm 간격)
    max_steps: int = 300                # 30 s
    # 적재 질량
    mass_mode: str = 'random'           # 'random': train_masses 균등 / 'fixed': fixed_mass
    train_masses: Tuple[float, ...] = (0.0, 2.0, 10.0, 25.0)
    fixed_mass: float = 0.0
    observe_mass: bool = False          # True면 제안 방법
    mass_hide_prob: float = 0.0         # 학습 중 질량을 "모름"으로 가릴 확률 (제안 방법 0.2)
    # 보상 (v1 튜닝 후보 G, README "v1 튜닝 기록". config/env_v0.toml과 같아야 한다)
    w_progress: float = 5.0             # /m
    w_time: float = 0.1                 # /스텝
    w_near: float = 0.0                 # /m (안전거리 침범량). 장애물만, 벽은 제외
    d_safe: float = 0.3                 # m. 명세 긴급정지 거리
    w_smooth: float = 0.0               # 정규화된 명령 변화량(|Δv|/v_max + |Δω|/ω_max)당
    r_collision: float = -100.0
    w_impact: float = 40.0              # 충돌 순간 속도 [m/s]당 추가 벌점. 0이면 속도와 무관
    r_arrival: float = 100.0
    r_timeout: float = -25.0
    w_overshoot: float = 12.5           # /m
    # 종료 조건
    arrive_dist: float = 0.2            # m
    arrive_speed: float = 0.1           # m/s
    overshoot_fail: float = 2.0         # m. 목표를 이만큼 지나면 실패로 끝낸다
    stop_speed: float = 0.1             # m/s. 실제 제동 거리를 잴 때 "멈췄다"로 보는 속도 (도착 기준과 같음)
    reaction_s: float = 0.1             # s. "설 수 있었는지" 판정에 쓰는 반응 지연 (정책 주기 한 번)
    # 관측 정규화 상수 (질량과 무관한 고정값)
    norm_goal_dist: float = 20.0
    norm_obs_dist: float = 10.0
    norm_rel_speed: float = 3.0
    norm_radius: float = 1.0
    norm_mass: float = 30.0
    # 질량 칸에 넣는 값 (README "제안 방법 질량 입력 튜닝"). 'mass': m / norm_mass, 'brake_ratio': a_max(m) / a_max(0 kg)
    # (빈 차 대비 제동 능력, B2가 쓰는 a_max(m)과 같은 정보). 모르면 둘 다 (0, 0). 제안 방법은 설정 파일
    # [variants.proposed]에서 brake_ratio를 쓴다. 기본값 mass는 이 옵션이 없던 예전 실행(config.json)과 맞춘 것이다
    mass_obs_mode: str = 'mass'
    norm_lateral: float = 1.0           # m. 경로 옆 오차, 벽까지 거리
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    robot: RobotParams = DEFAULT_PARAMS


@dataclass
class _Obstacle:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    hold_until: float = 0.0   # s. 이 시각까지 제자리에 서 있다 (멈춰 서는 장애물)


def collision_penalty(c: 'EnvConfig', impact_speed: Optional[float]) -> float:
    """충돌 종료 보상. r_collision에 충돌 순간 속도에 비례한 벌점(w_impact)을 더한다.

    속도와 무관하면 0.1 m/s로 살짝 닿는 것과 1.5 m/s로 박는 것이 같은 벌이라, 돌발 장애물이 나올 수 있는
    구간에서 미리 감속할 이유가 약하다 (v1 튜닝 진단: 멈춰 선 장애물 충돌 대부분이 "이미 설 수 없던" 경우).
    """
    return c.r_collision - c.w_impact * abs(impact_speed or 0.0)


class PayloadNavEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, config: EnvConfig = EnvConfig()):
        super().__init__()
        if config.mass_mode not in ('random', 'fixed'):
            raise ValueError(f"mass_mode는 'random' 또는 'fixed'여야 한다: {config.mass_mode}")
        if not 0.0 <= config.mass_hide_prob <= 1.0:
            raise ValueError(f'mass_hide_prob는 [0, 1] 범위여야 한다: {config.mass_hide_prob}')
        if config.mass_obs_mode not in MASS_OBS_MODES:
            raise ValueError(f'mass_obs_mode는 {MASS_OBS_MODES} 중 하나여야 한다: {config.mass_obs_mode}')
        self.cfg = config
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(-OBS_CLIP, OBS_CLIP, shape=(OBS_DIM,), dtype=np.float32)
        self._history: deque = deque(maxlen=HISTORY)

    # ---------------------------------------------------------------- gym API
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """options (평가·테스트용):
            scenario: Scenario      시나리오를 직접 지정
            popout: bool|None       True면 S1(돌발 장애물), False면 S2(없음)로 강제. 기본은 popout_prob
            payload_kg: float       실제 적재 질량 지정
            mass_obs_kg: float|None 관측에 넣을 질량 (오차 조건). None이면 "모름". 기본은 실제 질량
                                    이 값을 주면 학습용 질량 가림(mass_hide_prob)은 적용하지 않는다
        시나리오를 질량보다 먼저 뽑으므로, 같은 seed면 질량과 관계없이 같은 시나리오가 나온다.
        """
        super().reset(seed=seed)
        options = options or {}
        self.scenario: Scenario = options.get('scenario') or sample_scenario(
            self.np_random, self.cfg.scenario, self.cfg.robot, options.get('popout'))
        if 'payload_kg' in options:
            self.payload = float(options['payload_kg'])
        elif self.cfg.mass_mode == 'fixed':
            self.payload = self.cfg.fixed_mass
        else:
            self.payload = float(self.np_random.choice(self.cfg.train_masses))
        if 'mass_obs_kg' in options:
            mass_obs = options['mass_obs_kg']
        elif self.cfg.observe_mass and self.np_random.random() < self.cfg.mass_hide_prob:
            mass_obs = None
        else:
            mass_obs = self.payload
        self.mass_obs: Optional[float] = None if mass_obs is None else float(mass_obs)

        sc = self.scenario
        self.half_width = 0.5 * sc.corridor_width
        self.state = RobotState(y=sc.initial_y, theta=sc.initial_heading, v=sc.initial_speed)
        self.goal = (sc.goal_distance, 0.0)
        self.v_cmd, self.w_cmd = sc.initial_speed, 0.0
        self.t = 0.0
        self.steps = 0
        self._substeps_done = 0  # 시간을 정수 카운터로 세어 부동소수 오차가 쌓이지 않게 한다
        self.obstacles: List[_Obstacle] = []
        self._popout_pending = sc.popout is not None
        self.popout_appeared = False
        self.popout_stoppable: Optional[bool] = None
        self.popout_blocked: Optional[bool] = None
        self._popout_origin: Optional[Tuple[float, float]] = None  # 나타난 순간 로봇 위치
        self._popout_reach = 0.0                                   # 그때 로봇 앞면~장애물 중심
        self.popout_stop_distance: Optional[float] = None
        self.popout_stop_margin: Optional[float] = None
        self.popout_result: Optional[str] = None   # 'stopped' / 'passed' / 'collision' / None
        self.collision_with: Optional[str] = None  # 'obstacle' / 'wall'
        self.impact_speed: Optional[float] = None
        self.prev_dist = self._goal_dist()
        self.prev_overshoot = max(0.0, self._overshoot())
        self._history.clear()
        for _ in range(HISTORY):
            self._history.append(self._history_entry())
        return self._observe(), self._info(None, math.inf)

    def step(self, action):
        p, c = self.cfg.robot, self.cfg
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        prev_v_cmd, prev_w_cmd = self.v_cmd, self.w_cmd
        self.v_cmd = 0.5 * (a[0] + 1.0) * p.v_max   # [−1, 1] → [0, v_max], 질량과 무관
        self.w_cmd = a[1] * p.w_max

        sub_dt = c.dt / c.substeps
        min_clear = math.inf
        collided = False
        for _ in range(c.substeps):
            self.state = dynamics_step(self.state, self.v_cmd, self.w_cmd, self.payload, sub_dt, p)
            self._substeps_done += 1
            self.t = self._substeps_done * sub_dt
            for obs in self.obstacles:
                if self.t > obs.hold_until:
                    obs.x += obs.vx * sub_dt
                    obs.y += obs.vy * sub_dt
            if self._popout_pending and self.state.x >= self.scenario.popout.trigger_x:
                self._spawn_popout()
            for obs in self.obstacles:
                min_clear = min(min_clear, footprint_clearance(self.state, obs.x, obs.y, obs.radius, p))
            if min_clear <= 0.0:
                collided, self.collision_with = True, 'obstacle'
            elif self._wall_clearance() <= 0.0:
                collided, self.collision_with = True, 'wall'
            if collided:
                self.impact_speed = self.state.v
                break
            self._track_popout_stop()
        self.steps += 1

        dist = self._goal_dist()
        overshoot = max(0.0, self._overshoot())
        smooth = abs(self.v_cmd - prev_v_cmd) / p.v_max + abs(self.w_cmd - prev_w_cmd) / p.w_max
        reward = (c.w_progress * (self.prev_dist - dist)
                  - c.w_time
                  - c.w_near * max(0.0, c.d_safe - min_clear)
                  - c.w_smooth * smooth
                  # 지나친 거리의 최댓값이 늘어난 만큼만 벌한다. 되돌아와도 보상은 없다
                  - c.w_overshoot * max(0.0, overshoot - self.prev_overshoot))
        self.prev_dist, self.prev_overshoot = dist, max(overshoot, self.prev_overshoot)

        outcome = None
        if collided:
            reward += collision_penalty(c, self.impact_speed)
            outcome = 'collision'
            if self.popout_result is None and self.popout_appeared and self.collision_with == 'obstacle':
                self.popout_result = 'collision'
        elif dist < c.arrive_dist and abs(self.state.v) < c.arrive_speed:
            reward += c.r_arrival
            outcome = 'success'
        elif overshoot > c.overshoot_fail:
            outcome = 'overshoot'
        elif self.steps >= c.max_steps:
            # 남은 시간을 관측하므로 시간 초과도 상태로 정해지는 실패다 (truncated가 아니라 terminated)
            reward += c.r_timeout
            outcome = 'timeout'
        terminated = outcome is not None

        self._history.append(self._history_entry())
        return self._observe(), float(reward), terminated, False, self._info(outcome, min_clear)

    # ---------------------------------------------------------------- 내부
    def _goal_dist(self) -> float:
        return math.hypot(self.goal[0] - self.state.x, self.goal[1] - self.state.y)

    def _overshoot(self) -> float:
        return self.state.x - self.goal[0]  # 진행 방향(+x)으로 목표를 지난 거리

    def _wall_distances(self) -> Tuple[float, float]:
        """로봇 몸체에서 왼쪽(+y)·오른쪽(−y) 벽까지 거리 [m]. 0 이하면 벽에 닿았다."""
        ys = [y for _, y in footprint_corners(self.state, self.cfg.robot)]
        return self.half_width - max(ys), min(ys) + self.half_width

    def _wall_clearance(self) -> float:
        return min(self._wall_distances())

    def _spawn_popout(self):
        """경로(통로 중심선) 기준 좌표로 로봇 앞 gap 거리에 장애물을 만든다."""
        self._popout_pending = False
        po, s, p = self.scenario.popout, self.state, self.cfg.robot
        front_x = s.x + 0.5 * p.length
        x = front_x + po.gap + po.radius
        limit = max(0.0, self.half_width - po.radius)   # 통로 안에 생기게 한다
        y = min(max(s.y + po.lateral_offset, -limit), limit)
        self.obstacles.append(_Obstacle(x, y, 0.0, po.direction * po.speed, po.radius,
                                        hold_until=self.t + po.dwell_s))
        self.popout_appeared = True
        # 분석용: 나타난 순간 실제 질량으로, 반응 지연 뒤 급정지해서 간격 안에 설 수 있었는지
        self.popout_stoppable = bool(po.gap >= s.v * self.cfg.reaction_s
                                     + braking_distance(s.v, self.payload, p))
        # 분석용: 장애물 옆으로 로봇(폭 + 양쪽 5 cm)이 지나갈 공간이 없으면 막힘
        need = p.width + 0.1
        left_space = self.half_width - (y + po.radius)
        right_space = (y - po.radius) + self.half_width
        self.popout_blocked = bool(max(left_space, right_space) < need)
        self._popout_origin = (float(s.x), float(s.y))
        self._popout_reach = po.gap + po.radius

    def _track_popout_stop(self):
        """장애물이 나타난 뒤 처음 멈출 때까지 간 거리를 잰다.

        멈추기 전에 장애물이 있던 자리(나타날 때 장애물 중심)를 지나면 'passed'로 본다.
        장애물이 비키거나 옆으로 비켜 가서 멈추지 않고 지나간 경우다. 이때 제동 거리는 없다.
        """
        if self._popout_origin is None or self.popout_result is not None:
            return
        traveled = math.hypot(self.state.x - self._popout_origin[0],
                              self.state.y - self._popout_origin[1])
        if traveled > self._popout_reach:
            self.popout_result = 'passed'
        elif self.state.v < self.cfg.stop_speed:
            self.popout_result = 'stopped'
            self.popout_stop_distance = traveled
            self.popout_stop_margin = self.scenario.popout.gap - traveled

    def _history_entry(self):
        p = self.cfg.robot
        return (self.state.v / p.v_max, self.state.w / p.w_max,
                self.v_cmd / p.v_max, self.w_cmd / p.w_max)

    def _observe(self) -> np.ndarray:
        c, p, s = self.cfg, self.cfg.robot, self.state
        cos_t, sin_t = math.cos(s.theta), math.sin(s.theta)

        def to_local(dx, dy):
            return cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy

        gx, gy = to_local(self.goal[0] - s.x, self.goal[1] - s.y)
        heading_err = math.atan2(gy, gx)
        obs = [math.hypot(gx, gy) / c.norm_goal_dist, math.sin(heading_err), math.cos(heading_err),
               s.v / p.v_max, s.w / p.w_max, self.v_cmd / p.v_max, self.w_cmd / p.w_max]

        nearest = sorted(self.obstacles,
                         key=lambda o: footprint_clearance(s, o.x, o.y, o.radius, p))[:K_OBSTACLES]
        robot_vx, robot_vy = s.v * cos_t, s.v * sin_t
        for o in nearest:
            moving = self.t > o.hold_until
            lx, ly = to_local(o.x - s.x, o.y - s.y)
            rvx, rvy = to_local((o.vx if moving else 0.0) - robot_vx,
                                (o.vy if moving else 0.0) - robot_vy)
            obs += [lx / c.norm_obs_dist, ly / c.norm_obs_dist, rvx / c.norm_rel_speed,
                    rvy / c.norm_rel_speed, o.radius / c.norm_radius, 1.0]
        obs += [0.0] * 6 * (K_OBSTACLES - len(nearest))

        for entry in self._history:
            obs += entry

        # 경로(통로 중심선, +x 방향): 옆 오차(왼쪽 +), 방향 오차. 벽까지 거리 (OBS_CLIP에서 잘린다)
        left, right = self._wall_distances()
        obs += [s.y / c.norm_lateral, wrap_angle(s.theta) / math.pi,
                left / c.norm_lateral, right / c.norm_lateral]
        obs.append(1.0 - self.steps / c.max_steps)   # 남은 시간 비율

        known = c.observe_mass and self.mass_obs is not None
        obs += [self._mass_feature() if known else 0.0, 1.0 if known else 0.0]
        return np.clip(np.asarray(obs, dtype=np.float32), -OBS_CLIP, OBS_CLIP)

    def _mass_feature(self) -> float:
        c = self.cfg
        if c.mass_obs_mode == 'brake_ratio':
            return max_linear_accel(self.mass_obs, c.robot) / max_linear_accel(0.0, c.robot)
        return self.mass_obs / c.norm_mass

    def _info(self, outcome: Optional[str], min_clear: float) -> dict:
        po = self.scenario.popout
        return {
            'outcome': outcome,
            'is_success': outcome == 'success',
            'collision_with': self.collision_with,
            'impact_speed': self.impact_speed,
            'payload_kg': self.payload,
            'time': self.t,
            'v': self.state.v,
            'min_clearance': min_clear,
            'overshoot': self.prev_overshoot,
            'corridor_width': self.scenario.corridor_width,
            'popout_appeared': self.popout_appeared,
            'popout_kind': None if po is None else ('standing' if po.dwell_s > 0.0 else 'crossing'),
            'popout_stoppable': self.popout_stoppable,
            'popout_blocked': self.popout_blocked,
            'popout_result': self.popout_result,
            'popout_stop_distance': self.popout_stop_distance,
            'popout_stop_margin': self.popout_stop_margin,
            'mass_known': self.cfg.observe_mass and self.mass_obs is not None,
        }
