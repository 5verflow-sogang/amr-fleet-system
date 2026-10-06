"""v0 시나리오: 창고 통로에서 목표 정밀 정지(S2) + 선반 사각지대 돌발 장애물(S1) (명세서 4-2절).

로봇은 폭이 정해진 직선 통로(중심선 y = 0, 진행 방향 +x)를 따라 목표까지 간다. 경로는 통로 중심선이다.

시나리오는 적재 질량과 무관하게 뽑는다. 질량에 따라 분포가 달라지면, 질량을 보지 않는
비교군이 시나리오에서 질량을 짐작할 수 있다.

돌발 장애물은 로봇이 경로상 x_trigger 지점을 지날 때 나타난다 (시간이 아니라 위치 기준).
그래서 어떤 정책이든 같은 지점에서 같은 장애물을 만난다. 빨리 달려 장애물을 피해 가는 일이 없다.
x_trigger는 "가장 무거운 학습 질량(25 kg)으로, 반응 지연 뒤 초기 속도에서 급정지했을 때 멈추는 거리"
이후로 뽑는다. 그래서 미리 속도를 줄여 두면 모든 학습 질량에서 피할 수 있고, 높은 속도를 계속 유지할지는
정책이 질량을 보고 정한다.

돌발 장애물은 두 종류다.
- 가로지름(crossing): 진행선을 가로질러 지나간다. 속도를 줄이면 비켜 갈 때까지 기다리지 않아도 된다.
- 멈춰 섬(standing): 진행선에 들어와 dwell 동안 서 있다가 옆으로 비킨다. 작업자가 선반 사이에서
  튀어나와 멈춰 선 상황이다. 통로가 좁으면 옆으로 비켜 갈 수 없어 장애물 앞에서 서야 한다.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from .dynamics import DEFAULT_PARAMS, RobotParams, max_linear_accel

Range = Tuple[float, float]


@dataclass(frozen=True)
class ScenarioConfig:
    goal_distance: Range = (10.0, 20.0)      # m. 출발점에서 목표까지 (진행 방향 +x)
    initial_speed: Range = (1.0, 2.0)        # m/s. 명세 S1 주행 속도
    heading_noise_deg: float = 5.0           # 출발 방향 오차
    start_lateral_offset: float = 0.1        # m. 출발 위치의 통로 중심선 대비 옆 오차 (최대)
    corridor_width: Range = (1.2, 2.0)       # m. 통로 폭 (선반 사이 통로). 로봇 폭 0.4 m
    popout_prob: float = 0.7                 # 에피소드에 돌발 장애물이 있을 확률
    popout_gap: Range = (0.5, 1.5)           # m. 나타날 때 로봇 앞면~장애물 표면 (명세 S1)
    popout_speed: Range = (0.3, 1.5)         # m/s. 비킬 때 옆으로 움직이는 속도 (명세 동적 장애물)
    popout_lateral_offset: float = 0.2       # m. 나타날 때 로봇 진행선에서 옆으로 벗어난 최대 거리
    obstacle_radius: Range = (0.25, 0.4)     # m. 작업자(지름 0.5 m) ~ 소형 물체
    standing_prob: float = 0.5               # 돌발 장애물 중 멈춰 서는 종류의 비율
    standing_dwell_s: Range = (1.5, 3.0)     # s. 멈춰 선 장애물이 진행선에 서 있는 시간
    guard_payload_kg: float = 25.0           # 출현 지점 기준 질량 (가장 무거운 학습 질량)
    guard_reaction_s: float = 0.1            # s. 반응 지연 (정책 주기 한 번)
    guard_margin_m: float = 0.5              # m. 출현 지점 최소 거리에 더하는 여유
    trigger_goal_margin_m: float = 3.0       # m. 출현 지점은 목표보다 이만큼 앞에 둔다


@dataclass(frozen=True)
class Popout:
    trigger_x: float       # m. 로봇 중심이 경로상 이 지점을 지나면 나타난다
    gap: float             # m. 나타날 때 로봇 앞면~장애물 표면 (경로 방향)
    lateral_offset: float  # m. 나타날 때 로봇 위치 기준 옆 오프셋 (왼쪽 +)
    speed: float           # m/s
    direction: int         # +1: 왼쪽으로, -1: 오른쪽으로 비킴
    radius: float          # m
    dwell_s: float = 0.0   # s. 0이면 가로지름, 0보다 크면 진행선에 이만큼 서 있다가 비킴


@dataclass(frozen=True)
class Scenario:
    goal_distance: float
    initial_speed: float
    initial_heading: float  # rad
    popout: Optional[Popout]
    corridor_width: float = 1e6   # m. 기본값은 사실상 벽 없음 (테스트에서 시나리오를 직접 만들 때)
    initial_y: float = 0.0        # m


def min_trigger_x(initial_speed: float, config: 'ScenarioConfig',
                  params: RobotParams = DEFAULT_PARAMS) -> float:
    """출현 지점의 최소 x: 25 kg으로 반응 지연 뒤 초기 속도에서 급정지했을 때 멈추는 거리 + 여유."""
    a = max_linear_accel(config.guard_payload_kg, params)
    return (initial_speed * config.guard_reaction_s + initial_speed ** 2 / (2.0 * a)
            + config.guard_margin_m)


def sample_scenario(rng: np.random.Generator, config: ScenarioConfig = ScenarioConfig(),
                    params: RobotParams = DEFAULT_PARAMS) -> Scenario:
    def uniform(bounds: Range) -> float:
        return float(rng.uniform(bounds[0], bounds[1]))

    goal_distance = uniform(config.goal_distance)
    initial_speed = uniform(config.initial_speed)
    heading = math.radians(uniform((-config.heading_noise_deg, config.heading_noise_deg)))
    width = uniform(config.corridor_width)
    initial_y = uniform((-config.start_lateral_offset, config.start_lateral_offset))

    popout = None
    if rng.random() < config.popout_prob:
        lo = min_trigger_x(initial_speed, config, params)
        hi = goal_distance - config.trigger_goal_margin_m
        if hi <= lo:
            raise ValueError(f'목표가 너무 가까워 출현 지점을 둘 수 없다: [{lo:.2f}, {hi:.2f}]')
        popout = Popout(
            trigger_x=uniform((lo, hi)),
            gap=uniform(config.popout_gap),
            lateral_offset=uniform((-config.popout_lateral_offset, config.popout_lateral_offset)),
            speed=uniform(config.popout_speed),
            direction=int(rng.choice((-1, 1))),
            radius=uniform(config.obstacle_radius),
            dwell_s=(uniform(config.standing_dwell_s)
                     if rng.random() < config.standing_prob else 0.0),
        )
    return Scenario(goal_distance, initial_speed, heading, popout, width, initial_y)
