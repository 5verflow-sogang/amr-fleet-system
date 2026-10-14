"""RL 평가 환경용 정책 (ROS 비의존). 정책 = obs(dict, rl_eval.policy_obs) → (v, ω) 함수.

여기 정책은 환경이 끝까지 도는지 보는 시험 주행기다. 비교군(B1·B2 DWA, R1·R2·제안 PPO)이 아니다.
학습한 PPO는 load_policy('모듈:함수')로 붙인다 (연우). 함수는 obs dict를 받아 (v, ω)를 돌려주면 된다.

- cruise: 출발 속도를 유지하며 통로 중심선을 따라가고, 목표 앞에서 토크 한계 가속(질량 앎)으로 멈춘다.
  장애물은 보지 않는다
- brake: cruise + 진행선 앞 장애물까지 거리가 (반응 지연 + 제동 거리 + 여유)보다 짧으면 급정지
"""

import importlib
import math
from typing import Callable, Tuple

ROBOT_LENGTH = 0.6
ROBOT_WIDTH = 0.4
WHEEL_TORQUE = 3.0
WHEEL_RADIUS = 0.1
ROBOT_MASS = 28.9
MU_G = 0.7 * 9.81


def max_accel(payload_kg: float) -> float:
    """dynamics.max_linear_accel과 같은 식."""
    return min(2 * WHEEL_TORQUE / (WHEEL_RADIUS * (ROBOT_MASS + payload_kg)), MU_G)


def _steer(obs) -> float:
    # 통로 중심선(y = 0, +x) 따라가기
    return max(-1.5, min(1.5, -2.0 * obs['y'] - 2.5 * obs['theta']))


def cruise(obs) -> Tuple[float, float]:
    a = 0.8 * max_accel(obs['payload_kg'])
    remaining = obs['goal_x'] - obs['x'] - 0.05
    v = min(obs['initial_speed'], math.sqrt(max(0.0, 2.0 * a * remaining)))
    return v, _steer(obs)


def brake(obs) -> Tuple[float, float]:
    v, w = cruise(obs)
    a = max_accel(obs['payload_kg'])
    front = obs['x'] + ROBOT_LENGTH / 2
    for o in obs['obstacles']:
        in_path = abs(o['y'] - obs['y']) < ROBOT_WIDTH / 2 + o['radius'] + 0.1
        ahead = o['x'] - o['radius'] - front
        need = obs['v'] * 0.1 + obs['v'] ** 2 / (2.0 * a) + 0.3
        if in_path and 0.0 < ahead < need + 0.5:
            return 0.0, w
    return v, w


BUILTIN = {'cruise': cruise, 'brake': brake}


def load_policy(spec: str) -> Callable:
    """'cruise'·'brake' 또는 '패키지.모듈:함수' (PYTHONPATH에 있어야 한다)."""
    if spec in BUILTIN:
        return BUILTIN[spec]
    if ':' not in spec:
        raise ValueError(f'정책은 {sorted(BUILTIN)} 또는 모듈:함수: {spec}')
    module, func = spec.split(':', 1)
    return getattr(importlib.import_module(module), func)
