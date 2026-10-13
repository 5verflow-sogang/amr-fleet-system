"""규칙 정책: 안전 순항 속도를 지키는 참고 기준선 (명세서 4-2절).

정식 비교군(B1·B2·R1·R2·제안)이 아니라, 환경이 "미리 속도를 줄이면 모든 학습 질량에서 피할 수 있게"
만들어졌는지 확인하고 RL 결과를 읽는 기준으로 쓴다. 관측 벡터가 아니라 환경 내부 상태(로봇 위치·속도,
장애물 목록)를 바로 읽는다. 장애물은 나타난 뒤에만 보이므로 출현 지점은 모른다 (RL 정책과 같은 조건).

규칙:
- 조향: 통로 중심선을 따라간다. ω 행동 = clip((−2y − 2θ) / 1.5, −1, 1)
- 순항: 안전 순항 속도 v_cruise로 달린다. 출현 지점을 모르므로 언제 장애물이 나와도 서야 한다.
  가장 짧은 출현 간격 gap_min(0.5 m) 안에, 반응 지연 t_r(0.1 s) 뒤 급정지로 서는 가장 빠른 속도다.

      v·t_r + v² / (2·a_max(m)) = gap_min
      → v_cruise(m) = a_max(m) · (√(t_r² + 2·gap_min / a_max(m)) − t_r)

  a_max(m)는 dynamics.max_linear_accel. 출현 지점 하한(scenarios.min_trigger_x)과 "설 수 있었는지" 판정
  (env popout_stoppable)과 같은 식이다. 0 kg 약 1.25 m/s, 25 kg 약 0.95 m/s.
- 장애물: 진행선(로봇 몸체 폭 + 양쪽 여유) 안에 장애물이 보이면 급정지한다(조향 0, 토크를 제동에만 쓴다).
  멈춰 선 장애물이면 진행선에서 비킬 때까지 서서 기다렸다가 다시 간다. 옆으로 비켜 가지는 않는다.
- 목표: 남은 거리 d에서 v ≤ √(2·k·a_max(m)·(d − d_stop))로 줄여 목표 바로 앞에서 선다.
  k(0.5)는 명령 지연·조향 몫을 감안한 여유다.

변형 두 가지:
- mass_agnostic: 질량을 모른다. 가장 무거운 학습 질량(25 kg, guard_payload_kg) 기준으로 순항·감속한다
- mass_aware: 질량을 안다. 관측 질량(env.mass_obs, 정보 조건 exact면 실제 질량)으로 순항·감속한다.
  질량이 "모름"(unknown)이면 mass_agnostic과 같다
"""

from dataclasses import dataclass
import math

import numpy as np

from .dynamics import DEFAULT_PARAMS, RobotParams, max_linear_accel

RULE_VARIANTS = {'mass_agnostic': 'rule_agnostic', 'mass_aware': 'rule_aware'}  # CLI 이름 → variant


def safe_cruise_speed(payload_kg: float, gap_m: float = 0.5, reaction_s: float = 0.1,
                      params: RobotParams = DEFAULT_PARAMS) -> float:
    """gap_m 앞에 장애물이 나타나도 반응 지연 뒤 급정지로 설 수 있는 가장 빠른 속도 [m/s]."""
    a = max_linear_accel(payload_kg, params)
    return a * (math.sqrt(reaction_s ** 2 + 2.0 * gap_m / a) - reaction_s)


@dataclass
class RulePolicy:
    mass_aware: bool
    decel_fraction: float = 0.5     # 목표 앞 감속에 쓰는 a_max 비율
    stop_offset_m: float = 0.05     # m. 목표 이만큼 앞을 정지점으로 본다 (도착 반경 0.2 m 안)
    lateral_margin_m: float = 0.1   # m. 진행선 판정에서 몸체 옆에 더하는 여유

    @property
    def name(self) -> str:
        return RULE_VARIANTS['mass_aware' if self.mass_aware else 'mass_agnostic']

    def planning_mass(self, env) -> float:
        """순항·감속을 정하는 질량. 모르면 가장 무거운 학습 질량."""
        guard = env.cfg.scenario.guard_payload_kg
        if self.mass_aware and env.mass_obs is not None:
            return env.mass_obs
        return guard

    def cruise_speed(self, env) -> float:
        c = env.cfg
        return min(c.robot.v_max, safe_cruise_speed(self.planning_mass(env), c.scenario.popout_gap[0],
                                                    c.reaction_s, c.robot))

    def path_blocked(self, env) -> bool:
        """로봇 앞(몸체 뒤끝보다 앞) 진행선 안에 장애물이 있는지."""
        s, p = env.state, env.cfg.robot
        for o in env.obstacles:
            ahead = o.x + o.radius > s.x - 0.5 * p.length
            in_lane = abs(o.y - s.y) < o.radius + 0.5 * p.width + self.lateral_margin_m
            if ahead and in_lane:
                return True
        return False

    def __call__(self, env) -> np.ndarray:
        """환경 상태 → 정규화 행동 [v, ω] ∈ [−1, 1]² (env.step: v = (a₀ + 1)/2·v_max, ω = a₁·ω_max)."""
        p, s = env.cfg.robot, env.state
        if self.path_blocked(env):
            return np.array([-1.0, 0.0])
        a = self.decel_fraction * max_linear_accel(self.planning_mass(env), p)
        remaining = max(0.0, env.goal[0] - s.x - self.stop_offset_m)
        v = min(self.cruise_speed(env), math.sqrt(2.0 * a * remaining))
        w = max(-1.0, min(1.0, (-2.0 * s.y - 2.0 * s.theta) / 1.5))
        return np.array([2.0 * v / p.v_max - 1.0, w])
