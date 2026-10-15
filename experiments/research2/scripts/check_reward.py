"""보상 검증: 보상이 "안전을 지키며 가장 빨리"를 가장 높게 평가하는지 본다 (README "v1 튜닝 기록" 보상 검증).

RL 결과를 보지 않는 검사다. 규칙 정책(질량 앎, payload_nav/rule_policy.py)의 순항 속도를 배율로 바꿔 가며
학습 분포(돌발 확률 popout_prob, 학습 질량)에서 평균 보상(return)을 잰다. 안전 순항 속도(배율 1.0)가 가장 높아야
보상이 과제(명세 안전 우선)와 맞는다. 더 빠르고 더 부딪히는 배율이 높으면 보상이 과속을 장려하는 것이다.

    uv run python scripts/check_reward.py                         # config/env_v0.toml
    uv run python scripts/check_reward.py config/env_v0.toml raw/<실행>/config.toml --lists 100 101

평가 목록은 튜닝용(100번대)만 쓴다. 0~9는 최종 평가용이다.
"""

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_nav import PayloadNavEnv, load_experiment  # noqa: E402
from payload_nav.rule_policy import RulePolicy  # noqa: E402
from payload_nav.seeds import eval_seed  # noqa: E402

SCALES = (0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5)


@dataclass
class ScaledRule(RulePolicy):
    """순항 속도만 scale배 한 규칙 정책. 장애물이 진행선에 있으면 급정지하는 것은 같다."""
    scale: float = 1.0

    def cruise_speed(self, env) -> float:
        return min(env.cfg.robot.v_max, self.scale * super().cruise_speed(env))


def episode_return(env, policy, seed: int, payload: float) -> tuple[float, str, float]:
    """(보상 합, 결과, 시간 [s]). 시나리오는 학습과 같이 돌발 확률로 뽑는다 (popout None)."""
    env.reset(seed=seed, options={'payload_kg': payload, 'mass_obs_kg': payload, 'popout': None})
    total = 0.0
    while True:
        _, reward, terminated, truncated, info = env.step(policy(env))
        total += reward
        if terminated or truncated:
            return total, info['outcome'], info['time']


def check(config: Path, lists=(100, 101), episodes: int = 200, scales=SCALES) -> list[dict]:
    """배율마다 평균 보상·충돌률·시간 초과율·성공 도착 시간과 질량별 평균 보상."""
    base = load_experiment(config).env
    env = PayloadNavEnv(replace(base, observe_mass=True, mass_hide_prob=0.0))
    rows = []
    for scale in scales:
        policy = ScaledRule(mass_aware=True, scale=scale)
        returns, outcomes, times, per_mass = [], [], [], {}
        for m in base.train_masses:
            mass_returns = []
            for k in lists:
                for ep in range(episodes):
                    ret, outcome, t = episode_return(env, policy, eval_seed(k, ep), m)
                    mass_returns.append(ret)
                    outcomes.append(outcome)
                    if outcome == 'success':
                        times.append(t)
            returns += mass_returns
            per_mass[m] = float(np.mean(mass_returns))
        rows.append({'scale': scale, 'return': float(np.mean(returns)),
                     'collision_rate': float(np.mean([o == 'collision' for o in outcomes])),
                     'timeout_rate': float(np.mean([o == 'timeout' for o in outcomes])),
                     'success_time_s': float(np.mean(times)) if times else None, 'per_mass': per_mass})
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument('configs', nargs='*', type=Path, default=[ROOT / 'config' / 'env_v0.toml'])
    p.add_argument('--lists', nargs='+', type=int, default=[100, 101], help='평가 목록 (튜닝용 100번대)')
    p.add_argument('--episodes', type=int, default=200, help='목록·질량마다 에피소드 수')
    p.add_argument('--scales', nargs='+', type=float, default=list(SCALES))
    args = p.parse_args()
    if any(k < 100 for k in args.lists):
        sys.exit('평가 목록 0~99는 쓰지 않는다 (0~9는 최종 평가용)')
    for config in args.configs:
        c = load_experiment(config).env
        print(f'== {config}  (진행 {c.w_progress}, 시간 {c.w_time}, 충돌 {c.r_collision} − {c.w_impact}·|v|, '
              f'도착 {c.r_arrival}, 시간 초과 {c.r_timeout})')
        rows = check(config, args.lists, args.episodes, args.scales)
        masses = '/'.join(f'{m:g}' for m in c.train_masses)
        print(f'{"배율":>5s} {"평균 보상":>9s} {"충돌":>6s} {"시간 초과":>7s} {"성공 도착 s":>10s}   질량별 평균 보상 ({masses} kg)')
        for r in rows:
            t = '-' if r['success_time_s'] is None else f'{r["success_time_s"]:.2f}'
            print(f'{r["scale"]:5.1f} {r["return"]:9.2f} {100 * r["collision_rate"]:5.1f}% '
                  f'{100 * r["timeout_rate"]:6.1f}% {t:>10s}   '
                  + ' / '.join(f'{v:.1f}' for v in r['per_mass'].values()))
        best = max(rows, key=lambda r: r['return'])
        print(f'→ 평균 보상이 가장 높은 배율: {best["scale"]:g} (안전 속도 1.0)\n')


if __name__ == '__main__':
    main()
