"""보상 함정 점검: 규칙 정책(질량 앎)과 그 실패 변형들을 같은 보상으로 채점해, 실패 비용의 순서·간격을 본다.

check_reward.py는 순항 속도만 바꿔 "안전 속도가 최적인가"를 본다. 그런데 규칙 정책은 늘 목표에서 서므로
"안 서고 지나치기"·"기어가기"·"안 가기" 같은 다른 실패가 얼마나 싼지는 보이지 않는다. 학습 초반 정책은 아직
정확히 서지 못하므로, 이 실패들이 너무 싸거나(지나치기) 너무 비싸면(느리게 가기) 그쪽 국소해에 빠진다
(탐색 실험 후보 I: 지나치기가 시간 초과보다 16배 싸서 10개 시드 중 8개가 무너졌다).

    uv run python scripts/check_reward_traps.py config/env_v0.toml config/env_v3_info.toml

행동: stop(규칙 정책 그대로), fast(순항 1.2배), rush(목표에서 안 서고 지나침), creep(순항 0.3배), halt(안 움직임).
평가 목록은 튜닝용(100번대)만 쓴다.
"""

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from check_reward import ScaledRule, episode_return  # noqa: E402
from payload_nav import PayloadNavEnv, load_experiment  # noqa: E402
from payload_nav.seeds import eval_seed  # noqa: E402


@dataclass
class Halt(ScaledRule):
    def __call__(self, env) -> np.ndarray:
        return np.array([-1.0, 0.0])    # 선속도 0


BEHAVIORS = {
    'stop': lambda: ScaledRule(mass_aware=True, scale=1.0),
    'fast': lambda: ScaledRule(mass_aware=True, scale=1.2),
    'rush': lambda: ScaledRule(mass_aware=True, scale=1.0, stop_offset_m=-5.0),   # 정지점을 목표 5 m 뒤로
    'creep': lambda: ScaledRule(mass_aware=True, scale=0.3),
    'halt': lambda: Halt(mass_aware=True),
}


def check(config: Path, lists, episodes: int) -> dict:
    base = load_experiment(config).env
    env = PayloadNavEnv(replace(base, observe_mass=True, mass_hide_prob=0.0))
    out = {}
    for name, make in BEHAVIORS.items():
        policy = make()
        rets, outcomes = [], []
        for m in base.train_masses:
            for k in lists:
                for ep in range(episodes):
                    r, o, _ = episode_return(env, policy, eval_seed(k, ep), m)
                    rets.append(r)
                    outcomes.append(o)
        top = max(set(outcomes), key=outcomes.count)
        out[name] = (float(np.mean(rets)), top, outcomes.count(top) / len(outcomes))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('configs', nargs='+', type=Path)
    p.add_argument('--lists', nargs='+', type=int, default=[100, 101])
    p.add_argument('--episodes', type=int, default=50)
    args = p.parse_args()
    for config in args.configs:
        res = check(config, args.lists, args.episodes)
        stop = res['stop'][0]
        print(f'== {config.name}')
        for name, (ret, top, frac) in res.items():
            print(f'  {name:6s} 평균 보상 {ret:9.1f}  stop 대비 {ret - stop:+9.1f}  (주 결과 {top} {frac:.0%})')


if __name__ == '__main__':
    main()
