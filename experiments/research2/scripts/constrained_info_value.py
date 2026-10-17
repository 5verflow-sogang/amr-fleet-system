"""제약 RL(충돌 ≤ 한도에서 가장 빨리)의 질량 정보 값어치를 규칙 정책으로 미리 잰다.

    cd experiments/research2 && uv run python scripts/constrained_info_value.py

질량별 순항 배율을 바꿔 가며 충돌률·도착 시간을 재고,
  모를 때: 모든 질량에 같은 순항 속도(25 kg 기준 × s) 하나
  알 때: 질량마다 순항 속도(그 질량 안전 속도 × s_m)를 따로
로 "평균 충돌 ≤ 한도"를 지키는 가장 짧은 평균 시간을 찾는다. 학습 분포(돌발 확률 0.7, 학습 질량 균등)다.
"""
import itertools
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT))
from check_reward import ScaledRule, episode_return  # noqa: E402
from payload_nav import PayloadNavEnv, load_experiment  # noqa: E402
from payload_nav.seeds import eval_seed  # noqa: E402

LISTS, EPISODES = (100, 101), 150
SCALES = (1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8)


def run(job):
    aware, scale, m = job
    base = load_experiment(ROOT / 'config' / 'env_v0.toml').env
    env = PayloadNavEnv(replace(base, observe_mass=True, mass_hide_prob=0.0))
    pol = ScaledRule(mass_aware=aware, scale=scale)
    coll, times = [], []
    for k in LISTS:
        for ep in range(EPISODES):
            _, o, t = episode_return(env, pol, eval_seed(k, ep), m)
            coll.append(o == 'collision')
            times.append(t if o == 'success' else 30.0)   # 실패는 시간 한도로 친다
    return aware, scale, m, float(np.mean(coll)), float(np.mean(times))


if __name__ == '__main__':
    masses = load_experiment(ROOT / 'config' / 'env_v0.toml').env.train_masses
    jobs = [(a, s, m) for a in (True, False) for s in SCALES for m in masses]
    with Pool(8) as pool:
        res = {(a, s, m): (c, t) for a, s, m, c, t in pool.map(run, jobs)}
    print('질량별 충돌률 / 평균 시간 (알 때 배율):')
    for s in SCALES:
        print(f'  ×{s:.1f} ' + '  '.join(f'{m:g}kg {res[(True, s, m)][0]*100:4.1f}%/{res[(True, s, m)][1]:5.2f}s' for m in masses))
    for limit in (0.01, 0.02, 0.05):
        ag = [(np.mean([res[(False, s, m)][1] for m in masses]), s) for s in SCALES
              if np.mean([res[(False, s, m)][0] for m in masses]) <= limit]
        best_ag = min(ag) if ag else (float('nan'), None)
        best_aw = (float('inf'), None, None)
        for combo in itertools.product(SCALES, repeat=len(masses)):
            c = np.mean([res[(True, s, m)][0] for s, m in zip(combo, masses)])
            if c <= limit:
                t = np.mean([res[(True, s, m)][1] for s, m in zip(combo, masses)])
                if t < best_aw[0]:
                    best_aw = (t, combo, [res[(True, s, m)][0] for s, m in zip(combo, masses)])
        gain = best_ag[0] - best_aw[0]
        print(f'\n충돌 한도 {limit:.0%}: 모를 때 {best_ag[0]:.2f}s (배율 {best_ag[1]}) / 알 때 {best_aw[0]:.2f}s '
              f'(배율 {best_aw[1]}, 질량별 충돌 {[f"{c*100:.1f}%" for c in best_aw[2]]}) → 정보로 {gain:.2f}s ({gain/best_ag[0]*100:.1f}%) 단축')
