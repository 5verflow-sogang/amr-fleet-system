"""질량 정보의 값어치 (RL 없이): 규칙 정책으로 "질량을 알 때 얻는 최고 보상 − 모를 때 얻는 최고 보상"을 잰다.

모를 때 최고 = 질량 모르는 규칙 정책의 순항 배율을 바꿔 가며 얻은 최고 평균 보상 (R2가 배울 수 있는 상한의 근사).
이 값이 작으면 보상이 질량 정보를 쓸 이유를 주지 않는다 (G 1.8점 / 160점 = 1.1%). 보상 후보는 VARIANTS에 더한다.

    cd experiments/research2 && uv run python scripts/info_value.py
"""
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

LISTS, EPISODES = (100, 101), 100
AWARE_SCALES = (0.9, 1.0, 1.1)
AGNOSTIC_SCALES = (1.0, 1.05, 1.1, 1.15, 1.2, 1.3)
VARIANTS = {
    'G (지금)': {},
    'A2 충돌 −300': {'r_collision': -300.0, 'r_timeout': -75.0},
    'B1 돌발 0.9': {'scenario.popout_prob': 0.9},
    'A2+B1': {'r_collision': -300.0, 'r_timeout': -75.0, 'scenario.popout_prob': 0.9},
    '시간 ×3 (−0.3/스텝)': {'w_time': 0.3},
    '시간 ×5 (−0.5/스텝)': {'w_time': 0.5},
    '시간 ×5 + A2': {'w_time': 0.5, 'r_collision': -300.0, 'r_timeout': -75.0},
    'I (탐색 1)': {'w_time': 0.5, 'r_collision': -1000.0, 'r_timeout': -250.0},
    'I2 (탐색 2)': {'w_time': 0.5, 'r_collision': -1000.0, 'r_timeout': -250.0, 'w_overshoot': 125.0},
}


def make_cfg(over):
    base = load_experiment(ROOT / 'config' / 'env_v0.toml').env
    sc = {k.split('.', 1)[1]: v for k, v in over.items() if k.startswith('scenario.')}
    top = {k: v for k, v in over.items() if not k.startswith('scenario.')}
    return replace(base, scenario=replace(base.scenario, **sc), **top)


def run(job):
    name, aware, scale = job
    cfg = make_cfg(VARIANTS[name])
    env = PayloadNavEnv(replace(cfg, observe_mass=True, mass_hide_prob=0.0))
    pol = ScaledRule(mass_aware=aware, scale=scale)
    per_mass, coll = {}, []
    for m in cfg.train_masses:
        rets = []
        for k in LISTS:
            for ep in range(EPISODES):
                r, outcome, _ = episode_return(env, pol, eval_seed(k, ep), m)
                rets.append(r)
                coll.append(outcome == 'collision')
        per_mass[m] = float(np.mean(rets))
    return name, aware, scale, float(np.mean(list(per_mass.values()))), float(np.mean(coll)), per_mass


if __name__ == '__main__':
    jobs = [(n, True, s) for n in VARIANTS for s in AWARE_SCALES] + \
           [(n, False, s) for n in VARIANTS for s in AGNOSTIC_SCALES]
    with Pool(8) as pool:
        res = pool.map(run, jobs)
    print(f'{"보상·환경":20s} {"앎 최고(배율)":>14s} {"모름 최고(배율, 충돌)":>22s} {"정보 값어치":>10s} {"비율":>6s}  앎이 1.0에서 최고?')
    for n in VARIANTS:
        aw = [r for r in res if r[0] == n and r[1]]
        ag = [r for r in res if r[0] == n and not r[1]]
        ba, bg = max(aw, key=lambda r: r[3]), max(ag, key=lambda r: r[3])
        gap = ba[3] - bg[3]
        print(f'{n:20s} {ba[3]:8.1f} ({ba[2]:.2f}) {bg[3]:8.1f} ({bg[2]:.2f}, {bg[4]*100:4.1f}%) '
              f'{gap:10.1f} {gap/abs(ba[3])*100:5.1f}%  {"예" if ba[2] == 1.0 else "아니오"}')
