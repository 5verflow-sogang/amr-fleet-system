"""v3 정보 실험 ③ 반응 곡선: 같은 상태에서 질량 칸만 바꿨을 때 제안 정책의 속도 명령이 어떻게 변하나.

    uv run python scripts/probe_v3_mass_response.py raw/v3_confirm/runs.txt --out raw/v3_info

상태는 장애물 없는 S2에서 정책이 정확한 질량(10 kg)을 보고 달리는 동안, 목표까지 6 m 넘게 남은 순항 구간에서
에피소드마다 3개(1.0·2.0·3.0 s)를 뽑는다. 그 상태의 관측에서 질량 칸 두 개(제동 능력 비율, 앎 여부)만 바꿔
결정적 정책의 v_cmd를 잰다 (질량 0~30 kg, 그리고 "모름" (0, 0)). 오라클 기준은 규칙 정책의 안전 순항 속도
safe_cruise_speed(m)이다. 출력: <out>/mass_response.csv (시드·질량별 평균 v_cmd), mass_response.png
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from evaluate import eval_env_config  # noqa: E402
from payload_nav import PayloadNavEnv, eval_seed, load_experiment, max_linear_accel  # noqa: E402
from payload_nav.rule_policy import safe_cruise_speed  # noqa: E402

GRID = [0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0]
STATE_TIMES = (1.0, 2.0, 3.0)
EPISODES = 40


def collect_states(env, act, seed_list):
    states = []
    for ep in range(EPISODES):
        obs, _ = env.reset(seed=eval_seed(seed_list, ep), options={'payload_kg': 10.0, 'mass_obs_kg': 10.0, 'popout': False})
        while True:
            if any(abs(env.t - t) < 1e-6 for t in STATE_TIMES) and env.goal[0] - env.state.x > 6.0:
                states.append(obs.copy())
            obs, _, term, trunc, _ = env.step(act(obs))
            if term or trunc:
                break
    return np.array(states)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', type=Path)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    base = load_experiment(ROOT / 'config' / 'env_v0.toml').env
    rows = []
    for line in args.runs.read_text().splitlines():
        cand, variant, seed, run = line.split()
        if variant != 'proposed':
            continue
        meta = json.loads((Path(run) / 'config.json').read_text())
        cfg, _ = eval_env_config(meta['env'], base)
        # 질량 칸이 제동 능력 비율로 학습된 제안 정책이어야 아래에서 바꿔 넣는 값의 뜻이 맞는다
        assert cfg.observe_mass and cfg.mass_obs_mode == 'brake_ratio', f'{run}: 질량 칸 정의가 brake_ratio가 아니다'
        env = PayloadNavEnv(cfg)
        model = PPO.load(Path(run) / 'best_model.zip', device='cpu')
        vn = VecNormalize.load(str(Path(run) / 'best_vecnormalize.pkl'), DummyVecEnv([lambda: PayloadNavEnv(cfg)]))
        act = lambda o: model.predict(vn.normalize_obs(o), deterministic=True)[0]  # noqa: E731
        states = collect_states(env, act, int(seed))
        a0 = max_linear_accel(0.0, cfg.robot)
        for m in GRID + [None]:
            x = states.copy()
            x[:, -2] = 0.0 if m is None else max_linear_accel(m, cfg.robot) / a0   # 제동 능력 비율 (brake_ratio)
            x[:, -1] = 0.0 if m is None else 1.0                                    # 앎 여부
            a = model.predict(vn.normalize_obs(x), deterministic=True)[0]
            v_cmd = 0.5 * (np.clip(a[:, 0], -1, 1) + 1.0) * cfg.robot.v_max
            rows.append({'seed': int(seed), 'mass': 'unknown' if m is None else m, 'v_cmd': float(v_cmd.mean()), 'n_states': len(x)})
        print(f'시드 {seed}: 상태 {len(states)}개')
    with open(args.out / 'mass_response.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    # 그림: 0 kg 대비 비율 (시드별로 나눈 뒤 평균, 95% CI)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = 'AppleGothic'
    plt.rcParams['axes.unicode_minus'] = False
    seeds = sorted({r['seed'] for r in rows})
    by = {(r['seed'], r['mass']): r['v_cmd'] for r in rows}
    rng = np.random.default_rng(0)
    ratio = np.array([[by[(s, m)] / by[(s, 0.0)] for m in GRID] for s in seeds])
    unk = np.array([by[(s, 'unknown')] / by[(s, 0.0)] for s in seeds])
    boot = np.array([ratio[rng.integers(0, len(seeds), len(seeds))].mean(0) for _ in range(2000)])
    lo, hi = np.percentile(boot, [2.5, 97.5], axis=0)
    oracle = [safe_cruise_speed(m) / safe_cruise_speed(0.0) for m in GRID]
    fig, ax = plt.subplots(figsize=(7, 4.6))
    fig.patch.set_facecolor('#fcfcfb')
    ax.set_facecolor('#fcfcfb')
    ax.fill_between(GRID, lo, hi, color='#2a78d6', alpha=0.18, lw=0)
    ax.plot(GRID, ratio.mean(0), '-o', color='#2a78d6', lw=2, ms=7, mec='#fcfcfb', mew=2, label='제안 정책의 속도 명령')
    ax.plot(GRID, oracle, '--s', color='#52514e', lw=1.6, ms=6, label='물리적 안전 속도 (오라클)')
    ax.axhline(unk.mean(), color='#eb6834', ls=':', lw=1.8, label=f'무게 "모름"일 때 ({unk.mean():.3f})')
    for xv in (15.0, 30.0):
        ax.axvline(xv, color='#e7e6e1', lw=6, zorder=0)
    ax.text(15.0, 0.995, '학습 때 안 본 질량', ha='center', va='top', fontsize=8, color='#8a8983',
            transform=ax.get_xaxis_transform())
    ax.set_xlabel('관측에 넣은 질량 (kg)', color='#52514e')
    ax.set_ylabel('속도 명령 (0 kg일 때 = 1)', color='#52514e')
    ax.set_title(f'같은 상태에서 질량 칸만 바꿨을 때 (제안 정책 {len(seeds)}개, 띠는 95% CI)', fontsize=11, loc='left', color='#21211f')
    ax.grid(True, color='#e7e6e1', lw=0.8)
    ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    ax.legend(frameon=False, fontsize=9)
    plt.tight_layout()
    plt.savefig(args.out / 'mass_response.png', dpi=150, facecolor='#fcfcfb')
    print('비율 (시드 평균):', dict(zip(GRID, np.round(ratio.mean(0), 3))), '모름:', round(float(unk.mean()), 3))


if __name__ == '__main__':
    main()
