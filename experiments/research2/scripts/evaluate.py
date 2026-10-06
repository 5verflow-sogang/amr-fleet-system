"""학습한 정책을 질량별로 평가해 CSV로 남긴다 (명세서 4-5절).

    uv run python scripts/evaluate.py raw/proposed_s0_* raw/r2_s0_* --episodes 200
    uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs scale:1.3   # 질량 정보 +30% 오차
    uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs none        # 질량 정보 없음

같은 에피소드 번호는 질량·비교군과 관계없이 같은 시나리오다 (seed 고정).
"""

import argparse
import csv
from dataclasses import fields
import json
from pathlib import Path
import sys

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_nav import EnvConfig, PayloadNavEnv, RobotParams, ScenarioConfig  # noqa: E402
from payload_nav.evaluation import run_episode  # noqa: E402


def load_env_config(d: dict) -> EnvConfig:
    def tuples(cls, values):
        names = {f.name for f in fields(cls)}
        return cls(**{k: tuple(v) if isinstance(v, list) else v
                      for k, v in values.items() if k in names})
    env = dict(d)
    env['scenario'] = tuples(ScenarioConfig, env['scenario'])
    env['robot'] = tuples(RobotParams, env['robot'])
    return tuples(EnvConfig, env)


def mass_obs_for(mode: str, payload: float):
    if mode == 'true':
        return payload
    if mode == 'none':
        return None  # 질량을 모름 (플래그 0)
    if mode.startswith('scale:'):
        return payload * float(mode.split(':', 1)[1])
    raise ValueError(f'--mass-obs는 true, none, scale:<배율> 중 하나: {mode}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--masses', type=float, nargs='+', default=[0, 2, 10, 25, 15, 30])
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--seed-base', type=int, default=100_000)  # 학습·검증 seed와 겹치지 않게
    parser.add_argument('--mass-obs', default='true')
    parser.add_argument('--checkpoint', choices=['best', 'final'], default='best')
    parser.add_argument('--out', type=Path, default=ROOT / 'raw' / 'eval' / 'episodes.csv')
    args = parser.parse_args()

    rows = []
    runs = [r for r in args.runs if (r / f'{args.checkpoint}_model.zip').exists()]
    for skipped in sorted(set(args.runs) - set(runs)):
        print(f'건너뜀: {skipped} ({args.checkpoint}_model.zip 없음, 학습이 끝나지 않은 실행)')
    if not runs:
        sys.exit('평가할 실행이 없다')
    for run in runs:
        meta = json.loads((run / 'config.json').read_text())
        env = PayloadNavEnv(load_env_config(meta['env']))
        model = PPO.load(run / f'{args.checkpoint}_model.zip', device='cpu')
        vecnorm = VecNormalize.load(str(run / f'{args.checkpoint}_vecnormalize.pkl'),
                                    DummyVecEnv([lambda: PayloadNavEnv(env.cfg)]))
        vecnorm.training = False
        for mass in args.masses:
            for ep in range(args.episodes):
                options = {'payload_kg': mass, 'mass_obs_kg': mass_obs_for(args.mass_obs, mass)}
                result = run_episode(model, vecnorm, env, args.seed_base + ep, options)
                rows.append({'run': run.name, 'variant': meta['variant'], 'mass_obs': args.mass_obs,
                             'episode': ep, **result})

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    # 충돌 속도는 충돌한 에피소드, 제동 거리 여유는 장애물 앞에서 멈춘(stopped) 에피소드만 평균 낸다
    print(f'{"run":36s} {"kg":>4s} {"성공":>5s} {"충돌":>5s} {"초과":>5s} {"근접":>5s} '
          f'{"시간s":>6s} {"충돌v":>6s} {"여유m":>6s} {"멈춤":>4s} {"minTTC":>7s}')

    def mean_or_dash(values, width):
        return f'{np.mean(values):{width}.2f}' if values else f'{"-":>{width}s}'

    for run in runs:
        for mass in args.masses:
            sel = [r for r in rows if r['run'] == run.name and r['payload_kg'] == mass]
            impacts = [r['impact_speed'] for r in sel if r['collision']]
            margins = [r['popout_stop_margin'] for r in sel if r['popout_result'] == 'stopped']
            ttcs = [r['min_ttc'] for r in sel if np.isfinite(r['min_ttc'])]
            print(f'{run.name:36s} {mass:4.0f} {np.mean([r["success"] for r in sel]):5.2f} '
                  f'{np.mean([r["collision"] for r in sel]):5.2f} '
                  f'{np.mean([r["timeout"] for r in sel]):5.2f} '
                  f'{np.mean([r["near_miss"] for r in sel]):5.2f} '
                  f'{np.mean([r["time_s"] for r in sel]):6.2f} {mean_or_dash(impacts, 6)} '
                  f'{mean_or_dash(margins, 6)} {len(margins):4d} {mean_or_dash(ttcs, 7)}')
    print(f'CSV: {args.out}')


if __name__ == '__main__':
    main()
