"""학습한 정책을 질량·정보 조건·시나리오별로 평가해 CSV로 남긴다 (명세서 4-5절, 실험 설계 3·5절).

    uv run python scripts/evaluate.py raw/proposed_s0_* raw/r2_s0_*                  # S1·S2 각 200개
    uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs exact plus30 minus30 unknown
    uv run python scripts/evaluate.py raw/proposed_s0_* --scenarios mixed            # 학습과 같은 섞인 분포

같은 시나리오 시드는 질량·비교군·정보 조건과 관계없이 같은 시나리오다. S1과 S2는 같은 시드 번호를
쓰므로 시나리오는 (scenario, scenario_seed)로 구분한다.
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
from payload_nav import (EVAL_SEED_BASE, EnvConfig, PayloadNavEnv, RobotParams,  # noqa: E402
                         ScenarioConfig)
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


# 시나리오 이름 → reset 옵션 popout (S1: 돌발 장애물, S2: 정밀 정지, mixed: 학습과 같이 popout_prob)
SCENARIOS = {'S1': True, 'S2': False, 'mixed': None}
# 정보 조건(실험 설계 3절) → 관측에 넣을 질량 배율. None이면 "모름"(앎 여부 0)
MASS_OBS = {'exact': 1.0, 'plus30': 1.3, 'minus30': 0.7, 'unknown': None}
MASS_OBS_ALIASES = {'true': 'exact', 'none': 'unknown', 'scale:1.3': 'plus30', 'scale:0.7': 'minus30'}


def mass_obs_label(mode: str) -> str:
    label = MASS_OBS_ALIASES.get(mode, mode)
    if label not in MASS_OBS:
        raise ValueError(f'--mass-obs는 {", ".join(MASS_OBS)} 중 하나: {mode}')
    return label


def mass_obs_for(label: str, payload: float):
    scale = MASS_OBS[label]
    return None if scale is None else payload * scale


def rate(values):
    return f'{np.mean(values):5.2f}' if values else f'{"-":>5s}'


def mean_or_dash(values, width):
    return f'{np.mean(values):{width}.2f}' if values else f'{"-":>{width}s}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--masses', type=float, nargs='+', default=[0, 2, 10, 25, 15, 30])
    parser.add_argument('--episodes', type=int, default=200, help='시나리오 종류마다 에피소드 수')
    parser.add_argument('--seed-base', type=int, default=EVAL_SEED_BASE)  # 학습·검증 seed와 겹치지 않게
    parser.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=['S1', 'S2'])
    parser.add_argument('--mass-obs', nargs='+', default=['exact'],
                        help='정보 조건: exact, plus30, minus30, unknown (예전 표기 true, none, '
                             'scale:1.3, scale:0.7도 받는다). 질량을 보지 않는 비교군은 exact로 한 번만 평가')
    parser.add_argument('--checkpoint', choices=['best', 'final'], default='best')
    parser.add_argument('--out', type=Path, default=ROOT / 'raw' / 'eval' / 'episodes.csv')
    args = parser.parse_args()
    labels = list(dict.fromkeys(mass_obs_label(m) for m in args.mass_obs))

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
        # 질량을 보지 않는 비교군(R1·R2)은 정보 조건이 결과를 바꾸지 않는다
        run_labels = labels if env.cfg.observe_mass else ['exact']
        for label in run_labels:
            for scenario in args.scenarios:
                for mass in args.masses:
                    for ep in range(args.episodes):
                        seed = args.seed_base + ep
                        options = {'payload_kg': mass, 'mass_obs_kg': mass_obs_for(label, mass),
                                   'popout': SCENARIOS[scenario]}
                        result = run_episode(model, vecnorm, env, seed, options)
                        rows.append({'run': run.name, 'variant': meta['variant'],
                                     'train_seed': meta['args']['seed'], 'scenario': scenario,
                                     'scenario_seed': seed, 'mass_obs': label,
                                     'mass_obs_kg': options['mass_obs_kg'], 'episode': ep, **result})

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    # 충돌 속도는 충돌한 에피소드, 제동 거리 여유는 장애물 앞에서 멈춘(stopped) 에피소드만 평균 낸다.
    # 돌발 장애물 충돌률의 분모는 장애물이 나타난 에피소드 전체다. 설 수 없던(stoppable 0) 에피소드도
    # 빼지 않는다: 출현 순간 속도는 정책이 고른 것이라, 빼면 너무 빨리 달려 생긴 충돌이 사라진다.
    # "못섬"은 그 충돌 중 나타난 순간 이미 설 수 없던 개수다 (원인 분해용)
    print(f'{"run":36s} {"info":7s} {"sc":5s} {"kg":>4s} {"성공":>5s} {"충돌":>5s} {"초과":>5s} '
          f'{"근접":>5s} {"시간s":>6s} {"충돌v":>6s} {"여유m":>6s} {"멈춤":>4s} {"minTTC":>7s} '
          f'{"출현":>4s} {"장충돌":>5s} {"못섬":>4s}')
    for run in runs:
        for label in dict.fromkeys(r['mass_obs'] for r in rows if r['run'] == run.name):
            for scenario in args.scenarios:
                for mass in args.masses:
                    sel = [r for r in rows if r['run'] == run.name and r['mass_obs'] == label
                           and r['scenario'] == scenario and r['payload_kg'] == mass]
                    impacts = [r['impact_speed'] for r in sel if r['collision']]
                    margins = [r['popout_stop_margin'] for r in sel if r['popout_result'] == 'stopped']
                    ttcs = [r['min_ttc'] for r in sel if np.isfinite(r['min_ttc'])]
                    appeared = [r for r in sel if r['popout_appeared']]
                    hits = [r for r in appeared if r['popout_result'] == 'collision']
                    print(f'{run.name:36s} {label:7s} {scenario:5s} {mass:4.0f} '
                          f'{rate([r["success"] for r in sel])} {rate([r["collision"] for r in sel])} '
                          f'{rate([r["timeout"] for r in sel])} {rate([r["near_miss"] for r in sel])} '
                          f'{np.mean([r["time_s"] for r in sel]):6.2f} {mean_or_dash(impacts, 6)} '
                          f'{mean_or_dash(margins, 6)} {len(margins):4d} {mean_or_dash(ttcs, 7)} '
                          f'{len(appeared):4d} {rate([r["popout_result"] == "collision" for r in appeared])} '
                          f'{sum(not r["popout_stoppable"] for r in hits):4d}')
    print(f'CSV: {args.out}')


if __name__ == '__main__':
    main()
