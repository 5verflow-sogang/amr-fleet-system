"""학습한 정책을 질량·정보 조건·시나리오별로 평가해 CSV로 남긴다 (명세서 4-5절, 실험 설계 3·5절).

    uv run python scripts/evaluate.py raw/proposed_s0_* raw/r2_s0_*                  # S1·S2 각 200개
    uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs exact plus30 minus30 unknown
    uv run python scripts/evaluate.py raw/proposed_s0_* --scenarios mixed            # 학습과 같은 섞인 분포
    uv run python scripts/evaluate.py --rule mass_agnostic mass_aware --eval-list 100   # 규칙 정책 (참고 기준선)
    uv run python scripts/evaluate.py --dwa b1 b2 --eval-list 0 --mass-obs exact plus30 minus30 unknown  # DWA

결과: raw/eval/<시각>/episodes.csv와 meta.json (실행마다 새 폴더라 앞 결과를 덮어쓰지 않는다).

평가 시나리오는 학습 시드 k마다 다른 목록 k를 쓴다 (seeds.py). 같은 시나리오 시드는 질량·비교군·정보
조건과 관계없이 같은 시나리오다. S1과 S2는 같은 시드 번호를 쓰므로 시나리오는 (scenario,
scenario_seed)로 구분한다.

--rule은 학습 없이 규칙 정책(payload_nav/rule_policy.py, 참고 기준선)을 같은 시나리오·열로 평가한다.
학습 시드가 없으므로 --eval-list가 필요하다. 환경 설정은 --config(기본 config/env_v0.toml)의 [env]다.
run·variant 열은 rule_agnostic / rule_aware다.

--dwa는 DWA 비교군 B1·B2(payload_nav/dwa_policy.py, #41 C++ 원본을 그대로 부름)를 같은 방식으로 평가한다.
--rule처럼 --eval-list가 필요하고, 시드 k의 RL과 짝지으려면 목록 k로 평가한다 (실험 설계 2절). run·variant 열은
dwa_b1 / dwa_b2, variant 열은 b1 / b2다(analyze.py가 이 이름으로 H1·H2를 판정한다). B1은 질량을 보지 않으므로 정보 조건을 exact로 한 번만 평가한다.
"""

import argparse
import csv
from dataclasses import asdict, fields, replace
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np
import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_nav import (EnvConfig, PayloadNavEnv, RobotParams, ScenarioConfig,  # noqa: E402
                         eval_seed, load_experiment)
from payload_nav.seeds import EVAL_LIST_STRIDE, EVAL_SEED_BASE  # noqa: E402
from payload_nav.evaluation import run_policy_episode  # noqa: E402
from payload_nav.dwa_policy import DWA_VARIANTS, DwaPolicy  # noqa: E402
from payload_nav.rule_policy import RULE_VARIANTS, RulePolicy  # noqa: E402


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


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_info() -> dict:
    """평가에 쓴 코드 (하네스 meta.json의 git_sha와 같은 뜻). 커밋 안 된 변경이 있으면 dirty."""
    def git(*cmd):
        out = subprocess.run(['git', *cmd], cwd=ROOT, capture_output=True, text=True)
        return out.stdout.strip() if out.returncode == 0 else None
    status = git('status', '--porcelain', '--', '.')
    dirty = bool(status) if status is not None else None
    return {'git_sha': git('rev-parse', 'HEAD'), 'git_dirty': dirty}


def rate(values):
    return f'{np.mean(values):5.2f}' if values else f'{"-":>5s}'


def mean_or_dash(values, width):
    return f'{np.mean(values):{width}.2f}' if values else f'{"-":>{width}s}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='*', type=Path, help='학습 실행 폴더 (raw/<variant>_s<seed>_<시각>)')
    parser.add_argument('--rule', nargs='+', choices=list(RULE_VARIANTS), default=[],
                        help='규칙 정책(참고 기준선)도 평가한다. --eval-list가 필요하다')
    parser.add_argument('--dwa', nargs='+', choices=list(DWA_VARIANTS), default=[],
                        help='DWA 비교군(B1·B2)도 평가한다. --eval-list가 필요하다')
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'env_v0.toml',
                        help='--rule·--dwa가 쓰는 환경 설정 ([env])')
    parser.add_argument('--masses', type=float, nargs='+', default=[0, 2, 10, 25, 15, 30])
    parser.add_argument('--episodes', type=int, default=200, help='시나리오 종류마다 에피소드 수')
    parser.add_argument('--eval-list', type=int, default=None,
                        help='평가 목록 번호. 기본은 실행의 학습 시드 (시드 k ↔ 목록 k). DWA처럼 시드가 '
                             '없는 비교군을 시드 k와 짝지을 때 쓴다')
    parser.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=['S1', 'S2'])
    parser.add_argument('--mass-obs', nargs='+', default=['exact'],
                        help='정보 조건: exact, plus30, minus30, unknown (예전 표기 true, none, '
                             'scale:1.3, scale:0.7도 받는다). 질량을 보지 않는 비교군은 exact로 한 번만 평가')
    parser.add_argument('--checkpoint', choices=['best', 'final'], default='best')
    parser.add_argument('--out-dir', type=Path, default=None,
                        help='결과 폴더. 기본 raw/eval/<시각>. 이미 episodes.csv가 있으면 멈춘다')
    args = parser.parse_args()
    labels = list(dict.fromkeys(mass_obs_label(m) for m in args.mass_obs))
    if not 0 < args.episodes <= EVAL_LIST_STRIDE:
        sys.exit(f'--episodes는 1~{EVAL_LIST_STRIDE}여야 한다 (평가 목록 하나의 크기)')
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out_dir = args.out_dir or ROOT / 'raw' / 'eval' / stamp
    if (out_dir / 'episodes.csv').exists():
        sys.exit(f'이미 결과가 있다: {out_dir / "episodes.csv"} (다른 --out-dir을 쓴다)')

    if (args.rule or args.dwa) and args.eval_list is None:
        sys.exit('--rule·--dwa는 학습 시드가 없어 --eval-list가 필요하다 (튜닝은 100번대, 0~9는 최종 평가용)')

    rows = []
    runs = [r for r in args.runs if (r / f'{args.checkpoint}_model.zip').exists()]
    for skipped in sorted(set(args.runs) - set(runs)):
        print(f'건너뜀: {skipped} ({args.checkpoint}_model.zip 없음, 학습이 끝나지 않은 실행)')
    if not runs and not args.rule and not args.dwa:
        sys.exit('평가할 실행이 없다')
    run_meta = []
    targets = []   # (run 이름, variant, 학습 시드, 평가 목록, env, act(obs), 정보 조건 목록)
    for run in runs:
        meta = json.loads((run / 'config.json').read_text())
        train_seed = meta['args']['seed']
        eval_list = train_seed if args.eval_list is None else args.eval_list
        env = PayloadNavEnv(load_env_config(meta['env']))
        model = PPO.load(run / f'{args.checkpoint}_model.zip', device='cpu')
        vecnorm = VecNormalize.load(str(run / f'{args.checkpoint}_vecnormalize.pkl'),
                                    DummyVecEnv([lambda: PayloadNavEnv(env.cfg)]))
        vecnorm.training = False
        # 질량을 보지 않는 비교군(R1·R2)은 정보 조건이 결과를 바꾸지 않는다
        run_labels = labels if env.cfg.observe_mass else ['exact']
        ckpt = run / f'{args.checkpoint}_model.zip'
        run_meta.append({
            'run': str(run), 'variant': meta['variant'], 'train_seed': train_seed,
            'env_seed_base': meta.get('env_seed_base'), 'mass_obs': run_labels,
            'eval_list': eval_list,
            'scenario_seeds': [eval_seed(eval_list, 0), eval_seed(eval_list, args.episodes - 1)],
            'checkpoint': str(ckpt), 'checkpoint_sha256': sha256(ckpt),
            'vecnormalize_sha256': sha256(run / f'{args.checkpoint}_vecnormalize.pkl'),
            'config_toml_sha256': sha256(run / 'config.toml'),
        })

        def act(obs, model=model, vecnorm=vecnorm):
            return model.predict(vecnorm.normalize_obs(obs), deterministic=True)[0]
        targets.append((run.name, meta['variant'], train_seed, eval_list, env, act, run_labels))
    if args.rule or args.dwa:
        base = load_experiment(args.config).env
    for rule in args.rule:
        policy = RulePolicy(mass_aware=rule == 'mass_aware')
        # observe_mass는 mass_known 열에만 쓰인다 (규칙 정책은 관측 벡터를 읽지 않는다)
        env = PayloadNavEnv(replace(base, observe_mass=policy.mass_aware))
        # 질량을 보지 않는 규칙은 정보 조건이 결과를 바꾸지 않는다
        run_labels = labels if policy.mass_aware else ['exact']
        run_meta.append({
            'run': policy.name, 'variant': policy.name, 'train_seed': None, 'mass_obs': run_labels,
            'eval_list': args.eval_list,
            'scenario_seeds': [eval_seed(args.eval_list, 0), eval_seed(args.eval_list, args.episodes - 1)],
            'rule': {'name': rule, **asdict(policy)},
            'config': str(args.config), 'config_toml_sha256': sha256(args.config),
        })
        targets.append((policy.name, policy.name, None, args.eval_list, env,
                        lambda obs, policy=policy, env=env: policy(env), run_labels))
    for variant in args.dwa:
        policy = DwaPolicy(variant=variant)
        env = PayloadNavEnv(replace(base, observe_mass=policy.mass_aware))
        run_labels = labels if policy.mass_aware else ['exact']   # B1은 질량을 보지 않는다
        run_meta.append({
            'run': policy.name, 'variant': variant, 'train_seed': None, 'mass_obs': run_labels,
            'eval_list': args.eval_list,
            'scenario_seeds': [eval_seed(args.eval_list, 0), eval_seed(args.eval_list, args.episodes - 1)],
            'dwa': {'variant': variant, **asdict(policy.params)},
            'config': str(args.config), 'config_toml_sha256': sha256(args.config),
        })
        # variant 열은 명세 비교군 이름 b1·b2 (analyze.py가 이 이름으로 H1·H2를 판정한다), run 열은 dwa_b1·dwa_b2
        targets.append((policy.name, variant, None, args.eval_list, env,
                        lambda obs, policy=policy, env=env: policy(env), run_labels))

    for name, variant, train_seed, eval_list, env, act, run_labels in targets:
        for label in run_labels:
            for scenario in args.scenarios:
                for mass in args.masses:
                    for ep in range(args.episodes):
                        seed = eval_seed(eval_list, ep)
                        options = {'payload_kg': mass, 'mass_obs_kg': mass_obs_for(label, mass),
                                   'popout': SCENARIOS[scenario]}
                        result = run_policy_episode(act, env, seed, options)
                        rows.append({'run': name, 'variant': variant,
                                     'train_seed': train_seed, 'eval_list': eval_list,
                                     'scenario': scenario, 'scenario_seed': seed, 'mass_obs': label,
                                     'mass_obs_kg': options['mass_obs_kg'], 'episode': ep, **result})

    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / 'episodes.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    # 결과를 나중에 추적·재현하는 데 필요한 정보 (하네스 meta.json과 같은 방식)
    (out_dir / 'meta.json').write_text(json.dumps({
        'created': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        **git_info(),
        'command': sys.argv,
        'eval': {'episodes': args.episodes, 'masses': args.masses, 'scenarios': args.scenarios,
                 'mass_obs': labels, 'checkpoint': args.checkpoint,
                 'eval_seed_base': EVAL_SEED_BASE, 'eval_list_stride': EVAL_LIST_STRIDE,
                 'eval_list_override': args.eval_list, 'rules': args.rule,
                 'dwa': args.dwa},
        'runs': run_meta,
        'host': {'machine': platform.machine(), 'processor': platform.processor() or None,
                 'cpu_count': os.cpu_count(), 'torch_threads': torch.get_num_threads()},
        'versions': {'python': platform.python_version(), 'torch': torch.__version__,
                     'stable_baselines3': stable_baselines3.__version__,
                     'numpy': np.__version__},
    }, indent=2, ensure_ascii=False))

    # 충돌 속도는 충돌한 에피소드, 제동 거리 여유는 장애물 앞에서 멈춘(stopped) 에피소드만 평균 낸다.
    # 돌발 장애물 충돌률의 분모는 장애물이 나타난 에피소드 전체다. 설 수 없던(stoppable 0) 에피소드도
    # 빼지 않는다: 출현 순간 속도는 정책이 고른 것이라, 빼면 너무 빨리 달려 생긴 충돌이 사라진다.
    # "못섬"은 그 충돌 중 나타난 순간 이미 설 수 없던 개수다 (원인 분해용)
    print(f'{"run":36s} {"info":7s} {"sc":5s} {"kg":>4s} {"성공":>5s} {"충돌":>5s} {"초과":>5s} '
          f'{"근접":>5s} {"시간s":>6s} {"충돌v":>6s} {"여유m":>6s} {"멈춤":>4s} {"minTTC":>7s} '
          f'{"출현":>4s} {"장충돌":>5s} {"못섬":>4s}')
    for name, *_ in targets:
        for label in dict.fromkeys(r['mass_obs'] for r in rows if r['run'] == name):
            for scenario in args.scenarios:
                for mass in args.masses:
                    sel = [r for r in rows if r['run'] == name and r['mass_obs'] == label
                           and r['scenario'] == scenario and r['payload_kg'] == mass]
                    impacts = [r['impact_speed'] for r in sel if r['collision']]
                    margins = [r['popout_stop_margin'] for r in sel if r['popout_result'] == 'stopped']
                    ttcs = [r['min_ttc'] for r in sel if np.isfinite(r['min_ttc'])]
                    appeared = [r for r in sel if r['popout_appeared']]
                    hits = [r for r in appeared if r['popout_result'] == 'collision']
                    print(f'{name:36s} {label:7s} {scenario:5s} {mass:4.0f} '
                          f'{rate([r["success"] for r in sel])} {rate([r["collision"] for r in sel])} '
                          f'{rate([r["timeout"] for r in sel])} {rate([r["near_miss"] for r in sel])} '
                          f'{np.mean([r["time_s"] for r in sel]):6.2f} {mean_or_dash(impacts, 6)} '
                          f'{mean_or_dash(margins, 6)} {len(margins):4d} {mean_or_dash(ttcs, 7)} '
                          f'{len(appeared):4d} {rate([r["popout_result"] == "collision" for r in appeared])} '
                          f'{sum(not r["popout_stoppable"] for r in hits):4d}')
    print(f'결과: {out_dir / "episodes.csv"}, {out_dir / "meta.json"}')


if __name__ == '__main__':
    main()
