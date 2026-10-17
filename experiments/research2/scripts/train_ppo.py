"""연구 2 v0 PPO 학습 (명세서 4-3절).

    uv run python scripts/train_ppo.py --variant proposed --seed 0
    uv run python scripts/train_ppo.py --variant r2 --seed 0 --steps 200000   # 짧게 (파이프라인 확인)
    uv run python scripts/train_ppo.py --variant r2 --config config/env_v0_collision20.toml
    uv run python scripts/train_ppo.py --variant r2 --config config/env_v2_b1.toml --val-config config/env_v0.toml

비교군·환경·PPO 설정은 설정 파일(기본 config/env_v0.toml)에서 읽는다.
결과: raw/<variant>_s<seed>[_<tag>]_<시각>/ 아래 config.toml(쓴 설정 원본), config.json(풀어 쓴 값),
best_model.zip, best_vecnormalize.pkl, final_model.zip, final_vecnormalize.pkl, TensorBoard 로그.

--val-config를 주면 검증(eval/*: 체크포인트 선택, eval_common/*)의 시나리오 분포([env.scenario])를 그 파일
것으로 쓴다. 학습 시나리오만 바꾼 후보가 기준선과 같은 시나리오로 체크포인트를 고르게 하려는 것이다.
안 주면 학습 시나리오 그대로다. 평가(evaluate.py)는 늘 --config의 시나리오를 쓴다.
"""

import argparse
from collections import deque
import copy
from dataclasses import asdict, replace
import datetime
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import VecNormalize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_nav import VAL_SEED_BASE, PayloadNavEnv, env_config_for, load_experiment  # noqa: E402
from payload_nav.evaluation import run_episode  # noqa: E402
from payload_nav.training import make_model  # noqa: E402


class SuccessEvalCallback(BaseCallback):
    """학습 질량만으로 구성한 검증 환경의 성공률로 체크포인트를 고른다 (학습 보상 아님)."""

    def __init__(self, env_config, vecnorm, run_dir, every_steps, episodes, seed_base=VAL_SEED_BASE):
        super().__init__()
        self.env = PayloadNavEnv(env_config)
        self.vecnorm, self.run_dir = vecnorm, run_dir
        self.every_steps, self.episodes, self.seed_base = every_steps, episodes, seed_base
        self.last_eval, self.best = 0, -1.0

    def _on_step(self) -> bool:
        if self.num_timesteps - self.last_eval < self.every_steps:
            return True
        self.last_eval = self.num_timesteps
        results = [run_episode(self.model, self.vecnorm, self.env, self.seed_base + i)
                   for i in range(self.episodes)]
        success = float(np.mean([r['success'] for r in results]))
        self.logger.record('eval/success_rate', success)
        self.logger.record('eval/collision_rate', float(np.mean([r['collision'] for r in results])))
        self.logger.record('eval/mean_time_s', float(np.mean([r['time_s'] for r in results])))
        self.logger.record('eval/timeout_rate', float(np.mean([r['outcome'] == 'timeout' for r in results])))
        if success > self.best:
            self.best = success
            self.model.save(self.run_dir / 'best_model.zip')
            self.vecnorm.save(str(self.run_dir / 'best_vecnormalize.pkl'))
        return True


class CommonEvalCallback(BaseCallback):
    """모든 비교군이 같은 조건으로 재는 곡선 비교용 검증 (eval_common/*). 체크포인트 선택에는 쓰지 않는다.

    eval/*는 비교군의 학습 질량으로 재서(R1은 0 kg만) 비교군끼리 나란히 놓으면 안 된다 (README "학습 곡선 비교").
    여기서는 검증 시나리오 하나를 학습 질량마다 한 번씩 잰다(에피소드 i → 시나리오 i // 개수, 질량 i % 개수).
    질량마다 다른 시나리오를 주면 돌발 장애물 비율 등이 질량마다 달라져(200회에서 0.66~0.78) 질량별 속도·충돌에 섞인다.
    질량을 지정하면 학습용 질량 가림이 꺼지므로 제안 방법은 정확한 질량을, R1·R2는 원래대로 "모름"을 본다.
    질량별 평균 속도와 가장 무거운 질량의 충돌률로 "무거우면 천천히"가 학습 중에 생기는지 본다.
    """

    def __init__(self, env_config, vecnorm, every_steps, episodes, seed_base=VAL_SEED_BASE):
        super().__init__()
        self.env = PayloadNavEnv(env_config)
        self.masses = tuple(env_config.train_masses)
        self.vecnorm, self.every_steps, self.episodes, self.seed_base = vecnorm, every_steps, episodes, seed_base
        self.last_eval = 0

    def evaluate(self, model) -> dict:
        n = len(self.masses)
        results = [run_episode(model, self.vecnorm, self.env, self.seed_base + i // n,
                               {'payload_kg': self.masses[i % n]})
                   for i in range(self.episodes)]
        out = {'success_rate': float(np.mean([r['success'] for r in results])),
               'collision_rate': float(np.mean([r['collision'] for r in results])),
               'timeout_rate': float(np.mean([r['outcome'] == 'timeout' for r in results])),
               'mean_time_s': float(np.mean([r['time_s'] for r in results]))}
        heaviest = max(self.masses)
        for m in self.masses:
            rs = [r for r in results if r['payload_kg'] == m]
            out[f'speed_{m:g}kg'] = float(np.mean([r['mean_speed'] for r in rs]))
        out[f'collision_{heaviest:g}kg'] = float(np.mean([r['collision'] for r in results
                                                         if r['payload_kg'] == heaviest]))
        return out

    def _on_step(self) -> bool:
        if self.num_timesteps - self.last_eval < self.every_steps:
            return True
        self.last_eval = self.num_timesteps
        for key, value in self.evaluate(self.model).items():
            self.logger.record(f'eval_common/{key}', value)
        return True


class OutcomeLogCallback(BaseCallback):
    """학습 중 끝난 에피소드의 결과 비율(최근 window개)을 롤아웃마다 기록한다 (outcome/*).

    검증 평가(eval/*)는 정확하지만 듬성듬성하고, 학습 중 성공률만으로는 왜 실패했는지가 곡선에 남지 않는다.
    학습 중 결과라 무작위 행동이 섞여 있다.
    """

    OUTCOMES = ('success', 'collision', 'timeout', 'overshoot')

    def __init__(self, window=200):
        super().__init__()
        self.recent = deque(maxlen=window)

    def _on_step(self) -> bool:
        for done, info in zip(self.locals['dones'], self.locals['infos']):
            if done:
                self.recent.append(info.get('outcome'))
        return True

    def _on_rollout_end(self) -> None:
        if self.recent:
            for outcome in self.OUTCOMES:
                self.logger.record(f'outcome/{outcome}_rate',
                                   sum(o == outcome for o in self.recent) / len(self.recent))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', required=True, help='설정 파일의 [variants.*] 이름 (r1, r2, proposed)')
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'env_v0.toml')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--steps', type=int, default=1_000_000)
    parser.add_argument('--n-envs', type=int, default=None, help='기본은 설정 파일 [ppo] n_envs')
    parser.add_argument('--eval-every', type=int, default=50_000)
    parser.add_argument('--eval-episodes', type=int, default=100)
    parser.add_argument('--common-eval-episodes', type=int, default=200,
                        help='비교군 공통 검증(eval_common/*) 횟수. 학습 질량에 고르게 나눈다. 0이면 끈다')
    parser.add_argument('--tag', default='',
                        help='결과 폴더 이름에 붙일 후보 이름 (raw/<variant>_s<seed>_<tag>_<시각>). 같은 비교군·시드를 '
                             '설정만 바꿔 동시에 돌릴 때 구분용')
    parser.add_argument('--val-config', type=Path, default=None,
                        help='검증(eval/*·eval_common/*) 시나리오([env.scenario])를 이 파일 것으로. 기본은 학습 시나리오')
    parser.add_argument('--init-from', type=Path, default=None,
                        help='이어 학습: 이 실행 폴더의 best_model(신경망 가중치)·best_vecnormalize(관측 정규화)에서 시작한다. '
                             '보상 정규화와 옵티마이저는 새로 시작한다 (보상을 바꿔 이어 학습할 때)')
    parser.add_argument('--threads', type=int, default=int(os.environ.get('OMP_NUM_THREADS', '2')))
    args = parser.parse_args()
    torch.set_num_threads(args.threads)

    experiment = load_experiment(args.config)
    env_config = env_config_for(experiment, args.variant)
    ppo = dict(experiment.ppo)
    n_envs = ppo.pop('n_envs', 8)
    if args.n_envs:
        n_envs = args.n_envs
    net_arch = list(ppo.pop('net_arch', [64, 64]))
    # 검증 환경: 시나리오만 --val-config 것, 나머지(관측·보상·질량)는 학습과 같다
    val_config = env_config
    if args.val_config:
        val_config = replace(env_config, scenario=load_experiment(args.val_config).env.scenario)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    tag = f'_{args.tag}' if args.tag else ''
    # 같은 이름(같은 초에 시작한 같은 비교군·시드)이 이미 있으면 번호를 붙인다. 동시 실행이 서로 덮어쓰지 않게
    run_dir = ROOT / 'raw' / f'{args.variant}_s{args.seed}{tag}_{stamp}'
    for n in range(2, 100):
        try:
            run_dir.mkdir(parents=True)
            break
        except FileExistsError:
            run_dir = ROOT / 'raw' / f'{args.variant}_s{args.seed}{tag}_{stamp}-{n}'
    else:
        sys.exit(f'결과 폴더를 만들지 못했다: {run_dir}')
    shutil.copyfile(args.config, run_dir / 'config.toml')
    # 학습 환경 시드는 학습 시드끼리 겹치지 않게 따로 둔다 (seeds.py, training.py)
    model, venv, env_seed = make_model(env_config, n_envs, args.seed, ppo, net_arch,
                                       tensorboard_log=str(run_dir / 'tb'))
    if args.init_from:
        # 신경망 구조는 설정 파일(net_arch)이 같아야 한다. 행동 표준편차(log_std)도 가중치라 같이 온다
        model.policy.load_state_dict(PPO.load(args.init_from / 'best_model.zip', device='cpu').policy.state_dict())
        old = VecNormalize.load(str(args.init_from / 'best_vecnormalize.pkl'), venv.venv)
        venv.obs_rms = copy.deepcopy(old.obs_rms)
    (run_dir / 'config.json').write_text(json.dumps(
        {'variant': args.variant,
         'args': {**vars(args), 'config': str(args.config),
                  'val_config': str(args.val_config) if args.val_config else None,
                  'init_from': str(args.init_from) if args.init_from else None},
         'n_envs': n_envs, 'env_seed_base': env_seed, 'ppo': {**ppo, 'net_arch': net_arch},
         'env': asdict(env_config), 'val_scenario': asdict(val_config.scenario),
         'torch': torch.__version__}, indent=2, ensure_ascii=False))
    callback = SuccessEvalCallback(val_config, venv, run_dir, args.eval_every, args.eval_episodes)
    callbacks = [callback, OutcomeLogCallback()]
    if args.common_eval_episodes > 0:
        callbacks.append(CommonEvalCallback(val_config, venv, args.eval_every, args.common_eval_episodes))
    model.learn(args.steps, callback=callbacks)
    model.save(run_dir / 'final_model.zip')
    venv.save(str(run_dir / 'final_vecnormalize.pkl'))
    print(f'저장: {run_dir} (검증 최고 성공률 {callback.best:.2f})')


if __name__ == '__main__':
    main()
