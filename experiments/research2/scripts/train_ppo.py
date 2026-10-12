"""연구 2 v0 PPO 학습 (명세서 4-3절).

    uv run python scripts/train_ppo.py --variant proposed --seed 0
    uv run python scripts/train_ppo.py --variant r2 --seed 0 --steps 200000   # 짧게 (파이프라인 확인)
    uv run python scripts/train_ppo.py --variant r2 --config config/env_v0_collision20.toml

비교군·환경·PPO 설정은 설정 파일(기본 config/env_v0.toml)에서 읽는다.
결과: raw/<variant>_s<seed>_<시각>/ 아래 config.toml(쓴 설정 원본), config.json(풀어 쓴 값),
best_model.zip, best_vecnormalize.pkl, final_model.zip, final_vecnormalize.pkl, TensorBoard 로그.
"""

import argparse
from collections import deque
from dataclasses import asdict
import datetime
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback

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
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    run_dir = ROOT / 'raw' / f'{args.variant}_s{args.seed}_{stamp}'
    run_dir.mkdir(parents=True)
    shutil.copyfile(args.config, run_dir / 'config.toml')
    # 학습 환경 시드는 학습 시드끼리 겹치지 않게 따로 둔다 (seeds.py, training.py)
    model, venv, env_seed = make_model(env_config, n_envs, args.seed, ppo, net_arch,
                                       tensorboard_log=str(run_dir / 'tb'))
    (run_dir / 'config.json').write_text(json.dumps(
        {'variant': args.variant, 'args': {**vars(args), 'config': str(args.config)},
         'n_envs': n_envs, 'env_seed_base': env_seed, 'ppo': {**ppo, 'net_arch': net_arch},
         'env': asdict(env_config), 'torch': torch.__version__}, indent=2, ensure_ascii=False))
    callback = SuccessEvalCallback(env_config, venv, run_dir, args.eval_every, args.eval_episodes)
    model.learn(args.steps, callback=[callback, OutcomeLogCallback()])
    model.save(run_dir / 'final_model.zip')
    venv.save(str(run_dir / 'final_vecnormalize.pkl'))
    print(f'저장: {run_dir} (검증 최고 성공률 {callback.best:.2f})')


if __name__ == '__main__':
    main()
