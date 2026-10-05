"""설치 확인용: CartPole PPO 학습 속도(steps/s)를 잰다.

    uv run python scripts/smoke_ppo.py                # 기본 5만 스텝
    uv run python scripts/smoke_ppo.py --steps 5000   # 짧게 (CI)
"""

import argparse
import os
import time

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=50_000)
    parser.add_argument('--threads', type=int, default=int(os.environ.get('OMP_NUM_THREADS', '2')))
    args = parser.parse_args()

    # 학교 서버는 nproc가 호스트 전체(256)를 보여 줘서 기본값이면 스레드를 수백 개 만들고 오히려 느려진다
    torch.set_num_threads(args.threads)

    model = PPO('MlpPolicy', make_vec_env('CartPole-v1', n_envs=2, seed=0), seed=0, device='cpu')
    start = time.perf_counter()
    model.learn(args.steps)
    elapsed = time.perf_counter() - start
    print(f'torch {torch.__version__}, 스레드 {args.threads}: '
          f'{args.steps / elapsed:.0f} steps/s ({args.steps} steps, {elapsed:.1f}s)')


if __name__ == '__main__':
    main()
