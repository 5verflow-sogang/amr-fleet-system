"""학습 환경과 PPO 모델 만들기 (scripts/train_ppo.py와 테스트가 같이 쓴다).

stable-baselines3를 import하므로 payload_nav/__init__.py에서 내보내지 않는다. 환경만 쓰는 코드가
SB3·torch를 불러오지 않게 하기 위해서다.
"""

from typing import Optional, Sequence, Tuple

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from .config import ConstraintConfig
from .env import EnvConfig, PayloadNavEnv
from .lagrangian import PPOLagrangian
from .seeds import train_env_seed


def make_model(env_config: EnvConfig, n_envs: int, train_seed: int, ppo_kwargs: dict,
               net_arch: Sequence[int], tensorboard_log: Optional[str] = None,
               verbose: int = 1, constraint: Optional[ConstraintConfig] = None) -> Tuple[PPO, VecNormalize, int]:
    """병렬 학습 환경(VecNormalize)과 PPO 모델을 만든다. 반환: (모델, 환경, 학습 환경 시드).

    ppo_kwargs의 lr_schedule: "constant"(기본) 또는 "linear"(learning_rate에서 학습 끝에 0까지 선형 감소).
    constraint가 있으면(설정 파일 [constraint], v3) PPO-Lagrangian, 없으면 일반 PPO다. 보상 정규화(norm_reward)는
    보상에만 걸리고 비용(info['cost'])은 그대로 쓴다.
    """
    ppo_kwargs = dict(ppo_kwargs)
    schedule = ppo_kwargs.pop('lr_schedule', 'constant')
    if schedule == 'linear':
        lr0 = ppo_kwargs.get('learning_rate', 3e-4)
        # SB3는 남은 학습 비율(1 → 0)을 넘긴다
        ppo_kwargs['learning_rate'] = lambda progress_remaining: lr0 * progress_remaining
    elif schedule != 'constant':
        raise ValueError(f'lr_schedule은 constant 또는 linear여야 한다: {schedule}')
    env_seed = train_env_seed(train_seed, n_envs)
    venv = make_vec_env(PayloadNavEnv, n_envs=n_envs, seed=env_seed,
                        env_kwargs={'config': env_config})
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True,
                        gamma=ppo_kwargs.get('gamma', 0.99))
    algo = dict(constraint=constraint) if constraint is not None else {}
    model = (PPOLagrangian if constraint is not None else PPO)(
        'MlpPolicy', venv, seed=train_seed, device='cpu', verbose=verbose, **ppo_kwargs,
        policy_kwargs=dict(net_arch=dict(pi=list(net_arch), vf=list(net_arch))),
        tensorboard_log=tensorboard_log, **algo,
    )
    # PPO(seed=)는 모델을 만들면서 env.seed(train_seed)를 다시 불러 환경 i의 시드를 train_seed + i로
    # 덮어쓴다 (SB3 BaseAlgorithm.set_random_seed). 그러면 학습 시드 k와 k+1이 환경을 n_envs − 1개
    # 공유하므로 학습 환경 시드를 다시 건다. 학습이 시작될 때(첫 reset) 적용된다
    venv.seed(env_seed)
    return model, venv, env_seed
