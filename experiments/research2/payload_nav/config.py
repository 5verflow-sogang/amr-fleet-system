"""실험 설정 파일(TOML)을 읽어 EnvConfig와 PPO 설정을 만든다.

오타로 없는 키를 적으면 조용히 무시되지 않도록 모르는 키는 오류로 처리한다.
"""

from dataclasses import dataclass, fields, replace
from pathlib import Path
import tomllib
from typing import Dict, Optional

from .dynamics import RobotParams
from .env import EnvConfig
from .scenarios import ScenarioConfig


@dataclass(frozen=True)
class ConstraintConfig:
    """제약 RL(v3) 설정. 설정 파일에 [constraint]가 있을 때만 PPO-Lagrangian으로 학습한다.

    제약: 학습 분포에서 에피소드 충돌 비율(비용 = 충돌 1) ≤ cost_limit. λ는 로그 공간 PID 라그랑지안
    (payload_nav/lagrangian.py PIDLagrangian)으로 롤아웃마다 갱신한다.
    """
    cost_limit: float                   # δ. 에피소드 충돌 비율 상한 (예: 0.01)
    cost_gamma: float = 1.0             # 비용 할인. 1이면 늦게 나는 충돌도 같은 무게 (제약이 "충돌 확률"이 되게)
    cost_gae_lambda: float = 0.95
    pid_kp: float = 0.5                 # 로그 공간 이득 (오차 e = ln((J_c + δ) / 2δ))
    pid_ki: float = 0.1
    pid_kd: float = 0.0
    lambda_init: float = 0.1
    lambda_min: float = 0.01            # 로그 공간이라 0이 될 수 없다. 이 정도면 정책 이득에 거의 영향이 없다
    lambda_max: float = 100.0
    cost_ema: float = 0.8               # 롤아웃 충돌 비율 이동평균 계수 (이전 값 비중). 롤아웃당 에피소드가 100개 남짓이라 잡음을 줄인다
    cost_vf_lr: float = 3e-4
    cost_vf_epochs: int = 10
    cost_vf_batch_size: int = 256
    checkpoint_margin: float = 0.01     # 체크포인트 선택: 검증 충돌률 ≤ cost_limit + margin인 것만 "제약을 지킴"으로 본다

    def __post_init__(self):
        if not 0.0 < self.cost_limit < 1.0:
            raise ValueError(f'cost_limit은 (0, 1) 범위여야 한다: {self.cost_limit}')
        if not 0.0 < self.cost_gamma <= 1.0 or not 0.0 <= self.cost_gae_lambda <= 1.0:
            raise ValueError('cost_gamma는 (0, 1], cost_gae_lambda는 [0, 1] 범위여야 한다')
        if min(self.pid_kp, self.pid_ki, self.pid_kd, self.lambda_init) < 0.0:
            raise ValueError('PID 이득·lambda_init은 0 이상이어야 한다')
        if not 0.0 < self.lambda_min < self.lambda_max or self.lambda_init > self.lambda_max:
            raise ValueError('0 < lambda_min < lambda_max, lambda_init ≤ lambda_max여야 한다')
        if not 0.0 <= self.cost_ema < 1.0:
            raise ValueError(f'cost_ema는 [0, 1) 범위여야 한다: {self.cost_ema}')
        if self.cost_vf_epochs < 1 or self.cost_vf_batch_size < 1:
            raise ValueError('cost_vf_epochs·cost_vf_batch_size는 1 이상이어야 한다')


@dataclass(frozen=True)
class Experiment:
    env: EnvConfig                 # 비교군 덮어쓰기 전 공통 설정
    variants: Dict[str, dict]      # 비교군 이름 → EnvConfig 덮어쓸 값
    ppo: dict
    constraint: Optional[ConstraintConfig] = None   # [constraint]가 없으면 None (일반 PPO, v1·v2)


def _build(cls, values: dict, where: str):
    names = {f.name for f in fields(cls)}
    unknown = set(values) - names
    if unknown:
        raise ValueError(f'[{where}]에 모르는 키가 있다: {sorted(unknown)}')
    return cls(**{k: tuple(v) if isinstance(v, list) else v for k, v in values.items()})


def load_experiment(path: Path) -> Experiment:
    data = tomllib.loads(Path(path).read_text(encoding='utf-8'))
    env = dict(data.get('env', {}))
    scenario = _build(ScenarioConfig, env.pop('scenario', {}), 'env.scenario')
    robot = _build(RobotParams, env.pop('robot', {}), 'env.robot')
    base = _build(EnvConfig, {**env, 'scenario': scenario, 'robot': robot}, 'env')
    variants = data.get('variants', {})
    if not variants:
        raise ValueError('[variants]가 비어 있다')
    for name, override in variants.items():
        _build(EnvConfig, {**env, **override, 'scenario': scenario, 'robot': robot},
               f'variants.{name}')  # 키 검사
    constraint = _build(ConstraintConfig, data['constraint'], 'constraint') if 'constraint' in data else None
    return Experiment(base, variants, dict(data.get('ppo', {})), constraint)


def env_config_for(experiment: Experiment, variant: str) -> EnvConfig:
    if variant not in experiment.variants:
        raise ValueError(f'비교군 {variant!r}이 설정에 없다: {sorted(experiment.variants)}')
    override = {k: tuple(v) if isinstance(v, list) else v
                for k, v in experiment.variants[variant].items()}
    return replace(experiment.env, **override)
