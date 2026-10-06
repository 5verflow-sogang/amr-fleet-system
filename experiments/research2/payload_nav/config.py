"""실험 설정 파일(TOML)을 읽어 EnvConfig와 PPO 설정을 만든다.

오타로 없는 키를 적으면 조용히 무시되지 않도록 모르는 키는 오류로 처리한다.
"""

from dataclasses import dataclass, fields, replace
from pathlib import Path
import tomllib
from typing import Dict

from .dynamics import RobotParams
from .env import EnvConfig
from .scenarios import ScenarioConfig


@dataclass(frozen=True)
class Experiment:
    env: EnvConfig                 # 비교군 덮어쓰기 전 공통 설정
    variants: Dict[str, dict]      # 비교군 이름 → EnvConfig 덮어쓸 값
    ppo: dict


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
    return Experiment(base, variants, dict(data.get('ppo', {})))


def env_config_for(experiment: Experiment, variant: str) -> EnvConfig:
    if variant not in experiment.variants:
        raise ValueError(f'비교군 {variant!r}이 설정에 없다: {sorted(experiment.variants)}')
    override = {k: tuple(v) if isinstance(v, list) else v
                for k, v in experiment.variants[variant].items()}
    return replace(experiment.env, **override)
