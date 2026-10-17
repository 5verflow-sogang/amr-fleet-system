"""scripts/evaluate.py의 평가 환경 구성. 학습 시나리오를 바꾼 실행도 --config의 시나리오로 평가하는지 본다."""

from dataclasses import asdict, replace
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import evaluate  # noqa: E402
from payload_nav import PayloadNavEnv, eval_seed, load_experiment  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'env_v0.toml'


def run_env(cfg):
    """train_ppo.py가 config.json에 적는 env (JSON을 거쳐 튜플이 리스트가 된다)."""
    return json.loads(json.dumps(asdict(cfg)))


def popout_gaps(cfg, eval_list=0, episodes=20):
    env = PayloadNavEnv(cfg)
    gaps = []
    for ep in range(episodes):
        env.reset(seed=eval_seed(eval_list, ep), options={'payload_kg': 25.0, 'popout': True})
        gaps.append(env.scenario.popout.gap)
    return gaps


def test_same_scenario_is_not_flagged():
    base = load_experiment(CONFIG).env
    cfg, differs = evaluate.eval_env_config(run_env(base), base)
    assert not differs
    assert cfg == base


def test_changed_training_scenario_is_evaluated_on_config_scenario():
    base = load_experiment(CONFIG).env
    trained = replace(base, scenario=replace(base.scenario, popout_gap=(0.5, 1.0)), observe_mass=True)
    cfg, differs = evaluate.eval_env_config(run_env(trained), base)
    assert differs
    assert cfg.scenario == base.scenario
    assert cfg.observe_mass   # 시나리오 밖 설정은 실행 것 그대로
    assert popout_gaps(cfg) == popout_gaps(base)
    assert popout_gaps(trained) != popout_gaps(base)
