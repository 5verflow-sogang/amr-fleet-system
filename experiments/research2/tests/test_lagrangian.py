"""제약 RL(v3): 비용 신호, [constraint] 설정."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from payload_nav import ConstraintConfig, PayloadNavEnv, env_config_for, load_experiment

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


def _run(env, action, seed, options):
    env.reset(seed=seed, options=options)
    while True:
        _, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            return reward, info


def test_cost_is_one_only_on_collision():
    env = PayloadNavEnv(load_experiment(CONFIG_DIR / 'env_v0.toml').env)
    # 최고 속도로 직진하면 돌발 장애물에 부딪힌다 (S1, 25 kg)
    _, info = _run(env, np.array([1.0, 0.0]), 0, {'payload_kg': 25.0, 'popout': True})
    assert info['outcome'] == 'collision' and info['cost'] == 1.0
    # 정지 명령만 내면 충돌 없이 끝난다
    _, info = _run(env, np.array([-1.0, 0.0]), 0, {'payload_kg': 0.0, 'popout': False})
    assert info['outcome'] != 'collision' and info['cost'] == 0.0


def test_cost_is_zero_before_termination():
    env = PayloadNavEnv(load_experiment(CONFIG_DIR / 'env_v0.toml').env)
    _, info = env.reset(seed=0, options={'payload_kg': 0.0, 'popout': False})
    assert info['cost'] == 0.0
    _, _, terminated, _, info = env.step(np.array([0.0, 0.0]))
    assert not terminated and info['cost'] == 0.0


def test_v3_collision_reward_is_zero():
    # v3는 충돌을 보상에서 빼고 비용으로만 다룬다. 충돌 스텝 보상에 큰 벌점이 없어야 한다
    exp = load_experiment(CONFIG_DIR / 'env_v3.toml')
    assert (exp.env.r_collision, exp.env.w_impact) == (0.0, 0.0)
    env = PayloadNavEnv(env_config_for(exp, 'r2'))
    reward, info = _run(env, np.array([1.0, 0.0]), 0, {'payload_kg': 25.0, 'popout': True})
    assert info['outcome'] == 'collision' and info['cost'] == 1.0
    assert abs(reward) < 5.0   # 진행·시간 항뿐


def test_v3_differs_from_ab_only_in_collision_terms():
    # 한 번에 정식화 하나만 바꾼다: 충돌 벌점 외에는 v2 설정 ab와 같아야 한다
    v3 = load_experiment(CONFIG_DIR / 'env_v3.toml')
    ab = v3.env
    assert ab.r_timeout == -75.0 and ab.scenario.popout_prob == 0.9   # ab 값
    assert v3.variants == load_experiment(CONFIG_DIR / 'env_v0.toml').variants
    assert v3.ppo == load_experiment(CONFIG_DIR / 'env_v0.toml').ppo


def test_constraint_section_is_optional():
    assert load_experiment(CONFIG_DIR / 'env_v0.toml').constraint is None
    c = load_experiment(CONFIG_DIR / 'env_v3.toml').constraint
    assert isinstance(c, ConstraintConfig) and c.cost_limit == 0.01 and c.cost_gamma == 1.0


def test_constraint_unknown_key_is_error(tmp_path):
    text = (CONFIG_DIR / 'env_v3.toml').read_text(encoding='utf-8') + 'cost_limt = 0.02\n'
    bad = tmp_path / 'bad.toml'
    bad.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError, match='constraint'):
        load_experiment(bad)


@pytest.mark.parametrize('field, value', [('cost_limit', 0.0), ('cost_limit', 1.0), ('cost_gamma', 0.0),
                                          ('pid_kp', -0.1), ('lambda_max', 0.0), ('lambda_min', 0.0), ('lambda_init', 200.0),
                                          ('cost_ema', 1.0), ('cost_vf_batch_size', 0)])
def test_constraint_value_check(field, value):
    with pytest.raises(ValueError):
        replace(ConstraintConfig(cost_limit=0.01), **{field: value})


# ---------------------------------------------------------------- PPO-Lagrangian 본체
import torch as th  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.buffers import RolloutBuffer  # noqa: E402
from gymnasium import spaces  # noqa: E402

from payload_nav.lagrangian import PIDLagrangian, PPOLagrangian, combine_advantages, gae  # noqa: E402
from payload_nav.training import make_model  # noqa: E402

SMALL_PPO = dict(n_steps=128, batch_size=64, n_epochs=2, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                 clip_range=0.2, target_kl=None)


def _crash_env_config():
    """짧고 충돌이 잦은 환경 (무작위 행동으로도 롤아웃마다 충돌이 나게): 25 kg 고정, 늘 돌발 장애물, 6 s."""
    base = load_experiment(CONFIG_DIR / 'env_v3.toml').env
    return replace(base, mass_mode='fixed', fixed_mass=25.0, max_steps=60,
                   scenario=replace(base.scenario, popout_prob=1.0))


def test_gae_matches_sb3():
    rng = np.random.default_rng(0)
    n, e = 50, 3
    buf = RolloutBuffer(n, spaces.Box(-1, 1, (2,)), spaces.Box(-1, 1, (1,)), gamma=0.97, gae_lambda=0.9, n_envs=e)
    buf.rewards = rng.normal(size=(n, e)).astype(np.float32)
    buf.values = rng.normal(size=(n, e)).astype(np.float32)
    buf.episode_starts = (rng.random((n, e)) < 0.1).astype(np.float32)
    last_v, last_d = rng.normal(size=e).astype(np.float32), rng.random(e) < 0.5
    buf.compute_returns_and_advantage(th.as_tensor(last_v), last_d)
    adv, ret = gae(buf.rewards, buf.values, buf.episode_starts, last_v, last_d, 0.97, 0.9)
    np.testing.assert_allclose(adv, buf.advantages, rtol=1e-5, atol=1e-6)
    np.testing.assert_allclose(ret, buf.returns, rtol=1e-5, atol=1e-6)


def _pid(**kw):
    return PIDLagrangian(ConstraintConfig(cost_limit=0.01, **kw))


def test_pid_rises_when_violated_and_falls_when_satisfied():
    pid = _pid(pid_kp=0.5, pid_ki=0.1, cost_ema=0.0)
    up = [pid.update(0.05) for _ in range(10)]         # 5% > 1%
    assert all(b > a for a, b in zip(up, up[1:])) and up[0] > pid.c.lambda_init
    down = [pid.update(0.0) for _ in range(60)]        # 0% < 1%
    assert all(b <= a for a, b in zip(down, down[1:])) and down[-1] < up[0]
    assert min(down) >= pid.c.lambda_min


def test_pid_holds_at_limit():
    pid = _pid(pid_kp=0.5, pid_ki=0.1, cost_ema=0.0, lambda_init=2.0)
    vals = [pid.update(0.01) for _ in range(20)]        # 충돌 비율 = δ면 오차 0 → 그대로
    assert vals == pytest.approx([2.0] * 20)


def test_pid_recovers_within_training_after_early_violation():
    # 초반 위반으로 λ가 상한까지 오른 뒤 충돌이 0이 되면, 300만 스텝(약 180 롤아웃) 안에 충분히 내려와야 한다
    # (선형 오차 (J − δ)/δ였다면 거의 안 내려온다. PIDLagrangian 설명)
    pid = _pid(pid_kp=0.5, pid_ki=0.1, cost_ema=0.8)
    for _ in range(40):
        pid.update(0.3)
    assert pid.value == pytest.approx(pid.c.lambda_max)
    n = 0
    while pid.value > 1.0:
        pid.update(0.0)
        n += 1
    assert n < 100


def test_pid_is_roughly_symmetric_in_log_space():
    # 같은 배율의 위반·준수(5δ ↔ 0)에서 λ가 롤아웃당 움직이는 비율이 같은 자릿수여야 한다
    up, down = _pid(pid_kp=0.0, pid_ki=0.1, cost_ema=0.0, lambda_init=1.0), _pid(pid_kp=0.0, pid_ki=0.1, cost_ema=0.0, lambda_init=1.0)
    r_up = up.update(0.05) / 1.0
    r_down = 1.0 / down.update(0.0)
    assert 1.0 < r_down < r_up < 2.0 * r_down


def test_pid_clamps_and_skips_empty_rollouts():
    pid = _pid(pid_kp=10.0, pid_ki=10.0, lambda_max=3.0, cost_ema=0.0)
    assert pid.update(0.5) == pytest.approx(3.0)              # 상한
    assert pid.log_integral == pytest.approx(np.log(3.0))     # 적분 누적 방지
    assert pid.update(None) == pytest.approx(3.0)             # 끝난 에피소드가 없으면 그대로
    low = _pid(pid_kp=10.0, pid_ki=10.0, cost_ema=0.0)
    assert low.update(0.0) == pytest.approx(low.c.lambda_min) # 하한


def test_pid_ema_and_fixed_mode():
    pid = _pid(cost_ema=0.5, pid_kp=0.0, pid_ki=0.0, lambda_init=0.0)
    assert pid.update(0.04) == 0.0 and pid.cost_ema == pytest.approx(0.04)   # 첫 롤아웃은 그 값
    assert pid.update(0.0) == 0.0 and pid.cost_ema == pytest.approx(0.02)    # 이득 0이면 λ 고정 (0 가능)


def test_make_model_without_constraint_is_plain_ppo():
    model, _, _ = make_model(_crash_env_config(), 2, 300, SMALL_PPO, [32, 32], verbose=0)
    assert type(model) is PPO


def test_costs_are_aligned_with_rollout_buffer():
    c = ConstraintConfig(cost_limit=0.01)
    model, venv, _ = make_model(_crash_env_config(), 2, 300, dict(SMALL_PPO, n_steps=512), [32, 32],
                                verbose=0, constraint=c)
    assert isinstance(model, PPOLagrangian)
    _, callback = model._setup_learn(512 * 2, callback=None)
    model.collect_rollouts(model.env, callback, model.rollout_buffer, n_rollout_steps=model.n_steps)
    costs, starts = model._costs, model.rollout_buffer.episode_starts
    hits = np.argwhere(costs == 1.0)
    assert len(hits) > 0                                         # 충돌이 실제로 났다
    for t, i in hits:                                            # 비용 1인 스텝 다음은 새 에피소드
        nxt = model._last_episode_starts[i] if t == len(costs) - 1 else starts[t + 1, i]
        assert nxt == 1.0
    assert set(np.unique(costs)) <= {0.0, 1.0}
    # 끝난 에피소드의 비용 합 중 1인 것의 개수 = 버퍼의 비용 1 개수 (롤아웃 시작 전부터 이어진 에피소드도 비용은 끝날 때만)
    assert sum(ec == 1.0 for ec in model._episode_costs) == len(hits)


def test_lambda_zero_matches_plain_ppo():
    # λ가 늘 0이면 정책 업데이트가 일반 PPO와 같아야 한다 (비용 경로가 정책·환경 난수를 건드리지 않는지도 함께 본다)
    # 두 모델이 torch 전역 난수를 같이 쓰므로, 모델마다 "만들고 바로 학습"해야 같은 난수 상태에서 시작한다
    cfg = _crash_env_config()
    zero = ConstraintConfig(cost_limit=0.01, pid_kp=0.0, pid_ki=0.0, pid_kd=0.0, lambda_init=0.0)
    runs = {}
    for name, c in (('ppo', None), ('lag', zero)):
        model, _, _ = make_model(cfg, 2, 301, SMALL_PPO, [32, 32], verbose=0, constraint=c)
        init = {k: v.clone() for k, v in model.policy.state_dict().items()}
        _, callback = model._setup_learn(128 * 2, callback=None)
        model.collect_rollouts(model.env, callback, model.rollout_buffer, n_rollout_steps=model.n_steps)
        buf = {k: getattr(model.rollout_buffer, k).copy() for k in ('observations', 'actions', 'rewards', 'advantages')}
        model.train()
        runs[name] = (init, buf, model.policy.state_dict(), model)
    (init_p, buf_p, after_p, _), (init_l, buf_l, after_l, lag) = runs['ppo'], runs['lag']
    for k in init_p:
        assert th.equal(init_p[k], init_l[k])                    # 같은 초기 정책 (비용 가치망 초기화가 난수를 안 건드림)
    for k in buf_p:
        np.testing.assert_array_equal(buf_p[k], buf_l[k])        # 같은 롤아웃 (비용 기록이 난수·환경을 안 건드림)
    changed = False
    for k in after_p:
        # 업데이트 하나: 보상 이득 전체 표준화 뒤 PPO 미니배치 표준화 = 미니배치 표준화만 (부동소수 차이만 남는다)
        th.testing.assert_close(after_l[k], after_p[k], rtol=1e-5, atol=1e-6)
        changed |= not th.equal(after_p[k], init_p[k])
    assert changed                                               # 실제로 학습이 일어났다
    assert lag.lagrangian.value == 0.0


def test_lambda_rises_with_collisions_and_changes_policy_update():
    cfg = _crash_env_config()
    c = ConstraintConfig(cost_limit=0.01, pid_kp=0.5, pid_ki=0.5)
    lag, _, _ = make_model(cfg, 2, 302, SMALL_PPO, [32, 32], verbose=0, constraint=c)
    lag.learn(128 * 2 * 3)
    assert lag.lagrangian.cost_ema > 0.01 and lag.lagrangian.value > c.lambda_init


def test_saved_model_loads_as_plain_ppo(tmp_path):
    c = ConstraintConfig(cost_limit=0.01)
    lag, _, _ = make_model(_crash_env_config(), 2, 303, SMALL_PPO, [32, 32], verbose=0, constraint=c)
    lag.learn(128 * 2)
    lag.save(tmp_path / 'm.zip')
    loaded = PPO.load(tmp_path / 'm.zip', device='cpu')           # evaluate.py와 같은 방식
    obs = np.zeros((1, lag.observation_space.shape[0]), dtype=np.float32)
    np.testing.assert_allclose(loaded.predict(obs, deterministic=True)[0], lag.predict(obs, deterministic=True)[0])


def test_combined_advantage_penalizes_steps_before_collision():
    # 같은 보상 이득이라도 비용 이득(충돌 직전 스텝에 큼)이 있으면 λ > 0일 때 그 스텝 이득이 더 낮아진다
    rng = np.random.default_rng(0)
    adv_r = rng.normal(size=(100, 2)).astype(np.float32)
    adv_c = np.zeros_like(adv_r)
    adv_c[40, 0] = 0.9                                           # 충돌로 이어진 스텝
    base = combine_advantages(adv_r, adv_c, 0.0)
    pen = combine_advantages(adv_r, adv_c, 5.0)
    np.testing.assert_allclose(base, (adv_r - adv_r.mean()) / (adv_r.std() + 1e-8), rtol=1e-5, atol=1e-6)
    others = np.ones_like(adv_r, dtype=bool)
    others[40, 0] = False
    # 순위: 충돌 스텝은 다른 스텝들보다 상대적으로 크게 내려간다
    drop_hit = base[40, 0] - pen[40, 0] * 6.0                    # (1 + λ)로 나눈 것을 되돌려 비교
    drop_other = (base[others] - pen[others] * 6.0).mean()
    assert drop_hit > 4.0 and abs(drop_other) < 0.1


def test_checkpoint_selection_prefers_feasible_then_return(tmp_path, monkeypatch):
    # 검증 충돌률 ≤ δ + margin(= 0.02)인 것 중 보상 합 최대. 없으면 충돌률 최소를 임시로 둔다
    import importlib.util
    import json
    from stable_baselines3.common.logger import configure
    spec = importlib.util.spec_from_file_location('train_ppo', CONFIG_DIR.parent / 'scripts' / 'train_ppo.py')
    train_ppo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(train_ppo)
    c = ConstraintConfig(cost_limit=0.01, checkpoint_margin=0.01)
    seq = iter([(0.05, 50.0), (0.03, 40.0), (0.01, 30.0), (0.0, 20.0), (0.015, 60.0), (0.03, 99.0)])

    def fake_run_episode(model, vecnorm, env, seed, options=None):
        return dict(cur)

    monkeypatch.setattr(train_ppo, 'run_episode', fake_run_episode)

    class Fake:
        logger = configure(None, [])
        saves = 0

        def save(self, path):
            Fake.saves += 1

    class FakeVec:
        def save(self, path):
            pass

    cfg = load_experiment(CONFIG_DIR / 'env_v3.toml').env
    cb = train_ppo.SuccessEvalCallback(cfg, FakeVec(), tmp_path, every_steps=1, episodes=4, constraint=c)
    cb.model = Fake()   # BaseCallback.logger는 model.logger를 쓴다
    picked = []
    for step, (col, ret) in enumerate(seq, start=1):
        cur = {'success': True, 'collision': col, 'time_s': 10.0, 'outcome': 'success', 'return': ret}
        cb.num_timesteps = step
        cb._on_step()
        picked.append(json.loads((tmp_path / 'best_info.json').read_text())['step'])
    # 1: 임시(못 지킴) → 2: 충돌 더 낮음 → 3: 처음 지킴 → 4: 지키지만 보상 낮음(유지) → 5: 지키고 보상 높음 → 6: 못 지킴(유지)
    assert picked == [1, 2, 3, 3, 5, 5]
    info = json.loads((tmp_path / 'best_info.json').read_text())
    assert info['feasible'] and info['return'] == 60.0 and info['threshold'] == pytest.approx(0.02)
    assert Fake.saves == 4


def test_cost_limit_annealing_schedule():
    c = ConstraintConfig(cost_limit=0.01, cost_limit_start=0.5, anneal_steps=1000)
    assert c.limit_at(0) == pytest.approx(0.5)
    assert c.limit_at(500) == pytest.approx(np.sqrt(0.5 * 0.01))      # 기하 평균 (비율로 줄임)
    assert c.limit_at(1000) == 0.01 and c.limit_at(5000) == 0.01
    assert all(c.limit_at(t) >= c.limit_at(t + 100) for t in range(0, 1000, 100))
    assert ConstraintConfig(cost_limit=0.01).limit_at(0) == 0.01        # 기본은 조이지 않음
    with pytest.raises(ValueError):
        ConstraintConfig(cost_limit=0.05, cost_limit_start=0.01, anneal_steps=10)


def test_pid_uses_current_limit():
    # 같은 충돌 비율이라도 그때의 δ가 느슨하면 λ가 내려가고, 조이면 올라간다
    loose, tight = _pid(cost_ema=0.0, lambda_init=1.0), _pid(cost_ema=0.0, lambda_init=1.0)
    assert loose.update(0.1, limit=0.5) < 1.0 < tight.update(0.1, limit=0.01)


def test_v3_config_anneals():
    c = load_experiment(CONFIG_DIR / 'env_v3.toml').constraint
    assert c.limit_at(0) == pytest.approx(0.5) and c.limit_at(c.anneal_steps) == c.cost_limit == 0.01
    assert c.lambda_max == 20.0 and c.pid_kp == 0.2


# ---------------------------------------------------------------- 벌점 곡선 실험 (보상 자르기 한도)
def test_clip_reward_default_and_config():
    cfg = load_experiment(CONFIG_DIR / 'env_v0.toml')
    _, venv, _ = make_model(cfg.env, 1, 0, {'n_steps': 64, 'batch_size': 64}, [8], verbose=0)
    assert venv.clip_reward == 10.0                                    # 기본은 SB3 그대로 (v1·v2 재현)
    pen = load_experiment(CONFIG_DIR / 'env_v3_pen1000.toml')
    ppo = dict(pen.ppo)
    ppo.pop('n_envs'), ppo.pop('net_arch')
    _, venv, _ = make_model(pen.env, 1, 0, dict(ppo, n_steps=64, batch_size=64), [8], verbose=0)
    assert venv.clip_reward == 100.0


@pytest.mark.parametrize('p', [100, 300, 1000])
def test_penalty_configs_differ_from_v0_only_in_collision_and_clip(p):
    v0, pen = load_experiment(CONFIG_DIR / 'env_v0.toml'), load_experiment(CONFIG_DIR / f'env_v3_pen{p}.toml')
    assert pen.env == replace(v0.env, r_collision=-float(p))
    assert pen.variants == v0.variants and pen.constraint is None
    assert pen.ppo == dict(v0.ppo, clip_reward=100.0)
