from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest
from stable_baselines3.common.env_checker import check_env

from payload_nav import (OBS_DIM, EnvConfig, PayloadNavEnv, Popout, RobotState, Scenario,
                         braking_distance, env_config_for, load_experiment, max_linear_accel,
                         step)
from payload_nav.scenarios import ScenarioConfig, min_trigger_x, sample_scenario
from payload_nav.seeds import (EVAL_LIST_STRIDE, EVAL_SEED_BASE, MAX_EVAL_LISTS,
                               TRAIN_ENV_SEED_BASE, TRAIN_ENV_SEED_STRIDE, VAL_SEED_BASE,
                               eval_seed, train_env_seed)
from payload_nav.training import make_model

# 관측 뒤쪽 칸: ... 경로 옆 오차, 경로 방향 오차, 왼쪽 벽, 오른쪽 벽, 남은 시간, 질량 값, 질량 앎 여부
LAT, HEAD, WALL_L, WALL_R, TIME_LEFT, MASS, KNOWN = range(OBS_DIM - 7, OBS_DIM)


def _run(env, policy, max_steps=400):
    """policy(env) → action을 끝날 때까지 돌려 마지막 (reward, info)를 돌려준다."""
    for _ in range(max_steps):
        _, reward, terminated, truncated, info = env.step(np.asarray(policy(env)))
        if terminated or truncated:
            return reward, info
    return reward, info


# ---------------------------------------------------------------- 동역학
@pytest.mark.parametrize('payload, measured', [(0.0, 0.254), (10.0, 0.337), (25.0, 0.462)])
def test_braking_distance_matches_gazebo(payload, measured):
    # #42·#44 Gazebo 급정지 실측(1 m/s)과 3 cm 안. 차이는 측정 지연(#55 t_delay 0.022 s)
    assert abs(braking_distance(1.0, payload) - measured) < 0.03


def test_step_reproduces_braking_distance():
    state, moved = RobotState(v=1.0), 0.0
    while state.v > 0.0:
        nxt = step(state, 0.0, 0.0, 25.0, 0.025)
        moved += nxt.x - state.x
        state = nxt
    assert moved == pytest.approx(braking_distance(1.0, 25.0), abs=1e-9)


def test_heavier_payload_accelerates_slower():
    assert max_linear_accel(25.0) < max_linear_accel(0.0)


def test_braking_while_turning_decelerates_less():
    # 두 바퀴 토크를 나눠 쓰므로 급정지와 급회전을 동시에 최대로 할 수 없다 (B1)
    straight = step(RobotState(v=1.0), 0.0, 0.0, 25.0, 0.1)
    turning = step(RobotState(v=1.0), 0.0, 1.5, 25.0, 0.1)
    assert turning.v > straight.v
    assert 0.0 < turning.w < 1.5


def test_heavier_payload_turns_slower():
    # 회전 가속도 바퀴 토크 한계를 받는다. DiffDrive 한계(3.2 rad/s²)보다 작아지는 질량에서 확인
    light = step(RobotState(), 0.0, 1.5, 0.0, 0.1).w
    heavy = step(RobotState(), 0.0, 1.5, 300.0, 0.1).w
    assert heavy < light


# ---------------------------------------------------------------- 환경 규약·관측
def test_check_env():
    check_env(PayloadNavEnv(), warn=False)


@pytest.mark.parametrize('observe, mass_obs, expected', [
    (False, 10.0, (0.0, 0.0)),          # R1·R2: 모름
    (True, 10.0, (10.0 / 30.0, 1.0)),   # 제안: 앎
    (True, 0.0, (0.0, 1.0)),            # 제안: 0 kg을 앎
    (True, None, (0.0, 0.0)),           # 제안 + 평가 "정보 없음": 모름. 0 kg과 구분된다
])
def test_mass_slots(observe, mass_obs, expected):
    env = PayloadNavEnv(EnvConfig(observe_mass=observe, mass_obs_mode='mass'))
    obs, _ = env.reset(seed=0, options={'payload_kg': 10.0, 'mass_obs_kg': mass_obs})
    assert obs.shape == (OBS_DIM,) == (48,)
    assert (obs[MASS], obs[KNOWN]) == pytest.approx(expected)



@pytest.mark.parametrize('mass_obs', [0.0, 10.0, 25.0, None])
def test_mass_slot_brake_ratio(mass_obs):
    # brake_ratio: 질량 대신 빈 차 대비 제동 능력 a_max(m) / a_max(0). 모르면 그대로 (0, 0)
    env = PayloadNavEnv(EnvConfig(observe_mass=True, mass_obs_mode='brake_ratio'))
    obs, _ = env.reset(seed=0, options={'payload_kg': 10.0, 'mass_obs_kg': mass_obs})
    if mass_obs is None:
        assert (obs[MASS], obs[KNOWN]) == pytest.approx((0.0, 0.0))
    else:
        ratio = max_linear_accel(mass_obs) / max_linear_accel(0.0)
        assert (obs[MASS], obs[KNOWN]) == pytest.approx((ratio, 1.0))
    with pytest.raises(ValueError, match='mass_obs_mode'):
        PayloadNavEnv(EnvConfig(mass_obs_mode='speed'))


def test_path_wall_and_time_slots():
    scenario = Scenario(goal_distance=20.0, initial_speed=0.0, initial_heading=0.0, popout=None,
                        corridor_width=1.6, initial_y=0.1)
    env = PayloadNavEnv()
    obs, _ = env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
    assert obs[LAT] == pytest.approx(0.1)
    assert obs[HEAD] == pytest.approx(0.0)
    assert obs[WALL_L] == pytest.approx(0.8 - 0.1 - 0.2)   # 반폭 − 옆 오차 − 로봇 반폭
    assert obs[WALL_R] == pytest.approx(0.8 + 0.1 - 0.2)
    assert obs[TIME_LEFT] == pytest.approx(1.0)
    obs, *_ = env.step(np.array([-1.0, 0.0]))
    assert obs[TIME_LEFT] == pytest.approx(1.0 - 1 / 300)


def test_scenario_does_not_depend_on_mass():
    env = PayloadNavEnv()
    env.reset(seed=3, options={'payload_kg': 0.0})
    light = env.scenario
    env.reset(seed=3, options={'payload_kg': 25.0})
    assert env.scenario == light


# ---------------------------------------------------------------- 통로·종료
def test_steering_into_wall_collides():
    scenario = Scenario(goal_distance=20.0, initial_speed=1.0, initial_heading=0.0, popout=None,
                        corridor_width=1.2)
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
    reward, info = _run(env, lambda e: [0.0, 1.0])
    assert info['outcome'] == 'collision'
    assert info['collision_with'] == 'wall'
    assert info['impact_speed'] > 0.0
    assert reward < -5.0


def test_timeout_is_terminal_with_penalty():
    scenario = Scenario(goal_distance=20.0, initial_speed=0.0, initial_heading=0.0, popout=None)
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
    for i in range(300):
        _, reward, terminated, truncated, info = env.step(np.array([-1.0, 0.0]))
        assert not truncated
        if i < 299:
            assert not terminated
    assert terminated and info['outcome'] == 'timeout'
    c = env.cfg
    assert reward == pytest.approx(-c.w_time + c.r_timeout)   # 마지막 스텝 시간 벌점 + 시간 초과


def test_braking_to_goal_succeeds():
    scenario = Scenario(goal_distance=3.0, initial_speed=0.0, initial_heading=0.0, popout=None)
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})

    def policy(e):
        remaining = e.goal[0] - e.state.x
        return [min(0.5, math.sqrt(max(0.0, remaining))) - 1.0, 0.0]
    _, info = _run(env, policy)
    assert info['outcome'] == 'success'


def test_returning_after_overshoot_gives_no_reward():
    # 목표를 지나쳤다가 되돌아와 멈춰 있어도 보상이 생기면 안 된다 (보상 해킹 방지)
    scenario = Scenario(goal_distance=2.0, initial_speed=0.5, initial_heading=0.0, popout=None)
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
    while env.state.x < env.goal[0] + 1.0:                 # 0.5 m/s로 목표를 1 m 지나친다
        env.step(np.array([-0.5, 0.0]))
    while abs(env.state.theta) < math.pi - 0.1:            # 제자리에서 돌아선다
        env.step(np.array([-1.0, 1.0]))
    while env.state.x > env.goal[0] + 0.5:                 # 목표 쪽으로 0.5 m 되돌아온다
        env.step(np.array([-0.5, 0.0]))
    for _ in range(5):                                     # 완전히 선다
        env.step(np.array([-1.0, 0.0]))
    assert env.state.v == 0.0
    for _ in range(10):                                    # 멈춰 있는 동안 보상은 시간 페널티뿐
        _, reward, terminated, truncated, _ = env.step(np.array([-1.0, 0.0]))
        assert not (terminated or truncated)
        assert reward < 0.0


# ---------------------------------------------------------------- 돌발 장애물: 위치 기준 출현
def test_trigger_after_25kg_stopping_distance():
    env = PayloadNavEnv()
    cfg = ScenarioConfig()
    for seed in range(300):
        env.reset(seed=seed)
        sc = env.scenario
        if sc.popout is not None:
            v0 = sc.initial_speed
            assert sc.popout.trigger_x >= v0 * 0.1 + v0 ** 2 / (2 * max_linear_accel(25.0))
            assert sc.popout.trigger_x >= min_trigger_x(v0, cfg) - 1e-9
            assert sc.popout.trigger_x <= sc.goal_distance - cfg.trigger_goal_margin_m


@pytest.mark.parametrize('cruise', [0.6, 2.0])
def test_every_policy_meets_the_obstacle_at_the_same_place(cruise):
    # 시간 기준 출현에서는 빨리 달리면 장애물을 덜 만났다 (허점 A1). 위치 기준이면 속도와 무관하다
    env = PayloadNavEnv()
    met = 0
    for seed in range(60):
        env.reset(seed=seed, options={'payload_kg': 0.0})
        po = env.scenario.popout
        if po is None:
            continue
        while not env.popout_appeared:
            s = env.state                           # 통로 중심선을 따라가게 조향
            w = max(-1.0, min(1.0, (-2.0 * s.y - 2.0 * s.theta) / 1.5))
            _, _, terminated, _, _ = env.step(np.array([cruise - 1.0, w]))
            assert not terminated
        met += 1
        # 나타난 순간 로봇 위치: 서브스텝 하나 동안 간 거리 안 (속도는 초기 속도 2 m/s 이하)
        x_at_spawn = env._popout_origin[0]
        assert po.trigger_x <= x_at_spawn <= po.trigger_x + 2.0 * 0.025 + 1e-9
    assert met > 0


def _standing(gap=1.0, width=1.2, offset=0.0, dwell=3.0, v0=1.0):
    return Scenario(goal_distance=20.0, initial_speed=v0, initial_heading=0.0, corridor_width=width,
                    popout=Popout(trigger_x=1.0, gap=gap, lateral_offset=offset, speed=1.0,
                                  direction=1, radius=0.3, dwell_s=dwell))


def _brake_on_sight(cruise_action):
    return lambda e: [-1.0, 0.0] if e.obstacles else [cruise_action, 0.0]


@pytest.mark.parametrize('payload', [0.0, 25.0])
def test_stop_distance_when_braking_at_appearance(payload):
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(), 'payload_kg': payload})
    while True:
        _, _, terminated, _, info = env.step(np.asarray(_brake_on_sight(0.0)(env)))
        if info['popout_result'] or terminated:
            break
    assert info['popout_result'] == 'stopped'
    d = info['popout_stop_distance']
    # "멈춤"은 0.1 m/s 미만이라 완전 정지보다 조금 짧고, 반응은 최대 0.1 s(1 m/s × 0.1) 늦다
    ideal = braking_distance(1.0, payload)
    assert ideal - braking_distance(0.1, payload) - 1e-9 <= d <= ideal + 0.1
    assert info['popout_stop_margin'] == pytest.approx(1.0 - d)


def test_stoppable_flag_includes_reaction_delay():
    # 나타난 순간 판정이 "설 수 있음"이면, 다음 정책 스텝에 급정지한 로봇은 부딪히지 않는다 (D1)
    env = PayloadNavEnv()
    for payload in (0.0, 25.0):
        for v0 in np.linspace(1.0, 2.0, 11):
            for gap in np.linspace(0.5, 1.5, 11):
                env.reset(seed=0, options={'scenario': _standing(gap=gap, v0=v0),
                                           'payload_kg': payload})
                _, info = _run(env, _brake_on_sight(v0 - 1.0), max_steps=60)
                if env.popout_stoppable:
                    assert env.popout_result == 'stopped', (payload, v0, gap)


def test_full_speed_into_standing_obstacle_collides():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(), 'payload_kg': 25.0})
    _, info = _run(env, lambda e: [0.0, 0.0])    # 1 m/s 유지
    assert info['popout_result'] == 'collision'
    assert info['collision_with'] == 'obstacle'


def test_blocked_flag_depends_on_corridor_width():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(width=1.2), 'payload_kg': 0.0})
    _run(env, lambda e: [0.0, 0.0], max_steps=12)
    assert env.popout_blocked is True             # 0.6 − 0.3 = 0.3 m < 로봇 폭 0.4 + 0.1
    env.reset(seed=0, options={'scenario': _standing(width=2.0, offset=0.2), 'payload_kg': 0.0})
    _run(env, lambda e: [0.0, 0.0], max_steps=12)
    assert env.popout_blocked is False            # 오른쪽에 1.0 + 0.2 − 0.3 = 0.9 m


def test_obstacle_spawns_inside_corridor():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(width=1.2, offset=0.2), 'payload_kg': 0.0})
    while not env.obstacles:
        env.step(np.array([0.0, 0.0]))
    o = env.obstacles[0]
    assert abs(o.y) + o.radius <= 0.6 + 1e-9


def test_standing_obstacle_holds_then_leaves():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(dwell=2.0, v0=0.5), 'payload_kg': 0.0})
    while not env.obstacles:
        env.step(np.array([-0.5, 0.0]))
    start = (env.obstacles[0].x, env.obstacles[0].y)
    for _ in range(15):                                 # 1.5 s: 아직 서 있다
        _, _, _, _, info = env.step(np.array([-1.0, 0.0]))
    assert (env.obstacles[0].x, env.obstacles[0].y) == start
    assert info['popout_kind'] == 'standing'
    for _ in range(10):                                 # 2 s가 지나면 비킨다
        env.step(np.array([-1.0, 0.0]))
    assert env.obstacles[0].y > start[1] + 0.3


def test_passed_when_obstacle_clears_without_stopping():
    scenario = Scenario(goal_distance=20.0, initial_speed=1.0, initial_heading=0.0,
                        popout=Popout(trigger_x=1.0, gap=1.0, lateral_offset=0.2, speed=1.5,
                                      direction=1, radius=0.25))
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
    for _ in range(40):
        _, _, terminated, _, info = env.step(np.array([0.0, 0.0]))
        assert not terminated
    assert info['popout_result'] == 'passed'
    assert info['popout_stop_distance'] is None


def test_standing_ratio():
    env = PayloadNavEnv()
    kinds = []
    for seed in range(2000):
        env.reset(seed=seed)
        if env.scenario.popout is not None:
            kinds.append(env.scenario.popout.dwell_s > 0.0)
    assert np.mean(kinds) == pytest.approx(0.5, abs=0.05)


# ---------------------------------------------------------------- 설정 파일
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'env_v0.toml'


def test_config_file_matches_code_defaults():
    # 설정 파일과 코드 기본값이 어긋나면 문서와 실제 실행이 달라진다
    assert load_experiment(CONFIG).env == EnvConfig()


def test_config_variants():
    exp = load_experiment(CONFIG)
    assert set(exp.variants) == {'r1', 'r2', 'proposed'}
    r1, r2, prop = (env_config_for(exp, v) for v in ('r1', 'r2', 'proposed'))
    assert (r1.mass_mode, r1.fixed_mass, r1.observe_mass) == ('fixed', 0.0, False)
    assert (r2.mass_mode, r2.observe_mass, r2.mass_hide_prob) == ('random', False, 0.0)
    assert (prop.mass_mode, prop.observe_mass, prop.mass_hide_prob) == ('random', True, 0.2)
    # 질량 칸 표현은 제안 방법만 brake_ratio (질량 입력 튜닝). R1·R2는 질량 칸이 늘 (0, 0)이라 영향 없다
    assert (r1.mass_obs_mode, r2.mass_obs_mode, prop.mass_obs_mode) == ('mass', 'mass', 'brake_ratio')
    # 비교군끼리 질량 설정 말고는 같다
    assert replace(r2, observe_mass=True, mass_hide_prob=0.2, mass_obs_mode='brake_ratio') == prop
    assert exp.ppo['net_arch'] == [128, 128]


def test_old_run_config_reads_as_mass_slot():
    # mass_obs_mode가 생기기 전 실행의 config.json(env)은 m / norm_mass로 학습했다. 다시 평가할 때 그대로 읽혀야 한다
    import importlib.util
    from dataclasses import asdict
    spec = importlib.util.spec_from_file_location(
        'evaluate', Path(__file__).resolve().parents[1] / 'scripts' / 'evaluate.py')
    evaluate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluate)
    old = asdict(EnvConfig(observe_mass=True, mass_hide_prob=0.2))
    del old['mass_obs_mode']
    assert evaluate.load_env_config(old).mass_obs_mode == 'mass'


def test_config_rejects_unknown_key(tmp_path):
    bad = tmp_path / 'bad.toml'
    bad.write_text(CONFIG.read_text(encoding='utf-8').replace('w_near =', 'w_nera ='),
                   encoding='utf-8')
    with pytest.raises(ValueError, match='w_nera'):
        load_experiment(bad)


# ---------------------------------------------------------------- 질량 가림
def test_mass_hidden_at_configured_rate():
    env = PayloadNavEnv(EnvConfig(observe_mass=True, mass_hide_prob=0.2))
    hidden = 0
    for seed in range(2000):
        obs, _ = env.reset(seed=seed)
        hidden += obs[KNOWN] == 0.0
    assert 0.17 < hidden / 2000 < 0.23


def test_mass_never_hidden_when_given_explicitly():
    env = PayloadNavEnv(EnvConfig(observe_mass=True, mass_hide_prob=1.0))
    obs, _ = env.reset(seed=0, options={'payload_kg': 10.0, 'mass_obs_kg': 10.0})
    assert obs[KNOWN] == 1.0


# ---------------------------------------------------------------- 평가용 S1·S2 강제
def test_forced_popout_sets_scenario_kind():
    for seed in range(50):
        s1 = sample_scenario(np.random.default_rng(seed), popout=True)
        s2 = sample_scenario(np.random.default_rng(seed), popout=False)
        assert s1.popout is not None and s2.popout is None


def test_forcing_keeps_random_stream():
    # 저절로 나온 종류로 강제하면 같은 시나리오다 (popout_prob 난수를 강제해도 소비한다)
    for seed in range(50):
        mixed = sample_scenario(np.random.default_rng(seed))
        forced = sample_scenario(np.random.default_rng(seed), popout=mixed.popout is not None)
        assert forced == mixed


def test_reset_option_forces_popout():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'popout': True})
    assert env.scenario.popout is not None
    env.reset(seed=0, options={'popout': False})
    assert env.scenario.popout is None


# ---------------------------------------------------------------- 시드 규칙
def test_train_env_seeds_do_not_overlap():
    n_envs, used = 8, set()
    for train_seed in range(200):   # 본 실험 0~99 + 예비 100~199
        base = train_env_seed(train_seed, n_envs)
        seeds = set(range(base, base + n_envs))
        assert not seeds & used
        used |= seeds
    # 검증·평가 시나리오 범위와도 겹치지 않는다
    last_eval = eval_seed(MAX_EVAL_LISTS - 1, EVAL_LIST_STRIDE - 1)
    assert min(used) >= TRAIN_ENV_SEED_BASE > last_eval
    assert EVAL_SEED_BASE > VAL_SEED_BASE + EVAL_LIST_STRIDE


def test_train_env_seed_rejects_too_many_envs():
    with pytest.raises(ValueError):
        train_env_seed(0, TRAIN_ENV_SEED_STRIDE + 1)


def test_eval_lists_differ_per_seed():
    # 학습 시드 k는 평가 목록 k를 푼다. 목록끼리 겹치지 않고, 목록 하나는 에피소드 200개를 담는다
    used = set()
    for k in range(200):
        seeds = {eval_seed(k, e) for e in range(200)}
        assert len(seeds) == 200 and not seeds & used
        used |= seeds
    assert eval_seed(0, 0) == EVAL_SEED_BASE
    assert eval_seed(3, 5) == EVAL_SEED_BASE + 3 * EVAL_LIST_STRIDE + 5


@pytest.mark.parametrize('eval_list, episode', [(-1, 0), (MAX_EVAL_LISTS, 0), (0, -1),
                                                (0, EVAL_LIST_STRIDE)])
def test_eval_seed_rejects_out_of_range(eval_list, episode):
    with pytest.raises(ValueError):
        eval_seed(eval_list, episode)


# ---------------------------------------------------------------- 학습 환경 시드 (실제 PPO로 확인)
def _first_training_scenarios(train_seed, n_envs=8):
    """train_ppo.py와 같은 경로(make_model)로 만들고, 학습 첫 reset에서 각 병렬 환경이 뽑은 시나리오."""
    model, venv, _ = make_model(EnvConfig(), n_envs, train_seed,
                                {'n_steps': 64, 'batch_size': 64}, [8], verbose=0)
    model.env.reset()   # learn()을 시작할 때와 같은 reset
    return [(s.goal_distance, s.initial_speed, s.corridor_width)
            for s in venv.venv.get_attr('scenario')]


def test_training_envs_differ_between_seeds():
    # PPO(seed=)가 환경 시드를 train_seed + i로 덮어쓰면 시드 0과 1이 8개 중 7개를 공유한다
    assert not set(_first_training_scenarios(0)) & set(_first_training_scenarios(1))


def test_training_envs_reproducible():
    assert _first_training_scenarios(5) == _first_training_scenarios(5)


# ---------------------------------------------------------------- 보상 검증 (scripts/check_reward.py)
def test_reward_prefers_safe_cruise_speed():
    # 보상이 안전 순항 속도(규칙 정책, 충돌 0)를 과속(1.5배, 충돌 있음)보다 높게 평가해야 한다
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'check_reward', Path(__file__).resolve().parents[1] / 'scripts' / 'check_reward.py')
    check_reward = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(check_reward)
    safe, fast = check_reward.check(CONFIG, lists=(100,), episodes=20, scales=(1.0, 1.5))
    assert safe['collision_rate'] == 0.0 and fast['collision_rate'] > 0.0
    assert safe['return'] > fast['return']


# ---------------------------------------------------------------- 비교군 공통 검증 (train_ppo.py eval_common/*)
def _train_ppo_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location('train_ppo', Path(__file__).resolve().parents[1] / 'scripts'
                                                  / 'train_ppo.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_common_eval_same_conditions_for_every_variant():
    exp = load_experiment(CONFIG)
    train_ppo = _train_ppo_module()
    # R1(0 kg 고정)도 학습 질량 전부로, 제안 방법은 가림 없이 정확한 질량으로 잰다
    for variant, known in (('r1', False), ('r2', False), ('proposed', True)):
        cfg = env_config_for(exp, variant)
        model, venv, _ = make_model(cfg, 1, 0, {'n_steps': 64, 'batch_size': 64}, [8], verbose=0)
        cb = train_ppo.CommonEvalCallback(cfg, venv, every_steps=1, episodes=8)
        assert cb.masses == exp.env.train_masses
        env = PayloadNavEnv(cfg)
        scenarios = []
        for i in range(8):
            _, info = env.reset(seed=cb.seed_base + i // 4, options={'payload_kg': cb.masses[i % 4]})
            assert info['payload_kg'] == cb.masses[i % 4] and info['mass_known'] is known
            scenarios.append(env.scenario)
        # 질량끼리 같은 시나리오 (0~3번이 시나리오 0, 4~7번이 시나리오 1)
        assert scenarios[0] == scenarios[3] and scenarios[4] == scenarios[7] and scenarios[0] != scenarios[4]
        out = cb.evaluate(model)
        assert set(out) == {'success_rate', 'collision_rate', 'timeout_rate', 'mean_time_s',
                            'speed_0kg', 'speed_2kg', 'speed_10kg', 'speed_25kg', 'collision_25kg'}
        assert all(0.0 <= out[k] <= 1.0 for k in ('success_rate', 'collision_rate', 'collision_25kg'))


# ---------------------------------------------------------------- v1 튜닝 옵션
def test_collision_penalty_scales_with_impact_speed():
    from payload_nav.env import collision_penalty
    flat = EnvConfig(w_impact=0.0)
    assert collision_penalty(flat, 1.2) == flat.r_collision                # w_impact 0이면 속도와 무관
    c = EnvConfig(r_collision=-20.0, w_impact=40.0)
    assert collision_penalty(c, 0.5) == pytest.approx(-40.0)
    assert collision_penalty(c, -1.0) == pytest.approx(-60.0)               # 후진 충돌도 속도 크기
    assert collision_penalty(c, None) == pytest.approx(-20.0)


def test_linear_lr_schedule():
    model, _, _ = make_model(EnvConfig(), 2, 0, {'n_steps': 64, 'batch_size': 64, 'learning_rate': 3e-4,
                                                 'lr_schedule': 'linear'}, [8], verbose=0)
    assert model.lr_schedule(1.0) == pytest.approx(3e-4)
    assert model.lr_schedule(0.25) == pytest.approx(7.5e-5)
    with pytest.raises(ValueError):
        make_model(EnvConfig(), 2, 0, {'lr_schedule': 'cosine'}, [8], verbose=0)


# ---------------------------------------------------------------- 학습 곡선 요약 (plot_curves.py)
def test_curve_summary():
    from payload_nav.curves import curve_summary
    steps = [1, 2, 3, 4, 5]
    values = [0.1, 0.5, 0.9, 0.7, 0.7]
    s = curve_summary(steps, values, final_frac=0.25, threshold=0.8)
    assert s['auc'] == pytest.approx(0.58)
    assert s['final'] == pytest.approx(0.7)          # 마지막 25% 구간 = 스텝 4·5
    assert s['best'] == pytest.approx(0.9)
    assert s['worsen'] == pytest.approx(0.2)         # 최고 뒤 무너진 정도
    assert s['steps_to'] == 3
    assert curve_summary(steps, [0.1] * 5, threshold=0.8)['steps_to'] is None
    assert curve_summary(steps, values)['steps_to'] is None   # 기준값이 없으면 도달 스텝도 없다


def test_curve_summary_lower_is_better():
    # 충돌률처럼 낮을수록 좋은 지표: 가장 좋은 값은 최저, 나빠진 양은 최종 − 최저
    from payload_nav.curves import curve_summary
    s = curve_summary([1, 2, 3, 4, 5], [0.4, 0.2, 0.05, 0.1, 0.1], higher_is_better=False, threshold=0.1)
    assert s['best'] == pytest.approx(0.05)
    assert s['worsen'] == pytest.approx(0.05)
    assert s['steps_to'] == 3                        # 처음 0.1 이하


def test_curve_summary_until_uses_same_window():
    # 학습량이 다른 두 실행은 공통 스텝까지 잘라야 "최종"이 같은 구간의 평균이 된다
    from payload_nav.curves import common_horizon, curve_summary
    long_run = ([1, 2, 3, 4, 5, 6, 7, 8], [0.1, 0.5, 0.8, 0.9, 0.9, 0.6, 0.6, 0.6])
    short_run = ([1, 2, 3, 4], [0.2, 0.6, 0.8, 0.9])
    h = common_horizon([long_run, short_run])
    assert h == 4
    s = curve_summary(*long_run, until=h)
    assert s['last_step'] == 4 and s['points'] == 4
    assert s['final'] == pytest.approx(0.9)          # 잘라내지 않으면 0.6
    assert curve_summary(*long_run)['final'] == pytest.approx(0.6)


def test_mean_band_uses_common_range():
    from payload_nav.curves import mean_band
    grid, mean, low, high = mean_band([([0, 10], [0.0, 1.0]), ([0, 5], [0.0, 0.5])], num=3)
    assert grid.tolist() == [0.0, 2.5, 5.0]          # 두 시드가 다 있는 0~5까지만
    assert mean.tolist() == pytest.approx([0.0, 0.25, 0.5])
    assert (low <= mean).all() and (mean <= high).all()
