"""규칙 정책 (참고 기준선, payload_nav/rule_policy.py)."""

import numpy as np
import pytest

from payload_nav import PayloadNavEnv, Popout, Scenario, max_linear_accel
from payload_nav.rule_policy import RulePolicy, safe_cruise_speed
from payload_nav.scenarios import ScenarioConfig


def _episode(env, policy, max_steps=400):
    """끝날 때까지 돌려 (마지막 info, 속도 기록)을 돌려준다."""
    speeds = []
    for _ in range(max_steps):
        _, _, terminated, truncated, info = env.step(policy(env))
        speeds.append(env.state.v)
        if terminated or truncated:
            break
    return info, speeds


def _standing(gap=0.5, width=1.2, dwell=2.0, v0=1.0):
    return Scenario(goal_distance=15.0, initial_speed=v0, initial_heading=0.0, corridor_width=width,
                    popout=Popout(trigger_x=5.0, gap=gap, lateral_offset=0.0, speed=0.5,
                                  direction=1, radius=0.3, dwell_s=dwell))


@pytest.mark.parametrize('payload, spec', [(0.0, 1.25), (25.0, 0.95)])
def test_cruise_speed_matches_spec(payload, spec):
    # 명세서 4-2절: 간격 0.5 m, 반응 0.1 s 기준 0 kg 약 1.25 m/s, 25 kg 약 0.95 m/s
    assert safe_cruise_speed(payload) == pytest.approx(spec, abs=0.01)


@pytest.mark.parametrize('payload', [0.0, 10.0, 25.0, 30.0])
def test_cruise_speed_stops_exactly_within_gap(payload):
    # 반응 지연 + 제동 거리 = 최소 간격 (env popout_stoppable·min_trigger_x와 같은 식)
    v = safe_cruise_speed(payload)
    assert v * 0.1 + v ** 2 / (2 * max_linear_accel(payload)) == pytest.approx(0.5)


def test_heavier_payload_cruises_slower():
    assert safe_cruise_speed(30.0) < safe_cruise_speed(25.0) < safe_cruise_speed(0.0)


def test_agnostic_plans_for_heaviest_training_mass():
    env = PayloadNavEnv()
    env.reset(seed=0, options={'payload_kg': 0.0})
    guard = ScenarioConfig().guard_payload_kg
    assert RulePolicy(mass_aware=False).cruise_speed(env) == pytest.approx(safe_cruise_speed(guard))
    assert RulePolicy(mass_aware=True).cruise_speed(env) == pytest.approx(safe_cruise_speed(0.0))
    env.reset(seed=0, options={'payload_kg': 0.0, 'mass_obs_kg': None})   # 정보 없음 → 25 kg 기준
    assert RulePolicy(mass_aware=True).cruise_speed(env) == pytest.approx(safe_cruise_speed(guard))


@pytest.mark.parametrize('aware', [False, True])
@pytest.mark.parametrize('payload', [0.0, 25.0, 30.0])
def test_reaches_goal_without_obstacle(aware, payload):
    scenario = Scenario(goal_distance=12.0, initial_speed=2.0, initial_heading=0.05, popout=None,
                        corridor_width=1.2, initial_y=0.1)
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': scenario, 'payload_kg': payload})
    info, speeds = _episode(env, RulePolicy(aware))
    assert info['outcome'] == 'success'
    # 초기 속도에서 줄인 뒤에는 순항 속도를 넘지 않는다
    assert max(speeds[30:]) <= RulePolicy(aware).cruise_speed(env) + 1e-6


@pytest.mark.parametrize('aware', [False, True])
@pytest.mark.parametrize('payload', [0.0, 2.0, 10.0, 25.0])
def test_stops_for_standing_obstacle_then_continues(aware, payload):
    # 가장 짧은 간격(0.5 m), 비켜 갈 수 없는 좁은 통로: 서서 기다렸다가 비키면 목표까지 간다
    env = PayloadNavEnv()
    env.reset(seed=0, options={'scenario': _standing(), 'payload_kg': payload})
    info, _ = _episode(env, RulePolicy(aware))
    # 순항 속도가 "설 수 있음" 경계값이라 popout_stoppable은 부동소수 오차로 False가 될 수 있다
    assert env.popout_blocked is True
    assert info['popout_result'] == 'stopped'
    assert info['popout_stop_margin'] > 0.0
    assert info['outcome'] == 'success'


def test_aware_is_faster_on_light_payload():
    scenario = Scenario(goal_distance=15.0, initial_speed=1.0, initial_heading=0.0, popout=None,
                        corridor_width=1.6)
    env = PayloadNavEnv()
    times = {}
    for aware in (False, True):
        env.reset(seed=0, options={'scenario': scenario, 'payload_kg': 0.0})
        info, _ = _episode(env, RulePolicy(aware))
        assert info['outcome'] == 'success'
        times[aware] = info['time']
    assert times[True] < times[False]


@pytest.mark.parametrize('aware', [False, True])
def test_deterministic(aware):
    env = PayloadNavEnv()
    runs = []
    for _ in range(2):
        env.reset(seed=7, options={'payload_kg': 10.0, 'popout': True})
        actions = []
        for _ in range(400):
            a = RulePolicy(aware)(env)
            actions.append(a)
            *_, terminated, truncated, info = env.step(a)
            if terminated or truncated:
                break
        runs.append((np.array(actions), info['outcome'], info['time']))
    np.testing.assert_array_equal(runs[0][0], runs[1][0])
    assert runs[0][1:] == runs[1][1:]


@pytest.mark.parametrize('aware', [False, True])
def test_succeeds_on_random_training_scenarios(aware):
    # 학습 질량에서는 출현 지점·간격과 관계없이 피할 수 있다 (검증 시드, 평가 목록과 겹치지 않음)
    env = PayloadNavEnv()
    policy = RulePolicy(aware)
    for payload in (0.0, 25.0):
        for ep in range(20):
            env.reset(seed=10_000 + ep, options={'payload_kg': payload, 'popout': True})
            info, _ = _episode(env, policy)
            assert info['outcome'] == 'success', (payload, ep, info['outcome'])
