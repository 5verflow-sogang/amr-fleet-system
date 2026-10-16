"""DWA 비교군 B1·B2 (payload_nav/dwa_policy.py, amr_planning_core C++ 원본 호출).

판단은 C++ 원본이 하므로 여기서는 연결(costmap 래스터, 좌표 변환, 질량 가정, 플러그인 동작)을 본다.
C++ 알고리즘 자체의 동작은 amr_planning_core test_dwa.cpp가 본다.
"""

from dataclasses import replace

import numpy as np
import pytest

from payload_nav import PayloadNavEnv, Scenario, max_linear_accel
from payload_nav.dwa_policy import DwaPolicy


def _scenario(goal=10.0, v0=0.0, width=1.6, popout=None):
    return Scenario(goal_distance=goal, initial_speed=v0, initial_heading=0.0, corridor_width=width,
                    popout=popout)


def _env(scenario, payload=0.0, mass_obs='exact'):
    env = PayloadNavEnv()
    options = {'scenario': scenario, 'payload_kg': payload}
    options['mass_obs_kg'] = payload if mass_obs == 'exact' else mass_obs
    env.reset(seed=0, options=options)
    return env


def _obstacle(x, y, vx=0.0, vy=0.0, r=0.3, hold_until=-1.0):
    return type('O', (), dict(x=x, y=y, vx=vx, vy=vy, radius=r, hold_until=hold_until))()


def test_costmap_walls_follow_nav2_inflation():
    # 통로 1.6 m (벽 |y| ≥ 0.8). 가운데 열에서 벽까지 셀 거리로 Nav2 inflation 비용
    env = _env(_scenario(width=1.6))
    ox, oy, n, cost = DwaPolicy().costmap(env)
    assert n == 60 and ox == pytest.approx(-1.5) and oy == pytest.approx(-1.5)
    col = cost.reshape(n, n)[:, 30]
    cy = oy + (np.arange(n) + 0.5) * 0.05
    assert (col[np.abs(cy) >= 0.8] == 254).all()       # 벽
    assert col[30] == 0                                # 중심선: 벽까지 0.8 m > inflation 0.5 m
    edge = int(np.argmax(cy >= 0.8))                   # 위쪽 벽 첫 행. 그 아래 4칸(0.2 m ≤ 0.21)은 치명 253
    assert (col[edge - 4:edge] == 253).all() and col[edge - 5] < 253


def test_costmap_marks_scanned_obstacle():
    env = _env(_scenario())
    env.obstacles = [_obstacle(1.0, 0.0)]
    ox, oy, n, cost = DwaPolicy().costmap(env)
    i = int((0.0 - oy) / 0.05) * n + int((1.0 - ox) / 0.05)
    assert cost[i] == 254


def test_empty_corridor_accelerates_forward():
    env = _env(_scenario())
    status, v, w = DwaPolicy().compute_velocity_command(env, (1.5, 0.0), acc_v=1.0)
    assert status == 'ok' and v == pytest.approx(0.1) and abs(w) < 0.2


def test_obstacle_ahead_slows_or_blocks():
    env = _env(_scenario(v0=1.0))
    env.obstacles = [_obstacle(1.2, 0.0, hold_until=10.0)]
    status, v, _ = DwaPolicy().compute_velocity_command(env, (1.5, 0.0), acc_v=1.0)
    assert status == 'no_valid' or v < 1.0


@pytest.mark.parametrize('payload', [0.0, 25.0])
def test_b1_uses_empty_robot_limit_and_b2_observed_mass(payload):
    env = _env(_scenario(), payload=payload)
    assert DwaPolicy('b1').acc_v(env) == pytest.approx(max_linear_accel(0.0))
    assert DwaPolicy('b2').acc_v(env) == pytest.approx(max_linear_accel(payload))


def test_b2_without_mass_info_equals_b1():
    # 실험 설계 3절: 정보 없음이면 B2 = B1
    env = _env(_scenario(), payload=25.0, mass_obs=None)
    assert DwaPolicy('b2').acc_v(env) == pytest.approx(DwaPolicy('b1').acc_v(env))


def test_names():
    assert DwaPolicy('b1').name == 'dwa_b1'
    with pytest.raises(ValueError):
        DwaPolicy('b3')


def test_rotate_to_goal_inside_tolerance():
    # 목표 반경 안이면 선속도 0, 제자리 회전 (C++ rotateToGoalYaw)
    env = _env(_scenario(goal=10.0))
    env.state = replace(env.state, x=9.9, theta=1.0)
    v, w = DwaPolicy().command(env)
    assert v == 0.0 and w < 0.0


def test_runs_whole_episode():
    env = _env(_scenario(goal=10.0, v0=1.0))
    policy = DwaPolicy('b2')
    for _ in range(300):
        _, _, terminated, truncated, info = env.step(policy(env))
        if terminated or truncated:
            break
    assert info['outcome'] in ('success', 'timeout', 'overshoot', 'collision')
