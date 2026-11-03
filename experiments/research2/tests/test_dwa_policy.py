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


def test_sample_counts_are_fixed_paper_values():
    # 논문 비교군 값 20 x 30 고정 - 창고 스택(nav2_params.yaml)이 샘플 수를 줄여도 따라가지 않는다
    p = DwaPolicy().params
    assert (p.v_samples, p.w_samples) == (20, 30)
    cfg = DwaPolicy().config(1.0)
    assert (cfg.v_samples, cfg.w_samples) == (20, 30)


def test_rotate_in_place_is_off_paper_value():
    # 논문 값: 제자리 회전 끔 - 창고 스택(nav2_params.yaml)이 45°로 켜도 따라가지 않는다. C++ 기본값(45°)이
    # 아니라 이 값이 C API로 넘어가야 한다
    p = DwaPolicy().params
    assert p.rotate_in_place_min_angle <= 0.0
    cfg = DwaPolicy().config(1.0)
    assert cfg.rotate_in_place_min_angle <= 0.0
    # 제자리 회전 샘플의 풋프린트 여유도 논문 값 0 (C++ 기본값 0.05가 아니다)
    assert p.rotate_in_place_clearance == 0.0
    assert cfg.rotate_in_place_clearance == 0.0


def test_avoid_mode_is_off_paper_value():
    # 논문 값: TTC 회피 모드 끔 - 창고 스택(nav2_params.yaml)이 켜도 따라가지 않는다. C API로 0이 넘어가야 한다
    p = DwaPolicy().params
    assert p.avoid_mode_enabled is False
    assert DwaPolicy().config(1.0).avoid_mode_enabled == 0


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


def test_evaluate_output_is_read_by_analyze(tmp_path):
    # evaluate.py --dwa 결과의 variant 열이 analyze.py가 H1·H2에 쓰는 이름(b1·b2)이어야 한다 (본 실험 시험 실행에서 발견)
    import importlib.util
    from pathlib import Path
    import subprocess
    import sys
    root = Path(__file__).resolve().parents[1]
    out = tmp_path / 'dwa'
    subprocess.run([sys.executable, str(root / 'scripts' / 'evaluate.py'), '--dwa', 'b1', 'b2', '--eval-list', '150',
                    '--episodes', '1', '--masses', '10', '--scenarios', 'S1', '--mass-obs', 'exact', 'unknown',
                    '--out-dir', str(out)], check=True, cwd=root, capture_output=True)
    spec = importlib.util.spec_from_file_location('analyze', root / 'scripts' / 'analyze.py')
    analyze = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyze)
    rows, _ = analyze.load([out / 'episodes.csv'])
    assert {r['variant'] for r in rows} == {'b1', 'b2'}
    import csv
    with (out / 'episodes.csv').open(newline='', encoding='utf-8') as fh:
        assert {r['run'] for r in csv.DictReader(fh)} == {'dwa_b1', 'dwa_b2'}


@pytest.mark.parametrize('payload', [0.0, 25.0])
def test_path_call_stops_at_goal(payload):
    # 경로를 넘기면(플러그인과 같음, #195) 목표 앞 감속이 켜져 빠르게 다가가도 지나치거나 벽에 닿지 않는다
    env = _env(_scenario(goal=8.0, v0=1.5), payload=payload)
    policy = DwaPolicy('b2')
    for _ in range(300):
        _, _, terminated, truncated, info = env.step(policy(env))
        if terminated or truncated:
            break
    assert info['outcome'] == 'success', info['outcome']


def test_point_call_is_kept_for_comparison():
    # follow_path=False는 #185 처음 방식(지점 하나, final_goal 없음). 같은 상태에서 두 호출 모두 동작한다
    env = _env(_scenario(goal=8.0, v0=1.5))
    for follow in (True, False):
        policy = DwaPolicy('b2', replace(DwaPolicy().params, follow_path=follow))
        v, _ = policy.command(env)
        assert 0.0 <= v <= policy.params.max_v
