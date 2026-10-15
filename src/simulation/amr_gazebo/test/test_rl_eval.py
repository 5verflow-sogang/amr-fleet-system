"""RL 평가 환경 (rl_eval.py, rl_policies.py). 연우 2D 환경(experiments/research2)과 같은 시나리오·지표인지 본다.

2D 환경 자체와 비교하는 테스트는 gymnasium이 있을 때만 돈다 (ROS 이미지에는 없다).
RL_EVAL_REQUIRE_2D=1이면 건너뛰지 않고 실패한다 (CI rl-env.yml: 연우 시나리오가 바뀌면 여기서 잡는다).
"""
import importlib.util
import math
import os
import xml.etree.ElementTree as ET

import pytest

from amr_gazebo.rl_eval import (
    EpisodeTracker, Sample, brake_metrics, eval_seed, layout_for, load_research2, obstacle_sdf, obstacle_spot,
    policy_obs, scenario_for, walls_sdf)
from amr_gazebo.rl_policies import brake, cruise, load_policy, max_accel

HERE = os.path.dirname(os.path.abspath(__file__))
R2_DIR = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..', 'experiments', 'research2'))
REQUIRE_2D = os.environ.get('RL_EVAL_REQUIRE_2D') == '1'
needs_r2 = pytest.mark.skipif(not REQUIRE_2D and not os.path.isdir(R2_DIR), reason='experiments/research2 없음')


@pytest.fixture(scope='module')
def r2():
    return load_research2(R2_DIR)


def _load(name):
    spec = importlib.util.spec_from_file_location(f'r2_{name}', os.path.join(R2_DIR, 'payload_nav', f'{name}.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@needs_r2
def test_eval_seed_matches_research2():
    seeds = _load('seeds')
    for k, ep in ((0, 0), (3, 17), (9, 999)):
        assert eval_seed(k, ep) == seeds.eval_seed(k, ep)


@needs_r2
def test_scenario_is_deterministic(r2):
    a, b = scenario_for(r2, 100_003, True), scenario_for(r2, 100_003, True)
    assert a == b
    assert a.popout is not None
    assert scenario_for(r2, 100_003, False).popout is None
    # S1·S2 강제는 popout만 바꾸고 나머지 난수 흐름은 같다 (2D sample_scenario 규칙)
    assert scenario_for(r2, 100_003, False).goal_distance == a.goal_distance


@needs_r2
def test_scenario_matches_2d_env(r2):
    if not REQUIRE_2D:
        pytest.importorskip('gymnasium')
    import sys
    # 패키지 __init__(tomllib, Python 3.11)을 거치지 않고 env.py만 load_research2와 같은 이름 공간에 읽는다
    name = 'payload_nav_gz.env'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, os.path.join(R2_DIR, 'payload_nav', 'env.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    env = sys.modules[name].PayloadNavEnv()
    for seed in (100_000, 100_001, 101_005):
        for popout in (True, False):
            env.reset(seed=seed, options={'popout': popout, 'payload_kg': 10.0})
            ours = scenario_for(r2, seed, popout)
            assert ours.goal_distance == pytest.approx(env.scenario.goal_distance)
            assert ours.corridor_width == pytest.approx(env.scenario.corridor_width)
            assert ours.initial_speed == pytest.approx(env.scenario.initial_speed)
            assert (ours.popout is None) == (env.scenario.popout is None)
            if ours.popout:
                assert ours.popout.trigger_x == pytest.approx(env.scenario.popout.trigger_x)
                assert ours.popout.dwell_s == pytest.approx(env.scenario.popout.dwell_s)


@needs_r2
@pytest.mark.parametrize('payload', [0.0, 10.0, 25.0])
def test_layout_runup_reaches_initial_speed(r2, payload):
    sc = scenario_for(r2, 100_000, True)
    lay = layout_for(r2, sc, payload)
    a = r2.dynamics.max_linear_accel(payload)
    assert lay.runup_m > sc.initial_speed ** 2 / (2 * a)
    # 가속 구간 시작은 출발 방향 선 위 (0, initial_y)에서 runup_m 뒤
    assert math.hypot(lay.start_x, lay.start_y - sc.initial_y) == pytest.approx(lay.runup_m)
    assert lay.wall_x0 < lay.start_x and lay.wall_x1 > sc.goal_distance + 2.0
    assert lay.half_width == pytest.approx(sc.corridor_width / 2)


@needs_r2
def test_walls_inner_faces(r2):
    sc = scenario_for(r2, 100_002, False)
    lay = layout_for(r2, sc, 0.0)
    root = ET.fromstring(walls_sdf(lay))
    ys = []
    for link in root.findall('./model/link'):
        y = float(link.find('pose').text.split()[1])
        thick = float(link.find('collision/geometry/box/size').text.split()[1])
        ys.append(abs(y) - thick / 2)
    assert ys == pytest.approx([lay.half_width] * 2, abs=1e-4)   # SDF는 소수 4자리
    masks = [int(c.text, 16) for c in root.findall('./model/link/collision/surface/contact/collide_bitmask')]
    assert masks == [0x04, 0x04]


def test_obstacle_sdf_valid():
    root = ET.fromstring(obstacle_sdf(0.3))
    assert root.find('./model/link/collision/geometry/cylinder/radius').text == '0.3000'
    # 장애물과 벽은 비트마스크가 겹치지 않아 서로 통과한다 (2D처럼 장애물이 통로 밖으로 비킨다)
    obstacle_mask = int(root.find('./model/link/collision/surface/contact/collide_bitmask').text, 16)
    assert obstacle_mask & 0x04 == 0
    assert {p.get('name') for p in root.findall('./model/plugin')} == {
        'ignition::gazebo::systems::VelocityControl', 'ignition::gazebo::systems::OdometryPublisher'}


@needs_r2
def test_obstacle_spot_like_env(r2):
    sc = scenario_for(r2, 100_000, True)
    po = sc.popout
    x, y = obstacle_spot(r2, sc, 5.0, 0.05, 0.8)
    assert x == pytest.approx(5.0 + 0.3 + po.gap + po.radius)
    assert abs(y) <= 0.8 - po.radius + 1e-9
    _, y_far = obstacle_spot(r2, sc, 5.0, 10.0, 0.8)   # 통로 밖으로 나가면 자른다
    assert y_far == pytest.approx(0.8 - po.radius)


def run_straight(r2, sc, payload, v_of_x, dt=0.01, obstacle=None, appear=True):
    """중심선을 따라 1D로 움직여 tracker에 넣는다 (토크 한계 가감속)."""
    tr = EpisodeTracker(r2, sc, payload, sc.corridor_width / 2)
    a = r2.dynamics.max_linear_accel(payload)
    x, v, t, next_tick = 0.0, sc.initial_speed, 0.0, 0.0
    while tr.outcome is None and t < 40.0:
        s = Sample(t, x, 0.0, 0.0, v, 0.0)
        if appear and tr.popout_triggered(s):
            tr.appear(s, obstacle(s))
        tr.update(s, obstacle(s) if (obstacle and tr.popout_appeared) else None)
        if t >= next_tick - 1e-9:
            tr.control_tick(s)
            next_tick += 0.1
        target = v_of_x(x)
        v = min(target, v + a * dt) if target > v else max(target, v - a * dt)
        x += v * dt
        t += dt
    return tr


@needs_r2
def test_tracker_success_and_metrics(r2):
    sc = scenario_for(r2, 100_001, False)
    a = r2.dynamics.max_linear_accel(10.0)
    tr = run_straight(r2, sc, 10.0, lambda x: min(sc.initial_speed, math.sqrt(max(0.0, 2 * 0.9 * a * (sc.goal_distance - x)))))
    m = tr.metrics()
    assert m['outcome'] == 'success' and m['success']
    assert m['time_s'] > 0 and 0 < m['mean_speed'] <= sc.initial_speed
    assert m['popout_appeared'] is False and m['popout_kind'] is None


@needs_r2
def test_tracker_overshoot(r2):
    sc = scenario_for(r2, 100_001, False)
    tr = run_straight(r2, sc, 0.0, lambda x: sc.initial_speed)
    assert tr.outcome == 'overshoot'


@needs_r2
def test_tracker_wall_collision(r2):
    sc = scenario_for(r2, 100_001, False)
    tr = EpisodeTracker(r2, sc, 0.0, sc.corridor_width / 2)
    tr.update(Sample(0.0, 1.0, sc.corridor_width / 2 - 0.1, 0.0, 1.0, 0.0))
    assert tr.outcome == 'collision' and tr.collision_with == 'wall' and tr.impact_speed == 1.0


@needs_r2
def test_tracker_popout_stopped_and_collision(r2):
    sc = scenario_for(r2, 100_000, True)
    po = sc.popout
    spot = {}

    def obstacle(s):
        if 'xy' not in spot:
            spot['xy'] = (s.x + 0.3 + po.gap + po.radius, 0.0)
        return spot['xy']

    # 나타나자마자 급정지: 설 수 있으면 stopped, 여유 = 간격 − 제동 거리
    stopped = {}

    def v_brake(x):
        if x >= po.trigger_x:
            stopped['yes'] = True
        return 0.0 if stopped else sc.initial_speed
    tr = run_straight(r2, sc, 0.0, v_brake, obstacle=obstacle)
    m = tr.metrics()
    assert m['popout_appeared'] and m['popout_gap_actual'] if 'popout_gap_actual' in m else tr.popout_gap_actual
    if tr.popout_stoppable:
        assert m['popout_result'] == 'stopped'
        assert m['popout_stop_margin'] == pytest.approx(tr.popout_gap_actual - m['popout_stop_distance'])
    # 장애물을 무시하고 달리면 충돌
    spot.clear()
    tr2 = run_straight(r2, sc, 0.0, lambda x: sc.initial_speed, obstacle=obstacle)
    assert tr2.outcome == 'collision' and tr2.collision_with == 'obstacle'
    assert tr2.metrics()['popout_result'] == 'collision'


def _obs(**kw):
    base = dict(t=0.0, x=0.0, y=0.0, theta=0.0, v=1.5, w=0.0, goal_x=15.0, goal_y=0.0, half_width=0.8,
                initial_speed=1.5, payload_kg=10.0, remaining_s=30.0, dt=0.1, obstacles=[])
    base.update(kw)
    return base


def test_cruise_and_brake():
    v, _ = cruise(_obs())
    assert v == pytest.approx(1.5)
    v_near, _ = cruise(_obs(x=14.9))
    assert v_near < 0.5
    _, w = cruise(_obs(y=0.2))
    assert w < 0   # 중심선 쪽(오른쪽)으로
    ahead = [{'x': 1.5, 'y': 0.0, 'vx': 0.0, 'vy': 0.0, 'radius': 0.3}]
    assert brake(_obs(obstacles=ahead))[0] == 0.0
    beside = [{'x': 1.5, 'y': 0.7, 'vx': 0.0, 'vy': 0.0, 'radius': 0.3}]
    assert brake(_obs(obstacles=beside))[0] > 0.0
    assert max_accel(25.0) < max_accel(0.0)


def test_load_policy():
    assert load_policy('brake') is brake
    assert load_policy('amr_gazebo.rl_policies:cruise') is cruise
    with pytest.raises(ValueError):
        load_policy('nope')


@needs_r2
def test_policy_obs_hides_obstacle_until_seen(r2):
    sc = scenario_for(r2, 100_000, True)
    lay = layout_for(r2, sc, 0.0)
    obs = policy_obs(Sample(1.0, 2.0, 0.0, 0.0, 1.0, 0.0), sc, lay, 0.0, [])
    assert obs['obstacles'] == [] and obs['remaining_s'] == pytest.approx(29.0)


def test_brake_metrics_model_brake():
    """모델대로(지연 없이 토크 한계 a) 서면 차 0, 지연 0, 감속 a. 지연 τ를 넣으면 차 ≈ v·τ."""
    a, v0, dt = 2.0, 1.5, 0.01

    def trace(delay):
        out, t, x, v = [], 0.0, 0.0, v0
        while t < 2.0:
            t += dt
            x += v * dt if t <= delay else 0.0
            if t > delay:
                v_new = max(0.0, v - a * dt)
                x += 0.5 * (v + v_new) * dt
                v = v_new
            out.append((t, x, 0.0, 0.0, v, 0.0))
        return out

    m = brake_metrics(trace(0.0), 0.0, 0.0, v0, a)
    assert m['model_dist'] == pytest.approx(v0 ** 2 / (2 * a))
    assert m['excess'] == pytest.approx(0.0, abs=0.01)
    assert m['dead_time_s'] == pytest.approx(0.0, abs=dt)
    assert m['decel_fit'] == pytest.approx(a, rel=0.01)
    m = brake_metrics(trace(0.1), 0.0, 0.0, v0, a)
    assert m['excess'] == pytest.approx(v0 * 0.1, abs=0.01)
    assert m['dead_time_s'] == pytest.approx(0.1, abs=dt)
