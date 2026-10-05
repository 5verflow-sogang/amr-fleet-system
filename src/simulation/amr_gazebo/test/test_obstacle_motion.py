"""obstacle_motion 단위 테스트 (rclpy 없이). 실제 yaml로 오래 굴려 경로·속도 범위를 확인한다."""

import math
import os
import random

import pytest
import yaml

from amr_gazebo.obstacle_motion import ObstacleMotion, body_twist, catmull_rom

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'dynamic_obstacles.yaml')
DT = 0.05


def _specs():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)['obstacles']


def _simulate(spec, seconds, seed=1):
    """이상적인 추종 (명령 속도를 그대로 적분)으로 굴려 위치·속도 기록을 돌려준다."""
    m = ObstacleMotion(spec, random.Random(seed))
    xy = (spec['spawn']['x'], spec['spawn']['y'])
    trace, speeds = [xy], []
    for _ in range(int(seconds / DT)):
        vx, vy = m.step(xy)
        speeds.append(math.hypot(vx, vy))
        xy = (xy[0] + vx * DT, xy[1] + vy * DT)
        trace.append(xy)
    return m, trace, speeds


def test_yaml_meets_spec():
    """원 명세: 5개 이상, 0.3~1.5 m/s, 직선·곡선·무작위."""
    specs = _specs()
    assert len(specs) >= 5
    assert {s['pattern'] for s in specs} == {'straight', 'curve', 'random'}
    for s in specs:
        lo, hi = s['speed_range_mps']
        assert 0.3 <= lo <= hi <= 1.5, s['id']
        assert (s['spawn']['x'], s['spawn']['y']) == (s['waypoints'][0]['x'], s['waypoints'][0]['y']), s['id']


@pytest.mark.parametrize('spec', _specs(), ids=lambda s: s['id'])
def test_speed_within_range(spec):
    _, _, speeds = _simulate(spec, 300)
    lo, hi = spec['speed_range_mps']
    assert lo - 1e-9 <= min(speeds) and max(speeds) <= hi + 1e-9


@pytest.mark.parametrize('spec', [s for s in _specs() if s['pattern'] == 'straight'], ids=lambda s: s['id'])
def test_straight_shuttles_between_ends(spec):
    _, trace, _ = _simulate(spec, 400)
    a, b = spec['waypoints'][0], spec['waypoints'][-1]
    length = math.dist((a['x'], a['y']), (b['x'], b['y']))
    # 직선 위에만 있다
    for x, y in trace:
        cross = (b['x'] - a['x']) * (y - a['y']) - (b['y'] - a['y']) * (x - a['x'])
        assert abs(cross) / length < 1e-6
    # 반대쪽 끝에 닿고 다시 출발점 근처로 돌아온다 (왕복)
    far = max(range(len(trace)), key=lambda i: math.dist(trace[i], (a['x'], a['y'])))
    assert math.dist(trace[far], (b['x'], b['y'])) < 0.2
    assert min(math.dist(p, (a['x'], a['y'])) for p in trace[far:]) < 0.2


@pytest.mark.parametrize('spec', [s for s in _specs() if s['pattern'] == 'curve'], ids=lambda s: s['id'])
def test_curve_passes_all_waypoints(spec):
    _, trace, _ = _simulate(spec, 400)
    for w in spec['waypoints']:
        assert min(math.dist(p, (w['x'], w['y'])) for p in trace) < 0.2, (spec['id'], w)


@pytest.mark.parametrize('spec', [s for s in _specs() if s['pattern'] == 'random'], ids=lambda s: s['id'])
def test_random_stays_in_area_and_changes_goal(spec):
    m, trace, _ = _simulate(spec, 300)
    x0, x1, y0, y1 = m.area
    for x, y in trace:
        assert x0 - 0.2 <= x <= x1 + 0.2 and y0 - 0.2 <= y <= y1 + 0.2
    # 무작위 목표가 여러 번 바뀌어 영역 대부분을 돌아다닌다 (네 사분면 모두 방문)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    assert len({(x > cx, y > cy) for x, y in trace}) == 4


def test_same_seed_same_motion():
    spec = next(s for s in _specs() if s['pattern'] == 'random')
    assert _simulate(spec, 60, seed=7)[1] == _simulate(spec, 60, seed=7)[1]
    assert _simulate(spec, 60, seed=7)[1] != _simulate(spec, 60, seed=8)[1]


def test_catmull_rom_hits_points_and_spacing():
    pts = [(0.0, 0.0), (4.0, 2.0), (8.0, 0.0)]
    curve = catmull_rom(pts, spacing=0.5)
    for p in pts:
        assert min(math.dist(p, c) for c in curve) < 1e-9
    gaps = [math.dist(a, b) for a, b in zip(curve, curve[1:])]
    assert max(gaps) < 0.7


def test_body_twist():
    # 동쪽(+x)으로 가는데 몸체가 북쪽(yaw 90°)을 보면, 몸체 기준으로는 오른쪽(−y)으로 가고 시계 방향으로 돈다
    bx, by, wz = body_twist(1.0, 0.0, math.pi / 2)
    assert bx == pytest.approx(0.0, abs=1e-9) and by == pytest.approx(-1.0)
    assert wz < 0
    assert body_twist(0.0, 0.0, 1.0) == (0.0, 0.0, 0.0)


def test_rejects_bad_spec():
    base = {'id': 'x', 'pattern': 'straight', 'speed_range_mps': [0.5, 1.0],
            'waypoints': [{'x': 0, 'y': 0}, {'x': 1, 'y': 0}]}
    with pytest.raises(ValueError):
        ObstacleMotion({**base, 'pattern': 'zigzag'}, random.Random(0))
    with pytest.raises(ValueError):
        ObstacleMotion({**base, 'speed_range_mps': [1.0, 0.5]}, random.Random(0))
    with pytest.raises(ValueError):
        ObstacleMotion({**base, 'waypoints': [{'x': 0, 'y': 0}]}, random.Random(0))
