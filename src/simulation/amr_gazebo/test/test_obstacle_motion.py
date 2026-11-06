"""obstacle_motion 단위 테스트 (rclpy 없이). 실제 yaml로 오래 굴려 경로·속도 범위를 확인한다."""

import math
import os
import random

import pytest
import yaml

from amr_gazebo.obstacle_motion import (
    ROBOT_LENGTH, ROBOT_WIDTH, YIELD_DISTANCE, YIELD_PATIENCE_S, YIELD_SIDE_MARGIN, ObstacleMotion, blocks_path,
    body_twist,
    catmull_rom, segment_box_distance)

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


# --- yield_to_robots: 멈춘 로봇 앞에서 선다 ---

HL, HW = 0.5 * ROBOT_LENGTH, 0.5 * ROBOT_WIDTH
CLEAR = 0.25 + YIELD_SIDE_MARGIN        # 작업자 반지름 + 옆 여유
# I17처럼 x = 3에서 남쪽으로 1.0 m/s로 건너는 작업자. 로봇은 그 선 위 (3, 1.5)에서 동쪽을 본다
WORKER = {'id': 'worker_cross', 'type': 'worker', 'pattern': 'straight', 'speed_range_mps': [1.0, 1.0],
          'spawn': {'x': 3.0, 'y': 4.0}, 'waypoints': [{'x': 3.0, 'y': 4.0}, {'x': 3.0, 'y': 0.8}],
          'yield_to_robots': ['amr_01']}


def _box_clearance(xy, robot):
    """작업자 몸(원 r 0.25)과 로봇 차체 사이 거리 (avoidance_eval footprint_clearance와 같은 계산)."""
    x, y, yaw, _ = robot
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = c * (xy[0] - x) + s * (xy[1] - y), -s * (xy[0] - x) + c * (xy[1] - y)
    return math.hypot(max(abs(lx) - HL, 0.0), max(abs(ly) - HW, 0.0)) - 0.25


def _walk(robot_at, seconds, spec=WORKER, name='amr_01', clock=True, motion=None):
    """robot_at(t) → RobotState인 로봇 옆을 걷게 한다. (시각, 위치, 서 있는지) 기록을 돌려준다.

    clock=False면 시각을 주지 않는다 (돌아서지 않고 계속 기다린다).
    """
    m = motion or ObstacleMotion(spec, random.Random(1))
    xy, log = (spec['spawn']['x'], spec['spawn']['y']), []
    for k in range(int(seconds / DT)):
        t = k * DT
        vx, vy = m.step(xy, {name: robot_at(t)}, t if clock else None)
        log.append((t, xy, m.waiting))
        xy = (xy[0] + vx * DT, xy[1] + vy * DT)
    return log


def test_segment_box_distance_cases():
    assert segment_box_distance(-1.0, 0.0, 1.0, 0.0, HL, HW) == 0.0                    # 가로지름
    assert segment_box_distance(-0.1, 0.0, 0.1, 0.05, HL, HW) == 0.0                   # 안에 있음
    assert segment_box_distance(-1.0, 0.5, 1.0, 0.5, HL, HW) == pytest.approx(0.3)     # 옆으로 나란히
    assert segment_box_distance(0.5, -1.0, 0.5, 1.0, HL, HW) == pytest.approx(0.2)     # 축과 나란하고 바깥
    assert segment_box_distance(1.0, 0.0, 0.5, 0.0, HL, HW) == pytest.approx(0.2)      # 끝점이 가장 가까움
    # 꼭짓점 (0.3, 0.2)이 선분 x + y = 0.6에 가장 가깝다
    assert segment_box_distance(0.0, 0.6, 0.6, 0.0, HL, HW) == pytest.approx(0.1 / math.sqrt(2))
    assert segment_box_distance(1.0, 1.0, 1.0, 1.0, HL, HW) == pytest.approx(math.hypot(0.7, 0.8))  # 점


def test_segment_box_distance_matches_sampling():
    """무작위 선분 300개: 선분을 촘촘히 찍어 잰 거리와 같다 (찍은 간격만큼만 크다)."""
    rng = random.Random(3)
    for _ in range(300):
        ax, ay, bx, by = (rng.uniform(-1.5, 1.5) for _ in range(4))
        exact = segment_box_distance(ax, ay, bx, by, HL, HW)
        n = 2000
        sampled = min(math.hypot(max(abs(ax + (bx - ax) * k / n) - HL, 0.0),
                                 max(abs(ay + (by - ay) * k / n) - HW, 0.0)) for k in range(n + 1))
        step = math.hypot(bx - ax, by - ay) / n
        assert exact - 1e-12 <= sampled <= exact + step


def test_blocks_path_only_when_approaching():
    robot = (3.0, 1.5, 0.0, 0.0)          # 차체 y 1.3~1.7
    assert blocks_path((3.0, 2.4), (0.0, -1.0), YIELD_DISTANCE, robot, CLEAR)          # 0.5 m 가면 0.2 m
    assert not blocks_path((3.0, 2.6), (0.0, -1.0), YIELD_DISTANCE, robot, CLEAR)      # 0.5 m 가도 0.4 m
    # 이미 지나쳐 붙어 있어도 멀어지는 쪽으로는 간다. 되돌아오면 막힌다
    assert not blocks_path((3.0, 1.0), (0.0, -1.0), YIELD_DISTANCE, robot, CLEAR)
    assert blocks_path((3.0, 1.0), (0.0, 1.0), YIELD_DISTANCE, robot, CLEAR)
    # 옆으로 나란히 지나가는 길: 가장 가까운 점 쪽으로 가지 않는다
    assert not blocks_path((3.0, 1.95), (1.0, 0.0), YIELD_DISTANCE, robot, CLEAR)
    # 옆 여유: 차체 옆 끝(x 3.3)에서 0.25 m인 선은 막히고 0.35 m인 선은 막히지 않는다
    assert blocks_path((3.55, 2.0), (0.0, -1.0), YIELD_DISTANCE, robot, CLEAR)
    assert not blocks_path((3.65, 2.0), (0.0, -1.0), YIELD_DISTANCE, robot, CLEAR)


def test_stops_before_stopped_robot_and_keeps_distance():
    robot = (3.0, 1.5, 0.0, 0.0)
    log = _walk(lambda t: robot, 15.0, clock=False)
    stop = next(i for i, (_, _, waiting) in enumerate(log) if waiting)
    gap = _box_clearance(log[stop][1], robot)
    # 몸이 0.5 m 더 가면 0.05 m 안으로 닿는 곳에서 선다: 몸과 차체 사이 0.55 m 앞 한 주기(1.0 m/s × 0.05 s) 안
    limit = YIELD_DISTANCE + YIELD_SIDE_MARGIN
    assert limit - 0.05 - 1e-9 <= gap <= limit + 1e-9
    # 로봇이 서 있는 동안 계속 서 있고, 닿지 않는다
    assert all(waiting for _, _, waiting in log[stop:])
    assert log[-1][1] == log[stop][1]
    assert min(_box_clearance(xy, robot) for _, xy, _ in log) == pytest.approx(gap)


def test_robot_heading_changes_stop_point():
    """차체가 남북으로 서 있으면 (yaw 90°) 위쪽 변이 0.1 m 더 높아 0.1 m 먼저 선다."""
    stop_y = {}
    for yaw in (0.0, math.pi / 2):
        log = _walk(lambda t, yaw=yaw: (3.0, 1.5, yaw, 0.0), 6.0)
        stop_y[yaw] = next(xy for _, xy, waiting in log if waiting)[1]
    assert stop_y[math.pi / 2] - stop_y[0.0] == pytest.approx(0.1, abs=0.05 + 1e-9)
    # 옆으로 비낀 로봇: 동서로 서면 길을 막고 (x 3.22~), 남북으로 서면 막지 않는다 (x 3.32~)
    assert any(w for _, _, w in _walk(lambda t: (3.52, 1.5, 0.0, 0.0), 6.0))
    assert not any(w for _, _, w in _walk(lambda t: (3.52, 1.5, math.pi / 2, 0.0), 6.0))


def test_does_not_yield_to_moving_robot():
    """움직이는 로봇에는 비키지 않는다 (로봇이 피해야 하는 시험). 0.06 m/s로 기어가도 마찬가지."""
    for speed in (0.06, 0.5):
        log = _walk(lambda t, v=speed: (3.0 + v * (t - 1.5), 1.5, 0.0, v), 6.0)
        assert not any(waiting for _, _, waiting in log)


def test_waits_until_robot_leaves_path_even_after_it_starts():
    """5 s까지 서 있던 로봇이 0.1 m/s로 동쪽으로 떠난다. 차체가 길(x 3 ± 0.3)에서 비킬 때까지 기다린다."""
    def robot(t):
        x = 3.0 if t < 5.0 else 3.0 + 0.1 * (t - 5.0)
        return (x, 1.5, 0.0, 0.0 if t < 5.0 else 0.1)

    log = _walk(robot, 20.0)
    waiting_t = [t for t, _, waiting in log if waiting]
    # 차체 왼쪽 끝 x − 0.3이 3.3을 넘는 x 3.6, 즉 t = 11 s까지 기다린다
    assert waiting_t[0] < 2.0 and waiting_t[-1] == pytest.approx(11.0, abs=DT + 1e-9)
    assert all(t > 11.0 - DT for t, _, waiting in log if not waiting and t > waiting_t[0])
    # 다시 걸어 남쪽 끝까지 가고, 떠나는 로봇 뒤로 옆 여유보다 멀리 지나간다
    assert min(xy[1] for t, xy, _ in log if t > 11.0) < 1.0
    assert min(_box_clearance(xy, robot(t)) for t, xy, _ in log) > YIELD_SIDE_MARGIN


def test_ignores_other_robots_and_specs_without_yield():
    stopped = (3.0, 1.5, 0.0, 0.0)
    assert not any(w for _, _, w in _walk(lambda t: stopped, 6.0, name='amr_02'))
    plain = {k: v for k, v in WORKER.items() if k != 'yield_to_robots'}
    assert not any(w for _, _, w in _walk(lambda t: stopped, 6.0, spec=plain))
    # 기존처럼 로봇 없이 불러도 된다
    assert ObstacleMotion(WORKER, random.Random(0)).step((3.0, 4.0)) == pytest.approx((0.0, -1.0))


def test_turns_back_when_robot_stays_stopped():
    """로봇이 계속 멈춰 있으면 YIELD_PATIENCE_S 뒤 온 쪽(북쪽)으로 돌아선다. 다시 와도 같은 자리에서 선다."""
    robot = (3.0, 1.5, 0.0, 0.0)
    m = ObstacleMotion(WORKER, random.Random(1))
    log = _walk(lambda t: robot, 20.0, motion=m)
    stop = next(i for i, (_, _, w) in enumerate(log) if w)
    walk = next(i for i in range(stop, len(log)) if not log[i][2])
    assert log[walk][0] - log[stop][0] == pytest.approx(YIELD_PATIENCE_S, abs=DT + 1e-9)
    assert m.turn_backs >= 1
    assert log[walk + 10][1][1] > log[walk][1][1] + 0.4                  # 북쪽으로 걷는다
    # 북쪽 끝(4.0)에서 다시 내려와 같은 간격에서 또 선다. 닿지 않는다
    again = next(i for i in range(walk, len(log)) if log[i][2])
    assert log[again][1] == pytest.approx(log[stop][1], abs=1e-6)
    assert min(_box_clearance(xy, robot) for _, xy, _ in log) == pytest.approx(_box_clearance(log[stop][1], robot))


def test_patience_restarts_when_robot_moves():
    """5~6 s에 길 위에서 조금 움직이고 다시 서면, 다시 선 6 s부터 YIELD_PATIENCE_S를 센다."""
    def robot(t):
        x = 3.0 + 0.1 * min(max(t - 5.0, 0.0), 1.0)
        return (x, 1.5, 0.0, 0.1 if 5.0 <= t < 6.0 else 0.0)

    m = ObstacleMotion(WORKER, random.Random(1))
    log = _walk(robot, 14.0, motion=m)
    stop = next(i for i, (_, _, w) in enumerate(log) if w)
    walk = next(i for i in range(stop, len(log)) if not log[i][2])
    assert log[stop][0] < 2.0
    assert log[walk][0] == pytest.approx(6.0 + YIELD_PATIENCE_S, abs=DT + 1e-9) and m.turn_backs == 1


def test_robot_stopping_next_to_worker_does_not_deadlock():
    """로봇이 작업자가 이미 0.07 m 옆까지 왔을 때 선다 (검수 지적). 작업자 몸이 로봇 앞 긴급정지 띠 안이고
    로봇은 후진하지 않으므로, 작업자가 서기만 하면 둘 다 멈춘 채로 남는다. YIELD_PATIENCE_S 뒤 돌아선다."""
    spec = {**WORKER, 'spawn': {'x': 3.15, 'y': 4.0}, 'waypoints': [{'x': 3.15, 'y': 4.0}, {'x': 3.15, 'y': 0.8}]}

    def robot(t):          # 동쪽으로 0.5 m/s, 2.04 s 뒤 x 2.65(앞면 2.95)에서 선다
        moving = t < 2.04
        return (1.625 + 0.5 * min(t, 2.05), 1.5, 0.0, 0.5 if moving else 0.0)

    m = ObstacleMotion(spec, random.Random(1))
    log = _walk(robot, 10.0, spec=spec, motion=m)
    stop = next(i for i, (_, _, w) in enumerate(log) if w)
    assert _box_clearance(log[stop][1], robot(log[stop][0])) < 0.1          # 바로 옆에서 선다
    walk = next(i for i in range(stop, len(log)) if not log[i][2])
    assert log[walk][0] - log[stop][0] == pytest.approx(YIELD_PATIENCE_S, abs=DT + 1e-9) and m.turn_backs == 1
    assert max(xy[1] for _, xy, _ in log[walk:]) > log[stop][1][1] + 1.0    # 북쪽으로 떠났다
    assert min(_box_clearance(xy, robot(t)) for t, xy, _ in log) > 0.0


def test_rejects_bad_yield_config():
    with pytest.raises(ValueError):          # 몸 크기를 모른다
        ObstacleMotion({k: v for k, v in WORKER.items() if k != 'type'}, random.Random(0))
    with pytest.raises(ValueError):          # 목록이 아니라 글자 하나하나가 로봇 이름이 된다
        ObstacleMotion({**WORKER, 'yield_to_robots': 'amr_01'}, random.Random(0))
    with pytest.raises(ValueError):          # curve는 구간이 짧아 앞을 못 본다
        ObstacleMotion({**WORKER, 'pattern': 'curve'}, random.Random(0))


def test_rejects_bad_robot_size():
    for size in ([0.565], [0.565, 0.0], [-0.5, 0.5]):
        with pytest.raises(ValueError):
            ObstacleMotion({**WORKER, 'yield_robot_size': size}, random.Random(0))


def test_customer_yields_to_serving_robot_size():
    """식당 손님(customer, r 0.25)도 멈춘 로봇 앞에서 선다. 서빙 로봇(0.565 x 0.537 m)은 AMR보다 폭이 넓어
    옆으로 0.27 m 떨어진 선도 막는다 (AMR 크기로 보면 막히지 않는다)."""
    customer = {**WORKER, 'id': 'customer_cross', 'type': 'customer'}
    # 로봇(3, 1.5, 동쪽)의 오른쪽(동쪽) 변에서 옆으로: AMR 반길이 0.3 / 서빙 0.2825는 x 방향이라 북쪽을 보게 돌려 반폭이 x로 오게 한다
    robot = (3.0, 1.5, math.pi / 2, 0.0)
    line_x = 3.0 + 0.2685 + 0.25          # 서빙 로봇 옆 끝에서 손님 몸 반지름만큼 = 닿는 선
    spec = {**customer, 'spawn': {'x': line_x, 'y': 4.0},
            'waypoints': [{'x': line_x, 'y': 4.0}, {'x': line_x, 'y': 0.8}]}
    assert not any(w for _, _, w in _walk(lambda t: robot, 6.0, spec=spec, clock=False))
    serving = {**spec, 'yield_robot_size': [0.565, 0.537]}
    log = _walk(lambda t: robot, 6.0, spec=serving, clock=False)
    assert any(w for _, _, w in log)


@pytest.mark.parametrize('name', ['dynamic_obstacles_restaurant.yaml', 'dynamic_obstacles_restaurant_peak.yaml'])
def test_restaurant_guests_yield_except_curve(name):
    path = os.path.join(os.path.dirname(__file__), '..', 'config', name)
    with open(path, encoding='utf-8') as f:
        specs = yaml.safe_load(f)['obstacles']
    for s in specs:
        ObstacleMotion(s, random.Random(0))          # 설정이 받아들여진다
        if s['pattern'] == 'curve':
            assert not s.get('yield_to_robots'), s['id']
        else:
            assert s['yield_to_robots'] == ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05'], s['id']
            assert s['yield_robot_size'] == [0.565, 0.537], s['id']
