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


# 식당 손님 파일 → 손님 수: 기본은 일과 손님 10명, 혼잡도 시연 1-1·1-2는 몰림 손님 10명을 더한 20명. 모두 일과(routine)가 있다
RESTAURANT_GUESTS = {'dynamic_obstacles_restaurant_routine.yaml': 10, 'dynamic_obstacles_restaurant_routine_peak.yaml': 20}


def _restaurant_guests(name):
    with open(os.path.join(os.path.dirname(__file__), '..', 'config', name), encoding='utf-8') as f:
        return yaml.safe_load(f)['obstacles']


@pytest.mark.parametrize('name', sorted(RESTAURANT_GUESTS))
def test_restaurant_guests_have_routines_and_yield_to_serving_robots(name):
    specs = _restaurant_guests(name)
    assert len(specs) == RESTAURANT_GUESTS[name]
    for s in specs:
        assert ObstacleMotion(s, random.Random(0)).pattern == 'routine', s['id']      # 설정이 받아들여진다
        assert len({st['name'] for st in s['steps']}) >= 2, s['id']                   # 한 자리에만 있지 않는다
        assert s['yield_to_robots'] == ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05'], s['id']
        assert s['yield_robot_size'] == [0.565, 0.537], s['id']
        assert (s['yield_patience_s'], s['yield_progress_m']) == (3.0, 0.1), s['id']  # 멈춘 로봇 앞에서 3 s 기다리고 비켜 간다


def test_restaurant_has_no_scripted_walkers_or_standing_guests_left():
    # 예전 걷는 손님(통로 직선 왕복·곡선·무작위)과 서 있기만 하던 손님 파일은 지웠다. 식당 손님은 일과 손님뿐이고,
    # 직원(_staff.yaml, 칸막이 안 주방 왕복)만 straight로 남는다
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    files = sorted(n for n in os.listdir(config_dir) if n.startswith('dynamic_obstacles_restaurant'))
    assert files == sorted(RESTAURANT_GUESTS) + ['dynamic_obstacles_restaurant_staff.yaml']


def test_peak_guests_are_the_default_guests_plus_a_crowd_that_holds_the_west_aisle():
    # 몰림 파일 = 식당 기본 손님 10명(일과 그대로) + 몰림 손님 10명. 한 파일이라 20명 모두 혼잡도에 센다
    base, peak = (_restaurant_guests(n) for n in sorted(RESTAURANT_GUESTS))
    assert [g['id'] for g in peak[:10]] == [g['id'] for g in base]
    for b, p in zip(base, peak):
        assert [(s['name'], s['wait_s']) for s in b['steps']] == [(s['name'], s['wait_s']) for s in p['steps']]
    crowd = peak[10:]
    assert [g['id'] for g in crowd] == [f'crowd_{i:02d}' for i in range(1, 11)]
    zones = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'fleet', 'amr_fleet_manager', 'config',
                         'congestion_zones_restaurant.yaml')
    with open(zones, encoding='utf-8') as f:
        zones = {z['id']: z for z in yaml.safe_load(f)['zones']}

    def inside(zone, p):
        x0, x1, y0, y1 = zones[zone]['area']
        return x0 <= p[0] <= x1 and y0 <= p[1] <= y1

    # 처음 자리에서 200 s 넘게 머문다: 장면의 핵심 구간(1-1 약 140 s, 1-2 180 s) 동안 몰림이 그대로다
    for g in crowd:
        first = g['steps'][0]
        assert first['path'] == [[g['spawn']['x'], g['spawn']['y']]] and first['wait_s'][0] >= 200.0, g['id']
        assert first['evade_dirs'], g['id']                                          # 로봇이 오면 비킬 곳이 있다
    start = [(g['spawn']['x'], g['spawn']['y']) for g in crowd]
    assert sum(inside('west_aisle', p) for p in start) == 4 > zones['west_aisle']['capacity']   # 혼잡도 1.0
    assert sum(inside('stall_front', p) for p in start) == 1                         # 1-1 켬 3호 비용 15.8 (1/6)
    assert not any(inside('buffet_line', p) for p in start)
    # 기본 손님은 처음에 서쪽 통로·매장 앞에 없다 (1-1 0초의 혼잡도는 몰림 손님이 정한다)
    assert not [g['id'] for g in base for z in ('west_aisle', 'stall_front', 'buffet_line')
                if inside(z, (g['spawn']['x'], g['spawn']['y']))]


# 제자리에서 도는 로봇: 위치는 그대로인데 속력이 0과 0.1 m/s를 오간다 (식당 리허설에서 손님과 서로 기다림)
def _jittering(t):
    return (3.0, 1.5, 0.0, 0.1 if int(t * 2) % 2 else 0.0)


def test_default_patience_keeps_waiting_for_a_jittering_robot():
    log = _walk(_jittering, 12.0)
    stop = next(i for i, (_, _, waiting) in enumerate(log) if waiting)
    assert all(waiting for _, _, waiting in log[stop:])


def test_progress_patience_walks_again_when_the_robot_does_not_move():
    spec = {**WORKER, 'yield_patience_s': 3.0, 'yield_progress_m': 0.1}
    m = ObstacleMotion(spec, random.Random(1))
    log = _walk(_jittering, 12.0, spec=spec, motion=m)
    stop = next(i for i, (_, _, waiting) in enumerate(log) if waiting)
    walk = next(i for i in range(stop, len(log)) if not log[i][2])
    assert log[walk][0] - log[stop][0] == pytest.approx(3.0, abs=2 * DT)
    assert m.turn_backs >= 1
    assert max(xy[1] for _, xy, _ in log[walk:]) > log[stop][1][1] + 1.0    # 북쪽으로 떠났다


def test_progress_patience_restarts_when_the_robot_moves():
    spec = {**WORKER, 'yield_patience_s': 3.0, 'yield_progress_m': 0.1}

    def creeping(t):    # 2 s마다 0.15 m씩 동쪽으로 간다 (7.5 s까지 길은 계속 막고 있다)
        return (3.0 + 0.15 * int(t / 2.0), 1.5, 0.0, 0.0)

    m = ObstacleMotion(spec, random.Random(1))
    log = _walk(creeping, 7.5, spec=spec, motion=m)
    assert any(waiting for _, _, waiting in log)
    assert m.turn_backs == 0                   # 3 s 안에 0.1 m 넘게 움직여 타이머가 다시 시작된다


def test_rejects_bad_patience():
    for extra in ({'yield_patience_s': 0.0}, {'yield_progress_m': -0.1}):
        with pytest.raises(ValueError):
            ObstacleMotion({**WORKER, **extra}, random.Random(0))


# ---- keep_away_m: 로봇 근처면 손님이 선다 (움직이는 로봇 포함) ----

def _line(x0=0.0, x1=10.0, pattern='straight'):
    pts = [{'x': x0, 'y': 0.0}, {'x': x1, 'y': 0.0}]
    if pattern == 'curve':
        pts.insert(1, {'x': 0.5 * (x0 + x1), 'y': 0.5})
    return {'id': 'g', 'type': 'customer', 'pattern': pattern, 'speed_range_mps': [0.5, 0.5],
            'spawn': {'x': x0, 'y': 0.0}, 'waypoints': pts}


def test_keep_away_off_by_default_walks_into_robot():
    m = ObstacleMotion(_line(), random.Random(0))
    assert m.step((2.0, 0.0), {'amr_01': (2.5, 0.0, 0.0, 0.5)}, 0.0) != (0.0, 0.0)


@pytest.mark.parametrize('pattern', ['straight', 'curve'])
def test_keep_away_stops_before_robot_ahead_even_if_moving(pattern):
    # 출발점(0, 0)에서 +x로 걷는다. 마주 오는 로봇이 1.0 m 앞
    m = ObstacleMotion(_line(pattern=pattern), random.Random(0), keep_away_m=1.2)
    assert m.step((0.0, 0.0), {'amr_01': (1.0, 0.0, math.pi, 0.5)}, 0.0) == (0.0, 0.0) and m.keeping_away
    # 로봇이 반경 밖(2.0 m)이면 걷는다
    assert m.step((0.0, 0.0), {'amr_01': (2.0, 0.0, math.pi, 0.5)}, 0.1) != (0.0, 0.0) and not m.keeping_away


def test_keep_away_walks_when_robot_is_behind():
    # 로봇이 등 뒤(걷는 방향 반대쪽)에 있으면 멀어지는 중이라 계속 걷는다
    m = ObstacleMotion(_line(), random.Random(0), keep_away_m=1.2)
    vx, _ = m.step((2.0, 0.0), {'amr_01': (1.5, 0.0, 0.0, 0.5)}, 0.0)
    assert vx > 0 and not m.keeping_away


def test_keep_away_resumes_when_robot_leaves():
    m = ObstacleMotion(_line(), random.Random(0), keep_away_m=1.2)
    assert m.step((2.0, 0.0), {'amr_01': (2.8, 0.0, 0.0, 0.0)}, 0.0) == (0.0, 0.0)
    assert m.step((2.0, 0.0), {'amr_01': (2.8, 3.0, 0.0, 0.5)}, 1.0) != (0.0, 0.0)
    assert m.turn_backs == 0


def test_keep_away_turns_back_after_patience():
    from amr_gazebo.obstacle_motion import KEEP_AWAY_PATIENCE_S
    m = ObstacleMotion(_line(), random.Random(0), keep_away_m=1.2)
    parked = {'amr_01': (2.8, 0.0, 0.0, 0.0)}
    assert m.step((2.0, 0.0), parked, 0.0) == (0.0, 0.0)
    assert m.step((2.0, 0.0), parked, KEEP_AWAY_PATIENCE_S - 0.1) == (0.0, 0.0)
    vx, _ = m.step((2.0, 0.0), parked, KEEP_AWAY_PATIENCE_S + 0.1)
    assert m.turn_backs == 1 and vx < 0          # 온 쪽(로봇 반대쪽)으로 걷는다
    vx, _ = m.step((1.98, 0.0), parked, KEEP_AWAY_PATIENCE_S + 0.2)
    assert vx < 0 and not m.keeping_away


def test_keep_away_simulated_guest_never_enters_radius_of_parked_robot():
    # 길 한가운데 멈춘 로봇: 손님은 1.2 m 안으로 들어가지 않고 돌아서 왕복한다
    m = ObstacleMotion(_line(), random.Random(0), keep_away_m=1.2)
    robot = {'amr_01': (6.0, 0.0, 0.0, 0.0)}
    xy, t, closest = (0.0, 0.0), 0.0, 99.0
    for _ in range(int(60 / DT)):
        vx, vy = m.step(xy, robot, t)
        xy = (xy[0] + vx * DT, xy[1] + vy * DT)
        t += DT
        closest = min(closest, math.dist(xy, (6.0, 0.0)))
    assert closest >= 1.2 - 0.5 * DT and m.turn_backs >= 2


def test_negative_keep_away_rejected():
    with pytest.raises(ValueError):
        ObstacleMotion(_line(), random.Random(0), keep_away_m=-1.0)


# ---- peers: 손님끼리 본다 (식당 시연, 손님이 한 점에 뭉치지 않게) ----

def _stand(gid, x, y, dirs=((0, 1), (0, -1), (1, 0), (-1, 0))):
    return {'id': gid, 'type': 'customer', 'pattern': 'stand', 'speed_range_mps': [0.02, 0.05],
            'spawn': {'x': x, 'y': y}, 'waypoints': [{'x': x - 0.2, 'y': y}, {'x': x + 0.2, 'y': y}],
            'evade_dirs': [list(d) for d in dirs]}


def test_peers_off_by_default_walks_into_guest():
    m = ObstacleMotion(_line(), random.Random(0))
    assert m.step((2.0, 0.0), {}, 0.0) != (0.0, 0.0)
    assert m.step((2.0, 0.0), {}, 0.0, peers=[]) != (0.0, 0.0)


def test_stops_for_guest_ahead_but_not_beside_or_behind():
    m = ObstacleMotion(_line(), random.Random(0))
    assert m.step((2.0, 0.0), {}, 0.0, peers=[(2.5, 0.1)]) == (0.0, 0.0) and m.peer_waiting
    m = ObstacleMotion(_line(), random.Random(0))
    assert m.step((2.0, 0.0), {}, 0.0, peers=[(2.5, 0.8)]) != (0.0, 0.0)     # 옆으로 0.8 m: 지나간다
    assert m.step((2.0, 0.0), {}, 0.0, peers=[(1.6, 0.0)]) != (0.0, 0.0)     # 등 뒤
    assert m.step((2.0, 0.0), {}, 0.0, peers=[(3.0, 0.0)]) != (0.0, 0.0)     # 1 m 앞: 아직 멀다


def test_walks_again_when_guest_ahead_leaves():
    m = ObstacleMotion(_line(), random.Random(0))
    assert m.step((2.0, 0.0), {}, 0.0, peers=[(2.5, 0.0)]) == (0.0, 0.0)
    vx, _ = m.step((2.0, 0.0), {}, 0.5, peers=[(2.5, 1.0)])
    assert vx > 0 and not m.peer_waiting and m.peer_turn_backs == 0


def test_turns_back_when_guest_ahead_stays():
    from amr_gazebo.obstacle_motion import PEER_PATIENCE_S
    m = ObstacleMotion(_line(), random.Random(0))
    still = [(2.5, 0.0)]
    assert m.step((2.0, 0.0), {}, 0.0, peers=still) == (0.0, 0.0)
    assert m.step((2.0, 0.0), {}, 0.5 * PEER_PATIENCE_S - 0.01, peers=still) == (0.0, 0.0)
    vx, _ = m.step((2.0, 0.0), {}, 1.5 * PEER_PATIENCE_S + 0.01, peers=still)
    assert m.peer_turn_backs == 1 and vx < 0


@pytest.mark.parametrize('seed', range(5))
def test_two_guests_walking_head_on_never_overlap(seed):
    # 같은 직선을 마주 보고 왕복하는 두 손님: 서로 보고 서거나 돌아서, 몸(반지름 0.25)이 겹치지 않는다
    rng = random.Random(seed)
    a = ObstacleMotion(_line(0.0, 6.0), random.Random(rng.random()))
    b = ObstacleMotion({**_line(6.0, 0.0), 'id': 'h'}, random.Random(rng.random()))
    pa, pb, t, closest = (0.0, 0.0), (6.0, 0.0), 0.0, 99.0
    for _ in range(int(60 / DT)):
        va = a.step(pa, {}, t, peers=[pb])
        vb = b.step(pb, {}, t, peers=[pa])
        pa = (pa[0] + va[0] * DT, pa[1] + va[1] * DT)
        pb = (pb[0] + vb[0] * DT, pb[1] + vb[1] * DT)
        t += DT
        closest = min(closest, math.dist(pa, pb))
    assert closest > 0.5 and a.peer_turn_backs + b.peer_turn_backs >= 2


def test_stand_evade_skips_spot_taken_by_another_guest():
    # 로봇이 +x로 다가온다. 위(+y)와 아래(-y)가 똑같이 진행선에서 멀다
    robot = {'amr_01': (-1.0, 0.0, 0.0, 0.5)}
    m = ObstacleMotion(_stand('g', 0.0, 0.0, dirs=((0, 1), (0, -1))), random.Random(0))
    vx, vy = m.step((0.0, 0.0), robot, 0.0, peers=[(0.0, 0.8)])     # 위 자리에 손님이 서 있다
    assert m.evading and m.claimed[1] < 0 and vy < 0
    m = ObstacleMotion(_stand('g', 0.0, 0.0, dirs=((0, 1), (0, -1))), random.Random(0))
    m.step((0.0, 0.0), robot, 0.0, peers=[(0.0, -0.8)])
    assert m.claimed[1] > 0


def test_stand_kiosk_queue_guests_evade_to_different_spots():
    # 키오스크 줄처럼 0.55 m 간격으로 앞뒤로 선 두 손님. 둘 다 같은 쪽이 가장 좋아도 겹치지 않게 비킨다
    robot = {'amr_01': (-1.2, -0.3, 0.0, 0.5)}
    a = ObstacleMotion(_stand('a', 0.0, 0.0), random.Random(0))
    b = ObstacleMotion(_stand('b', 0.0, -0.55), random.Random(1))
    a.step((0.0, 0.0), robot, 0.0, peers=[(0.0, -0.55)])
    b.step((0.0, -0.55), robot, 0.0, peers=[(0.0, 0.0), a.claimed])
    assert a.claimed and b.claimed and math.dist(a.claimed, b.claimed) >= 0.6


def test_stand_returning_home_waits_for_guest_in_the_way():
    m = ObstacleMotion(_stand('g', 0.0, 0.0), random.Random(0))
    m.step((0.0, 0.0), {'amr_01': (-1.0, 0.0, 0.0, 0.5)}, 0.0)
    assert m.evading
    far = {'amr_01': (-9.0, 0.0, 0.0, 0.5)}
    away = m.claimed
    assert m.step(away, far, 1.0) != (0.0, 0.0) and m.returning
    mid = (0.5 * away[0], 0.5 * away[1])
    assert m.step(away, far, 1.1, peers=[mid]) == (0.0, 0.0)
    # 오래 막히면 (둘 다 제자리로 가는 길) 그냥 간다
    assert m.step(away, far, 10.0, peers=[mid]) != (0.0, 0.0)


# ---- routine: 정해진 일과 (가서 머물기를 차례로, loop_from부터 되풀이) ----

def _routine(**kw):
    spec = {'id': 'r', 'type': 'customer', 'pattern': 'routine', 'speed_range_mps': [0.5, 0.5],
            'spawn': {'x': 0.0, 'y': 0.0},
            'steps': [{'name': 'seat', 'path': [[0.0, 0.0]], 'wait_s': [2.0, 2.0]},
                      {'name': 'water', 'path': [[2.0, 0.0], [2.0, 2.0]], 'wait_s': [1.0, 1.0],
                       'evade_dirs': [[1, 0], [-1, 0]]},
                      {'name': 'seat', 'path': [[2.0, 0.0], [0.0, 0.0]], 'wait_s': [3.0, 3.0]}],
            'loop_from': 1}
    spec.update(kw)
    return spec


def _run_routine(m, seconds, robots=None, xy=(0.0, 0.0)):
    t, names = 0.0, []
    for _ in range(int(seconds / DT)):
        vx, vy = m.step(xy, robots or {}, t)
        xy = (xy[0] + vx * DT, xy[1] + vy * DT)
        t += DT
        if not names or names[-1] != (m.step_name, m.dwelling):
            names.append((m.step_name, m.dwelling))
    return xy, names


def test_routine_walks_steps_in_order_and_loops():
    m = ObstacleMotion(_routine(), random.Random(0))
    _, names = _run_routine(m, 40.0)
    dwelt = [n for n, d in names if d]
    # seat(2 s) → water → seat → (loop_from 1) water → seat ...
    assert dwelt[:5] == ['seat', 'water', 'seat', 'water', 'seat']


def test_routine_stays_for_wait_s():
    m = ObstacleMotion(_routine(), random.Random(0))
    xy = (0.0, 0.0)
    assert m.step(xy, {}, 0.0) == (0.0, 0.0) and m.dwelling      # 첫 자리에 이미 있다
    assert m.step(xy, {}, 1.9) == (0.0, 0.0)
    vx, _ = m.step(xy, {}, 2.1)
    assert vx > 0 and not m.dwelling and m.step_name == 'water'


def test_routine_follows_path_points():
    m = ObstacleMotion(_routine(), random.Random(0))
    xy, _ = _run_routine(m, 2.1 + 2.0 / 0.5 + 0.5)               # 2 s 머문 뒤 (2, 0)까지 4 s 걷는다
    # 첫 경유점 (2, 0)을 꺾어 (2, 2) 쪽으로 걷는 중 (대각선으로 질러가지 않는다)
    assert abs(xy[0] - 2.0) < 0.15 and 0.0 < xy[1] < 1.0 and m.goal == (2.0, 2.0)


def test_routine_dwell_evades_robot_and_comes_back():
    m = ObstacleMotion(_routine(steps=[{'name': 'seat', 'path': [[0.0, 0.0]], 'wait_s': [100.0, 100.0],
                                        'evade_dirs': [[0, 1], [0, -1]]}], loop_from=0), random.Random(0))
    xy = (0.0, 0.0)
    assert m.step(xy, {}, 0.0) == (0.0, 0.0)
    coming = {'amr_01': (-1.0, 0.0, 0.0, 0.5)}
    vx, vy = m.step(xy, coming, 0.1)
    assert m.evading and abs(vy) > 0 and abs(vx) < 1e-9
    away = m.claimed
    far = {'amr_01': (-9.0, 0.0, 0.0, 0.5)}
    m.step(away, far, 0.2)
    assert m.returning and m.dwelling
    vx, vy = m.step(away, far, 0.3)
    assert math.hypot(vx, vy) > 0


def test_routine_stops_for_robot_and_peer_while_walking():
    # 다 머물고 water로 걷기 시작한 뒤 (로봇이 1.5 m 안에 있으면 걷기 전에 먼저 비킨다)
    m = ObstacleMotion(_routine(), random.Random(0), keep_away_m=1.2)
    m.step((0.0, 0.0), {}, 0.0)
    assert m.step((0.0, 0.0), {}, 2.1) != (0.0, 0.0)
    assert m.step((0.1, 0.0), {'amr_01': (1.2, 0.0, math.pi, 0.0)}, 2.2) == (0.0, 0.0) and m.keeping_away
    m = ObstacleMotion(_routine(), random.Random(0))
    m.step((0.0, 0.0), {}, 0.0)
    assert m.step((0.0, 0.0), {}, 2.1) != (0.0, 0.0)
    assert m.step((0.1, 0.0), {}, 2.2, peers=[(0.6, 0.0)]) == (0.0, 0.0) and m.peer_waiting


def test_routine_turn_back_steps_to_previous_point():
    from amr_gazebo.obstacle_motion import PEER_PATIENCE_S
    m = ObstacleMotion(_routine(), random.Random(0))
    m.step((0.0, 0.0), {}, 0.0)
    m.step((0.0, 0.0), {}, 2.1)                                 # water 단계로 걷기 시작
    xy = (2.0, 0.05)                                            # 첫 경유점 (2, 0)에 닿았다
    m.step(xy, {}, 3.0)
    assert m.goal == (2.0, 2.0)
    still = [(2.0, 0.6)]
    assert m.step(xy, {}, 3.1, peers=still) == (0.0, 0.0)
    m.step(xy, {}, 3.1 + 1.5 * PEER_PATIENCE_S + 0.1, peers=still)
    assert m.peer_turn_backs == 1 and m.goal == (2.0, 0.0)


def test_rejects_bad_routine():
    for bad in ({'steps': []}, {'loop_from': 3},
                {'steps': [{'name': 'a', 'path': [], 'wait_s': [1, 2]}]},
                {'steps': [{'name': 'a', 'path': [[0, 0]], 'wait_s': [3, 1]}]}):
        with pytest.raises(ValueError):
            ObstacleMotion(_routine(**bad), random.Random(0))


@pytest.mark.parametrize('spec_name, out_name', [
    ('guest_routines_restaurant.yaml', 'dynamic_obstacles_restaurant_routine.yaml'),
    ('guest_routines_restaurant_peak.yaml', 'dynamic_obstacles_restaurant_routine_peak.yaml')])
def test_restaurant_routine_files_are_up_to_date_and_valid(spec_name, out_name):
    # 생성 파일이 설정·지도와 맞는지 (설정을 고치고 다시 안 만든 것을 잡는다), 경유점 사이가 몸 여유를 지키는지
    from amr_gazebo.guest_routes import GridMap, build_obstacles, load_spec
    here = os.path.dirname(__file__)
    spec = load_spec(os.path.join(here, '..', 'config', spec_name))
    grid = GridMap.load(os.path.join(here, '..', '..', '..', 'localization', 'amr_mapping', 'maps',
                                     'restaurant_gt.yaml'))
    with open(os.path.join(here, '..', 'config', out_name), encoding='utf-8') as f:
        generated = yaml.safe_load(f)['obstacles']
    want = build_obstacles(spec, grid)
    assert [o['id'] for o in generated] == [o['id'] for o in want]
    for g, w in zip(generated, want):
        assert g['steps'] == w['steps'] and g['spawn'] == w['spawn'], g['id']
        assert ObstacleMotion(g, random.Random(0)).pattern == 'routine'
        prev = (g['spawn']['x'], g['spawn']['y'])
        for st in g['steps']:
            for p in st['path']:
                assert grid.segment_clear(prev, tuple(p), 0.3), (g['id'], st['name'], prev, p)
                prev = tuple(p)
