"""회피 KPI 평가(amr_safety.avoidance_eval) 단위 테스트. ROS 없이 합성 궤적으로 본다."""

import json
import math

import pytest

from amr_safety.avoidance_eval import (
    AvoidanceParams,
    AvoidanceRecorder,
    Pose,
    Shape,
    aggregate,
    aggregate_markdown,
    evaluate_run,
    footprint_clearance,
    load_recorder,
    polyline_distance,
    shape_for_id,
    transform_path,
)
from amr_safety.avoidance_eval.cli import report_main
from amr_safety.avoidance_eval.metrics import clopper_pearson
from amr_safety.avoidance_eval.obstacles import CORRIDOR_Y, CROSS_X, SPEC_SPEED, SPEED_RANGE, make_obstacles

L, W = 0.6, 0.4
WORKER = Shape('circle', radius=0.25)
ROBOT_BOX = Shape('box', length=0.6, width=0.4)
REF = [(0.0, 0.0), (6.0, 0.0)]
DT = 0.01


def detour_y(x, offset=0.6):
    """x 2.0~2.5 올라가고 3.5까지 offset 유지, 4.0까지 내려온다."""
    if x < 2.0 or x > 4.0:
        return 0.0
    if x < 2.5:
        return offset * (x - 2.0) / 0.5
    if x <= 3.5:
        return offset
    return offset * (4.0 - x) / 0.5


def drive(y_of_x=detour_y, speed=0.5, x_end=6.0, t0=0.0):
    n = int(round(x_end / speed / DT))
    return [Pose(t0 + i * DT, speed * i * DT, y_of_x(speed * i * DT), 0.0) for i in range(n + 1)]


def static(x, y, t_end=20.0, yaw=0.0):
    return [Pose(i * 0.02, x, y, yaw, 0.0) for i in range(int(t_end / 0.02) + 1)]


# --- 기하 ---

def test_circle_clearance_uses_rectangle_not_circle():
    r = Pose(0.0, 0.0, 0.0, 0.0)
    assert footprint_clearance(r, L, W, Pose(0, 1.0, 0.0), WORKER) == pytest.approx(1.0 - 0.3 - 0.25)
    assert footprint_clearance(r, L, W, Pose(0, 0.0, 1.0), WORKER) == pytest.approx(1.0 - 0.2 - 0.25)
    # 로봇이 90도 돌면 옆·앞이 바뀐다
    r90 = Pose(0.0, 0.0, 0.0, math.pi / 2)
    assert footprint_clearance(r90, L, W, Pose(0, 0.0, 1.0), WORKER) == pytest.approx(1.0 - 0.3 - 0.25)
    # 모서리 쪽: 모서리 (0.3, 0.2)까지 거리
    assert footprint_clearance(r, L, W, Pose(0, 1.3, 1.2), WORKER) == pytest.approx(math.hypot(1, 1) - 0.25)


def test_circle_overlap_is_negative():
    r = Pose(0.0, 0.0, 0.0, 0.0)
    assert footprint_clearance(r, L, W, Pose(0, 0.5, 0.0), WORKER) == pytest.approx(-0.05)
    assert footprint_clearance(r, L, W, Pose(0, 0.0, 0.0), WORKER) < -0.25


def test_box_clearance():
    r = Pose(0.0, 0.0, 0.0, 0.0)
    assert footprint_clearance(r, L, W, Pose(0, 1.0, 0.0, 0.0), ROBOT_BOX) == pytest.approx(0.4)
    assert footprint_clearance(r, L, W, Pose(0, 0.0, 1.0, 0.0), ROBOT_BOX) == pytest.approx(0.6)
    assert footprint_clearance(r, L, W, Pose(0, 0.5, 0.1, 0.3), ROBOT_BOX) == 0.0
    # 45도 돈 상자의 꼭짓점이 로봇 앞면을 향한다
    half_diag = math.hypot(0.3, 0.2)
    c = footprint_clearance(r, L, W, Pose(0, 1.0, 0.0, math.atan2(0.2, 0.3)), ROBOT_BOX)
    assert c == pytest.approx(1.0 - 0.3 - half_diag)


def test_shape_for_id_from_gazebo_models():
    assert shape_for_id('worker_1') == WORKER
    assert shape_for_id('forklift_2').length == pytest.approx(1.2)
    assert shape_for_id('robot_1') == ROBOT_BOX
    with pytest.raises(KeyError):
        shape_for_id('cart_1')
    assert shape_for_id('cart_1', default_radius=0.4).radius == pytest.approx(0.4)


def test_polyline_and_transform():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    assert polyline_distance(1.0, 0.5, path) == pytest.approx(0.5)
    assert polyline_distance(3.0, 1.0, path) == pytest.approx(1.0)
    assert polyline_distance(-1.0, 0.0, path) == pytest.approx(1.0)     # 끝점 밖은 끝점까지 거리
    assert transform_path([(1.0, 0.0)], [0.0, 1.5, 0.0]) == [(1.0, 1.5)]
    x, y = transform_path([(1.0, 0.0)], [1.0, 0.0, math.pi / 2])[0]
    assert (x, y) == pytest.approx((1.0, 1.0))


# --- 실행 하나 ---

def test_detour_around_obstacle_passes():
    res = evaluate_run(drive(), {'worker_1': static(3.0, 0.0)}, {'worker_1': WORKER}, REF)
    s = res.summary
    assert s['collision']['count'] == 0
    assert s['collision']['min_clearance'] == pytest.approx(0.6 - 0.2 - 0.25, abs=1e-3)
    assert s['collision']['near_miss']                       # 0.15 m < 0.3 m
    assert s['deviation']['max'] == pytest.approx(0.6, abs=1e-3)
    assert len(res.episodes) == 1
    ep = res.episodes[0]
    assert ep.obstacle_related and ep.trigger == 'deviation'
    # 이탈 0.1 m 초과: x = 2 + 0.5/6 → t = 4.167 s, 0.2 m 이하 복귀: x = 4 − 0.5/3 → t = 7.667 s
    assert ep.start_t == pytest.approx(4.167, abs=0.02)
    assert ep.return_s == pytest.approx(3.5, abs=0.03)
    assert s['avoidance']['max_return_s'] == pytest.approx(3.5, abs=0.03)
    assert s['reached_goal'] and s['encounter']
    assert s['kpi']['pass']


def test_collision_counted_once_per_contact():
    straight = drive(lambda x: 0.0)
    res = evaluate_run(straight, {'worker_1': static(3.0, 0.0)}, {'worker_1': WORKER}, REF)
    s = res.summary
    assert s['collision']['count'] == 1
    ev = s['collision']['events'][0]
    assert ev['obstacle'] == 'worker_1'
    assert ev['robot_speed'] == pytest.approx(0.5, abs=1e-6)
    assert not s['kpi']['collision_free'] and not s['kpi']['pass']
    assert sum(r.collision for r in res.samples) > 1


def test_large_deviation_fails():
    res = evaluate_run(drive(lambda x: detour_y(x, 1.3)), {'worker_1': static(3.0, 0.0)},
                       {'worker_1': WORKER}, REF)
    assert res.summary['deviation']['max'] == pytest.approx(1.3, abs=1e-3)
    assert not res.summary['kpi']['deviation_ok']


def test_slow_return_fails_and_never_return_is_reported():
    # 복귀가 느리다: x 3.5~6.0에 걸쳐 내려온다 (0.5 m/s로 5 s 넘게)
    def slow(x):
        return 0.0 if x < 2.0 else 0.6 if x < 3.0 else max(0.0, 0.6 * (6.0 - x) / 3.0)
    res = evaluate_run(drive(slow, x_end=8.0), {'worker_1': static(2.5, 0.0)}, {'worker_1': WORKER},
                       [(0.0, 0.0), (8.0, 0.0)])
    assert res.summary['avoidance']['max_return_s'] > 5.0
    assert not res.summary['kpi']['return_ok']

    # 끝까지 경로로 안 돌아온다
    res = evaluate_run(drive(lambda x: 0.0 if x < 2.0 else 0.6), {'worker_1': static(3.0, 0.0)},
                       {'worker_1': WORKER}, REF)
    assert res.summary['avoidance']['unreturned'] == 1
    assert res.episodes[0].return_s is None
    assert not res.summary['kpi']['return_ok']
    assert not res.summary['reached_goal']


def test_return_needs_hold():
    # 0.2 m 밑으로 잠깐(0.4 s) 내려왔다가 다시 벗어나면 그때는 복귀가 아니다
    def wobble(x):
        if 3.0 <= x < 3.2:
            return 0.1
        return detour_y(x)
    res = evaluate_run(drive(wobble), {'worker_1': static(3.0, 1.5)}, {'worker_1': WORKER}, REF)
    assert len(res.episodes) == 1
    assert res.episodes[0].return_s == pytest.approx(3.5, abs=0.03)


def test_tracking_deviation_without_obstacle_is_not_avoidance():
    obstacle_far = {'worker_1': static(3.0, 10.0)}
    res = evaluate_run(drive(), obstacle_far, {'worker_1': WORKER}, REF)
    s = res.summary
    assert s['avoidance']['episodes'] == 0 and s['avoidance']['tracking_episodes'] == 1
    assert not s['encounter'] and not s['kpi']['valid'] and not s['kpi']['pass']


def test_proximity_start_counts_approach_time():
    p = AvoidanceParams(start_on='proximity')
    res = evaluate_run(drive(), {'worker_1': static(3.0, 0.0)}, {'worker_1': WORKER}, REF, p)
    ep = res.episodes[0]
    assert ep.trigger == 'proximity'
    # 이격 1.5 m 안: 로봇 앞면이 장애물 원에서 1.5 m → x = 3 − 0.25 − 1.5 − 0.3 = 0.95 → t = 1.9 s
    assert ep.start_t == pytest.approx(1.9, abs=0.02)
    assert ep.return_s == pytest.approx(7.667 - 1.9, abs=0.03)


def test_samples_before_reference_are_ignored():
    robot = [Pose(t * 0.1, 0.0, 5.0) for t in range(10)] + drive(t0=1.0)
    res = evaluate_run(robot, {'worker_1': static(3.0, 0.0)}, {'worker_1': WORKER}, REF, reference_t=1.0)
    assert res.summary['deviation']['max'] == pytest.approx(0.6, abs=1e-3)


def test_obstacle_gap_is_unknown_not_interpolated():
    obs = [Pose(0.0, 3.0, 0.0, 0.0), Pose(10.0, 3.0, 0.0, 0.0)]          # 10 s 끊김
    res = evaluate_run(drive(lambda x: 0.0), {'worker_1': obs}, {'worker_1': WORKER}, REF)
    # 끊긴 사이(로봇이 장애물을 지나는 t 4~8 s)는 모르는 것으로 보고, 양 끝 샘플 시각만 잰다
    assert res.summary['collision']['count'] == 0
    assert res.summary['collision']['min_clearance'] == pytest.approx(5.0 - 3.0 - 0.3 - 0.25)


# --- 여러 실행 ---

def _run_summary(**over):
    s = evaluate_run(drive(), {'worker_1': static(3.0, 0.0)}, {'worker_1': WORKER}, REF).summary
    for k, v in over.items():
        s['kpi'][k] = v
    return s


def test_aggregate_verdicts():
    ok = _run_summary()
    a = aggregate([(f'r{i}', ok) for i in range(30)])
    assert a['verdict'] == 'PASS' and a['valid_runs'] == 30
    assert a['max_deviation'] == pytest.approx(0.6, abs=1e-3)

    a = aggregate([(f'r{i}', ok) for i in range(29)] + [('missing', None)])
    assert a['verdict'] == 'INSUFFICIENT' and a['valid_runs'] == 29

    bad = _run_summary(**{'pass': False, 'return_ok': False})
    a = aggregate([(f'r{i}', ok) for i in range(30)] + [('bad', bad)])
    assert a['verdict'] == 'FAIL' and a['failing_runs'] == ['bad']

    # 요약이 없는 실행이 있으면 유효 30회여도 판정 보류 (재지 못한 실행을 빼고 PASS 주지 않는다)
    a = aggregate([(f'r{i}', ok) for i in range(30)] + [('missing', None)])
    assert a['verdict'] == 'INSUFFICIENT' and a['insufficient_reasons'] == ['요약 없음 1회']


def _collided(summary, valid):
    s = json.loads(json.dumps(summary))
    s['collision']['count'] = 1
    s['kpi'].update({'collision_free': False, 'valid': valid, 'pass': False})
    if not valid:
        s['kpi']['invalid_reasons'] = ['기준 경로 없음']
    return s


def test_aggregate_counts_collisions_in_every_run():
    ok = _run_summary()
    # 무효 실행(기준 경로를 놓침)에서 난 충돌도 FAIL
    a = aggregate([(f'r{i}', ok) for i in range(30)] + [('no_plan', _collided(ok, valid=False))])
    assert a['verdict'] == 'FAIL' and a['failing_runs'] == ['no_plan']
    assert a['collisions'] == 1 and a['valid_runs'] == 30
    # 하네스 실패 실행: 요약이 없으면 판정 보류, 요약에 충돌이 있으면 FAIL
    a = aggregate([(f'r{i}', ok) for i in range(30)], failed=[('crash', 'timeout')])
    assert a['verdict'] == 'INSUFFICIENT' and a['insufficient_reasons'] == ['하네스 실패 1회']
    a = aggregate([(f'r{i}', ok) for i in range(30)], failed=[('crash', 'timeout', None)])
    assert a['verdict'] == 'INSUFFICIENT'
    a = aggregate([(f'r{i}', ok) for i in range(30)], failed=[('crash', 'timeout', _collided(ok, valid=True))])
    assert a['verdict'] == 'FAIL' and a['failing_runs'] == ['crash'] and a['valid_runs'] == 30
    assert a['harness_failed'] == [{'run': 'crash', 'outcome': 'timeout'}]
    md = aggregate_markdown(a)
    assert '하네스 실패' in md and '충돌 (건, 모든 실행) | 1' in md



def test_invalid_cause_when_obstacle_never_met():
    # 장애물이 멀리 있어 만나지 못한 실행: 목표 도착 / 거의 안 움직임 / 중간에 멈춤으로 나눈다
    far = {'worker_1': static(3.0, 10.0)}

    def reasons(robot):
        return evaluate_run(robot, far, {'worker_1': WORKER}, REF).summary['kpi']['invalid_reasons']
    assert reasons(drive()) == ['장애물이 1.5 m 안에 오지 않음: 목표 도착']
    assert reasons(drive(x_end=0.3)) == ['장애물이 1.5 m 안에 오지 않음: 로봇이 거의 안 움직임']
    assert reasons(drive(x_end=3.0)) == ['장애물이 1.5 m 안에 오지 않음: 목표 전에 멈춤']
    straight = evaluate_run(drive(lambda x: 0.0, x_end=3.0), far, {'worker_1': WORKER}, REF).summary
    assert straight['travel_m'] == pytest.approx(3.0, abs=1e-3)
    # 비켜 간 길(x 2.0~2.5에서 0.6 m 올라감)도 이동 거리에 들어간다
    detour = evaluate_run(drive(x_end=3.0), far, {'worker_1': WORKER}, REF).summary
    assert detour['travel_m'] == pytest.approx(3.0 + math.hypot(0.5, 0.6) - 0.5, abs=1e-3)


def test_aggregate_counts_invalid_reasons_and_pass_rate():
    ok = _run_summary()
    far = {'worker_1': static(3.0, 10.0)}
    stuck = evaluate_run(drive(x_end=0.3), far, {'worker_1': WORKER}, REF).summary
    a = aggregate([(f'r{i}', ok) for i in range(30)] + [('stuck1', stuck), ('stuck2', stuck), ('none', None)],
                  failed=[('crash', 'timeout')])
    assert a['invalid_reason_counts'] == {'장애물이 1.5 m 안에 오지 않음: 로봇이 거의 안 움직임': 2, '요약 없음': 1}
    assert a['harness_outcome_counts'] == {'timeout': 1}
    assert a['pass_rate']['passed'] == 30 and a['pass_rate']['valid'] == 30
    assert a['pass_rate']['ci95'] == [pytest.approx(0.025 ** (1 / 30), abs=1e-4), 1.0]
    md = aggregate_markdown(a)
    assert '2회: 장애물이 1.5 m 안에 오지 않음: 로봇이 거의 안 움직임' in md and '1회: 하네스 실패 (timeout)' in md
    assert '30/30 (88.4~100.0%)' in md


def test_clopper_pearson_known_values():
    lo, hi = clopper_pearson(30, 30)
    assert lo == pytest.approx(0.025 ** (1 / 30), abs=1e-6) and hi == 1.0      # 0.884: 실패율 상한 약 12%
    lo, hi = clopper_pearson(0, 30)
    assert lo == 0.0 and hi == pytest.approx(1 - 0.025 ** (1 / 30), abs=1e-6)
    lo, hi = clopper_pearson(15, 30)
    assert lo == pytest.approx(0.3130, abs=1e-3) and hi == pytest.approx(0.6870, abs=1e-3)
    assert clopper_pearson(0, 0) == (None, None)



def test_missing_data_is_invalid_with_reason(tmp_path):
    # 로봇 GT도 장애물 GT도 없어도 요약을 낸다 (하네스가 요약을 기다리다 끝나지 않게)
    assert evaluate_run([], {}, {}, REF).summary['kpi']['invalid_reasons'] == ['로봇 GT 없음']
    s = evaluate_run(drive(), {}, {}, REF).summary
    assert s['kpi']['invalid_reasons'] == ['장애물 GT 없음'] and not s['kpi']['valid']
    rec = AvoidanceRecorder('amr_01', [0.0, 1.5, 0.0], {'worker_1': WORKER}, stream_dir=str(tmp_path))
    summary = rec.save(str(tmp_path))
    rec.close()
    assert not summary['kpi']['valid'] and '로봇 GT 없음' in summary['kpi']['invalid_reasons']
    assert (tmp_path / 'avoidance_summary.json').exists()


# --- 장애물 설정 (시드) ---

def test_obstacles_from_seed_vary_and_repeat():
    assert make_obstacles(5) == make_obstacles(5)          # 같은 시드면 같은 시험
    specs = [make_obstacles(seed)['obstacles'][0] for seed in range(1, 31)]
    assert len({(o['spawn']['x'], o['spawn']['y']) for o in specs}) == 30
    speeds = [o['speed_range_mps'][0] for o in specs]
    assert speeds.count(SPEC_SPEED) == 10                   # 시드 3개 중 1개는 원 명세 1.0 m/s
    assert all(SPEED_RANGE[0] <= v <= SPEED_RANGE[1] for v in speeds) and len(set(speeds)) > 10
    southward = []
    for o in specs:
        (a, b) = o['waypoints']
        assert o['id'] == 'worker_cross' and o['pattern'] == 'straight' and o['type'] == 'worker'
        assert CROSS_X[0] <= o['spawn']['x'] <= CROSS_X[1] and a['x'] == b['x'] == o['spawn']['x']
        assert CORRIDOR_Y[0] <= o['spawn']['y'] <= CORRIDOR_Y[1]
        assert {a['y'], b['y']} == set(CORRIDOR_Y)
        # straight 패턴은 처음에 두 번째 waypoint로 간다. 몸 방향도 그쪽
        south = b['y'] < a['y']
        assert o['spawn']['yaw'] == pytest.approx(-math.pi / 2 if south else math.pi / 2, abs=1e-3)
        southward.append(south)
    assert 0 < sum(southward) < 30                          # 두 방향 모두 나온다


# --- 기록·재계산 ---

def test_recorder_round_trip(tmp_path):
    origin = [0.0, 1.5, 0.0]
    rec = AvoidanceRecorder('amr_01', origin, {'worker_1': WORKER}, stream_dir=str(tmp_path))
    for pose in static(3.0, 1.5):
        rec.add_obstacle('worker_1', pose)
    # 로봇 GT는 world, 경로는 map (world = map + (0, 1.5))
    for pose in drive():
        rec.add_robot(Pose(pose.t, pose.x, pose.y + 1.5, pose.yaw))
    assert rec.add_plan(0.0, 'map', REF)
    assert not rec.add_plan(1.0, 'map', [(0.0, 0.0), (3.0, 1.0), (6.0, 0.0)])   # 재계획은 기록만
    summary = rec.save(str(tmp_path))
    rec.close()
    assert summary['plans_received'] == 2
    assert summary['deviation']['max'] == pytest.approx(0.6, abs=1e-3)
    assert summary['kpi']['pass']
    header = (tmp_path / 'avoidance.csv').read_text(encoding='utf-8').splitlines()[0]
    assert header.startswith('timestamp,robot_x,robot_y')

    again = load_recorder(str(tmp_path))
    assert again.counts == rec.counts
    assert again.evaluate().summary['deviation'] == summary['deviation']
    stricter = load_recorder(str(tmp_path), AvoidanceParams(kpi_return_s=3.0))
    assert not stricter.evaluate().summary['kpi']['return_ok']


def test_fixed_reference_mode():
    with pytest.raises(ValueError):
        AvoidanceRecorder('amr_01', [0, 0, 0], {}, reference_mode='fixed')
    rec = AvoidanceRecorder('amr_01', [0.0, 1.5, 0.0], {}, reference_mode='fixed', fixed_path=REF)
    rec.add_robot(Pose(2.0, 1.0, 2.0))
    ref, t = rec.reference()
    assert ref == [(0.0, 1.5), (6.0, 1.5)] and t == 2.0


def test_report_cli(tmp_path, capsys):
    for i in range(3):
        run = tmp_path / f'nav_avoidance_{i}'
        (run / 'avoidance').mkdir(parents=True)
        (run / 'meta.json').write_text(json.dumps({'outcome': 'completed'}))
        (run / 'avoidance' / 'avoidance_summary.json').write_text(json.dumps(_run_summary()))
    failed = tmp_path / 'nav_avoidance_9'
    failed.mkdir()
    (failed / 'meta.json').write_text(json.dumps({'outcome': 'timeout'}))
    out = tmp_path / 'report.json'
    code = report_main([str(p) for p in sorted(tmp_path.iterdir()) if p.is_dir()]
                       + ['--runs', '3', '--json', str(out)])
    # 하네스 실패 실행(요약 없음)이 있으면 유효 3회를 채워도 판정 보류
    assert code == 1
    result = json.loads(out.read_text(encoding='utf-8'))
    assert result['verdict'] == 'INSUFFICIENT' and result['valid_runs'] == 3
    assert result['harness_failed'] == [{'run': 'nav_avoidance_9', 'outcome': 'timeout'}]
    assert 'I17' in capsys.readouterr().out
    # 실패 실행의 요약은 읽는다: 그 안의 충돌은 FAIL
    (failed / 'avoidance').mkdir()
    collided = _run_summary(**{'collision_free': False, 'pass': False})
    collided['collision']['count'] = 1
    (failed / 'avoidance' / 'avoidance_summary.json').write_text(json.dumps(collided))
    code = report_main([str(p) for p in sorted(tmp_path.iterdir()) if p.is_dir()]
                       + ['--runs', '3', '--json', str(out)])
    result = json.loads(out.read_text(encoding='utf-8'))
    assert code == 1 and result['verdict'] == 'FAIL'
    assert result['failing_runs'] == ['nav_avoidance_9/avoidance']
    # 실패 실행 없이 3회면 PASS
    for p in (failed / 'avoidance' / 'avoidance_summary.json', failed / 'meta.json'):
        p.unlink()
    (failed / 'avoidance').rmdir()
    failed.rmdir()
    assert report_main([str(p) for p in sorted(tmp_path.iterdir()) if p.is_dir()] + ['--runs', '3']) == 0
