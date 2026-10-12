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
    evaluate_run,
    footprint_clearance,
    load_recorder,
    polyline_distance,
    shape_for_id,
    transform_path,
)
from amr_safety.avoidance_eval.cli import report_main

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
    assert code == 0
    result = json.loads(out.read_text(encoding='utf-8'))
    assert result['verdict'] == 'PASS' and result['valid_runs'] == 3
    assert result['harness_failed'] == [{'run': 'nav_avoidance_9', 'outcome': 'timeout'}]
    assert 'I17' in capsys.readouterr().out
