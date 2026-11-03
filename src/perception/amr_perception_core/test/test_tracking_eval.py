import math

import numpy as np
import pytest

from amr_perception_core.clustering import transform_points_2d
from amr_perception_core.collision import footprint_clearance, footprint_time_to_collision
from amr_perception_core.tracking_eval import (
    associate_nearest,
    body_risks,
    EvalConfig,
    format_summary_ko,
    se2_compose,
    se2_inverse,
    TimeSeries,
    TrackingEvaluator,
    TrackObs,
    world_from_odom,
    wrap_angle,
)

DT = 0.1


def gt_line(ev, gid, t0, t1, start, velocity, dt=0.02):
    """start에서 velocity로 등속 이동하는 GT를 t0~t1 동안 dt 간격으로 넣는다."""
    for k in range(int(round((t1 - t0) / dt)) + 1):
        t = t0 + k * dt
        ev.add_gt(gid, t, start[0] + velocity[0] * (t - t0), start[1] + velocity[1] * (t - t0),
                  velocity[0], velocity[1])


def test_se2_inverse_and_world_from_odom():
    a = (1.0, -2.0, 0.7)
    x, y, th = se2_compose(a, se2_inverse(a))
    assert (x, y, th) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    # world → odom = (5, 1, 90°). 로봇은 odom (2, 0, 0)에 있다 → world (5, 3, 90°)
    world_odom = (5.0, 1.0, math.pi / 2)
    odom_base = (2.0, 0.0, 0.0)
    world_base = se2_compose(world_odom, odom_base)
    assert world_base == pytest.approx((5.0, 3.0, math.pi / 2))
    assert world_from_odom(world_base, odom_base) == pytest.approx(world_odom)


def test_track_transformed_rotates_velocity_and_covariance():
    tr = TrackObs(3, 1.0, 0.0, 1.0, 0.0, True, pos_cov=np.diag([0.04, 0.01]))
    out = tr.transformed((10.0, 0.0, math.pi / 2))
    assert (out.x, out.y) == pytest.approx((10.0, 1.0))
    assert (out.vx, out.vy) == pytest.approx((0.0, 1.0), abs=1e-12)
    np.testing.assert_allclose(out.pos_cov, np.diag([0.01, 0.04]), atol=1e-12)
    assert out.vel_cov is None and out.track_id == 3 and out.is_dynamic


def test_time_series_interpolates_angles_and_refuses_gaps():
    ts = TimeSeries(angle_columns=(1,))
    ts.add(0.0, (0.0, math.pi - 0.1))
    ts.add(1.0, (2.0, -math.pi + 0.1))
    ts.add(5.0, (10.0, 0.0))
    v = ts.at(0.5, max_gap=2.0)
    assert v[0] == pytest.approx(1.0)
    assert abs(wrap_angle(v[1] - math.pi)) < 1e-9   # ±π를 넘어 짧은 쪽으로 보간
    assert ts.at(3.0, max_gap=2.0) is None           # 간격 4 s > 2 s
    assert ts.at(-0.1, max_gap=2.0) is None          # 범위 밖은 외삽하지 않는다
    assert ts.at(5.0, max_gap=2.0)[0] == 10.0
    ts.add(0.5, (7.0, 0.0))                           # 늦게 온 샘플도 시간순으로 들어간다
    assert ts.at(0.5, max_gap=2.0)[0] == 7.0


def test_associate_nearest_is_one_to_one_and_gated():
    tracks = [[0.0, 0.0], [0.3, 0.0], [5.0, 5.0]]
    gts = [[0.25, 0.0], [10.0, 0.0]]
    pairs = associate_nearest(tracks, gts, gate=1.0)
    assert [(i, j) for i, j, _ in pairs] == [(1, 0)]   # 가장 가까운 쌍만, GT 하나에 트랙 하나
    assert pairs[0][2] == pytest.approx(0.05)
    assert associate_nearest([], gts, 1.0) == []


def perfect_run(noise=0.0, cov=None, seed=0):
    """GT 1개(1 m/s, x 방향)를 트랙 7번이 따라간다. 1.0 s부터 동적."""
    rng = np.random.default_rng(seed)
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['worker_1'])
    gt_line(ev, 'worker_1', 0.0, 8.0, (0.0, 0.0), (1.0, 0.0))
    for k in range(51):
        t = k * DT
        e = rng.normal(0.0, noise, 2) if noise else np.zeros(2)
        ev.add_frame(t, [TrackObs(7, t + e[0], e[1], 1.0, 0.0, t >= 1.0,
                                  pos_cov=cov)], robot_xy=(0.0, 3.0))
    return ev


def test_perfect_tracking_has_zero_errors():
    rows, episodes, summary = perfect_run().evaluate()
    o = summary['overall']
    assert summary['frames'] == 51
    assert o['pos']['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert o['vel']['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert o['heading']['mean'] == pytest.approx(0.0, abs=1e-6)
    # 5 s까지 프레임, GT는 8 s까지 → +1 s, +2 s 예측 모두 계산되고 등속이라 0
    assert o['pred_1']['n'] == 51 and o['pred_2']['n'] == 51
    assert o['pred_1']['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert o['hold_2']['rmse'] == pytest.approx(2.0)          # 정지 가정 예측은 2 s × 1 m/s
    assert o['id_switches'] == 0
    assert o['match_rate_in_range'] == 1.0
    assert summary['false_dynamic_per_frame'] == 0.0
    assert all(r['kind'] == 'match' for r in rows)


def test_prediction_skipped_without_future_gt():
    ev = TrackingEvaluator(EvalConfig(horizons=(1.0,)), gt_ids=['a'])
    gt_line(ev, 'a', 0.0, 1.5, (0.0, 0.0), (1.0, 0.0))
    for k in range(11):
        t = k * DT
        ev.add_frame(t, [TrackObs(0, t, 0.0, 1.0, 0.0, False)])
    _, _, summary = ev.evaluate()
    assert summary['overall']['pred_1']['n'] == 6     # t ≤ 0.5 s만 +1 s GT가 있다


def test_velocity_and_heading_error():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a'])
    gt_line(ev, 'a', 0.0, 3.0, (0.0, 0.0), (1.0, 0.0))
    for k in range(11):
        t = k * DT
        # 속도 (0, 1): 크기 같고 방향 90° 틀림 → 벡터 오차 √2
        ev.add_frame(t, [TrackObs(0, t + 0.1, 0.0, 0.0, 1.0, True)])
    rows, _, summary = ev.evaluate()
    o = summary['overall']
    assert o['vel']['rmse'] == pytest.approx(math.sqrt(2.0))
    assert o['speed']['mean'] == pytest.approx(0.0)
    assert o['heading']['mean'] == pytest.approx(90.0)
    assert o['pos']['rmse'] == pytest.approx(0.1)
    # +1 s 예측: (t+0.1, 1) vs (t+1, 0) → √(0.81 + 1)
    assert o['pred_1']['mean'] == pytest.approx(math.sqrt(0.81 + 1.0))


def test_radial_bias_sign_toward_sensor_is_negative():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a'])
    gt_line(ev, 'a', 0.0, 2.0, (5.0, 0.0), (0.0, 0.0))
    for k in range(5):
        ev.add_frame(k * DT, [TrackObs(0, 4.8, 0.0, 0.0, 0.0, False)], robot_xy=(0.0, 0.0))
    _, _, summary = ev.evaluate()
    assert summary['overall']['radial']['mean'] == pytest.approx(-0.2)


def test_nees_matches_chi_square_mean():
    """오차를 공분산 P에서 뽑으면 위치 NEES 평균 ≈ 2 (2자유도), χ² 95% 안 비율 ≈ 0.95."""
    rng = np.random.default_rng(1)
    cov = np.array([[0.04, 0.01], [0.01, 0.02]])
    chol = np.linalg.cholesky(cov)
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a'])
    gt_line(ev, 'a', 0.0, 300.0, (0.0, 0.0), (0.0, 0.0), dt=0.1)
    for k in range(3000):
        t = k * DT
        e = chol @ rng.normal(size=2)
        ev.add_frame(t, [TrackObs(0, e[0], e[1], 0.0, 0.0, False, pos_cov=cov)])
    _, _, summary = ev.evaluate()
    nees = summary['overall']['nees_pos']
    assert nees['mean'] == pytest.approx(2.0, abs=0.15)
    assert nees['within_chi2_95'] == pytest.approx(0.95, abs=0.015)
    assert summary['overall']['nees_vel']['n'] == 0     # 속도 공분산이 없으면 계산하지 않는다


def test_id_switch_counted_per_gt_object():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a', 'b'])
    gt_line(ev, 'a', 0.0, 3.0, (0.0, 0.0), (1.0, 0.0))
    gt_line(ev, 'b', 0.0, 3.0, (0.0, 5.0), (0.0, 0.0))
    for k in range(21):
        t = k * DT
        tid = 1 if t < 0.95 else 2                        # a: 1 → 2 (전환 1번)
        tracks = [TrackObs(tid, t, 0.0, 1.0, 0.0, True), TrackObs(9, 0.0, 5.0, 0.0, 0.0, False)]
        if 1.25 < t < 1.55:
            tracks = tracks[1:]                           # a를 잠시 놓침: 전환으로 세지 않는다
        ev.add_frame(t, tracks)
    _, _, summary = ev.evaluate()
    a, b = summary['per_object']['a'], summary['per_object']['b']
    assert a['id_switches'] == 1 and a['distinct_track_ids'] == 2
    assert b['id_switches'] == 0
    assert a['frames_matched'] == a['frames_in_range'] - 3
    assert summary['overall']['id_switches'] == 1


def test_false_and_duplicate_dynamic_tracks():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a'])
    gt_line(ev, 'a', 0.0, 2.0, (0.0, 0.0), (1.0, 0.0))
    for k in range(10):
        t = k * DT
        ev.add_frame(t, [
            TrackObs(0, t, 0.0, 1.0, 0.0, True),           # 짝
            TrackObs(1, t + 0.4, 0.0, 1.0, 0.0, True),     # GT 옆 두 번째 트랙 (중복)
            TrackObs(2, 20.0, 0.0, 2.0, 0.0, True),        # 벽 조각 (가짜 동적)
            TrackObs(3, 25.0, 0.0, 0.0, 0.0, False),       # 정적 트랙: 세지 않음
        ])
    rows, _, summary = ev.evaluate()
    assert summary['false_dynamic_per_frame'] == 1.0
    assert summary['duplicate_dynamic_per_frame'] == 1.0
    assert summary['frames_with_false_dynamic'] == 1.0
    assert summary['tracks_per_frame'] == 4.0 and summary['dynamic_tracks_per_frame'] == 3.0
    assert sum(r['kind'] == 'track_only' for r in rows) == 30


def test_dynamic_latency_from_motion_start_and_first_sight():
    """GT가 2 s부터 움직이고, 2.5 s에 처음 짝지어지고(가려짐), 3.2 s에 동적으로 판정된다."""
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    gt_line(ev, 'w', 0.0, 2.0, (0.0, 0.0), (0.0, 0.0))
    gt_line(ev, 'w', 2.0, 8.0, (0.0, 0.0), (0.8, 0.0))
    for k in range(61):
        t = k * DT
        x = max(0.0, 0.8 * (t - 2.0))
        tracks = [] if 1.95 < t < 2.45 else [TrackObs(4, x, 0.0, 0.8, 0.0, t >= 3.15)]
        ev.add_frame(t, tracks, robot_xy=(0.0, 4.0))
    _, episodes, summary = ev.evaluate()
    assert len(episodes) == 1
    ep = episodes[0]
    assert ep['t_start'] == pytest.approx(2.0)
    assert ep['t_seen'] == pytest.approx(2.5)
    assert ep['latency_from_start'] == pytest.approx(1.2)
    assert ep['latency_from_seen'] == pytest.approx(0.7)
    assert ep['speed_bin'] == '0.6-1.0' and not ep['censored'] and not ep['carried']
    lat = summary['latency']
    assert lat['classified'] == 1 and lat['from_seen']['mean'] == pytest.approx(0.7)
    assert lat['by_speed']['0.6-1.0']['classified'] == 1


def test_latency_episode_starts_when_entering_sensor_range():
    ev = TrackingEvaluator(EvalConfig(sensor_range=12.0), gt_ids=['f'])
    gt_line(ev, 'f', 0.0, 10.0, (-20.0, 0.0), (1.5, 0.0))      # x = 12 at t ≈ 5.33
    for k in range(91):
        t = k * DT
        x = -20.0 + 1.5 * t
        tracks = [TrackObs(1, x, 0.0, 1.5, 0.0, t >= 6.0)] if x > -12.0 else []
        ev.add_frame(t, tracks, robot_xy=(0.0, 0.0))
    _, episodes, summary = ev.evaluate()
    assert len(episodes) == 1
    ep = episodes[0]
    assert ep['t_start'] == pytest.approx(5.4)          # 범위 밖이던 프레임은 구간이 아니다
    assert ep['latency_from_start'] == pytest.approx(0.6)
    assert ep['speed_bin'] == '>=1.0'
    assert ep['dynamic_dist'] == pytest.approx(11.0)
    # 범위 밖 GT는 놓친 것으로 세지 않는다
    assert summary['per_object']['f']['match_rate_in_range'] == 1.0


def test_censored_and_carried_episodes_excluded_from_latency():
    ev = TrackingEvaluator(EvalConfig(episode_merge_gap=0.5), gt_ids=['w'])
    # 기록 처음부터 움직임(censored) → 1 s 정지 → 다시 움직임. 같은 트랙이 이미 동적(carried)
    gt_line(ev, 'w', 0.0, 2.0, (0.0, 0.0), (1.0, 0.0))
    gt_line(ev, 'w', 2.02, 3.0, (2.0, 0.0), (0.0, 0.0))
    gt_line(ev, 'w', 3.02, 5.0, (2.0, 0.0), (1.0, 0.0))
    for k in range(51):
        t = k * DT
        ev.add_frame(t, [TrackObs(5, min(t, 2.0) + max(0.0, t - 3.0), 0.0, 1.0, 0.0, t >= 0.5)])
    _, episodes, summary = ev.evaluate()
    assert [(e['censored'], e['carried']) for e in episodes] == [(1, 0), (0, 1)]
    assert summary['latency']['censored_or_carried'] == 2
    assert summary['latency']['from_seen']['n'] == 0


def test_object_appearing_after_recording_start_is_not_censored():
    """하네스는 장애물을 추적보다 늦게 띄운다: 트랙 프레임은 0 s부터, GT는 5 s에 10 m 앞에 나타나 다가온다."""
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    gt_line(ev, 'w', 5.0, 10.0, (10.0, 0.0), (-1.0, 0.0))
    for k in range(101):
        t = k * DT
        x = 10.0 - (t - 5.0)
        tracks = [TrackObs(1, x, 0.0, -1.0, 0.0, t >= 6.45)] if t >= 5.25 else []
        ev.add_frame(t, tracks, robot_xy=(0.0, 0.0))
    _, episodes, summary = ev.evaluate()
    assert len(episodes) == 1
    ep = episodes[0]
    assert not ep['censored'] and not ep['carried']
    assert ep['t_start'] == pytest.approx(5.0)
    assert ep['latency_from_seen'] == pytest.approx(1.2)
    assert ep['dynamic_dist'] == pytest.approx(8.5)
    assert summary['latency']['classified'] == 1


def test_not_classified_episode_counted():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    gt_line(ev, 'w', 0.0, 1.0, (0.0, 0.0), (0.0, 0.0))
    gt_line(ev, 'w', 1.02, 4.0, (0.0, 0.0), (0.4, 0.0))
    for k in range(41):
        t = k * DT
        ev.add_frame(t, [TrackObs(0, 0.4 * max(0.0, t - 1.0), 0.0, 0.4, 0.0, False)])
    _, _, summary = ev.evaluate()
    lat = summary['latency']
    assert lat['seen'] == 1 and lat['classified'] == 0 and lat['not_classified'] == 1
    assert lat['by_speed']['<0.6']['seen'] == 1


def shuttle(t0, t1, x_far, x_near, speed, dt=0.02):
    """장면 C 작업자: x_far에서 x_near 쪽으로 출발해 두 점 사이를 같은 속력으로 왕복한다 (y = 0).

    obstacle_motion straight처럼 끝에서 감속 없이 바로 돌아선다. (t, x, vx) 목록.
    """
    out, x, vx = [], x_far, -speed
    for k in range(int(round((t1 - t0) / dt)) + 1):
        out.append((t0 + k * dt, x, vx))
        x += vx * dt
        if x <= x_near:
            vx = speed
        elif x >= x_far:
            vx = -speed
    return out


# 5 s에 10 m 앞에 나타나 1.5 m/s로 다가와 x ≈ 1.7에서 돌아서기를 되풀이한다
SHUTTLE = shuttle(5.0, 30.0, 10.0, 1.7, 1.5)
TURNS = [b[0] for a, b in zip(SHUTTLE, SHUTTLE[1:]) if a[2] != b[2]]     # 가까운 끝, 먼 끝, ...


def approach_run(t_dynamic, gt=SHUTTLE, seen_from=5.3):
    """로봇 (0, 0) 정지, 작업자가 gt대로 정면에서 왕복한다. 트랙 프레임은 0 s부터 (장애물을 늦게 띄움).

    트랙은 seen_from부터 보이고(None이면 안 보임) t_dynamic부터 동적이다(None이면 끝까지 정적).
    """
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    for t, x, vx in gt:
        ev.add_gt('w', t, x, 0.0, vx, 0.0)
    for k in range(int(round(gt[-1][0] / DT)) + 1):
        t = k * DT
        tracks = []
        if seen_from is not None and t >= seen_from:
            _, x, vx = gt[min(len(gt) - 1, max(0, round((t - gt[0][0]) / 0.02)))]
            tracks = [TrackObs(1, x, 0.0, vx, 0.0, t_dynamic is not None and t >= t_dynamic)]
        ev.add_frame(t, tracks, robot_xy=(0.0, 0.0))
    return ev.evaluate()


def test_approach_judged_before_first_closest():
    _, episodes, summary = approach_run(7.0)        # 다가오다 7 m에서 판정
    assert len(episodes) == 1                       # 감속 없이 돌아서서 왕복 전체가 한 구간으로 이어진다
    ap = summary['approach']
    assert ap['judged_before_closest'] == 1 and ap['note'] == ''
    assert ap['dynamic_dist'] == pytest.approx(7.0, abs=1e-3)
    assert ap['dynamic_speed'] == pytest.approx(1.5)
    assert ap['ttc_s'] == pytest.approx((7.0 - 0.55) / 1.5, abs=1e-3)
    assert ap['ttc_s'] == episodes[0]['ttc_at_dynamic']
    # 첫 최근접 = 처음 돌아서는 곳 (10 Hz 프레임이라 0.1 s 안)
    assert ap['t_closest'] == pytest.approx(TURNS[0], abs=0.1)
    assert ap['closest_dist'] == pytest.approx(1.7, abs=0.06)
    assert ap['latency_from_seen'] == pytest.approx(1.7)


def test_approach_judged_after_turning_away_is_zero():
    # 돌아서서 멀어지는 중 6 m에서야 판정: 거리만 보면 TTC 3.7 s로 합격처럼 보인다
    t_dyn = next(t for t, x, vx in SHUTTLE if vx > 0 and x >= 6.0)
    _, episodes, summary = approach_run(t_dyn)
    ap = summary['approach']
    assert (ap['dynamic_dist'] - 0.55) / ap['dynamic_speed'] > 3.0
    assert ap['judged_before_closest'] == 0 and ap['ttc_s'] == 0.0 and ap['note'] == '최근접 뒤 판정'
    assert episodes[0]['ttc_at_dynamic'] is None
    assert '최근접 뒤 판정' in format_summary_ko(summary)


def test_approach_judged_on_second_approach_is_zero():
    # 다시 다가올 때 7 m에서 판정: 다가오는 중이어도 이미 한 번 판정 없이 1.7 m까지 왔다
    t_dyn = next(t for t, x, vx in SHUTTLE if t > TURNS[1] and x <= 7.0)
    _, _, summary = approach_run(t_dyn)
    ap = summary['approach']
    assert ap['t_dynamic'] > ap['t_closest']
    assert ap['ttc_s'] == 0.0 and ap['note'] == '최근접 뒤 판정'


def test_approach_never_judged_is_zero():
    for seen_from in (5.3, None):                   # 보였지만 정적 / 아예 안 보임
        _, _, summary = approach_run(None, seen_from=seen_from)
        ap = summary['approach']
        assert ap['ttc_s'] == 0.0 and ap['note'] == '판정 없음'
        assert ap['t_seen'] == (None if seen_from is None else pytest.approx(5.3))


def test_approach_unmeasurable_or_multiple_objects():
    # 기록 시작 전부터 움직이던 물체: 첫 판정을 다른 구간으로 대신하지 않고 비운다
    _, _, summary = approach_run(7.0, gt=shuttle(0.0, 20.0, 10.0, 1.7, 1.5))
    assert summary['approach']['ttc_s'] is None
    assert summary['approach']['note'] == '첫 구간이 기록 시작 전부터'
    # 구간 없음 (GT 없음)
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    ev.add_frame(0.0, [])
    ap = ev.evaluate()[2]['approach']
    assert ap['gt_id'] == 'w' and ap['ttc_s'] is None and ap['note'] == '구간 없음'
    # 로봇 위치 없음
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'])
    ev.add_frame(0.0, [])
    gt_line(ev, 'w', 1.0, 3.0, (5.0, 0.0), (-1.0, 0.0))
    for k in range(1, 31):
        ev.add_frame(k * DT, [TrackObs(1, 5.0 - (k * DT - 1.0), 0.0, -1.0, 0.0, True)] if k >= 10 else [])
    ap = ev.evaluate()[2]['approach']
    assert ap['ttc_s'] is None and ap['note'] == '로봇 위치 없음'
    # 물체가 여럿이면 만들지 않는다
    assert TrackingEvaluator(EvalConfig(), gt_ids=['a', 'b']).evaluate()[2]['approach'] is None


# 가짜 동적 트랙 때문에 늘어난 감속·정지. 로봇을 world의 엉뚱한 자리·방향에 둔다 (몸체 변환이 틀리면 드러나게)
ROBOT = (5.0, -3.0, 0.7)


def at_body(bx, by, bvx=0.0, bvy=0.0, robot=ROBOT, robot_velocity=(0.0, 0.0)):
    """몸체 좌표(앞 x, 왼쪽 y)의 위치와 로봇에 대한 상대 속도 → world 위치·속도."""
    x, y, th = robot
    c, s = math.cos(th), math.sin(th)
    return (x + c * bx - s * by, y + s * bx + c * by,
            c * bvx - s * bvy + robot_velocity[0], s * bvx + c * bvy + robot_velocity[1])


def safety_run(false_at, frames=30, robot_velocity=(0.0, 0.0), worker=None):
    """10 Hz 프레임 0 ~ frames−1. false_at(k)가 몸체 (bx, by, bvx, bvy)를 주면 그 프레임에 동적 트랙(반경 0.25)을 둔다.

    worker: 진짜 작업자의 몸체 위치. 멈춰 선 채 동적으로 판정된 트랙이 GT 위에 있다.
    """
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['w'] if worker else [])
    if worker:
        wx, wy, _, _ = at_body(*worker)
        gt_line(ev, 'w', 0.0, frames * DT, (wx, wy), (0.0, 0.0))
    for k in range(frames):
        tracks = [TrackObs(1, wx, wy, 0.0, 0.0, True, radius=0.25)] if worker else []
        body = false_at(k)
        if body is not None:
            x, y, vx, vy = at_body(*body, robot_velocity=robot_velocity)
            tracks.append(TrackObs(9, x, y, vx, vy, True, radius=0.25))
        ev.add_frame(k * DT, tracks, robot_xy=ROBOT[:2], robot_yaw=ROBOT[2],
                     robot_velocity=robot_velocity)
    return ev.evaluate()


def test_false_fast_approach_outside_warning_distance_stops():
    # 차체 앞 2.3 m (차체 거리 2.3 − 0.3 − 0.25 = 1.75 m, 1.5 m 밖)에 2.5 m/s로 다가오는 가짜가 한 프레임 나타난다.
    # 1.5 m 안 개수로는 세지 않지만 TTC 0.7 s ≤ 1 s라 안전 노드는 선다
    rows, _, summary = safety_run(lambda k: (2.3, 0.0, -2.5, 0.0) if k == 5 else None)
    row = next(r for r in rows if r['kind'] == 'track_only')
    assert row['clearance'] == pytest.approx(1.75) and row['ttc'] == pytest.approx(0.7)
    assert row['radius'] == 0.25 and row['tr_speed'] == pytest.approx(2.5)
    assert (row['tr_x'], row['tr_y']) == pytest.approx(at_body(2.3, 0.0)[:2])
    fs = summary['false_dynamic_safety']
    # 그 프레임 + 단계를 내리기 전 0.5 s = 0.6 s
    assert fs['stop_events'] == 1 and fs['limited_events'] == 1
    assert fs['stop_s'] == pytest.approx(0.6) and fs['limited_s'] == pytest.approx(0.6)
    assert fs['frames'] == 30 and fs['duration_s'] == pytest.approx(2.9)
    assert fs['stop_frac'] == pytest.approx(0.6 / 2.9)
    assert fs['false_min_clearance'] == pytest.approx(1.75)
    assert fs['false_min_ttc'] == pytest.approx(0.7)
    assert '정지만 20.7% (1번)' in format_summary_ko(summary)


def test_false_beside_robot_slows_but_does_not_stop():
    # 차체 왼쪽 1.9 m에서 나란히 가는 가짜가 1 s. 중심 거리 1.9 m, 반경 빼기 전 1.7 m는 1.5 m 밖이지만
    # 안전 노드 거리(차체 옆 0.2 m, 반경 0.25 m를 뺀 1.45 m)로는 안이다. 닿지 않으니 TTC는 없다
    _, _, summary = safety_run(lambda k: (0.0, 1.9, 1.0, 0.0) if 5 <= k < 15 else None)
    fs = summary['false_dynamic_safety']
    assert fs['limited_events'] == 1 and fs['stop_events'] == 0 and fs['stop_s'] == 0.0
    assert fs['limited_s'] == pytest.approx(1.0 + 0.5)
    assert fs['false_min_clearance'] == pytest.approx(1.45) and fs['false_min_ttc'] is None


def test_false_far_and_receding_has_no_effect():
    _, _, summary = safety_run(lambda k: (5.0, 0.0, 1.0, 0.0))
    fs = summary['false_dynamic_safety']
    assert summary['false_dynamic_per_frame'] == 1.0
    assert fs['limited_s'] == 0.0 and fs['limited_events'] == 0 and fs['stop_events'] == 0
    assert fs['false_min_clearance'] == pytest.approx(4.45) and fs['false_min_ttc'] is None


def test_false_adds_nothing_when_real_worker_already_slows_robot():
    # 작업자가 차체 앞 1 m에 멈춰 서 있어(차체 거리 0.45 m) 이미 감속 중이다
    def false_at(k):
        if 5 <= k < 15:
            return (0.0, 1.9, 1.0, 0.0)       # 감속만 부르는 가짜: 더한 것 없음
        if k == 20:
            return (2.3, 0.0, -2.5, 0.0)      # 정지를 부르는 가짜: 감속 중이어도 정지는 가짜 탓
        return None
    _, _, summary = safety_run(false_at, worker=(1.0, 0.0))
    assert summary['false_dynamic_per_frame'] == pytest.approx(11 / 30)
    fs = summary['false_dynamic_safety']
    assert fs['limited_events'] == 1 and fs['stop_events'] == 1
    assert fs['limited_s'] == pytest.approx(0.6) and fs['stop_s'] == pytest.approx(0.6)


def test_duplicate_next_to_worker_is_not_counted_as_false():
    # 작업자(앞 3 m) 0.5 m 옆의 두 번째 트랙(중복)이 TTC 0.78 s로 다가와도 진짜 물체 쪽 트랙이라 가짜 탓이 아니다
    _, _, summary = safety_run(lambda k: (2.5, 0.0, -2.5, 0.0) if k == 5 else None,
                               worker=(3.0, 0.0))
    assert summary['duplicate_dynamic_per_frame'] == pytest.approx(1 / 30)
    assert summary['false_dynamic_per_frame'] == 0.0
    fs = summary['false_dynamic_safety']
    assert fs['limited_s'] == 0.0 and fs['stop_events'] == 0
    assert fs['false_min_clearance'] is None and fs['false_min_ttc'] is None


def test_robot_driving_toward_stopped_false_track_stops():
    # 동적 판정은 트랙이 사라질 때까지 남는다: 멈춘 구조물 조각(world 속도 0)도 동적이다.
    # 로봇이 0.5 m/s로 다가가면 상대 속도로 TTC 0.45 / 0.5 = 0.9 s → 정지 (로봇 속도를 빼면 감속만)
    velocity = (0.5 * math.cos(ROBOT[2]), 0.5 * math.sin(ROBOT[2]))
    rows, _, summary = safety_run(lambda k: (1.0, 0.0, -0.5, 0.0) if k == 5 else None,
                                  robot_velocity=velocity)
    row = next(r for r in rows if r['kind'] == 'track_only')
    assert row['tr_speed'] == pytest.approx(0.0, abs=1e-12)
    assert row['clearance'] == pytest.approx(0.45) and row['ttc'] == pytest.approx(0.9)
    assert summary['false_dynamic_safety']['stop_events'] == 1


def test_frame_gap_counts_at_most_tracks_timeout():
    # 0~0.9 s, 4.0~4.9 s 두 덩어리. 사이 3.1 s는 tracks_timeout_s 1 s까지만 센다
    ev = TrackingEvaluator(EvalConfig())
    for t in [k * DT for k in range(10)] + [4.0 + k * DT for k in range(10)]:
        x, y, vx, vy = at_body(0.0, 1.9, 1.0, 0.0)
        ev.add_frame(t, [TrackObs(9, x, y, vx, vy, True, radius=0.25)], robot_xy=ROBOT[:2],
                     robot_yaw=ROBOT[2], robot_velocity=(0.0, 0.0))
    fs = ev.evaluate()[2]['false_dynamic_safety']
    assert fs['duration_s'] == pytest.approx(0.9 + 1.0 + 0.9)
    assert fs['limited_frac'] == pytest.approx(1.0) and fs['limited_events'] == 1


def node_style_risks(tracks, odom_base, robot_velocity, length=0.6, width=0.4, horizon=5.0):
    """odom 좌표계에서 amr_safety obstacle_infos·amr_tracking collision_risks를 그대로 옮긴 계산."""
    # 안전 노드: TF base ← odom으로 중심을 몸체 좌표로 옮겨 차체 거리 − 반경
    base_odom = se2_inverse(odom_base)
    body = transform_points_2d(np.array([(t.x, t.y) for t in tracks]), base_odom[2], base_odom[:2])
    clearances = [max(float(d) - t.radius, 0.0)
                  for d, t in zip(footprint_clearance(body, length, width), tracks)]
    # 추적 노드: odom → base TF의 위치·방향, 몸체 축으로 돌린 상대 위치·속도
    c, s = math.cos(odom_base[2]), math.sin(odom_base[2])

    def to_body(x, y):
        return (c * x + s * y, -s * x + c * y)

    ttcs = [footprint_time_to_collision(
        to_body(t.x - odom_base[0], t.y - odom_base[1]),
        to_body(t.vx - robot_velocity[0], t.vy - robot_velocity[1]), t.radius, length, width,
        horizon) for t in tracks]
    return list(zip(clearances, ttcs))


def test_body_risks_match_safety_and_tracking_nodes_in_odom_frame():
    """평가는 world에서, 노드는 odom에서 계산한다. 같은 장면이면 차체 거리·TTC가 같아야 한다."""
    rng = np.random.default_rng(7)
    checked = finite = 0
    for _ in range(500):
        world_odom = (rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-math.pi, math.pi))
        odom_base = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
        v_body = (rng.uniform(-1.0, 2.0), rng.uniform(-0.3, 0.3))
        c, s = math.cos(odom_base[2]), math.sin(odom_base[2])
        v_odom = (c * v_body[0] - s * v_body[1], s * v_body[0] + c * v_body[1])
        tracks = [TrackObs(i, odom_base[0] + rng.uniform(-4, 4), odom_base[1] + rng.uniform(-4, 4),
                           rng.uniform(-3, 3), rng.uniform(-3, 3), True, radius=rng.uniform(0.0, 1.3))
                  for i in range(4)]
        expected = node_style_risks(tracks, odom_base, v_odom)
        world_base = se2_compose(world_odom, odom_base)
        cw, sw = math.cos(world_base[2]), math.sin(world_base[2])
        v_world = (cw * v_body[0] - sw * v_body[1], sw * v_body[0] + cw * v_body[1])
        got = body_risks([t.transformed(world_odom) for t in tracks], world_base, v_world,
                         0.6, 0.4, 5.0)
        np.testing.assert_allclose(np.array(got), np.array(expected), rtol=0, atol=1e-9)
        checked += len(tracks)
        finite += sum(math.isfinite(ttc) for _, ttc in expected)
    assert checked == 2000 and finite > 200      # 닿는 경우(유한 TTC)도 충분히 들어 있다


def test_false_dynamic_safety_needs_robot_pose_and_velocity():
    def run(**robot):
        ev = TrackingEvaluator(EvalConfig(), gt_ids=['a'])
        gt_line(ev, 'a', 0.0, 2.0, (0.0, 0.0), (1.0, 0.0))
        for k in range(10):
            t = k * DT
            ev.add_frame(t, [TrackObs(0, t, 0.0, 1.0, 0.0, True), TrackObs(2, 3.0, 0.0, 2.0, 0.0, True)],
                         **robot)
        return ev.evaluate()
    old_rows, _, old = run(robot_xy=(0.0, -1.0))
    new_rows, _, new = run(robot_xy=(0.0, -1.0), robot_yaw=0.3, robot_velocity=(0.2, 0.0))
    assert old['false_dynamic_safety'] is None
    assert run(robot_xy=(0.0, -1.0), robot_yaw=0.3)[2]['false_dynamic_safety'] is None
    assert new['false_dynamic_safety']['frames'] == 10
    # 나머지 지표는 그대로
    assert {k: v for k, v in new.items() if k != 'false_dynamic_safety'} == {
        k: v for k, v in old.items() if k != 'false_dynamic_safety'}
    assert [{k: v for k, v in r.items() if k not in ('clearance', 'ttc')} for r in new_rows] == old_rows
    assert not any('clearance' in r for r in old_rows)
    assert '가짜 동적 때문에' not in format_summary_ko(old)


def test_missing_data_is_robust_and_summary_formats():
    ev = TrackingEvaluator(EvalConfig(), gt_ids=['a', 'b'])
    rows, episodes, summary = ev.evaluate()                 # 아무것도 없음
    assert rows == [] and episodes == [] and summary['frames'] == 0
    assert summary['overall']['pos']['rmse'] is None
    assert '추적 정확도' in format_summary_ko(summary)
    ev.add_frame(0.0, [TrackObs(0, 0.0, 0.0, 0.0, 0.0, True)])  # GT 없음
    ev.add_frame(float('nan'), [])                               # 버린다
    _, _, summary = ev.evaluate()
    assert summary['frames'] == 1 and summary['false_dynamic_per_frame'] == 1.0
    text = format_summary_ko(perfect_run(noise=0.05, cov=np.eye(2) * 0.0025).evaluate()[2])
    assert 'worker_1' in text and '동적 판정 지연' in text


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        EvalConfig(gate=0.0)
    with pytest.raises(ValueError):
        EvalConfig(horizons=(1.0, -2.0))
    with pytest.raises(ValueError):
        EvalConfig(closest_rise=0.0)
    with pytest.raises(ValueError):
        EvalConfig(ttc_offset=-0.1)
    for name in ('robot_length', 'robot_width', 'ttc_horizon'):
        with pytest.raises(ValueError):
            EvalConfig(**{name: 0.0})
