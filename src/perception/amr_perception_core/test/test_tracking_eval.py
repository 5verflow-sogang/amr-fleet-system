import math

import numpy as np
import pytest

from amr_perception_core.tracking_eval import (
    associate_nearest,
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
