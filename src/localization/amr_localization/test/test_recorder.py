import json
import math

import pytest

from amr_localization.eval import (
    STRAIGHT,
    EvalRecorder,
    Pose2D,
    associate,
    associate_interpolated,
    compute_errors,
    format_summary,
    interpolate,
    read_tum,
    summarize,
    transform_to_map,
)


def test_interpolate_midpoint():
    gt = [Pose2D(0.0, 0.0, 0.0, 0.0), Pose2D(0.1, 1.0, 2.0, 0.2)]
    p = interpolate(gt, 0.05, max_gap=0.2)
    assert (p.x, p.y, p.yaw) == pytest.approx((0.5, 1.0, 0.1))


def test_interpolate_yaw_takes_short_way_across_pi():
    gt = [Pose2D(0.0, 0, 0, math.pi - 0.1), Pose2D(0.1, 0, 0, -math.pi + 0.1)]
    p = interpolate(gt, 0.05, max_gap=0.2)
    # 0.2 rad 차이의 가운데 = ±π. 반대로 돌면 0 근처가 나온다
    assert abs(p.yaw) == pytest.approx(math.pi)


def test_interpolate_rejects_gap_and_out_of_range():
    gt = [Pose2D(0.0, 0, 0, 0), Pose2D(1.0, 1, 0, 0)]
    assert interpolate(gt, 0.5, max_gap=0.05) is None
    assert interpolate(gt, -0.1, max_gap=2.0) is None
    assert interpolate(gt, 1.1, max_gap=2.0) is None


def test_interpolation_removes_timing_error():
    # 정답 100 Hz, 추정 50 Hz이고 5 ms 어긋남. 추정값은 그 시각의 참값이다 (2 m/s 직진)
    speed = 2.0
    gt = [Pose2D(i * 0.01, i * 0.01 * speed, 0.0, 0.0) for i in range(300)]
    est = [Pose2D(0.005 + i * 0.02, (0.005 + i * 0.02) * speed, 0.0, 0.0) for i in range(140)]
    nearest = summarize([s.error for s in compute_errors(associate(gt, est, max_dt=0.01), gt)])
    interp = summarize([s.error for s in compute_errors(associate_interpolated(gt, est), gt)])
    # 가장 가까운 시점 방식은 5 ms × 2 m/s = 1 cm 오차가 생긴다
    assert nearest.rmse == pytest.approx(0.01, abs=1e-6)
    assert interp.rmse == pytest.approx(0.0, abs=1e-9)


def test_transform_to_map_translation_and_rotation():
    # map 원점이 world (0, 1.5)에 있고 90도 돌아 있다
    origin = (0.0, 1.5, math.pi / 2)
    [p] = transform_to_map([Pose2D(0.0, 0.0, 2.5, math.pi / 2)], origin)
    # world에서 원점 기준 +y 1 m = map 기준 +x 1 m, 방향은 0
    assert (p.x, p.y, p.yaw) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


def test_recorder_save_writes_harness_files(tmp_path):
    origin = (0.0, 1.5, 0.0)
    rec = EvalRecorder('amr_01', map_origin_in_world=origin)
    # world 기준으로 y=1.5 위를 1 m/s 직진. map 기준 추정은 y=0.02로 2 cm 어긋남
    for i in range(200):
        t = i * 0.01
        rec.add_gt(Pose2D(t, t, 1.5, 0.0))
        if i % 2 == 1:
            rec.add_est(Pose2D(t, t, 0.02, 0.0))
    summary = rec.save(tmp_path)

    assert (tmp_path / 'localization.csv').exists()
    assert (tmp_path / 'trajectories' / 'amr_01_gt.tum').exists()
    est = read_tum(tmp_path / 'trajectories' / 'amr_01_est.tum')
    assert len(est) == 100
    # 저장한 정답 궤적은 map 기준 (y=0)
    gt_map = read_tum(tmp_path / 'trajectories' / 'amr_01_gt.tum')
    assert gt_map[0].y == pytest.approx(0.0)

    straight = summary['segments'][STRAIGHT]
    assert straight['rmse'] == pytest.approx(0.02, abs=1e-6)
    assert summary['kpi_pass'][STRAIGHT] is True
    with open(tmp_path / 'localization_summary.json', encoding='utf-8') as f:
        assert json.load(f)['counts']['est'] == 100
    assert 'straight n=' in format_summary(summary)


def _fill(rec, n=300):
    # world y=1.5 위를 1 m/s 직진, 추정은 map 기준 y=0.02 (2 cm 어긋남)
    for i in range(n):
        t = i * 0.01
        rec.add_gt(Pose2D(t, t, 1.5, 0.0))
        if i % 2 == 1:
            rec.add_est(Pose2D(t, t, 0.02, 0.0))


def test_streaming_writes_raw_before_save(tmp_path):
    rec = EvalRecorder('amr_01', (0.0, 1.5, 0.0), stream_dir=str(tmp_path))
    _fill(rec)
    rec.flush()
    # save 전인데도 원자료가 파일에 있다 (강제 종료 대비)
    gt = read_tum(tmp_path / 'trajectories' / 'amr_01_gt.tum')
    est = read_tum(tmp_path / 'trajectories' / 'amr_01_est.tum')
    assert len(gt) == 300 and len(est) == 150
    assert gt[0].y == pytest.approx(0.0)   # map 기준으로 변환돼 있음
    rec.close()


def test_offline_recompute_matches_live(tmp_path):
    live = EvalRecorder('amr_01', (0.0, 1.5, 0.0), stream_dir=str(tmp_path))
    _fill(live)
    live_summary = live.save(str(tmp_path))
    live.close()
    # 원자료만으로 다시 계산 (eval_offline과 같은 경로)
    offline = EvalRecorder.from_tum(str(tmp_path), 'amr_01')
    off_summary = offline.save(str(tmp_path))
    assert off_summary['counts'] == live_summary['counts']
    for seg, s in live_summary['segments'].items():
        if s is None:
            assert off_summary['segments'][seg] is None
        else:
            assert off_summary['segments'][seg]['rmse'] == pytest.approx(s['rmse'], abs=1e-6)
    assert off_summary['map_origin_in_world'] is None


def test_precomputed_times_give_same_result():
    from amr_localization.eval.metrics import gt_velocity
    gt = [Pose2D(i * 0.01, i * 0.01, 0.0, 0.1 * i * 0.01) for i in range(50)]
    times = [p.t for p in gt]
    for t in (0.005, 0.123, 0.37):
        assert interpolate(gt, t, 0.05) == interpolate(gt, t, 0.05, times)
        assert gt_velocity(gt, t) == gt_velocity(gt, t, times)


def test_eval_offline_cli(tmp_path, capsys):
    from amr_localization.eval_offline import main
    rec = EvalRecorder('amr_01', (0.0, 1.5, 0.0), stream_dir=str(tmp_path))
    _fill(rec)
    rec.close()   # save 없이 끝남 = 강제 종료된 상황
    assert main([str(tmp_path), '--robot', 'amr_01']) == 0
    assert 'straight n=' in capsys.readouterr().out
    assert (tmp_path / 'localization.csv').exists()
