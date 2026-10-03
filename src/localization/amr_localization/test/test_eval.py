import csv
import math

import pytest

from amr_localization.eval import (
    MIN_SAMPLES,
    ROTATION,
    STATIONARY,
    STRAIGHT,
    Pose2D,
    associate,
    classify_segment,
    compute_errors,
    kpi_check,
    read_tum,
    summarize,
    summarize_by_segment,
    write_localization_csv,
    write_tum,
)
from amr_localization.eval.metrics import CSV_COLUMNS


def straight_line(n, dt=0.02, speed=1.0, y=0.0):
    """x축으로 등속 직진하는 궤적."""
    return [Pose2D(i * dt, i * dt * speed, y, 0.0) for i in range(n)]


def test_tum_roundtrip_keeps_yaw(tmp_path):
    poses = [Pose2D(0.0, 1.0, 2.0, 0.5), Pose2D(0.1, 1.5, 2.5, -2.9)]
    path = tmp_path / 'traj.tum'
    write_tum(path, poses)
    loaded = read_tum(path)
    assert len(loaded) == 2
    for a, b in zip(poses, loaded):
        assert b.t == pytest.approx(a.t)
        assert (b.x, b.y) == pytest.approx((a.x, a.y))
        assert b.yaw == pytest.approx(a.yaw)


def test_read_tum_rejects_wrong_column_count(tmp_path):
    path = tmp_path / 'bad.tum'
    path.write_text('0.0 1.0 2.0\n')
    with pytest.raises(ValueError):
        read_tum(path)


def test_associate_picks_nearest_within_max_dt():
    gt = [Pose2D(0.00, 0, 0, 0), Pose2D(0.02, 1, 0, 0), Pose2D(0.04, 2, 0, 0)]
    est = [Pose2D(0.021, 9, 0, 0), Pose2D(0.5, 9, 0, 0)]
    pairs = associate(gt, est, max_dt=0.005)
    # 0.021은 0.02와 짝, 0.5는 가까운 정답이 없어 버림
    assert [(g.t, e.t) for g, e in pairs] == [(0.02, 0.021)]


def test_constant_offset_gives_rmse_equal_to_offset():
    gt = straight_line(200)
    est = [Pose2D(p.t, p.x, p.y + 0.03, p.yaw) for p in gt]
    samples = compute_errors(associate(gt, est), gt)
    s = summarize([x.error for x in samples])
    assert s.n == 200
    assert s.rmse == pytest.approx(0.03)
    assert s.max == pytest.approx(0.03)


def test_rmse_differs_from_mean():
    # 오차 0.03, 0.04 → RMSE = sqrt((0.0009 + 0.0016) / 2)
    s = summarize([0.03, 0.04])
    assert s.rmse == pytest.approx(math.sqrt(0.00125))
    assert s.mean == pytest.approx(0.035)
    assert s.max == pytest.approx(0.04)


def test_summarize_empty_is_none():
    assert summarize([]) is None


def test_classify_segment():
    assert classify_segment(0.0, 0.0) == STATIONARY
    assert classify_segment(0.5, 0.0) == STRAIGHT
    assert classify_segment(0.0, 0.5) == ROTATION
    # 곡선 주행은 회전으로 센다
    assert classify_segment(0.5, 0.5) == ROTATION


def test_segments_from_gt_trajectory():
    dt = 0.02
    stop = [Pose2D(i * dt, 0.0, 0.0, 0.0) for i in range(50)]
    t0 = stop[-1].t
    drive = [Pose2D(t0 + (i + 1) * dt, (i + 1) * dt, 0.0, 0.0) for i in range(50)]
    t1, x1 = drive[-1].t, drive[-1].x
    spin = [Pose2D(t1 + (i + 1) * dt, x1, 0.0, (i + 1) * dt * 1.0) for i in range(50)]
    gt = stop + drive + spin
    summaries = summarize_by_segment(compute_errors(associate(gt, gt), gt))
    # 구간 경계에서 차분이 한 칸씩 걸치므로 대략 50개씩
    for seg in (STATIONARY, STRAIGHT, ROTATION):
        assert summaries[seg] is not None
        assert abs(summaries[seg].n - 50) <= 1
        assert summaries[seg].rmse == pytest.approx(0.0)


def test_yaw_error_wraps_around_pi():
    gt = [Pose2D(0.0, 0, 0, 3.1)]
    est = [Pose2D(0.0, 0, 0, -3.1)]
    sample = compute_errors(associate(gt, est), gt)[0]
    assert sample.yaw_error == pytest.approx(2 * math.pi - 6.2)


def test_kpi_check():
    gt = straight_line(MIN_SAMPLES)
    est = [Pose2D(p.t, p.x, p.y + 0.04, p.yaw) for p in gt]
    summaries = summarize_by_segment(compute_errors(associate(gt, est), gt))
    result = kpi_check(summaries)
    # 직선 기준 5cm, 오차 4cm, 샘플 100개 → 통과
    assert result[STRAIGHT] is True
    # 정지·회전 구간은 샘플이 없어 판정 불가
    assert result[STATIONARY] is None
    assert result[ROTATION] is None


def test_kpi_check_fails_below_min_samples():
    gt = straight_line(MIN_SAMPLES - 1)
    summaries = summarize_by_segment(compute_errors(associate(gt, gt), gt))
    assert kpi_check(summaries)[STRAIGHT] is False


def test_write_localization_csv(tmp_path):
    gt = straight_line(3)
    est = [Pose2D(p.t, p.x + 0.01, p.y, p.yaw) for p in gt]
    path = tmp_path / 'localization.csv'
    write_localization_csv(path, compute_errors(associate(gt, est), gt), robot='amr_01')
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    # 명세서 컬럼 6개가 맨 앞
    assert tuple(rows[0]) == CSV_COLUMNS
    assert rows[0][:6] == ['timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error']
    assert len(rows) == 4
    assert float(rows[1][5]) == pytest.approx(0.01)
    assert rows[1][8] == 'amr_01'
