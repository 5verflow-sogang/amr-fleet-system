import math

import numpy as np
import pytest

from amr_odometry.scan_filter import (
    filter_angle, filter_range, filter_scan, remove_outliers, ScanFilterConfig,
)

N = 720
INC = 2 * math.pi / N
A0 = -math.pi
CFG = ScanFilterConfig()


def wall(distance=2.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    return (np.full(N, distance) + rng.normal(0.0, noise, N)).astype(np.float32)


def test_range_filter_rep117_markers():
    r = np.array([0.05, 0.5, 30.0, np.nan, np.inf], dtype=np.float32)
    out = filter_range(r, 0.12, 25.0, CFG)
    assert out[0] == -np.inf          # 너무 가까움
    assert out[1] == pytest.approx(0.5)
    assert out[2] == np.inf           # 최대 거리 밖
    assert np.isnan(out[3])
    assert out[4] == np.inf


def test_range_filter_uses_narrower_of_sensor_and_config():
    cfg = ScanFilterConfig(range_min=0.3, range_max=10.0)
    out = filter_range(np.array([0.2, 5.0, 12.0], dtype=np.float32), 0.12, 25.0, cfg)
    assert out[0] == -np.inf and out[1] == pytest.approx(5.0) and out[2] == np.inf


def test_angle_filter_keeps_full_scan_with_rounded_pi():
    cfg = ScanFilterConfig(angle_min=-3.14159265, angle_max=3.14159265)
    out = filter_angle(wall(), A0, INC, cfg)
    assert np.isfinite(out).all()


def test_angle_filter_masks_outside_window():
    cfg = ScanFilterConfig(angle_min=-math.pi / 2, angle_max=math.pi / 2)
    out = filter_angle(wall(), A0, INC, cfg)
    angles = A0 + INC * np.arange(N)
    inside = (angles >= -math.pi / 2 - 1e-6) & (angles <= math.pi / 2 + 1e-6)
    assert np.isfinite(out[inside]).all()
    assert np.isnan(out[~inside]).all()
    assert len(out) == N


def test_outlier_removes_isolated_spike_only():
    r = wall(2.0)
    r[100] = 0.8      # 혼자 튄 점
    r[300:303] = 1.0  # 실제 작은 물체 (3빔 연속)
    out = remove_outliers(r, CFG)
    assert np.isnan(out[100])
    assert np.isfinite(out[300:303]).all()
    assert np.isfinite(np.delete(out, 100)).all()


def test_outlier_keeps_noisy_wall():
    # LiDAR 노이즈 σ 0.03 m인 벽은 하나도 지우면 안 된다
    out = remove_outliers(wall(5.0, noise=0.03, seed=1), CFG)
    assert np.isfinite(out).all()


def test_outlier_keeps_far_grazing_wall():
    # 15 m 근처에서 벽을 비스듬히 비추면 이웃 빔끼리 0.4 m씩 벌어진다 → 정상 점이다
    r = wall(2.0)
    r[200:240] = 15.0 + 0.4 * np.arange(40)
    out = remove_outliers(r, CFG)
    assert np.isfinite(out[200:240]).all()


def test_outlier_far_spike_is_removed():
    r = wall(3.0)
    r[400] = 12.0     # 3 m 벽 사이 혼자 튄 12 m
    out = remove_outliers(r, CFG)
    assert np.isnan(out[400])


def test_outlier_fixed_threshold_when_ratio_zero():
    r = wall(2.0)
    r[200:203] = 15.0 + 0.4 * np.arange(3)
    out = remove_outliers(r, ScanFilterConfig(outlier_ratio=0.0))
    assert np.isnan(out[200:203]).all()


def test_outlier_wraps_around_scan_ends():
    r = wall(2.0)
    r[0] = 0.5        # 처음 빔은 마지막 빔과 이웃이다
    out = remove_outliers(r, CFG)
    assert np.isnan(out[0]) and np.isfinite(out[-1]) and np.isfinite(out[1])


def test_outlier_point_next_to_no_return_is_removed():
    r = np.full(N, np.inf, dtype=np.float32)
    r[50] = 3.0       # 빈 공간 한가운데 점 하나 → 이웃 없음
    out = remove_outliers(r, CFG)
    assert np.isnan(out[50])


def test_outlier_disabled():
    r = wall(2.0)
    r[100] = 0.8
    out = remove_outliers(r, ScanFilterConfig(outlier_window=0))
    assert out[100] == pytest.approx(0.8)


def test_filter_scan_pipeline_keeps_layout():
    r = wall(2.0)
    r[10] = 0.05
    r[20] = 0.9
    out = filter_scan(r.tolist(), A0, INC, 0.12, 25.0)
    assert out.dtype == np.float32 and len(out) == N
    assert out[10] == -np.inf         # 거리 필터 표시는 아웃라이어 단계에서 지우지 않는다
    assert np.isnan(out[20])


@pytest.mark.parametrize('kwargs', [
    {'range_min': 1.0, 'range_max': 0.5}, {'angle_min': 1.0, 'angle_max': 0.0},
    {'outlier_window': -1}, {'outlier_window': 1, 'outlier_min_neighbors': 3},
    {'outlier_distance': 0.0}, {'outlier_ratio': -0.1},
])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ScanFilterConfig(**kwargs)
