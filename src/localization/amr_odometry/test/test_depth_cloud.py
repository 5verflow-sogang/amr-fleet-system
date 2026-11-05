import math

import numpy as np
import pytest

from amr_odometry.depth_cloud import (
    decode_depth, depth_to_cloud, depth_to_points, DepthCloudConfig, rate_limited, voxel_downsample,
)

# sensors.yaml rgbd_camera: 640×480, 수평 FOV 87° → fx = 320 / tan(43.5°) ≈ 337.2
W, H = 640, 480
FX = FY = (W / 2) / math.tan(math.radians(87.0) / 2)
CX, CY = W / 2, H / 2
ALL = DepthCloudConfig(stride=1, voxel_size=0.0)


def wall(z=2.0):
    return np.full((H, W), z, dtype=np.float32)


def test_wall_backprojects_to_plane_with_pinhole_xy():
    pts = depth_to_points(wall(2.0), FX, FY, CX, CY, ALL)
    assert len(pts) == W * H
    assert np.allclose(pts[:, 2], 2.0)
    # 왼쪽 위 픽셀 (0, 0): x = (0 − cx)·z/fx, y = (0 − cy)·z/fy. 광학 프레임이라 x 오른쪽·y 아래
    assert pts[0] == pytest.approx([-CX * 2.0 / FX, -CY * 2.0 / FY, 2.0], rel=1e-5)
    # 수평 FOV 끝 = z·tan(43.5°)
    assert pts[:, 0].min() == pytest.approx(-2.0 * math.tan(math.radians(43.5)), rel=1e-5)


def test_range_filter_drops_invalid_pixels():
    d = wall(2.0)
    d[0, :4] = [np.nan, np.inf, 0.1, 12.0]     # 측정 없음, 반사 없음, 최소 거리 안, 최대 거리 밖
    pts = depth_to_points(d, FX, FY, CX, CY, ALL)
    assert len(pts) == W * H - 4
    assert np.isfinite(pts).all()


def test_stride_keeps_one_pixel_per_block():
    pts = depth_to_points(wall(), FX, FY, CX, CY, DepthCloudConfig(stride=2, voxel_size=0.0))
    assert len(pts) == (W // 2) * (H // 2)
    # 솎아도 남은 픽셀의 좌표는 그대로 (0, 2, 4, …번 픽셀)
    assert pts[1, 0] == pytest.approx((2 - CX) * 2.0 / FX, rel=1e-5)


def test_voxel_averages_points_in_same_cell():
    p = np.array([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [0.21, 0.0, 0.0]], dtype=np.float32)
    out = voxel_downsample(p, 0.05)
    out = out[np.argsort(out[:, 0])]
    assert out == pytest.approx(np.array([[0.02, 0.02, 0.02], [0.21, 0.0, 0.0]]), abs=1e-6)


def test_voxel_off_and_empty_return_input():
    p = np.random.default_rng(0).random((10, 3)).astype(np.float32)
    assert voxel_downsample(p, 0.0) is p
    assert len(voxel_downsample(np.empty((0, 3), dtype=np.float32), 0.05)) == 0


def test_min_points_drops_isolated_voxel():
    p = np.array([[0.01, 0.0, 0.0], [0.02, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float32)
    out = voxel_downsample(p, 0.05, min_points=2)
    assert len(out) == 1 and out[0, 0] == pytest.approx(0.015)


def test_downsample_shrinks_wall_to_voxel_grid():
    # 2 m 벽 (보이는 폭 약 3.8 m × 2.8 m) → 5 cm 복셀 칸 수 정도로 준다
    cloud = depth_to_cloud(wall(2.0), FX, FY, CX, CY, DepthCloudConfig())
    width = 2 * 2.0 * math.tan(math.radians(43.5))
    height = width * H / W
    cells = (width / 0.05) * (height / 0.05)
    assert 0.9 * cells < len(cloud) < 1.2 * cells
    assert len(cloud) < W * H / 50
    # 각 복셀 안 점은 같은 면 위라 평균도 면 위
    assert np.allclose(cloud[:, 2], 2.0, atol=1e-5)


def test_decode_32fc1_with_row_padding():
    img = np.arange(6, dtype=np.float32).reshape(2, 3)
    padded = np.zeros((2, 4), dtype=np.float32)
    padded[:, :3] = img
    out = decode_depth(padded.tobytes(), '32FC1', 3, 2, 16)
    assert np.array_equal(out, img)


def test_decode_16uc1_millimetres_and_zero_is_missing():
    img = np.array([[0, 1500]], dtype=np.uint16)
    out = decode_depth(img.tobytes(), '16UC1', 2, 1, 4)
    assert np.isnan(out[0, 0]) and out[0, 1] == pytest.approx(1.5)


def test_decode_rejects_bad_input():
    with pytest.raises(ValueError):
        decode_depth(b'\0' * 12, 'rgb8', 2, 2, 6)
    with pytest.raises(ValueError):
        decode_depth(b'\0' * 8, '32FC1', 2, 1, 8, is_bigendian=True)
    with pytest.raises(ValueError):
        decode_depth(b'\0' * 8, '32FC1', 3, 1, 8)         # step이 width보다 짧다


@pytest.mark.parametrize('kwargs', [
    {'range_min': 2.0, 'range_max': 1.0}, {'stride': 0}, {'voxel_size': -0.1}, {'min_points_per_voxel': 0},
])
def test_config_validation(kwargs):
    with pytest.raises(ValueError):
        DepthCloudConfig(**kwargs)


def test_rate_limit_skips_frames_closer_than_period():
    assert not rate_limited(0.0, None, 10.0)            # 첫 영상은 처리
    assert rate_limited(1.05, 1.0, 10.0)                # 0.05 s < 0.1 s → 건너뜀
    assert not rate_limited(1.1, 1.0, 10.0)             # 한 주기 지남
    assert not rate_limited(1.01, 1.0, 0.0)             # 0이면 전부 처리


def test_rate_limit_does_not_stall_when_clock_goes_back():
    # /clock이 다시 시작해 시각이 뒤로 가면 건너뛰지 않고 처리한다 (last_stamp를 새로 잡게)
    assert not rate_limited(0.2, 120.0, 10.0)
