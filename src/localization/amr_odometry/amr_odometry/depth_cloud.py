"""Depth 영상 → 포인트클라우드 변환과 복셀 다운샘플링 (ROS 비의존).

Pinhole 역투영: 픽셀 (u, v)의 깊이 z [m]에서 광학 프레임 점 x = (u − cx)·z/fx, y = (v − cy)·z/fy, z.
CameraInfo K는 픽셀 i의 중심을 i로 본다. Gazebo depth 영상 값은 광축 방향 거리(z)다.

처리 순서: 픽셀 간격(stride)으로 솎기 → 거리 필터 → 역투영 → 복셀 다운샘플(복셀마다 점들의 평균 하나).
640×480 한 장이 30만 점이라 그대로 내면 받는 쪽(costmap 등)이 못 따라온다.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DepthCloudConfig:
    # 거리 필터 [m] (깊이 z 기준). 센서 사양(sensors.yaml rgbd_camera: 0.3~10 m)보다 좁히면 그 범위만 쓴다
    range_min: float = 0.3
    range_max: float = 10.0
    # 역투영 전에 가로·세로 stride 픽셀마다 하나만 쓴다 (1이면 전부). 2면 점 수·계산이 1/4
    stride: int = 2
    # 복셀 한 변 [m]. 0이면 다운샘플을 끈다
    voxel_size: float = 0.05
    # 점이 이보다 적은 복셀은 버린다 (1이면 끔). 혼자 튄 깊이 픽셀을 지우지만, 멀리서는 복셀 하나에 점이
    # 1개뿐이라(10 m에서 stride 2 픽셀 간격 약 6 cm) 먼 면도 같이 지워진다
    min_points_per_voxel: int = 1

    def __post_init__(self):
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError(f'0 <= range_min < range_max여야 한다: [{self.range_min}, {self.range_max}]')
        if self.stride < 1:
            raise ValueError(f'stride는 1 이상이어야 한다: {self.stride}')
        if self.voxel_size < 0.0:
            raise ValueError(f'voxel_size는 0 이상이어야 한다: {self.voxel_size}')
        if self.min_points_per_voxel < 1:
            raise ValueError(f'min_points_per_voxel은 1 이상이어야 한다: {self.min_points_per_voxel}')


def rate_limited(stamp: float, last_stamp, max_rate_hz: float) -> bool:
    """영상 시각 stamp [s]를 건너뛸지. 직전 처리 시각에서 1 / max_rate_hz가 안 지났으면 True.

    max_rate_hz가 0 이하이거나 처음 받은 영상이면 False. 시각이 뒤로 가면(/clock 다시 시작) 거르지 않는다:
    stamp − last_stamp가 음수인 동안 계속 건너뛰면 예전 시각을 따라잡을 때까지 아무것도 안 낸다 (#267 연우 리뷰).
    """
    if max_rate_hz <= 0 or last_stamp is None:
        return False
    return 0.0 <= stamp - last_stamp < 0.99 / max_rate_hz


def decode_depth(data: bytes, encoding: str, width: int, height: int, step: int,
                 is_bigendian: bool = False) -> np.ndarray:
    """sensor_msgs/Image 버퍼를 (height, width) float32 깊이 [m]로. 32FC1(m)과 16UC1(mm), little-endian만 받는다."""
    if is_bigendian:
        raise ValueError('big-endian depth 영상은 지원하지 않는다 (Gazebo는 little-endian)')
    if encoding == '32FC1':
        dtype, scale = np.float32, None
    elif encoding == '16UC1':
        dtype, scale = np.uint16, 0.001
    else:
        raise ValueError(f'지원하지 않는 depth encoding: {encoding}')
    item = np.dtype(dtype).itemsize
    if step % item or step < width * item or len(data) < step * height:
        raise ValueError(f'영상 크기가 맞지 않는다: {width}×{height}, step {step}, {len(data)} bytes')
    img = np.frombuffer(data, dtype=dtype, count=height * step // item).reshape(height, step // item)[:, :width]
    if scale is None:
        return img
    out = img.astype(np.float32) * scale
    out[img == 0] = np.nan       # 16UC1의 0은 측정 없음
    return out


def depth_to_points(depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                    config: DepthCloudConfig) -> np.ndarray:
    """깊이 영상을 광학 프레임 점 (N, 3) float32로. 범위 밖·NaN·inf 픽셀은 뺀다."""
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f'fx, fy는 양수여야 한다: {fx}, {fy}')
    s = config.stride
    z = np.asarray(depth, dtype=np.float32)[::s, ::s]
    v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(z) & (z >= config.range_min) & (z <= config.range_max)
    z, u, v = z[ok], u[ok], v[ok]
    pts = np.empty((z.size, 3), dtype=np.float32)
    pts[:, 0] = (u - cx) * z / fx
    pts[:, 1] = (v - cy) * z / fy
    pts[:, 2] = z
    return pts


def voxel_downsample(points: np.ndarray, voxel_size: float, min_points: int = 1) -> np.ndarray:
    """복셀마다 점들의 평균 하나를 남긴다. voxel_size 0이면 그대로 돌려준다."""
    if voxel_size <= 0.0 or len(points) == 0:
        return points
    cell = np.floor(points / voxel_size).astype(np.int64)
    cell -= cell.min(axis=0)
    span = cell.max(axis=0) + 1
    # 3축 칸 번호를 정수 하나로 합쳐 1차원 unique·bincount로 센다 (3열 unique·add.at보다 몇 배 빠르다)
    key = (cell[:, 0] * span[1] + cell[:, 1]) * span[2] + cell[:, 2]
    _, inverse, counts = np.unique(key, return_inverse=True, return_counts=True)
    keep = counts >= min_points
    mean = np.empty((int(keep.sum()), 3), dtype=np.float32)
    for axis in range(3):
        mean[:, axis] = np.bincount(inverse, weights=points[:, axis], minlength=len(counts))[keep] / counts[keep]
    return mean


def depth_to_cloud(depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                   config: DepthCloudConfig) -> np.ndarray:
    """역투영 + 복셀 다운샘플."""
    pts = depth_to_points(depth, fx, fy, cx, cy, config)
    return voxel_downsample(pts, config.voxel_size, config.min_points_per_voxel)
