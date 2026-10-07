"""스캔-지도 일치율. 위치를 잃었는지(Kidnapped) 판단하는 데 쓴다. ROS에 의존하지 않는다 (numpy만).

추정 자세에서 스캔 끝점을 지도에 찍었을 때, 점유 셀과 tol 이내에 떨어지는 끝점의 비율이다.
위치가 맞으면 거의 모든 끝점이 벽·랙 표면에 붙는다(10/6 실측: 정답 자세에서 빔 600여 개가 거의 다 한 칸 안).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridMap:
    """점유 격자와 점유 셀까지의 거리장."""

    resolution: float
    origin_x: float
    origin_y: float
    # [행 = y, 열 = x], 점유 셀까지 거리 [m]. max_dist를 넘으면 max_dist
    dist: np.ndarray

    @property
    def shape(self):
        return self.dist.shape


def distance_field(occupied: np.ndarray, resolution: float, max_dist: float) -> np.ndarray:
    """점유 셀까지의 거리(체비쇼프 거리 × 해상도 근사). 팽창을 max_dist까지만 반복한다.

    scipy 없이 돈다. 일치율 판정(tol 0.1 m 수준)에는 근사 거리로 충분하다.
    """
    steps = int(math.ceil(max_dist / resolution))
    dist = np.full(occupied.shape, max_dist, dtype=np.float32)
    dist[occupied] = 0.0
    reached = occupied.copy()
    for k in range(1, steps + 1):
        grown = reached.copy()
        grown[1:, :] |= reached[:-1, :]
        grown[:-1, :] |= reached[1:, :]
        grown[:, 1:] |= grown[:, :-1].copy()
        grown[:, :-1] |= grown[:, 1:].copy()
        new = grown & ~reached
        dist[new] = min(k * resolution, max_dist)
        reached = grown
    return dist


def grid_from_occupancy(data, width: int, height: int, resolution: float, origin_x: float, origin_y: float,
                        occupied_threshold: int = 65, max_dist: float = 0.5) -> GridMap:
    """nav_msgs/OccupancyGrid의 data(행 우선, 0~100, -1 미탐색) → GridMap."""
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    return GridMap(resolution, origin_x, origin_y, distance_field(cells >= occupied_threshold, resolution, max_dist))


def endpoints(pose: tuple[float, float, float], sensor_offset: tuple[float, float, float],
              ranges: np.ndarray, angle_min: float, angle_increment: float,
              range_min: float, range_max: float) -> np.ndarray:
    """로봇 자세(map 기준 x, y, yaw)와 센서 장착 위치(로봇 기준 x, y, yaw)로 유효 빔의 끝점 (N, 2)."""
    x, y, yaw = pose
    sx, sy, syaw = sensor_offset
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + c * sx - s * sy, y + s * sx + c * sy
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(len(r)) + yaw + syaw
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.stack([ox + r[ok] * np.cos(a[ok]), oy + r[ok] * np.sin(a[ok])], axis=1)


def match_ratio(grid: GridMap, points: np.ndarray, tol: float = 0.1) -> float:
    """끝점 중 점유 셀과 tol 이내인 비율. 지도 밖 끝점은 맞지 않은 것으로 센다. 끝점이 없으면 nan."""
    if len(points) == 0:
        return float('nan')
    i = np.floor((points[:, 0] - grid.origin_x) / grid.resolution).astype(int)
    j = np.floor((points[:, 1] - grid.origin_y) / grid.resolution).astype(int)
    h, w = grid.shape
    inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
    hit = np.zeros(len(points), dtype=bool)
    hit[inside] = grid.dist[j[inside], i[inside]] <= tol
    return float(hit.mean())
