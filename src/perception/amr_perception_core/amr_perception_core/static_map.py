"""R2 정적 지도 대조 (ROS 비의존): 지도에서 막힌 칸(벽·랙·기둥) 위의 스캔 점을 군집화 전에 뺀다.

크기 규칙(R1)만으로는 짧은 구조물(랙 모서리, 비스듬히 보여 잘린 벽 조각)이 장애물 후보로 남고,
로봇이 움직이면 잘린 위치가 따라 움직여 동적 트랙이 되기도 했다 (perception.md 1-2절 R2).
지도에 있는 것은 처음부터 추적하지 않는다.

- 막힌 칸: OccupancyGrid 값이 occupied_threshold 이상 (map_server 기본 0.65 → 65). 미지(−1)는 막히지 않음
- inflation [m]: 막힌 칸을 이만큼 부풀린다. 위치추정 오차와 지도 해상도만큼 벽 점이 칸 밖으로 나가기 때문.
  대가: 벽에 이 거리보다 가까이 붙은 사람·물체의 점도 빠진다 (안전 노드는 원본 스캔을 쓰므로 정지에는 영향 없음)
- 지도 밖의 점은 정적이 아니라고 본다
- 국소 정렬(best_offset): 위치추정이 지도와 어긋나면(Gazebo에서 0.25 m, 부풀림 0.15 m보다 큼) 벽 점이
  새어 나와 조각 트랙이 된다. 대조 직전에 스캔을 작은 범위에서 밀어 보고 지도 위 점이 가장 많은 평행이동을
  쓴다. 회전 오차는 IMU가 있어 작다(측정 0.1°)고 보고 평행이동만 찾는다
"""

import math
from typing import Sequence

import numpy as np


class StaticMapMask:

    def __init__(self, occupancy: np.ndarray, resolution: float, origin: Sequence[float],
                 occupied_threshold: int = 65, inflation: float = 0.15):
        """occupancy: (높이, 너비) int 배열, 행 = y 칸, 열 = x 칸 (OccupancyGrid.data를 reshape한 것).
        origin: 지도 (0, 0) 칸 모서리의 (x, y, yaw) [m, rad], map 좌표.
        """
        grid = np.asarray(occupancy)
        if grid.ndim != 2 or grid.size == 0:
            raise ValueError(f'occupancy는 비지 않은 2차원 배열이어야 한다: shape={grid.shape}')
        if not (math.isfinite(resolution) and resolution > 0.0):
            raise ValueError(f'resolution은 양수여야 한다: {resolution}')
        if not (math.isfinite(inflation) and inflation >= 0.0):
            raise ValueError(f'inflation은 0 이상이어야 한다: {inflation}')
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]))
        self.mask = _dilate(grid >= occupied_threshold, int(math.ceil(inflation / resolution)))

    @classmethod
    def from_grid_message(cls, width: int, height: int, resolution: float, origin_xy_yaw,
                          data: Sequence[int], **kwargs) -> 'StaticMapMask':
        return cls(np.asarray(data, dtype=np.int16).reshape(height, width), resolution,
                   origin_xy_yaw, **kwargs)

    def best_offset(self, points: np.ndarray, max_shift: float, step: float):
        """points를 ±max_shift 안에서 step 간격으로 밀어 지도 위(부풀린 막힌 칸) 비율이 가장 큰 (dx, dy).

        반환: (dx, dy, 그 비율, 밀지 않았을 때 비율). 비율이 같으면 덜 미는 쪽을 고른다.
        점이 없으면 (0, 0, 0, 0).
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if pts.shape[0] == 0:
            return 0.0, 0.0, 0.0, 0.0
        # 두 단계로 찾는다: 2·step 간격으로 넓게, 그 주변을 step 간격으로 (169개 → 약 58개, 같은 결과)
        coarse = self._search(pts, (0.0, 0.0), int(round(max_shift / (2 * step))), 2 * step)
        fine = self._search(pts, coarse[:2], 1, step)
        best = fine if fine[2] > coarse[2] else coarse
        frac0 = float(self.is_static(pts).mean())
        return best[0], best[1], best[2], frac0

    def _search(self, pts: np.ndarray, center, k: int, step: float):
        """center 주변 (2k+1)² 평행이동 중 지도 위 비율이 가장 큰 것. 같으면 center에 가까운 쪽."""
        offsets = np.array([(center[0] + i * step, center[1] + j * step)
                            for i in range(-k, k + 1) for j in range(-k, k + 1)])
        offsets = offsets[np.argsort(np.hypot(offsets[:, 0], offsets[:, 1]), kind='stable')]
        shifted = pts[None, :, :] + offsets[:, None, :]
        fractions = self.is_static(shifted.reshape(-1, 2)).reshape(len(offsets), -1).mean(axis=1)
        best = int(np.argmax(fractions))       # 같은 값이면 처음 = 가장 덜 민 것
        return float(offsets[best, 0]), float(offsets[best, 1]), float(fractions[best])

    def is_static(self, points: np.ndarray) -> np.ndarray:
        """map 좌표 (N, 2) 점마다 막힌 칸(부풀린 뒤) 위인지. NaN 점과 지도 밖은 False."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - ox, pts[:, 1] - oy
        # 지도 좌표 → 칸 좌표 (지도 원점 회전을 되돌린다)
        gx = (c * dx + s * dy) / self.resolution
        gy = (-s * dx + c * dy) / self.resolution
        out = np.zeros(pts.shape[0], dtype=bool)
        ok = np.isfinite(gx) & np.isfinite(gy)
        col = np.floor(gx[ok]).astype(int)
        row = np.floor(gy[ok]).astype(int)
        h, w = self.mask.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        hits = np.zeros(col.size, dtype=bool)
        hits[inside] = self.mask[row[inside], col[inside]]
        out[ok] = hits
        return out


def _dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """원판 모양으로 radius_cells 칸만큼 부풀린다 (경계 밖으로 감싸지 않는다)."""
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    out = np.zeros_like(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            # out[y, x] |= mask[y - dy, x - dx]
            ys, yd = _shift(dy, h)
            xs, xd = _shift(dx, w)
            out[ys, xs] |= mask[yd, xd]
    return out


def _shift(d: int, n: int):
    """길이 n 축을 d칸 미는 (받는 쪽, 주는 쪽) 슬라이스."""
    if d >= 0:
        return slice(d, n), slice(0, n - d)
    return slice(0, n + d), slice(-d, n)
