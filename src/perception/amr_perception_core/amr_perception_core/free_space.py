"""빈 공간 침입 증거 (ROS 비의존): 움직이는 물체는 LiDAR가 방금 비어 있다고 본 칸으로 들어온다.

R3(추적 속력)만으로는 로봇이 움직일 때 벽·랙 조각이 동적 트랙이 됐다 (Gazebo, 회전 중 프레임당 13.7개).
LiDAR가 회전 중심 앞에 있어 로봇이 움직이면 가까운 랙이 먼 벽을 가리는 경계가 바뀌고, 보이는 벽 조각의
중심이 벽을 따라 미끄러져 속도가 생긴다. 이 조각의 점은 늘 벽 위(빔이 맞는 칸)에 있고, 빔이 지나가는
(비어 있는) 칸으로 들어가지 않는다. 진짜 사람·지게차는 방금 빈 공간이었던 칸으로 들어온다.

고정 좌표계(odom) 격자 하나를 로봇 주변에 두고 칸마다 두 시각을 적는다.
- last_free: 빔이 칸을 지나간(비어 있다고 본) 마지막 시각. 빔 끝 endpoint_margin 앞까지만 적는다
- last_hit: 빔 끝(반사점)이 칸에 떨어진 마지막 시각

점이 놓인 칸이 다음이면 "빈 공간 침입" 증거다 (이번 스캔을 넣기 전 상태로 판정한다).
- now − last_free ≤ free_memory (최근에 비어 있었다)
- now − last_hit > hit_quiet (그동안 그 칸에 무엇이 맞은 적이 없다: 벽 칸, 비스듬한 빔이 벽 칸을 지나가는
  경우를 거른다)

빔이 물체 옆을 스치고 지나가면 물체 바로 옆 칸이 비어 있다고 적힌다. 로봇이 움직여 물체의 다른 면이 보이면
그 점이 "빈 칸 침입"이 되어, 랙 다리·기둥이 동적 트랙이 됐다 (녹화 재생: 빈칸 증거만으로 프레임당 2.6개 남음).
그래서 이번 스캔 반사점에서 hit_clearance 안의 칸은 비어 있다고 적지 않는다.

위치추정(map)이 아니라 odom만 쓰므로 위치추정 오차의 영향을 받지 않는다. odom이 천천히 흐르는 것은
free_memory(몇 초) 안에서는 작다.
"""

import math
from typing import Sequence

import numpy as np


class FreeSpaceGrid:

    def __init__(self, resolution: float = 0.1, size: float = 30.0, max_range: float = 12.0,
                 endpoint_margin: float = 0.2, free_memory: float = 3.0, hit_quiet: float = 3.0,
                 hit_clearance: float = 0.2):
        """resolution: 칸 크기 [m]. size: 격자 한 변 [m] (로봇이 가운데 절반을 벗어나면 다시 가운데로 옮긴다).
        max_range: 이보다 먼 구간은 비어 있다고 적지 않는다 [m] (먼 곳은 빔 간격이 칸보다 넓다).
        endpoint_margin: 빔 끝에서 이만큼 앞까지만 비어 있다고 적는다 [m] (거리 잡음, 물체 두께).
        hit_clearance: 같은 스캔의 반사점에서 이 거리 안의 칸은 비어 있다고 적지 않는다 [m] (스치는 빔).
        """
        for name, value in (('resolution', resolution), ('size', size), ('max_range', max_range),
                            ('free_memory', free_memory)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')
        for name, value in (('endpoint_margin', endpoint_margin), ('hit_quiet', hit_quiet),
                            ('hit_clearance', hit_clearance)):
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name}는 0 이상이어야 한다: {value}')
        self.resolution = float(resolution)
        self.cells = int(math.ceil(size / resolution))
        self.max_range = float(max_range)
        self.endpoint_margin = float(endpoint_margin)
        self.free_memory = float(free_memory)
        self.hit_quiet = float(hit_quiet)
        self.hit_clearance = float(hit_clearance)
        k = int(math.ceil(self.hit_clearance / self.resolution))
        span = range(-k, k + 1)
        self._clear_offsets = np.array([(dr, dc) for dr in span for dc in span
                                        if (dr * dr + dc * dc) * self.resolution ** 2
                                        <= self.hit_clearance ** 2 + 1e-12])
        self.last_free = np.full((self.cells, self.cells), -np.inf)
        self.last_hit = np.full((self.cells, self.cells), -np.inf)
        self.origin = None          # 격자 (0, 0) 칸 모서리의 고정 좌표 (x, y)

    # ------------------------------------------------------------------ 격자
    def _cell(self, points: np.ndarray):
        """(N, 2) 점 → (행, 열, 격자 안인지)."""
        col = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (col >= 0) & (col < self.cells) & (row >= 0) & (row < self.cells)
        return row, col, inside

    def _recenter(self, center: Sequence[float]) -> None:
        """로봇이 격자 가운데 절반을 벗어나면 로봇을 가운데로 두도록 격자를 옮긴다 (겹치는 칸은 유지)."""
        half = 0.5 * self.cells * self.resolution
        new_origin = (math.floor((center[0] - half) / self.resolution) * self.resolution,
                      math.floor((center[1] - half) / self.resolution) * self.resolution)
        if self.origin is None:
            self.origin = new_origin
            return
        cx = (center[0] - self.origin[0]) / self.resolution
        cy = (center[1] - self.origin[1]) / self.resolution
        quarter = 0.25 * self.cells
        if quarter <= cx <= 3 * quarter and quarter <= cy <= 3 * quarter:
            return
        shift_c = int(round((new_origin[0] - self.origin[0]) / self.resolution))
        shift_r = int(round((new_origin[1] - self.origin[1]) / self.resolution))
        self.last_free = _shifted(self.last_free, shift_r, shift_c)
        self.last_hit = _shifted(self.last_hit, shift_r, shift_c)
        self.origin = new_origin

    # ------------------------------------------------------------------ 판정·갱신
    def evidence(self, points: np.ndarray, now: float) -> np.ndarray:
        """점마다 "최근 비어 있던 칸에 들어왔다"인지 (bool). 격자 밖·NaN은 False."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        out = np.zeros(pts.shape[0], dtype=bool)
        if self.origin is None or pts.shape[0] == 0:
            return out
        ok = np.all(np.isfinite(pts), axis=1)
        row, col, inside = self._cell(pts[ok])
        r, c = row[inside], col[inside]
        recent_free = now - self.last_free[r, c] <= self.free_memory
        quiet = now - self.last_hit[r, c] > self.hit_quiet
        hits = np.zeros(row.size, dtype=bool)
        hits[inside] = recent_free & quiet
        out[ok] = hits
        return out

    def integrate(self, sensor_xy: Sequence[float], directions: np.ndarray, ranges: np.ndarray,
                  now: float) -> None:
        """스캔 한 장을 넣는다. directions: 빔마다 고정 좌표 단위 방향 (N, 2), ranges: 거리 [m].

        유효하지 않은 거리(NaN, 0 이하)는 건너뛴다. inf(반사 없음)는 max_range까지 비어 있다고 본다.
        """
        sx, sy = float(sensor_xy[0]), float(sensor_xy[1])
        self._recenter((sx, sy))
        d = np.asarray(directions, dtype=float).reshape(-1, 2)
        r = np.asarray(ranges, dtype=float).reshape(-1)
        with np.errstate(invalid='ignore'):
            usable = (np.isfinite(r) & (r > 0.0)) | np.isposinf(r)
        d, r = d[usable], r[usable]
        if r.size == 0:
            return
        hit = np.isfinite(r)
        # 비어 있다고 적을 길이: 반사점 endpoint_margin 앞까지, max_range까지
        free_len = np.where(hit, r - self.endpoint_margin, np.inf)
        free_len = np.minimum(free_len, self.max_range)
        # 빔마다 제 길이만큼, 칸 크기 간격으로 표본을 만든다. 대각선으로 지나가는 칸 몇 개를 건너뛸 수 있지만
        # (비어 있다고 덜 적힘 = 증거가 덜 생기는 안전한 쪽) 반 칸 간격의 절반 시간이다 (스캔당 약 30 → 15 ms)
        step = self.resolution
        counts = np.floor(np.maximum(free_len, 0.0) / step).astype(np.int64) + 1
        beam = np.repeat(np.arange(r.size), counts)
        starts = np.cumsum(counts) - counts
        dist = (np.arange(beam.size) - np.repeat(starts, counts)) * step
        res = self.resolution
        col = np.floor((sx - self.origin[0] + d[beam, 0] * dist) / res).astype(np.int64)
        row = np.floor((sy - self.origin[1] + d[beam, 1] * dist) / res).astype(np.int64)
        inside = (col >= 0) & (col < self.cells) & (row >= 0) & (row < self.cells)
        hit_pts = np.c_[sx + d[hit, 0] * r[hit], sy + d[hit, 1] * r[hit]]
        hrow, hcol, hinside = self._cell(hit_pts)
        hrow, hcol = hrow[hinside], hcol[hinside]
        if self.hit_clearance > 0.0 and hrow.size:
            # 이번 반사점 둘레 칸은 비어 있다고 적지 않는다 (물체 옆을 스치는 빔)
            near = np.zeros((self.cells, self.cells), dtype=bool)
            nr = (hrow[:, None] + self._clear_offsets[None, :, 0]).ravel()
            nc = (hcol[:, None] + self._clear_offsets[None, :, 1]).ravel()
            ok = (nr >= 0) & (nr < self.cells) & (nc >= 0) & (nc < self.cells)
            near[nr[ok], nc[ok]] = True
            inside[inside] &= ~near[row[inside], col[inside]]
        self.last_free[row[inside], col[inside]] = now
        self.last_hit[hrow, hcol] = now


def _shifted(grid: np.ndarray, shift_r: int, shift_c: int) -> np.ndarray:
    """격자 원점이 (shift_r, shift_c)칸 옮겨졌을 때의 새 배열. 새로 드러난 칸은 -inf."""
    out = np.full_like(grid, -np.inf)
    n_r, n_c = grid.shape
    src_r = slice(max(shift_r, 0), n_r + min(shift_r, 0))
    dst_r = slice(max(-shift_r, 0), n_r + min(-shift_r, 0))
    src_c = slice(max(shift_c, 0), n_c + min(shift_c, 0))
    dst_c = slice(max(-shift_c, 0), n_c + min(-shift_c, 0))
    if src_r.start < src_r.stop and src_c.start < src_c.stop:
        out[dst_r, dst_c] = grid[src_r, src_c]
    return out
