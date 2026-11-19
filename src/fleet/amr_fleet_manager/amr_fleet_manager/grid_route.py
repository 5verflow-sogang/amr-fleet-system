"""가짜 로봇(fake_robot)이 지도의 통로를 따라 움직이게 하는 경로 (8방향 A*). rclpy를 import하지 않는다.

가짜 로봇은 원래 목적지까지 직선으로 가서 대시보드 지도에서 테이블·벽을 뚫고 지나갔다 (식당 시연 3 녹화).
여기서는 지도(map_server yaml + pgm)의 빈 칸만 지나는 최단 경로를 찾는다.
- 칸 크기 cell_m(0.1 m)로 줄이고(원래 칸 넷 중 하나라도 막히면 막힘), 막힌 칸에서 clearance_m 안은 지나지 않는다
  (로봇 반폭). 그 밖이라도 prefer_m 안은 비용을 더 매겨 통로 가운데로 가게 한다
- 정차 자리는 가구 앞 0.15~0.2 m라 clearance 안에 있다: 출발·도착은 가장 가까운 지나갈 수 있는 칸으로 이어 붙인다
- 경로 점은 시야가 트인 점끼리 건너뛰어 줄인다 (꺾이는 곳만 남는다)
- 다른 로봇·사람은 모른다 (로봇끼리 겹쳐 지나갈 수 있다). 시연 영상에는 "로봇 이동은 단순화한 시뮬레이션"으로 밝힌다
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml

Point = Tuple[float, float]
Cell = Tuple[int, int]

# map_server 무압축 pgm에서 이 값 이상만 빈 칸으로 본다 (254 = 빈 곳, 205 = 모름, 0 = 막힘). 모르는 곳(벽 밖)은 막힘
FREE_MIN = 250
_STEPS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def read_pgm(path) -> Tuple[int, int, int, bytes]:
    """P5(바이너리) pgm → (너비, 높이, 최댓값, 픽셀). 맨 윗줄이 지도의 위쪽(큰 y)이다."""
    data = Path(path).read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    if fields[0] != b'P5':
        raise ValueError(f'{path}: P5 pgm이 아니다 ({fields[0]!r})')
    width, height, maxval = (int(f) for f in fields[1:])
    pixels = data[pos + 1:pos + 1 + width * height]
    return width, height, maxval, pixels


class GridRoute:
    """blocked[j][i]: 아래(작은 y)에서 j번째 줄, 왼쪽에서 i번째 칸이 막혔는지. origin = 왼쪽 아래 칸의 모서리 (m)."""

    def __init__(self, blocked: Sequence[Sequence[bool]], cell_m: float, origin: Point,
                 clearance_m: float = 0.3, prefer_m: float = 0.6):
        self.cell_m = cell_m
        self.origin = origin
        self.height = len(blocked)
        self.width = len(blocked[0]) if blocked else 0
        self.blocked = [list(row) for row in blocked]
        self.dist = self._distance_m()
        self.clearance_m = clearance_m
        self.prefer_m = prefer_m

    @classmethod
    def from_map_yaml(cls, path, cell_m: float = 0.1, **kw) -> 'GridRoute':
        """map_server yaml(image, resolution, origin) → GridRoute. cell_m은 resolution의 정수배로 맞춘다."""
        path = Path(path)
        meta = yaml.safe_load(path.read_text(encoding='utf-8'))
        width, height, _, pixels = read_pgm(path.parent / meta['image'])
        res = float(meta['resolution'])
        k = max(1, round(cell_m / res))
        rows, cols = height // k, width // k
        blocked = []
        for j in range(rows):                       # j: 아래에서 j번째 칸 줄
            top = height - (j + 1) * k              # pgm은 위가 0번 줄
            blocked.append([any(pixels[(top + dy) * width + i * k + dx] < FREE_MIN
                                for dy in range(k) for dx in range(k)) for i in range(cols)])
        ox, oy = meta['origin'][:2]
        # pgm 위쪽에서 남는 줄(height % k)은 버리고, 아래 모서리는 origin 그대로다
        return cls(blocked, res * k, (float(ox), float(oy)), **kw)

    # ------------------------------------------------------------------ 좌표
    def cell(self, p: Point) -> Cell:
        return (int(math.floor((p[0] - self.origin[0]) / self.cell_m)),
                int(math.floor((p[1] - self.origin[1]) / self.cell_m)))

    def center(self, c: Cell) -> Point:
        return (self.origin[0] + (c[0] + 0.5) * self.cell_m, self.origin[1] + (c[1] + 0.5) * self.cell_m)

    def inside(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def passable(self, c: Cell) -> bool:
        return self.inside(c) and self.dist[c[1]][c[0]] >= self.clearance_m

    # ------------------------------------------------------------------ 경로
    def route(self, start: Point, goal: Point) -> List[Point]:
        """start에서 goal까지 지나갈 점들 (start는 빼고 goal은 넣는다). 못 찾으면 [goal] (직선, 예전 동작)."""
        s, g = self._nearest_passable(self.cell(start)), self._nearest_passable(self.cell(goal))
        if s is None or g is None:
            return [goal]
        cells = self._astar(s, g)
        if cells is None:
            return [goal]
        # 첫 점(출발 칸)도 넣는다: 빼면 출발점에서 둘째 점까지 곧장 이어져 모서리를 가로지를 수 있다
        return [self.center(c) for c in self._shortcut(cells)] + [goal]

    def _distance_m(self) -> List[List[float]]:
        """칸마다 가장 가까운 막힌 칸까지 거리 (m, 8방향 다익스트라 근사). 막힌 칸은 0. 지도 밖은 inside()가 막는다."""
        inf = float('inf')
        dist = [[0.0 if b else inf for b in row] for row in self.blocked]
        heap = [(0.0, i, j) for j, row in enumerate(self.blocked) for i, b in enumerate(row) if b]
        heapq.heapify(heap)
        while heap:
            d, i, j = heapq.heappop(heap)
            if d > dist[j][i]:
                continue
            for di, dj, w in _STEPS:
                ni, nj = i + di, j + dj
                if 0 <= ni < self.width and 0 <= nj < self.height and d + w * self.cell_m < dist[nj][ni]:
                    dist[nj][ni] = d + w * self.cell_m
                    heapq.heappush(heap, (dist[nj][ni], ni, nj))
        return dist

    def _nearest_passable(self, c: Cell) -> Optional[Cell]:
        """c가 지나갈 수 없으면(가구 앞 정차 자리 등) 가장 가까운 지나갈 수 있는 칸 (BFS)."""
        c = (min(max(c[0], 0), self.width - 1), min(max(c[1], 0), self.height - 1))
        seen, queue = {c}, deque([c])
        while queue:
            cur = queue.popleft()
            if self.passable(cur):
                return cur
            for di, dj, _ in _STEPS:
                n = (cur[0] + di, cur[1] + dj)
                if self.inside(n) and n not in seen:
                    seen.add(n)
                    queue.append(n)
        return None

    def _astar(self, s: Cell, g: Cell) -> Optional[List[Cell]]:
        def h(c):
            dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
            return (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)) * self.cell_m

        best = {s: 0.0}
        came = {}
        heap = [(h(s), 0.0, s)]
        while heap:
            _, cost, cur = heapq.heappop(heap)
            if cur == g:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            if cost > best.get(cur, float('inf')):
                continue
            for di, dj, w in _STEPS:
                n = (cur[0] + di, cur[1] + dj)
                if not self.passable(n):
                    continue
                # 통로 가운데로: 벽·가구에서 prefer_m 안이면 가까울수록 비싸다
                near = max(0.0, self.prefer_m - self.dist[n[1]][n[0]]) / self.prefer_m
                c2 = cost + w * self.cell_m * (1.0 + 2.0 * near)
                if c2 < best.get(n, float('inf')):
                    best[n] = c2
                    came[n] = cur
                    heapq.heappush(heap, (c2 + h(n), c2, n))
        return None

    def _clear_line(self, a: Cell, b: Cell) -> bool:
        """a에서 b까지 직선이 지나갈 수 있는 칸만 지나는지 (칸 반 크기 간격으로 본다)."""
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2
        for k in range(1, n):
            t = k / n
            c = (int(round(a[0] + (b[0] - a[0]) * t)), int(round(a[1] + (b[1] - a[1]) * t)))
            if not self.passable(c):
                return False
        return True

    def _shortcut(self, cells: List[Cell]) -> List[Cell]:
        """시야가 트인 칸끼리 건너뛴다 (첫 칸·끝 칸과 꺾이는 곳만 남는다)."""
        out, i = [cells[0]], 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._clear_line(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return out
