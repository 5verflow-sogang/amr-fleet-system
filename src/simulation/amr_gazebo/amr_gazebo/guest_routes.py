"""손님 일과(routine) 경로를 정답 지도에서 미리 뽑는다. rclpy·numpy 없이 돈다 (단위 테스트용).

지도: amr_mapping maps/<이름>.pgm + .yaml (map_server 형식, trinary, map = world). 칠해진 칸(점유)에서
CLEARANCE_M 넘게 떨어진 칸만 손님 몸 중심이 지날 수 있다. A*로 칸을 잇고, 통로 가운데를 조금 더 좋아하게
벽 가까운 칸에 웃돈을 붙인다. 뽑은 칸 목록은 직선으로 이어도 여유가 지켜지는 점만 남겨 경유점으로 줄인다.
"""

from __future__ import annotations

import heapq
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Point = Tuple[float, float]
Cell = Tuple[int, int]

# 몸 중심이 지날 수 있는 칸: 점유 칸에서 이만큼 [m] 넘게 (손님 몸 반지름 0.25 + 여유)
CLEARANCE_M = 0.32
# 이보다 가까운 [m] 칸은 지날 수는 있지만 웃돈을 붙인다 (통로 가운데로 걷게)
PREFER_M = 0.7
PREFER_GAIN = 2.0
# 다른 손님이 머무는(서 있는) 자리에서 이만큼 [m] 안 칸은 웃돈을 붙이고, 줄인 직선도 이만큼 떨어져 지나게 한다
AVOID_SPOT_M = 0.6
# 웃돈은 가볍게: 옆에 빈 길이 있으면 그쪽으로 가고, 없으면 그냥 지나간다 (지나가다 만나면 손님끼리 선다).
# 20배였을 때는 앉는 자리 사이 통로가 모두 '비킬 곳'이 되어, 손님 20명에서 바로 앞 자리에 가려고 홀을 한 바퀴
# 돌았다 (몰림 손님 crowd_03: 6.1 m 길을 17.1 m로)
AVOID_SPOT_COST = 3.0
# 머무는 자리에서 비킬 수 있는 방향: 이만큼 [m] 비켜 선 길이 비어 있으면 (몸 반지름만큼 여유)
EVADE_STEP_M = 0.8
EVADE_CLEARANCE_M = 0.25
DIRS8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


class GridMap:
    """map_server 형식 지도. occupied[row][col] (row 0 = 맨 아래 y), clearance[row][col] [m]."""

    def __init__(self, occupied: List[List[bool]], resolution: float, origin: Point):
        self.occupied = occupied
        self.height = len(occupied)
        self.width = len(occupied[0])
        self.resolution = resolution
        self.origin = origin
        self.clearance = _distance_transform(occupied, resolution)

    @classmethod
    def load(cls, yaml_path: str) -> 'GridMap':
        meta = yaml.safe_load(Path(yaml_path).read_text(encoding='utf-8'))
        pgm = Path(yaml_path).parent / meta['image']
        width, height, maxval, data = _read_pgm(pgm.read_bytes())
        occ_th = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))
        occupied = []
        for r in range(height):                     # pgm 첫 줄 = 맨 위 (y 최대)
            row = data[(height - 1 - r) * width:(height - r) * width]
            occupied.append([((v / maxval) if negate else (1.0 - v / maxval)) > occ_th for v in row])
        return cls(occupied, float(meta['resolution']), (float(meta['origin'][0]), float(meta['origin'][1])))

    def cell(self, p: Point) -> Cell:
        return (int(math.floor((p[0] - self.origin[0]) / self.resolution)),
                int(math.floor((p[1] - self.origin[1]) / self.resolution)))

    def point(self, c: Cell) -> Point:
        return (self.origin[0] + (c[0] + 0.5) * self.resolution, self.origin[1] + (c[1] + 0.5) * self.resolution)

    def inside(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def clear_at(self, p: Point) -> float:
        c = self.cell(p)
        return self.clearance[c[1]][c[0]] if self.inside(c) else 0.0

    def segment_clear(self, a: Point, b: Point, clearance: float) -> bool:
        n = max(1, int(math.ceil(math.dist(a, b) / (0.5 * self.resolution))))
        return all(self.clear_at((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)) >= clearance
                   for k in range(n + 1))

    def snap(self, p: Point, clearance: float = CLEARANCE_M, max_m: float = 1.5) -> Point:
        """p에서 가장 가까운, 여유 clearance 넘는 칸 중심. 이미 넉넉하면 p 그대로."""
        if self.clear_at(p) >= clearance:
            return p
        c0 = self.cell(p)
        r = int(math.ceil(max_m / self.resolution))
        best: Optional[Tuple[float, Cell]] = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                c = (c0[0] + dx, c0[1] + dy)
                if self.inside(c) and self.clearance[c[1]][c[0]] >= clearance:
                    d = math.dist(p, self.point(c))
                    if best is None or d < best[0]:
                        best = (d, c)
        if best is None:
            raise ValueError(f'{p} 둘레 {max_m} m 안에 여유 {clearance} m 칸이 없다')
        return self.point(best[1])

    def plan(self, a: Point, b: Point, clearance: float = CLEARANCE_M,
             avoid: Sequence[Point] = ()) -> List[Point]:
        """a → b 경유점 (a 빼고 b 포함). 둘 다 snap된 점이어야 한다.
        avoid: 다른 손님이 서 있는 자리. AVOID_SPOT_M 안을 되도록 지나지 않는다 (a·b 둘레 자리는 빼고 본다)."""
        if math.dist(a, b) < 1e-6:
            return [b]
        avoid = [p for p in avoid if math.dist(p, a) > AVOID_SPOT_M and math.dist(p, b) > AVOID_SPOT_M]
        start, goal = self.cell(a), self.cell(b)
        res = self.resolution
        open_: List[Tuple[float, float, Cell]] = [(0.0, 0.0, start)]
        came: Dict[Cell, Cell] = {}
        g = {start: 0.0}
        while open_:
            _, gc, c = heapq.heappop(open_)
            if c == goal:
                break
            if gc > g.get(c, math.inf):
                continue
            for dx, dy in DIRS8:
                n = (c[0] + dx, c[1] + dy)
                if not self.inside(n):
                    continue
                cl = self.clearance[n[1]][n[0]]
                if cl < clearance and n != goal:
                    continue
                step = res * math.hypot(dx, dy) * (1.0 + PREFER_GAIN * max(0.0, PREFER_M - cl) / PREFER_M)
                if avoid:
                    q = self.point(n)
                    if any(math.dist(q, p) < AVOID_SPOT_M for p in avoid):
                        step *= AVOID_SPOT_COST
                ng = gc + step
                if ng < g.get(n, math.inf):
                    g[n] = ng
                    came[n] = c
                    heapq.heappush(open_, (ng + res * math.dist(n, goal), ng, n))
        if goal not in g:
            raise ValueError(f'{a} → {b} 길이 없다 (여유 {clearance} m)')
        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        pts = [a] + [self.point(c) for c in reversed(cells[:-1])]
        pts[-1] = b
        return _shortcut(self, pts, clearance, avoid)[1:]

    def evade_dirs(self, p: Point, step_m: float = EVADE_STEP_M,
                   clearance: float = EVADE_CLEARANCE_M) -> List[List[int]]:
        """p에서 step_m 비켜 선 길이 비어 있는 8방향."""
        out = []
        for dx, dy in DIRS8:
            n = math.hypot(dx, dy)
            q = (p[0] + step_m * dx / n, p[1] + step_m * dy / n)
            if self.segment_clear(p, q, clearance):
                out.append([dx, dy])
        return out


def _shortcut(m: GridMap, pts: List[Point], clearance: float, avoid: Sequence[Point] = ()) -> List[Point]:
    """앞에서부터 직선으로 갈 수 있는 가장 먼 점으로 건너뛴다. 원래 칸 경로보다 avoid 자리에 가까워지지 않게 한다."""
    def ok(a, b, sub):
        if not m.segment_clear(a, b, clearance):
            return False
        for p in avoid:
            limit = min(AVOID_SPOT_M, min(math.dist(p, q) for q in sub))
            if _seg_point_dist(a, b, p) < limit - 1e-9:
                return False
        return True

    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not ok(pts[i], pts[j], pts[i:j + 1]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _seg_point_dist(a: Point, b: Point, p: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = 0.0 if den <= 0.0 else min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den, 0.0), 1.0)
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def _read_pgm(raw: bytes) -> Tuple[int, int, int, bytes]:
    """P5 (binary) pgm. 주석(#) 줄을 건너뛴다."""
    tokens: List[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError(f'P5 pgm이 아니다: {tokens[0]!r}')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1
    return width, height, maxval, raw[pos:pos + width * height]


def _distance_transform(occupied: List[List[bool]], res: float) -> List[List[float]]:
    """칸마다 가장 가까운 점유 칸까지 거리 [m] (8방향 다익스트라 근사). 지도 밖도 점유로 본다."""
    h, w = len(occupied), len(occupied[0])
    dist = [[math.inf] * w for _ in range(h)]
    heap: List[Tuple[float, int, int]] = []
    for r in range(h):
        for c in range(w):
            if occupied[r][c] or r in (0, h - 1) or c in (0, w - 1):
                dist[r][c] = 0.0 if occupied[r][c] else res
                heap.append((dist[r][c], c, r))
    heapq.heapify(heap)
    while heap:
        d, c, r = heapq.heappop(heap)
        if d > dist[r][c]:
            continue
        for dx, dy in DIRS8:
            nc, nr = c + dx, r + dy
            if 0 <= nc < w and 0 <= nr < h:
                nd = d + res * math.hypot(dx, dy)
                if nd < dist[nr][nc]:
                    dist[nr][nc] = nd
                    heapq.heappush(heap, (nd, nc, nr))
    return dist


def build_obstacles(spec: dict, grid: GridMap, other_spots: Sequence[Point] = ()) -> List[dict]:
    """일과 설정(places·defaults·guests) → dynamic_obstacles yaml의 obstacles 목록 (pattern routine).

    경로는 다른 손님이 머무는 자리(이 설정의 모든 머무는 자리 + other_spots)를 비켜 뽑는다.
    """
    places = {name: grid.snap((float(p[0]), float(p[1]))) for name, p in spec['places'].items()}
    dwell = {places[n] for g in spec['guests'] for n in [g['start']] + [st['go'] for st in g['steps']]
             if n in places}
    spots = sorted(dwell) + [tuple(p) for p in other_spots]
    defaults = spec.get('defaults') or {}
    out = []
    for g in spec['guests']:
        names = [g['start']] + [st['go'] for st in g['steps']]
        unknown = [n for n in names if n not in places]
        if unknown:
            raise ValueError(f'{g["id"]}: places에 없는 자리 {unknown}')
        loop_from = int(g.get('loop_from', 0))
        # 되풀이가 이어지려면 loop_from 단계 직전 자리(0이면 start)와 마지막 단계 자리가 같아야 한다
        if names[loop_from] != names[-1]:
            raise ValueError(f'{g["id"]}: 마지막 자리 {names[-1]}와 loop_from {loop_from} 직전 자리 '
                             f'{names[loop_from]}가 달라 되풀이가 끊긴다')
        steps = []
        here = places[g['start']]
        for st in g['steps']:
            there = places[st['go']]
            path = grid.plan(here, there, avoid=spots)
            steps.append({'name': st['go'],
                          'path': [[round(x, 3), round(y, 3)] for x, y in path],
                          'wait_s': [float(v) for v in st['wait_s']],
                          'evade_dirs': grid.evade_dirs(there)})
            here = there
        x0, y0 = places[g['start']]
        ob = {'id': g['id'], 'type': 'customer', 'pattern': 'routine'}
        ob.update({k: v for k, v in defaults.items()})
        ob.update({k: v for k, v in g.items() if k not in ('id', 'start', 'steps', 'loop_from', 'note')})
        ob['spawn'] = {'x': round(x0, 3), 'y': round(y0, 3), 'yaw': 0.0}
        ob['steps'] = steps
        ob['loop_from'] = loop_from
        if 'note' in g:
            ob['note'] = g['note']
        out.append(ob)
    return out


def load_spec(path) -> dict:
    """일과 설정 파일. include(같은 폴더의 다른 일과 설정)가 있으면 그 places·guests를 앞에 잇고 defaults를 물려받는다.

    몰림 설정이 식당 기본 손님을 그대로 품을 때 쓴다 (기본 손님 + 몰림 손님을 한 파일로 띄워 모두 혼잡도에 센다).
    자리·손님 이름이 겹치면 오류다.
    """
    path = Path(path)
    spec = yaml.safe_load(path.read_text(encoding='utf-8'))
    places, guests, defaults = {}, [], {}
    for name in spec.get('include') or []:
        base = load_spec(path.parent / name)
        _merge_new(places, base['places'], f'{path.name}: include {name} 자리')
        guests += base['guests']
        defaults.update(base.get('defaults') or {})
    _merge_new(places, spec.get('places') or {}, f'{path.name}: 자리')
    guests += spec.get('guests') or []
    ids = [g['id'] for g in guests]
    twice = sorted({i for i in ids if ids.count(i) > 1})
    if twice:
        raise ValueError(f'{path.name}: 손님 id가 겹친다 {twice}')
    defaults.update(spec.get('defaults') or {})
    return {'places': places, 'guests': guests, 'defaults': defaults}


def _merge_new(into: dict, more: dict, what: str) -> None:
    twice = sorted(set(into) & set(more))
    if twice:
        raise ValueError(f'{what} 이름이 겹친다 {twice}')
    into.update(more)


def path_length(start: Sequence[float], path: Sequence[Sequence[float]]) -> float:
    pts = [tuple(start)] + [tuple(p) for p in path]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
