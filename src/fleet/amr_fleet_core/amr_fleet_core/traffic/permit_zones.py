"""지도에서 좁은 통로를 찾아 구역 허가(정원 1) 구역을 만든다. ROS 비의존.

    PYTHONPATH=src/fleet/amr_fleet_core python3 -m amr_fleet_core.traffic.permit_zones \\
        src/localization/amr_mapping/maps/restaurant_gt.yaml \\
        src/fleet/amr_fleet_manager/config/stations_restaurant.yaml \\
        -o src/fleet/amr_fleet_manager/config/zones_restaurant.yaml --png zones_restaurant.png

상용 서빙·물류 로봇은 설치 기사가 지도에 "한 대만 들어가는 구역"을 손으로 그린다. 여기서는 후보를 지도에서 뽑고
사람은 그림(--png)으로 확인만 한다. 결과 파일은 config/zones.yaml 형식이라 서버·구역 클라이언트를 고치지 않는다.

1. 좁은 칸: 지름 max_width_m 원이 들어가지 못하는 빈 칸 (통로 폭 < max_width_m. 서빙 로봇 두 대가 엇갈리려면 1.07 m + 여유)
2. 로봇이 다니는 곳만: 정차 자리 모든 쌍의 최단 경로(로봇 반폭만큼 벽에서 떨어진 칸, 8방향)가 지나는 좁은 칸 덩어리 중
   경로가 min_length_m 이상 지나는 것. 방 구석·조리대 옆처럼 원이 못 들어가지만 아무도 안 가는 곳이 빠진다
3. 덩어리(로봇 중심이 설 수 있는 칸)에 통로를 빠져나가는 경로 end_margin_m만큼을 더한 사각형: 나오는 로봇이 몸을 다
   빼기 전에 반납하면 기다리던 로봇이 들어와 입구에서 막힌다 (11/16 restaurant_headon, 중앙 통로 구역을 입구 앞까지 늘려 해결)
4. 겹치는 사각형은 하나로 묶는다: 교차로로 이어진 통로를 따로 두면 두 대가 서로 다른 팔로 들어와 교차로에서 만난다.
   교차로는 통로보다 넓어 좁은 칸이 아니지만 3의 여유가 교차로를 덮어 양쪽 팔 사각형이 겹친다
5. 오래 서 있는 자리(충전·대기, keep_out)가 들어가면 그 자리가 빠지도록 사각형 한 변을 줄인다. 거기 선 로봇이
   구역을 계속 잡고 있으면 다른 로봇이 못 들어간다. 매장·테이블·퇴식 자리는 싣고 내리는 2 s(amr_bt dwell_s)만 서므로 둔다

shortcut: map = world 지도만 다룬다 (식당 restaurant_gt, map_origin_in_world [0, 0, 0]). 다른 지도에 쓰려면
area_world로 바꾸는 world ← map 변환을 더한다.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

Rect = Tuple[float, float, float, float]   # x_min, x_max, y_min, y_max (m)

MAX_WIDTH_M = 1.3
ROBOT_HALF_WIDTH_M = 0.2685     # 서빙 로봇 반폭 (amr_bringup nav2_profile, body:=serving)
MIN_LENGTH_M = 1.0
END_MARGIN_M = 0.6
KEEP_OUT = ('charger_', 'wait_')
SNAP_MAX_M = 0.5                # 정차 자리에서 이보다 멀리 있는 칸으로만 갈 수 있으면 지도·정차 자리가 어긋난 것
FREE_MIN = 250                  # 빈 칸(254)만. 미관측(205)은 벽 밖이다 (amr_mapping tools/restaurant_map.py)


@dataclass
class Grid:
    free: np.ndarray            # bool, [행 = y(남→북), 열 = x]
    resolution: float
    origin: Tuple[float, float]

    def cell_center(self, rows, cols):
        return (self.origin[0] + (np.asarray(cols) + 0.5) * self.resolution,
                self.origin[1] + (np.asarray(rows) + 0.5) * self.resolution)


@dataclass
class Zone:
    rect: Rect
    width_m: float              # 가장 좁은 곳이 아니라 덩어리 안 통로 폭의 최댓값 (묶인 통로 중 가장 넓은 것)
    stations: List[str]


def read_pgm(path: Path) -> np.ndarray:
    """P5(8비트) PGM. 주석 줄(#)을 건너뛴다."""
    data = path.read_bytes()
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
    if fields[0] != b'P5' or int(fields[3]) > 255:
        raise ValueError(f'{path}: 8비트 P5 PGM만 읽는다')
    w, h = int(fields[1]), int(fields[2])
    return np.frombuffer(data, np.uint8, w * h, pos + 1).reshape(h, w)


def read_map(map_yaml: Path) -> Grid:
    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    if meta.get('negate', 0):
        raise ValueError(f'{map_yaml}: negate 0 지도만 다룬다')
    img = read_pgm(map_yaml.parent / meta['image'])[::-1]     # PGM 첫 줄이 북쪽 끝
    return Grid(img >= FREE_MIN, float(meta['resolution']), (float(meta['origin'][0]), float(meta['origin'][1])))


def read_stations(path: Path) -> Dict[str, Tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))['stations']
    return {str(k): (float(v[0]), float(v[1])) for k, v in data.items()}


def _clearance(grid: Grid) -> np.ndarray:
    """칸 중심에서 가장 가까운 막힌 칸 가장자리까지 (m). 폭 1.0 m 통로 가운데가 0.5다."""
    return ndimage.distance_transform_edt(grid.free) * grid.resolution - grid.resolution / 2


def _narrow(grid: Grid, clear: np.ndarray, max_width_m: float) -> np.ndarray:
    """지름 max_width_m 원이 어떤 자리에서도 덮지 못하는 빈 칸."""
    r = max_width_m / 2
    covered = ndimage.distance_transform_edt(~(clear >= r)) * grid.resolution <= r
    return grid.free & ~covered


def _on_shortest_paths(grid: Grid, traversable: np.ndarray, stations: Dict[str, Tuple[float, float]]) -> np.ndarray:
    """정차 자리 모든 쌍의 최단 경로 위 칸 (같은 길이 경로가 여럿이면 모두)."""
    n = int(traversable.sum())
    idx = np.full(traversable.shape, -1)
    idx[traversable] = np.arange(n)
    h, w = traversable.shape
    rows, cols, wts = [], [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = idx[0:h - dy, max(0, -dx):w - max(0, dx)]
        b = idx[dy:h, max(0, dx):w - max(0, -dx)]
        m = (a >= 0) & (b >= 0)
        rows.append(a[m])
        cols.append(b[m])
        wts.append(np.full(int(m.sum()), grid.resolution * math.hypot(dx, dy)))
    graph = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()

    tr, tc = np.nonzero(traversable)
    tx, ty = grid.cell_center(tr, tc)
    sources = []
    for name, (x, y) in stations.items():
        d2 = (tx - x) ** 2 + (ty - y) ** 2
        j = int(np.argmin(d2))
        if d2[j] > SNAP_MAX_M ** 2:
            raise ValueError(f'정차 자리 {name} ({x}, {y}) 근처 {SNAP_MAX_M} m 안에 로봇이 설 칸이 없다')
        sources.append(idx[tr[j], tc[j]])
    dist = dijkstra(graph, directed=False, indices=sources)
    on = np.zeros(n, bool)
    for i in range(len(sources)):
        for j in range(i + 1, len(sources)):
            d = dist[i, sources[j]]
            if np.isfinite(d):
                on |= np.abs(dist[i] + dist[j] - d) < 1e-6
    out = np.zeros(traversable.shape, bool)
    out[traversable] = on
    return out


def _bbox(grid: Grid, mask: np.ndarray) -> Rect:
    r, c = np.nonzero(mask)
    res, (ox, oy) = grid.resolution, grid.origin
    return (ox + c.min() * res, ox + (c.max() + 1) * res, oy + r.min() * res, oy + (r.max() + 1) * res)


def _overlap(a: Rect, b: Rect) -> bool:
    return a[0] <= b[1] and b[0] <= a[1] and a[2] <= b[3] and b[2] <= a[3]


def _inside(rect: Rect, p: Tuple[float, float]) -> bool:
    return rect[0] <= p[0] <= rect[1] and rect[2] <= p[1] <= rect[3]


def _clip_out(rect: Rect, p: Tuple[float, float], eps: float, res: float, keep: np.ndarray) -> Rect:
    """p(와 eps 여유)가 빠지도록 한 변을 칸 경계로 옮긴다. 덮던 좁은 통로 칸(keep, [N, 2] 좌표)을 가장 적게 잃는 변,
    같으면 넓이가 가장 많이 남는 변."""
    x0, x1, y0, y1 = rect
    lo = lambda v: math.floor(round(v / res, 6)) * res     # noqa: E731
    hi = lambda v: math.ceil(round(v / res, 6)) * res      # noqa: E731
    options = [(hi(p[0] + eps), x1, y0, y1), (x0, lo(p[0] - eps), y0, y1),
               (x0, x1, hi(p[1] + eps), y1), (x0, x1, y0, lo(p[1] - eps))]

    def score(r):
        x, y = keep[:, 0], keep[:, 1]
        kept = int(np.count_nonzero((x >= r[0]) & (x <= r[1]) & (y >= r[2]) & (y <= r[3])))
        return kept, (r[1] - r[0]) * (r[3] - r[2])

    return max((r for r in options if r[0] < r[1] and r[2] < r[3]), key=score)


def permit_zones(grid: Grid, stations: Dict[str, Tuple[float, float]], *, max_width_m: float = MAX_WIDTH_M,
                 robot_half_width_m: float = ROBOT_HALF_WIDTH_M, min_length_m: float = MIN_LENGTH_M,
                 end_margin_m: float = END_MARGIN_M, keep_out: Sequence[str] = KEEP_OUT,
                 debug: Optional[dict] = None) -> List[Zone]:
    clear = _clearance(grid)
    traversable = clear >= robot_half_width_m
    narrow = _narrow(grid, clear, max_width_m)
    on_path = _on_shortest_paths(grid, traversable, stations)

    pieces: List[Tuple[Rect, float]] = []
    kept = np.zeros(narrow.shape, bool)
    labels, count = ndimage.label(narrow)
    for k in range(1, count + 1):
        comp = labels == k
        used = comp & on_path
        if not used.any():
            continue
        x0, x1, y0, y1 = _bbox(grid, used)
        if max(x1 - x0, y1 - y0) < min_length_m:
            continue
        kept |= used
        # 여유는 경로가 통로를 빠져나가는 쪽으로만 (사각형 네 변으로 늘리면 옆 좌석·매장 자리까지 덮는다)
        near = ndimage.distance_transform_edt(~comp) * grid.resolution <= end_margin_m
        pieces.append((_bbox(grid, (comp & traversable) | (on_path & near)), 2 * float(clear[comp].max())))

    merged = True
    while merged:
        merged = False
        for i in range(len(pieces)):
            for j in range(i + 1, len(pieces)):
                (a, wa), (b, wb) = pieces[i], pieces[j]
                if _overlap(a, b):
                    pieces[i] = ((min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3])), max(wa, wb))
                    del pieces[j]
                    merged = True
                    break
            if merged:
                break

    keep_xy = np.column_stack(grid.cell_center(*np.nonzero(kept)))
    zones = []
    for rect, width in pieces:
        for name, p in stations.items():
            if name.startswith(tuple(keep_out)) and _inside(rect, p):
                rect = _clip_out(rect, p, robot_half_width_m, grid.resolution, keep_xy)
        rect = tuple(round(v, 3) for v in rect)
        zones.append(Zone(rect, round(width, 2), sorted(n for n, p in stations.items() if _inside(rect, p))))
    zones.sort(key=lambda z: (z.rect[0], z.rect[2]))
    if debug is not None:
        debug.update(narrow=narrow, on_path=on_path)
    return zones


def to_yaml(zones: List[Zone], header: str) -> str:
    lines = [f'# {line}' if line else '#' for line in header.splitlines()] + ['zones:']
    if not zones:
        lines[-1] = 'zones: []'
    for i, z in enumerate(zones, 1):
        x0, x1, y0, y1 = z.rect
        where = ', '.join(z.stations) if z.stations else '없음'
        lines += [f'  - id: narrow_{i:02d}',
                  '    capacity: 1',
                  f'    area_world: [{x0:g}, {x1:g}, {y0:g}, {y1:g}]',
                  f'    description: 통로 폭 {z.width_m:.2f} m 이하, {x1 - x0:.1f} x {y1 - y0:.1f} m. 안의 정차 자리 {where}']
    return '\n'.join(lines) + '\n'


def draw_png(path: Path, grid: Grid, zones: List[Zone], stations: Dict[str, Tuple[float, float]], debug: dict,
             scale: int = 3) -> None:
    from PIL import Image, ImageDraw   # 검토 그림에만 쓴다

    rgb = np.where(grid.free[..., None], 255, 40).repeat(3, axis=2).astype(np.uint8)
    rgb[debug['on_path'] & ~debug['narrow']] = (190, 210, 250)
    rgb[debug['narrow']] = (250, 190, 190)
    rgb[debug['narrow'] & debug['on_path']] = (215, 60, 60)
    h, w = grid.free.shape
    img = Image.fromarray(rgb[::-1]).resize((w * scale, h * scale), Image.NEAREST)
    draw = ImageDraw.Draw(img)
    res, (ox, oy) = grid.resolution, grid.origin

    def px(x, y):
        return (x - ox) / res * scale, (h - (y - oy) / res) * scale

    for i, z in enumerate(zones, 1):
        (a, b), (c, d) = px(z.rect[0], z.rect[3]), px(z.rect[1], z.rect[2])
        draw.rectangle([a, b, c, d], outline=(30, 60, 220), width=3)
        draw.text((a + 4, b + 4), f'narrow_{i:02d}', fill=(30, 60, 220))
    for (x, y) in stations.values():
        a, b = px(x, y)
        draw.ellipse([a - 3, b - 3, a + 3, b + 3], fill=(20, 140, 60))
    img.save(path)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('map_yaml', type=Path)
    p.add_argument('stations_yaml', type=Path)
    p.add_argument('-o', '--out', type=Path, required=True, help='config/zones.yaml 형식 출력')
    p.add_argument('--png', type=Path, help='검토 그림 (PIL 필요)')
    p.add_argument('--max-width', type=float, default=MAX_WIDTH_M)
    p.add_argument('--min-length', type=float, default=MIN_LENGTH_M)
    p.add_argument('--end-margin', type=float, default=END_MARGIN_M)
    a = p.parse_args(argv)

    grid, stations = read_map(a.map_yaml), read_stations(a.stations_yaml)
    debug: dict = {}
    zones = permit_zones(grid, stations, max_width_m=a.max_width, min_length_m=a.min_length,
                         end_margin_m=a.end_margin, debug=debug)
    header = HEADER.format(map=a.map_yaml.name, stations=a.stations_yaml.name, max_width=a.max_width,
                           min_length=a.min_length, end_margin=a.end_margin)
    a.out.write_text(to_yaml(zones, header), encoding='utf-8')
    if a.png:
        draw_png(a.png, grid, zones, stations, debug)
    for i, z in enumerate(zones, 1):
        print(f'narrow_{i:02d} {z.rect} 폭 {z.width_m} m, 정차 자리 {z.stations}')
    return 0


HEADER = """구역 허가(ZoneRequest/ZoneGrant) 대상 구역, 식당. 한 번에 한 대만 들어간다.
자동 생성: amr_fleet_core.traffic.permit_zones ({map}, {stations}). 손으로 고치지 말고 다시 만든다.
기준: 통로 폭 < {max_width:g} m, 정차 자리 사이 최단 경로가 {min_length:g} m 이상 지나는 곳, 긴 방향 양 끝 여유 {end_margin:g} m,
겹치면 하나로, 충전·대기 자리는 뺀다. 그림: --png (모듈 설명 참고)

area_world: [x_min, x_max, y_min, y_max] (m, Gazebo world 좌표 = 식당 map 좌표). 로봇 측 규칙은 zones.yaml과 같다."""

if __name__ == '__main__':
    raise SystemExit(main())
