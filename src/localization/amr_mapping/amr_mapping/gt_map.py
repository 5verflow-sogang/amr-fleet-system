"""정답 지도: 월드 SDF의 물체를 LiDAR 스캔 높이에서 잘라 pgm + yaml로 그린다 (SLAM 없이, ROS 비의존).

    ros2 run amr_mapping gt_map $(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/worlds/warehouse.sdf \\
        maps/warehouse_gt.yaml

SLAM 지도의 휨(#183, v1 서쪽·북쪽 끝 14~22 cm)과 벽 띠 두께(좁은 통로가 22칸으로 좁아짐) 없이 내비·Fleet 실험을
하려고 만든다. 위치추정 KPI(맵 품질)는 SLAM 지도(기본 v2)로 잰다.

- 좌표는 map이다. 물체를 TF world → map(`map_origin_in_world`, system.launch.py 기본
  [−0.0439, 1.4712, −0.00088])으로 옮겨 그리므로, launch에서 `map:=`만 바꿔 끼우면 구역 파일·측정 구역(map
  x −10~15, y −15~5) 좌표가 그대로 맞는다.
- 칸 크기·원점·칸 수는 기본 지도 v2 0.025 m(`warehouse_v2_0025`)와 같다 (#242 전에는 v1 칸).
- 칸 중심이 물체 안이면 점유(0)다. 경계가 칸 중간에 걸려도 0.6 m 통로는 0.025 m 칸 24개로 남는다.
- 외벽 안쪽 바닥은 빈 칸(254), 외벽 바깥은 미관측(205). 물체 속도 점유로 채운다 (SLAM 지도는 속이 미관측).
- 스캔 평면(지면 약 0.38 m, #92)에 걸리는 것만 그린다: 랙·기둥·도크 플랫폼·충전기 본체·외벽
  (`map_quality.MODELS`·`WALLS`, 모두 바닥부터 0.5 m 넘게 솟은 박스). 표지판(아랫면 0.7 m 이상, 충돌 없음)·
  바닥 표시·도크 앞 정차 구역 표시·ArUco 마커판(면에서 5 mm, 칸보다 얇다)은 뺀다.
"""

import argparse
import math
import os
import sys
from typing import List, Sequence, Tuple

import numpy as np

from amr_mapping.map_quality import NARROW_PASSAGES, Box, load_objects, signed_boundary_distance, write_png

MAP_ORIGIN_IN_WORLD = (-0.0439, 1.4712, -0.00088)   # system.launch.py 기본 (지도 v2 0.025 m 정렬값, #242)
RESOLUTION = 0.025
ORIGIN = (-30.1, -21.6)    # 지도 왼쪽 아래 칸 모서리 (map), v2 0.025 m와 같다
SIZE = (2411, 1609)        # 칸 수 (가로, 세로), v2 0.025 m와 같다
INSIDE = (30.0, 20.0)      # 외벽 안쪽 면 |x|, |y| (world)

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def cell_centers_world(size: Tuple[int, int], resolution: float, origin: Sequence[float],
                       map_origin_in_world: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """(행, 열) 칸 중심의 world 좌표. 첫 행이 지도 위쪽(+y)이다 (pgm 순서)."""
    w, h = size
    cols = np.arange(w)
    rows = np.arange(h)
    mx = origin[0] + (cols + 0.5) * resolution
    my = origin[1] + (h - 1 - rows + 0.5) * resolution
    mx, my = np.meshgrid(mx, my)
    ox, oy, oyaw = map_origin_in_world
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * mx - s * my, oy + s * mx + c * my


def rasterize(objs: List[Box], size: Tuple[int, int] = SIZE, resolution: float = RESOLUTION,
              origin: Sequence[float] = ORIGIN,
              map_origin_in_world: Sequence[float] = MAP_ORIGIN_IN_WORLD) -> np.ndarray:
    """물체 사각형을 칸 중심 기준으로 칠한 (행, 열) uint8 지도."""
    wx, wy = cell_centers_world(size, resolution, origin, map_origin_in_world)
    img = np.where((np.abs(wx) < INSIDE[0]) & (np.abs(wy) < INSIDE[1]), FREE, UNKNOWN).astype(np.uint8)
    for b in objs:
        # 물체 근처 칸만 본다 (외접원 안)
        r = math.hypot(b.hx, b.hy) + resolution
        near = (np.abs(wx - b.cx) <= r) & (np.abs(wy - b.cy) <= r)
        rows, cols = np.nonzero(near)
        inside = signed_boundary_distance(wx[rows, cols], wy[rows, cols], b) < 0.0
        img[rows[inside], cols[inside]] = OCCUPIED
    return img


def world_to_cell(x: float, y: float, shape: Tuple[int, int], resolution: float, origin: Sequence[float],
                  map_origin_in_world: Sequence[float]) -> Tuple[int, int]:
    """world 점이 든 칸 (행, 열)."""
    ox, oy, oyaw = map_origin_in_world
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    dx, dy = x - ox, y - oy
    mx, my = c * dx - s * dy, s * dx + c * dy
    col = int(math.floor((mx - origin[0]) / resolution))
    row = shape[0] - 1 - int(math.floor((my - origin[1]) / resolution))
    return row, col


def passage_widths(img: np.ndarray, resolution: float, origin: Sequence[float],
                   map_origin_in_world: Sequence[float], xr: Tuple[float, float], yr: Tuple[float, float],
                   step: float = 0.1, end_margin: float = 0.1) -> List[int]:
    """좁은 통로(world 사각형)를 짧은 축으로 가로지르며 빈 칸(254)이 몇 개 이어지는지 잰다.

    긴 축을 따라 step마다, 통로 중심선의 칸에서 양쪽으로 빈 칸이 끝날 때까지 센다. 지도가 휘거나 칸이
    하나 어긋나면 여기서 바로 드러난다. 중심 칸이 빈 칸이 아니면 0이다.
    """
    across_x = (xr[1] - xr[0]) < (yr[1] - yr[0])
    lo, hi = (yr if across_x else xr)
    mid = (sum(xr) / 2) if across_x else (sum(yr) / 2)
    widths = []
    for t in np.arange(lo + end_margin, hi - end_margin + 1e-9, step):
        x, y = (mid, t) if across_x else (t, mid)
        row, col = world_to_cell(x, y, img.shape, resolution, origin, map_origin_in_world)
        line = img[row, :] if across_x else img[:, col]
        i = col if across_x else row
        if line[i] != FREE:
            widths.append(0)
            continue
        a = i
        while a > 0 and line[a - 1] == FREE:
            a -= 1
        b = i
        while b < len(line) - 1 and line[b + 1] == FREE:
            b += 1
        widths.append(b - a + 1)
    return widths


def write_map(yaml_path: str, img: np.ndarray, resolution: float, origin: Sequence[float],
              header: str = '') -> str:
    """pgm(P5) + yaml을 v1과 같은 형식(trinary)으로 쓴다. pgm 경로를 돌려준다."""
    base = os.path.splitext(yaml_path)[0]
    pgm = base + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode() + img.tobytes())
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(header)
        f.write(f'image: {os.path.basename(pgm)}\nmode: trinary\nresolution: {resolution}\n'
                f'origin: [{origin[0]}, {origin[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sdf', help='월드 SDF (amr_gazebo/worlds/warehouse.sdf)')
    ap.add_argument('out_yaml', help='저장할 지도 yaml (pgm은 같은 이름으로)')
    ap.add_argument('--map-origin-in-world', nargs=3, type=float, default=list(MAP_ORIGIN_IN_WORLD),
                    metavar=('X', 'Y', 'YAW'), help='TF world → map (system.launch.py map_origin_in_world)')
    ap.add_argument('--resolution', type=float, default=RESOLUTION)
    ap.add_argument('--origin', nargs=2, type=float, default=list(ORIGIN), metavar=('X', 'Y'))
    ap.add_argument('--size', nargs=2, type=int, default=list(SIZE), metavar=('W', 'H'), help='칸 수')
    ap.add_argument('--png', help='README 표시용 PNG')
    a = ap.parse_args(argv)

    objs = load_objects(a.sdf)
    img = rasterize(objs, tuple(a.size), a.resolution, a.origin, a.map_origin_in_world)
    o = [np.format_float_positional(v) for v in a.map_origin_in_world]
    header = (f'# 정답 지도: 월드 SDF를 LiDAR 스캔 높이에서 잘라 그렸다 (amr_mapping gt_map, SLAM 아님).\n'
              f'# map_origin_in_world [{o[0]}, {o[1]}, {o[2]}]와 함께 쓴다 (system.launch.py 기본).\n')
    pgm = write_map(a.out_yaml, img, a.resolution, a.origin, header)
    counts = {v: int((img == v).sum()) for v in (OCCUPIED, FREE, UNKNOWN)}
    print(f'{pgm}: {img.shape[1]}×{img.shape[0]} 칸, 해상도 {a.resolution} m, 물체 {len(objs)}개, '
          f'점유 {counts[OCCUPIED]} / 빈 칸 {counts[FREE]} / 미관측 {counts[UNKNOWN]}')
    for name, xr, yr in NARROW_PASSAGES:
        w = passage_widths(img, a.resolution, a.origin, a.map_origin_in_world, xr, yr)
        print(f'{name}: 빈 칸 {min(w)}~{max(w)}칸 = {min(w) * a.resolution:.3f}~{max(w) * a.resolution:.3f} m '
              f'(설계 {min(xr[1] - xr[0], yr[1] - yr[0]):.1f} m, {len(w)}곳)')
    if a.png:
        write_png(a.png, img)
        print(f'PNG: {a.png}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
