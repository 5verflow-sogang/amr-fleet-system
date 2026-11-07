#!/usr/bin/env python3
"""식당 월드 정답 지도: 로봇 키보다 낮은 물체를 전부 점유로 칠한다 (테이블 상판 포함, SLAM 아님).

    python3 restaurant_map.py ../../simulation/amr_gazebo/worlds/restaurant.sdf maps/restaurant_gt.yaml \\
        --stations ../../fleet/amr_fleet_manager/config/stations_restaurant.yaml --png maps/restaurant_gt.png

왜 따로 만드나: 서빙 로봇 LiDAR(지면 0.38 m)에는 테이블 다리만 보인다. 스캔 높이에서 자른 지도(gt_map)나
SLAM 지도로는 테이블 밑이 빈 곳이라, 키 1.29 m 로봇이 경로를 테이블 밑으로 잡고 상판에 부딪힌다.
실제 서빙로봇도 지도에 가상 벽(금지 구역)을 그려 이 문제를 막는다. 여기서는 SDF 충돌체 중 높이 구간이
[바닥 위 3 cm, 로봇 키) 와 겹치는 것을 전부 칠해 같은 효과를 낸다 (Nav2 static layer가 그대로 읽는다).
AMCL은 스캔 점(다리)이 점유 칸 안에 떨어지므로 그대로 맞는다.

- 좌표: map = world (map_origin_in_world [0, 0, 0]). 0.05 m 칸, 원점 [-12.5, -8.5], 500 x 340 칸
  (--extent로 외벽 안쪽 범위를 바꾼다. 넓은 식당 restaurant_wide는 --extent -8 24.2 -12 12 → 664 x 500 칸)
- 외벽 안쪽 바닥은 빈 칸(254), 바깥은 미관측(205), 물체는 점유(0). 칸 중심이 물체 안이면 점유
- --stations를 주면 로봇 크기로 지나갈 수 있는지 잰다: 점유 칸을 로봇 반폭(내접원)만큼 부풀린 뒤
  wait_1에서 모든 정차 자세까지 이어지는지 (Nav2가 통과 불가로 보는 기준과 같다)

PyYAML·numpy가 필요하다 (PNG는 PIL). ROS 없이 돈다.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import yaml

RESOLUTION = 0.05
EXTENT = (-12.0, 12.0, -8.0, 8.0)   # 외벽 안쪽 면 [x_min, x_max, y_min, y_max] (--extent)
MARGIN = 0.5                        # 외벽 바깥으로 더 그리는 폭 (m)
ORIGIN = (-12.5, -8.5)
SIZE = (500, 340)           # 칸 수 (가로, 세로). set_extent가 EXTENT에서 다시 정한다
ROBOT_HEIGHT = 1.29         # body:=serving (Pudu BellaBot 크기)
ROBOT_HALF = (0.2825, 0.2685)   # 반길이, 반폭
FLOOR_EPS = 0.03            # 이보다 낮은 것(바닥 표시, 충전 패드)은 넘어간다
OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def _pose(el):
    p = el.find('pose') if el is not None else None
    v = [float(x) for x in p.text.split()] if p is not None and p.text else [0.0] * 6
    return v[0], v[1], v[2], v[5]


def _compose(a, b):
    """(x, y, z, yaw) 두 개를 잇는다 (roll·pitch는 이 월드에 없다)."""
    ax, ay, az, ayaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, az + bz, ayaw + byaw


def _model_shapes(model, pose, models_dir):
    """모델 안 충돌체 → [(cx, cy, yaw, hx, hy, zmin, zmax, kind)] (world)."""
    out = []
    for link in model.findall('link'):
        lp = _compose(pose, _pose(link))
        for col in link.findall('collision'):
            cx, cy, cz, cyaw = _compose(lp, _pose(col))
            g = col.find('geometry')
            box, cyl = g.find('box'), g.find('cylinder')
            if box is not None:
                sx, sy, sz = (float(v) for v in box.find('size').text.split())
                out.append((cx, cy, cyaw, sx / 2, sy / 2, cz - sz / 2, cz + sz / 2, 'box'))
            elif cyl is not None:
                r = float(cyl.find('radius').text)
                h = float(cyl.find('length').text)
                out.append((cx, cy, 0.0, r, r, cz - h / 2, cz + h / 2, 'cyl'))
    for inc in model.findall('include'):
        out += _include_shapes(inc, pose, models_dir)
    return out


def _include_shapes(inc, parent, models_dir):
    uri = inc.find('uri').text.strip()
    if not uri.startswith('model://'):
        raise SystemExit(f'model:// 가 아닌 include는 읽지 못한다: {uri}')
    sdf = ET.parse(os.path.join(models_dir, uri[len('model://'):], 'model.sdf')).getroot()
    pose = _compose(parent, _pose(inc))
    m = sdf.find('model')
    return _model_shapes(m, _compose(pose, _pose(m)), models_dir)


def load_shapes(sdf_path):
    world = ET.parse(sdf_path).getroot().find('world')
    models_dir = os.path.join(os.path.dirname(os.path.abspath(sdf_path)), '..', 'models')
    shapes = []
    for m in world.findall('model'):
        shapes += _model_shapes(m, _pose(m), models_dir)
    for inc in world.findall('include'):
        shapes += _include_shapes(inc, (0.0, 0.0, 0.0, 0.0), models_dir)
    return shapes


def set_extent(extent):
    """외벽 안쪽 범위로 지도 원점·칸 수를 정한다 (기본 식당은 원점 [-12.5, -8.5], 500 x 340 칸)."""
    global EXTENT, ORIGIN, SIZE
    x0, x1, y0, y1 = (float(v) for v in extent)
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f'--extent는 x_min x_max y_min y_max: {extent}')
    EXTENT = (x0, x1, y0, y1)
    ORIGIN = (x0 - MARGIN, y0 - MARGIN)
    SIZE = (int(round((x1 - x0 + 2 * MARGIN) / RESOLUTION)), int(round((y1 - y0 + 2 * MARGIN) / RESOLUTION)))


def cell_centers():
    w, h = SIZE
    xs = ORIGIN[0] + (np.arange(w) + 0.5) * RESOLUTION
    ys = ORIGIN[1] + (h - 1 - np.arange(h) + 0.5) * RESOLUTION   # 첫 행이 위쪽(+y)
    return np.meshgrid(xs, ys)


def rasterize(shapes, robot_height=ROBOT_HEIGHT):
    wx, wy = cell_centers()
    x0, x1, y0, y1 = EXTENT
    img = np.where((wx > x0) & (wx < x1) & (wy > y0) & (wy < y1), FREE, UNKNOWN).astype(np.uint8)
    used = 0
    for cx, cy, yaw, hx, hy, zmin, zmax, kind in shapes:
        if zmax <= FLOOR_EPS or zmin >= robot_height:
            continue
        used += 1
        dx, dy = wx - cx, wy - cy
        if kind == 'cyl':
            inside = dx * dx + dy * dy < hx * hx
        else:
            c, s = math.cos(-yaw), math.sin(-yaw)
            inside = (np.abs(c * dx - s * dy) < hx) & (np.abs(s * dx + c * dy) < hy)
        img[inside] = OCCUPIED
    return img, used


def to_cell(x, y):
    col = int(math.floor((x - ORIGIN[0]) / RESOLUTION))
    row = SIZE[1] - 1 - int(math.floor((y - ORIGIN[1]) / RESOLUTION))
    return row, col


def reachable(img, radius, start):
    """점유 칸을 radius만큼 부풀린 빈 공간에서 start 칸과 이어진 칸 (bool)."""
    blocked = img != FREE
    k = int(math.ceil(radius / RESOLUTION))
    grown = blocked.copy()
    h, w = img.shape
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            if (dr * dr + dc * dc) * RESOLUTION ** 2 > radius ** 2:
                continue
            src = blocked[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            grown[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    seen = np.zeros_like(blocked)
    if grown[start]:
        return seen, grown
    q = deque([start])
    seen[start] = True
    while q:
        r, c = q.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < h and 0 <= nc < w and not seen[nr, nc] and not grown[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return seen, grown


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sdf')
    ap.add_argument('out_yaml')
    ap.add_argument('--robot-height', type=float, default=ROBOT_HEIGHT)
    ap.add_argument('--stations', help='stations_restaurant.yaml: 로봇 크기로 모든 정차 자세에 갈 수 있는지 잰다')
    ap.add_argument('--png', help='README 표시용 PNG')
    ap.add_argument('--extent', type=float, nargs=4, default=EXTENT, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'),
                    help='외벽 안쪽 범위 [m] (기본 식당 -12 12 -8 8, 넓은 식당 -8 24.2 -12 12)')
    a = ap.parse_args(argv)
    set_extent(a.extent)

    img, used = rasterize(load_shapes(a.sdf), a.robot_height)
    base = os.path.splitext(a.out_yaml)[0]
    with open(base + '.pgm', 'wb') as f:
        f.write(f'P5\n{SIZE[0]} {SIZE[1]}\n255\n'.encode() + img.tobytes())
    with open(a.out_yaml, 'w', encoding='utf-8') as f:
        f.write('# 식당 정답 지도: 월드 SDF에서 서빙 로봇 키(1.29 m)보다 낮은 물체를 전부 칠했다 - 테이블 상판 포함\n'
                '# (amr_mapping tools/restaurant_map.py, SLAM 아님). map = world, map_origin_in_world [0, 0, 0]\n'
                f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n'
                f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{base}.pgm: {SIZE[0]}x{SIZE[1]} 칸, {RESOLUTION} m, 칠한 충돌체 {used}개, 점유 {int((img == OCCUPIED).sum())}칸')

    ok = True
    if a.stations:
        with open(a.stations, encoding='utf-8') as f:
            st = yaml.safe_load(f)['stations']
        start = to_cell(*st['wait_1'][:2])
        # 내접원(반폭): Nav2가 통과 불가로 보는 거리. 외접원: 그 자리에서 제자리 회전할 수 있는지
        for label, r in (('지나감(반폭)', ROBOT_HALF[1]), ('제자리 회전(외접원)', math.hypot(*ROBOT_HALF))):
            seen, grown = reachable(img, r, start)
            bad = [n for n, p in st.items() if not seen[to_cell(p[0], p[1])]]
            print(f'{label} r={r:.3f} m: 정차 자세 {len(st) - len(bad)}/{len(st)}곳 도달' + (f', 못 감: {bad}' if bad else ''))
            if label.startswith('지나감') and bad:
                ok = False
    if a.png:
        from PIL import Image
        Image.fromarray(img).save(a.png)
        print(f'PNG: {a.png}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
