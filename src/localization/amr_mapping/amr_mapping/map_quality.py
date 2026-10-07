"""지도 품질 평가: 저장한 지도(pgm + yaml)를 월드 SDF의 실제 물체 위치와 비교한다 (ROS 비의존).

    ros2 run amr_mapping map_quality maps/warehouse_v1.yaml \\
        $(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/worlds/warehouse.sdf --start 0 1.5 0

1. 점유 칸(값 0)을 map → world로 옮긴다. 매핑 시작 자세(world)가 map 원점이다.
2. 각 점유 칸을 가장 가까운 물체(랙·기둥·도크·충전기·외벽) 테두리에 붙이고 거리를 잰다 (0.5 m 안).
3. 물체별 |거리| 중앙값 → 전체 중앙값·최대로 판정한다 (인계 계약 #81: 중앙값 ≤ 0.05 m, 최대 ≤ 0.15 m).
4. 어느 물체와도 0.3 m 넘게 떨어진 점유 칸을 유령·이중 벽 후보로 센다 (벽 겹침 확인).
"""

import argparse
from dataclasses import dataclass
import math
import os
import struct
import sys
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET
import zlib

import numpy as np
import yaml

# 모델 이름 → (모델 원점 기준 중심 x, y, 크기 x, y). amr_gazebo/models의 충돌 형상과 같다
MODELS = {
    'warehouse_rack': (0.0, 0.0, 5.0, 1.2),
    'warehouse_pillar': (0.0, 0.0, 0.6, 0.6),
    'loading_dock': (0.75, 0.0, 1.5, 3.5),       # 플랫폼
    'charging_station': (0.15, 0.0, 0.3, 0.6),   # 본체
}
# 외벽 (warehouse.sdf outer_walls): 이름, 중심 x, y, 크기 x, y
WALLS = [('wall_north', 0.0, 20.1, 60.4, 0.2), ('wall_south', 0.0, -20.1, 60.4, 0.2),
         ('wall_east', 30.1, 0.0, 0.2, 40.0), ('wall_west', -30.1, 0.0, 0.2, 40.0)]

# 좁은 통로 (warehouse.sdf, amr_gazebo README): 이름, x 범위, y 범위. 안쪽이 미관측이면 경로 계획에서 못 쓴다
NARROW_PASSAGES = [
    ('좁은 통로 ① (rack_b3–rack_b4)', (-5.0, -4.4), (-2.6, -1.4)),
    ('좁은 통로 ② (rack_lane_divider–rack_b4)', (-4.4, 0.6), (-1.4, -0.8)),
]

MATCH_DISTANCE = 0.5   # m. 이 안의 점유 칸만 물체에 붙인다
GHOST_DISTANCE = 0.3   # m. 이보다 먼 점유 칸은 유령·이중 벽 후보
MEDIAN_LIMIT = 0.05    # m (#81)
MAX_LIMIT = 0.15       # m (#81)


@dataclass(frozen=True)
class Box:
    name: str
    kind: str
    cx: float
    cy: float
    yaw: float
    hx: float   # 반 크기
    hy: float


def load_objects(sdf_path: str) -> List[Box]:
    """월드 SDF의 include(랙·기둥·도크·충전기)와 외벽을 사각형 목록으로."""
    objs = []
    for inc in ET.parse(sdf_path).getroot().iter('include'):
        model = inc.findtext('uri', '').replace('model://', '')
        if model not in MODELS:
            continue
        x, y, _, _, _, yaw = [float(v) for v in inc.findtext('pose').split()]
        lx, ly, sx, sy = MODELS[model]
        c, s = math.cos(yaw), math.sin(yaw)
        objs.append(Box(inc.findtext('name'), model, x + c * lx - s * ly, y + s * lx + c * ly, yaw,
                        sx / 2, sy / 2))
    for name, x, y, sx, sy in WALLS:
        objs.append(Box(name, 'wall', x, y, 0.0, sx / 2, sy / 2))
    return objs


def signed_boundary_distance(px: np.ndarray, py: np.ndarray, b: Box) -> np.ndarray:
    """점들에서 사각형 테두리까지 부호 있는 거리 (바깥 +, 안쪽 −)."""
    c, s = math.cos(-b.yaw), math.sin(-b.yaw)
    dx0, dy0 = px - b.cx, py - b.cy
    lx, ly = c * dx0 - s * dy0, s * dx0 + c * dy0
    dx, dy = np.abs(lx) - b.hx, np.abs(ly) - b.hy
    outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    return np.where((dx > 0) | (dy > 0), outside, np.maximum(dx, dy))


def boundary_distance(px: np.ndarray, py: np.ndarray, b: Box) -> np.ndarray:
    """점들에서 사각형 테두리까지 거리 (안쪽도 양수)."""
    return np.abs(signed_boundary_distance(px, py, b))


def read_pgm(path: str) -> np.ndarray:
    """P5(바이너리) pgm → (행, 열) uint8. 첫 행이 지도 위쪽(+y)이다."""
    with open(path, 'rb') as f:
        data = f.read()
    parts, i = [], 0
    while len(parts) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    if parts[0] != b'P5':
        raise ValueError(f'P5 pgm만 읽는다: {parts[0]!r}')
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


def write_png(path: str, img: np.ndarray) -> None:
    """8비트 회색조 PNG (README 표시용, PIL 없이)."""
    h, _ = img.shape
    raw = b''.join(b'\x00' + img[r].tobytes() for r in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', img.shape[1], h, 8, 0, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def occupied_world_points(img: np.ndarray, resolution: float, origin: Tuple[float, float],
                          start: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """점유 칸 중심을 world 좌표로. start = 매핑 시작 자세 (x, y, yaw) in world."""
    h = img.shape[0]
    rows, cols = np.nonzero(img == 0)
    mx = origin[0] + (cols + 0.5) * resolution
    my = origin[1] + (h - 1 - rows + 0.5) * resolution
    c, s = math.cos(start[2]), math.sin(start[2])
    return start[0] + c * mx - s * my, start[1] + s * mx + c * my


def evaluate(wx: np.ndarray, wy: np.ndarray, objs: List[Box]) -> Dict:
    """물체별 테두리 오차 중앙값, 위치 어긋남, 유령 칸 수.

    - 테두리 오차 (#81 판정): 물체에 붙은 점유 칸의 |거리| 중앙값.
      LiDAR 노이즈로 벽이 여러 칸 두께로 찍히면 칸이 작은 지도일수록 커진다.
    - 위치 어긋남: 부호 있는 거리(바깥 +)의 중앙값의 크기. 띠 두께와 무관하게 벽 위치가
      실제와 얼마나 어긋났는지 본다. 해상도가 다른 지도를 비교할 때 쓴다.
    """
    if len(wx) == 0:
        raise ValueError('점유 칸이 없다')
    sd = np.stack([signed_boundary_distance(wx, wy, b) for b in objs])
    d = np.abs(sd)
    nearest, dmin = d.argmin(axis=0), d.min(axis=0)
    per, offsets = {}, []
    for k, b in enumerate(objs):
        sel = (nearest == k) & (dmin < MATCH_DISTANCE)
        if sel.any():
            per[b.name] = (b.kind, float(np.median(dmin[sel])), int(sel.sum()))
            offsets.append(abs(float(np.median(sd[k, sel]))))
        else:
            per[b.name] = (b.kind, float('nan'), 0)
    found = [v[1] for v in per.values() if v[2] > 0]
    median, worst = float(np.median(found)), float(np.max(found))
    return {
        'per_object': per,
        'missing': [n for n, v in per.items() if v[2] == 0],
        'median': median,
        'max': worst,
        'passed': median <= MEDIAN_LIMIT and worst <= MAX_LIMIT,
        'offset_median': float(np.median(offsets)),
        'offset_max': float(np.max(offsets)),
        'ghost_cells': int((dmin > GHOST_DISTANCE).sum()),
        'occupied_cells': int(len(wx)),
    }


def region_unknown_ratio(img: np.ndarray, resolution: float, origin: Tuple[float, float],
                         start: Tuple[float, float, float], xr: Tuple[float, float],
                         yr: Tuple[float, float], unknown_value: int = 205) -> float:
    """world 사각형(x 범위, y 범위) 안 칸 중 미관측 비율. 테두리 한 칸은 벽 띠라 뺀다."""
    h, w = img.shape
    rows, cols = np.mgrid[0:h, 0:w]
    mx = origin[0] + (cols + 0.5) * resolution
    my = origin[1] + (h - 1 - rows + 0.5) * resolution
    c, s = math.cos(start[2]), math.sin(start[2])
    wx, wy = start[0] + c * mx - s * my, start[1] + s * mx + c * my
    sel = ((wx > xr[0] + resolution) & (wx < xr[1] - resolution)
           & (wy > yr[0] + resolution) & (wy < yr[1] - resolution))
    return float((img[sel] == unknown_value).sum() / max(sel.sum(), 1))


def unknown_inside_ratio(img: np.ndarray, resolution: float, origin: Tuple[float, float],
                         start: Tuple[float, float, float], objs: List[Box],
                         unknown_value: int = 205, margin: float = 0.3) -> float:
    """창고 안 빈 바닥(외벽 안쪽, 물체에서 margin 넘게 떨어진 곳) 중 미관측(unknown) 칸 비율.

    칸이 작으면 먼 곳에서 0.5° 간격 빔 사이로 아무 빔도 지나가지 않는 칸이 생긴다.
    """
    h, w = img.shape
    rows, cols = np.mgrid[0:h, 0:w]
    mx = origin[0] + (cols + 0.5) * resolution
    my = origin[1] + (h - 1 - rows + 0.5) * resolution
    c, s = math.cos(start[2]), math.sin(start[2])
    wx, wy = start[0] + c * mx - s * my, start[1] + s * mx + c * my
    inside = (np.abs(wx) < 30.0 - margin) & (np.abs(wy) < 20.0 - margin)
    for b in objs:
        if b.kind != 'wall':
            inside &= signed_boundary_distance(wx, wy, b) > margin
    return float((img[inside] == unknown_value).sum() / max(inside.sum(), 1))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map_yaml')
    ap.add_argument('sdf')
    ap.add_argument('--start', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'),
                    help='매핑 시작 자세 (world)')
    ap.add_argument('--png', help='지도를 PNG로도 저장')
    a = ap.parse_args(argv)

    with open(a.map_yaml) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(a.map_yaml)), meta['image']))
    wx, wy = occupied_world_points(img, meta['resolution'], meta['origin'][:2], tuple(a.start))
    objs = load_objects(a.sdf)
    r = evaluate(wx, wy, objs)

    print(f'지도 {img.shape[1]}×{img.shape[0]} 칸, 해상도 {meta["resolution"]} m, 점유 칸 {r["occupied_cells"]}')
    groups = [('랙·기둥', ('warehouse_rack', 'warehouse_pillar')),
              ('도크·충전기', ('loading_dock', 'charging_station')), ('외벽', ('wall',))]
    for label, kinds in groups:
        g = {n: v for n, v in r['per_object'].items() if v[0] in kinds}
        meds = {n: v[1] for n, v in g.items() if v[2] > 0}
        if not meds:
            print(f'  {label}: 지도에 없음')
            continue
        worst = max(meds, key=meds.get)
        print(f'  {label}: {len(meds)}/{len(g)}개, 물체별 중앙값의 중앙값 {np.median(list(meds.values())) * 100:.1f} cm, '
              f'최대 {meds[worst] * 100:.1f} cm ({worst})')
    print(f'전체: 중앙값 {r["median"] * 100:.1f} cm, 최대 {r["max"] * 100:.1f} cm → '
          f'#81 기준(≤{MEDIAN_LIMIT * 100:.0f} cm, ≤{MAX_LIMIT * 100:.0f} cm) {"합격" if r["passed"] else "불합격"}')
    print(f'위치 어긋남 (부호 있는 중앙값, 띠 두께 무관): 중앙값 {r["offset_median"] * 100:.1f} cm, '
          f'최대 {r["offset_max"] * 100:.1f} cm')
    print(f'유령·이중 벽 후보: {r["ghost_cells"]}칸 ({r["ghost_cells"] / r["occupied_cells"] * 100:.2f}%)')
    ratio = unknown_inside_ratio(img, meta['resolution'], meta['origin'][:2], tuple(a.start), objs)
    print(f'창고 안 빈 바닥 중 미관측(unknown) 칸: {ratio * 100:.2f}%')
    for name, xr, yr in NARROW_PASSAGES:
        r_pass = region_unknown_ratio(img, meta['resolution'], meta['origin'][:2], tuple(a.start), xr, yr)
        print(f'{name} 안 미관측: {r_pass * 100:.0f}%')
    if r['missing']:
        print('지도에 없는 물체: ' + ', '.join(r['missing']))
    if a.png:
        write_png(a.png, img)
        print(f'PNG: {a.png}')
    return 0 if r['passed'] and not r['missing'] else 1


if __name__ == '__main__':
    sys.exit(main())
