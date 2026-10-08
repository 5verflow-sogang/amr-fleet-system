"""지도 품질 평가: 저장한 지도(pgm + yaml)를 월드 SDF의 실제 물체 위치와 비교한다 (ROS 비의존).

    ros2 run amr_mapping map_quality maps/warehouse_v1.yaml \\
        $(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/worlds/warehouse.sdf --start 0 1.5 0

1. 점유 칸(값 0)을 map → world로 옮긴다. 매핑 시작 자세(world)가 map 원점이다.
2. 지도 정렬: 점유 칸 전체를 월드 물체 테두리에 강체 정합해 (dx, dy, dθ)를 구한다. 이 보정을 더한 시작 자세가
   `map_origin_in_world` 보정값이다 (#96, 민지 하네스: 지도 정렬 오차와 위치추정 오차를 나눠 본다).
3. 구조 정확도 (#81 판정): 정렬한 뒤 물체마다 평행이동을 맞춰 몇 cm 밀렸는지 잰다 (양쪽 면을 같이 보므로
   벽 띠 두께와 무관). 물체별 밀림 중앙값 ≤ 0.05 m, 최대 ≤ 0.15 m, 유령·이중 벽 0칸이면 합격.
4. 어느 물체와도 0.3 m 넘게 떨어진 점유 칸을 유령·이중 벽 후보로 센다 (벽 겹침 확인).
5. 참고값: 테두리 |거리| 중앙값 (벽 띠 두께에 따라 바뀐다), 부호 있는 거리 중앙값.
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


def boundary_normal(px: np.ndarray, py: np.ndarray, b: Box) -> Tuple[np.ndarray, np.ndarray]:
    """점에 가장 가까운 테두리 변의 바깥 법선 (world)."""
    c, s = math.cos(-b.yaw), math.sin(-b.yaw)
    dx0, dy0 = px - b.cx, py - b.cy
    lx, ly = c * dx0 - s * dy0, s * dx0 + c * dy0
    use_x = (np.abs(lx) - b.hx) >= (np.abs(ly) - b.hy)
    nlx, nly = np.where(use_x, np.sign(lx), 0.0), np.where(use_x, 0.0, np.sign(ly))
    cw, sw = math.cos(b.yaw), math.sin(b.yaw)
    return cw * nlx - sw * nly, sw * nlx + cw * nly


def associate(px: np.ndarray, py: np.ndarray, objs: List[Box]):
    """점마다 가장 가까운 물체 번호, 부호 거리(바깥 +), 바깥 법선."""
    sd = np.stack([signed_boundary_distance(px, py, b) for b in objs])
    k = np.abs(sd).argmin(axis=0)
    d = sd[k, np.arange(px.size)]
    nx, ny = np.zeros(px.size), np.zeros(px.size)
    for i, b in enumerate(objs):
        sel = k == i
        if sel.any():
            nx[sel], ny[sel] = boundary_normal(px[sel], py[sel], b)
    return k, d, nx, ny


def apply_rigid(px: np.ndarray, py: np.ndarray, fit: Tuple[float, float, float],
                pivot: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """점을 pivot 기준으로 dθ 돌린 뒤 (dx, dy) 옮긴다."""
    dx, dy, dth = fit
    c, s = math.cos(dth), math.sin(dth)
    rx, ry = px - pivot[0], py - pivot[1]
    return pivot[0] + c * rx - s * ry + dx, pivot[1] + s * rx + c * ry + dy


def fit_rigid(px: np.ndarray, py: np.ndarray, objs: List[Box], pivot: Tuple[float, float],
              iterations: int = 10, huber: float = 0.05) -> Tuple[float, float, float]:
    """점유 칸을 물체 테두리에 맞추는 강체 보정 (dx, dy, dθ), pivot 기준 회전.

    점-변 거리 최소제곱에 Huber 가중을 줘서 한 물체가 크게 어긋나도 끌려가지 않는다. 0.3 m 안의 점만 쓴다.
    """
    fit = np.zeros(3)
    for _ in range(iterations):
        qx, qy = apply_rigid(px, py, tuple(fit), pivot)
        _, d, nx, ny = associate(qx, qy, objs)
        sel = np.abs(d) < GHOST_DISTANCE
        d, nx, ny, qx, qy = d[sel], nx[sel], ny[sel], qx[sel], qy[sel]
        rx, ry = qx - pivot[0], qy - pivot[1]
        a = np.column_stack([nx, ny, nx * -ry + ny * rx])
        w = np.sqrt(np.where(np.abs(d) <= huber, 1.0, huber / np.maximum(np.abs(d), 1e-9)))
        step, *_ = np.linalg.lstsq(a * w[:, None], -d * w, rcond=None)
        fit = fit + step
        if np.abs(step[:2]).max() < 1e-4 and abs(step[2]) < 1e-6:
            break
    return float(fit[0]), float(fit[1]), float(fit[2])


def object_shift(d: np.ndarray, nx: np.ndarray, ny: np.ndarray) -> float:
    """한 물체에 붙은 점들로 그 물체가 몇 m 밀렸는지 (평행이동 크기). 한쪽 면만 보인 방향은 0으로 둔다."""
    a = np.column_stack([nx, ny])
    t = -np.linalg.solve(a.T @ a + 0.01 * len(d) * np.eye(2), a.T @ d)
    return float(np.hypot(t[0], t[1]))


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
    """물체별 밀림(#81 판정), 테두리 오차, 부호 있는 거리, 유령 칸 수. 점은 지도 정렬을 한 뒤의 world 좌표다.

    - 물체별 밀림 (#81 판정): 물체에 붙은 점으로 평행이동을 맞춘 크기. 양쪽 면을 같이 보므로 벽 띠 두께와
      무관하고, 물체 하나가 옮겨진(지도가 접힌) 것을 잡는다.
    - 테두리 오차 (참고): 물체에 붙은 점유 칸의 |거리| 중앙값. 벽이 여러 칸 두께로 찍히면 커진다.
    - 부호 있는 거리 (참고): 부호 있는 거리(바깥 +) 중앙값의 크기. 물체를 통째로 옮기면 양쪽 면이 상쇄돼
      작게 나오므로 밀림 판정에는 쓰지 않는다.
    """
    if len(wx) == 0:
        raise ValueError('점유 칸이 없다')
    nearest, sd, nx, ny = associate(wx, wy, objs)
    dmin = np.abs(sd)
    per, offsets, shifts = {}, [], {}
    for k, b in enumerate(objs):
        sel = (nearest == k) & (dmin < MATCH_DISTANCE)
        if sel.any():
            per[b.name] = (b.kind, float(np.median(dmin[sel])), int(sel.sum()))
            offsets.append(abs(float(np.median(sd[sel]))))
            near = sel & (dmin < GHOST_DISTANCE)
            if near.sum() >= 3:
                shifts[b.name] = object_shift(sd[near], nx[near], ny[near])
        else:
            per[b.name] = (b.kind, float('nan'), 0)
    found = [v[1] for v in per.values() if v[2] > 0]
    ghost = int((dmin > GHOST_DISTANCE).sum())
    shift_median = float(np.median(list(shifts.values())))
    shift_max = float(np.max(list(shifts.values())))
    return {
        'per_object': per,
        'missing': [n for n, v in per.items() if v[2] == 0],
        'shifts': shifts,
        'shift_median': shift_median,
        'shift_max': shift_max,
        'shift_worst': max(shifts, key=shifts.get),
        'passed': shift_median <= MEDIAN_LIMIT and shift_max <= MAX_LIMIT and ghost == 0,
        'median': float(np.median(found)),
        'max': float(np.max(found)),
        'offset_median': float(np.median(offsets)),
        'offset_max': float(np.max(offsets)),
        'ghost_cells': ghost,
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
    ap.add_argument('--region', nargs=4, type=float, metavar=('X0', 'X1', 'Y0', 'Y1'),
                    help='이 world 구역의 점만으로 지도 정렬을 한 번 더 구해 같이 낸다 (하네스 측정 구역 비교용)')
    a = ap.parse_args(argv)

    with open(a.map_yaml) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(a.map_yaml)), meta['image']))
    wx, wy = occupied_world_points(img, meta['resolution'], meta['origin'][:2], tuple(a.start))
    objs = load_objects(a.sdf)
    start = tuple(a.start)
    pivot = (start[0], start[1])
    fit = fit_rigid(wx, wy, objs, pivot)
    r = evaluate(*apply_rigid(wx, wy, fit, pivot), objs)

    print(f'지도 {img.shape[1]}×{img.shape[0]} 칸, 해상도 {meta["resolution"]} m, 점유 칸 {r["occupied_cells"]}')
    print(f'지도 정렬 (지도 전체를 월드에 강체 정합, 시작점 기준 회전): dx {fit[0] * 100:+.1f} cm, '
          f'dy {fit[1] * 100:+.1f} cm, dθ {math.degrees(fit[2]):+.3f}°')
    print(f'  → map_origin_in_world 보정값: [{start[0] + fit[0]:.4f}, {start[1] + fit[1]:.4f}, '
          f'{start[2] + fit[2]:.5f}] (시작 자세 [{start[0]}, {start[1]}, {start[2]}] 대신)')
    if a.region:
        x0, x1, y0, y1 = a.region
        sel = (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
        rf = fit_rigid(wx[sel], wy[sel], objs, pivot)
        print(f'  구역 x {x0}~{x1}, y {y0}~{y1}만 쓴 정렬: dx {rf[0] * 100:+.1f} cm, dy {rf[1] * 100:+.1f} cm, '
              f'dθ {math.degrees(rf[2]):+.3f}° (점 {int(sel.sum())}개)')
    print(f'구조 정확도 (정렬 후 물체별 밀림): 중앙값 {r["shift_median"] * 100:.1f} cm, '
          f'최대 {r["shift_max"] * 100:.1f} cm ({r["shift_worst"]}), 유령·이중 벽 {r["ghost_cells"]}칸 '
          f'→ #81 기준(≤{MEDIAN_LIMIT * 100:.0f} cm, ≤{MAX_LIMIT * 100:.0f} cm, 0칸) '
          f'{"합격" if r["passed"] else "불합격"}')
    print('참고값 (정렬 후):')
    groups = [('랙·기둥', ('warehouse_rack', 'warehouse_pillar')),
              ('도크·충전기', ('loading_dock', 'charging_station')), ('외벽', ('wall',))]
    for label, kinds in groups:
        g = {n: v for n, v in r['per_object'].items() if v[0] in kinds}
        meds = {n: v[1] for n, v in g.items() if v[2] > 0}
        if not meds:
            print(f'  {label}: 지도에 없음')
            continue
        worst = max(meds, key=meds.get)
        print(f'  {label}: {len(meds)}/{len(g)}개, 테두리 |거리| 물체별 중앙값의 중앙값 '
              f'{np.median(list(meds.values())) * 100:.1f} cm, 최대 {meds[worst] * 100:.1f} cm ({worst})')
    print(f'  테두리 |거리| 전체: 중앙값 {r["median"] * 100:.1f} cm, 최대 {r["max"] * 100:.1f} cm (벽 띠 두께에 따라 바뀜)')
    print(f'  부호 있는 거리: 중앙값 {r["offset_median"] * 100:.1f} cm, 최대 {r["offset_max"] * 100:.1f} cm')
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
