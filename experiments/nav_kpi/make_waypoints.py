#!/usr/bin/env python3
"""Nav KPI 시나리오의 웨이포인트(출발·목표 50쌍)를 지도에서 고정해 waypoints.yaml로 쓴다. ROS 불필요.

    python3 experiments/nav_kpi/make_waypoints.py            # 결과를 바꿀 때만 다시 돌린다 (seed 고정)

- 투어: 스폰 자세 world (0, 1.5)에서 출발해 웨이포인트 50개를 차례로 간다. 앞 목표가 다음 쌍의 출발이라 50쌍이다.
- 점: 지도 v1 0.025 m에서 장애물·미탐색 칸까지 CLEARANCE_M 넘게 떨어지고, 스폰 자리와 이어진 빈 칸 중에서 고른다.
- 좁은 통로 ①(rack_b3–b4, 폭 0.6 m)을 지나야 가장 가까운 쌍을 양방향으로 하나씩 넣는다(FORCED).
  ②(rack_lane_divider–b4)는 바깥 우회와 길이가 비슷해 쌍으로 강제하지 못한다. 실제로 지났는지는 결과에서 GT로 본다.
- 쌍 사이 직선거리는 MIN_HOP_M~MAX_HOP_M로 섞는다 (짧은 이동·통로를 바꾸는 긴 이동).
좌표는 world다. 측정 노드가 map_origin_in_world로 map으로 바꾼다.
"""

import math
import random
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
# 점을 뽑은 지도 v1. 점은 world 좌표로 저장하므로 기본 지도가 v2가 돼도 다시 뽑지 않는다 (#249)
MAP_YAML = REPO / 'src/localization/amr_mapping/maps/warehouse_v1_0025.yaml'
MAP_ORIGIN_IN_WORLD = (-0.0562, 1.5317, 0.00009)   # 지도 정렬값 (amr_mapping README, #157)
SPAWN = (0.0, 1.5, 0.0)
SEED = 1
N_WAYPOINTS = 50
CLEARANCE_M = 0.7          # 로봇 외접원 반지름 0.36 m + 여유 (목표 자리에서 바로 회전할 수 있게)
REACH_CLEARANCE_M = 0.25   # 이 거리 이상 빈 칸으로 스폰과 이어져야 갈 수 있는 곳으로 본다 (로봇 반폭 0.2 m)
MIN_HOP_M, MAX_HOP_M = 3.0, 15.0   # 합이 약 450 m (0.5 m/s면 실행 하나 15분 남짓)
# 좁은 통로 ①을 지나는 쌍: B–C 통로 쪽 입구 앞 ↔ A–B 통로 쪽 (둘 다 world, 가장 가까운 후보 칸으로 옮긴다)
NARROW_1 = [(-4.7, -4.2), (-5.6, 1.6)]
FORCED = {10: NARROW_1[0], 11: NARROW_1[1], 30: NARROW_1[1], 31: NARROW_1[0]}


def load_map(path: Path):
    meta = yaml.safe_load(path.read_text())
    raw = (path.parent / meta['image']).read_bytes()
    # P5 PGM: 헤더 4토큰(매직, 폭, 높이, 최대값) 뒤가 픽셀
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos)
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)[::-1]  # 행 0 = 지도 아래
    return img, float(meta['resolution']), meta['origin']


def map_to_world(x, y, origin=MAP_ORIGIN_IN_WORLD):
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * x - s * y, oy + s * x + c * y


def world_to_map(x, y, origin=MAP_ORIGIN_IN_WORLD):
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = x - ox, y - oy
    return c * dx + s * dy, -s * dx + c * dy


def main():
    img, res, (mx0, my0, _) = load_map(MAP_YAML)
    free = img >= 250                                   # trinary: 빈 칸 254, 장애물 0, 미탐색 205
    clearance = ndimage.distance_transform_edt(free) * res

    def cell(wx, wy):
        x, y = world_to_map(wx, wy)
        return int(round((y - my0) / res)), int(round((x - mx0) / res))

    labels, _ = ndimage.label(clearance >= REACH_CLEARANCE_M)
    reachable = labels == labels[cell(*SPAWN[:2])]
    ok = reachable & (clearance >= CLEARANCE_M)
    rows, cols = np.nonzero(ok)
    cand = [map_to_world(mx0 + c * res, my0 + r * res) for r, c in zip(rows, cols)]
    cand = cand[::7]                                    # 후보를 성기게 (재현성만 필요)

    def nearest_ok(p):
        return min(cand, key=lambda q: (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)

    rng = random.Random(SEED)
    tour = [SPAWN[:2]]
    for i in range(N_WAYPOINTS):
        if i in FORCED:
            p = nearest_ok(FORCED[i])
        else:
            prev = tour[-1]
            while True:
                p = rng.choice(cand)
                if MIN_HOP_M <= math.dist(p, prev) <= MAX_HOP_M and (i + 1 not in FORCED or MIN_HOP_M < math.dist(p, FORCED[i + 1]) <= MAX_HOP_M):
                    break
        tour.append((round(float(p[0]), 2), round(float(p[1]), 2)))
    out = {
        'note': 'make_waypoints.py가 만든다. world 좌표 [x, y]. 첫 점은 스폰 자세이고 쌍 i = (점 i, 점 i+1)',
        'map': MAP_YAML.name, 'seed': SEED, 'clearance_m': CLEARANCE_M,
        'narrow_pairs': sorted(i for i in FORCED if i - 1 in FORCED),
        'waypoints': [list(p) for p in tour],
    }
    (HERE / 'waypoints.yaml').write_text(yaml.safe_dump(out, allow_unicode=True, sort_keys=False, width=200))
    hops = [math.dist(a, b) for a, b in zip(tour, tour[1:])]
    print(f'{len(tour) - 1}쌍, 직선거리 {min(hops):.1f}~{max(hops):.1f} m (합 {sum(hops):.0f} m), 좁은 통로 쌍 {out["narrow_pairs"]}')
    for i in sorted(FORCED):
        print(f'  점 {i + 1}: {tour[i + 1]} (원래 {FORCED[i]}, 여유 {clearance[cell(*tour[i + 1])]:.2f} m)')


if __name__ == '__main__':
    main()
