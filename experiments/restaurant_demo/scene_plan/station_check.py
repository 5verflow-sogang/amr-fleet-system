"""정차 자리마다 서빙 로봇이 설 수 있는지: restaurant_gt + 식당 프로필 costmap(detour.costmap)으로 잰다.

    python3 station_check.py --ws /ros2_ws/src [--names b01 b02 b03]

자리마다
- clear_m: 로봇 중심에서 가장 가까운 점유 칸까지 (inflation 기준 거리)
- center_cost: 중심 칸 비용 (253 이상이면 전역 계획이 그 자리로 경로를 못 낸다)
- footprint_lethal: 정차 자세(자리 yaw) 풋프린트가 점유(254) 칸에 닿는가 (DWA는 254만 충돌로 본다)
- turn_ok: 그 자리에서 제자리 회전할 수 있는가 (외접 반경 + rotate_in_place_clearance 0.05 m 안에 점유 칸이 없다)
- goal_reachable_m: 목표 0.25 m(xy_goal_tolerance) 안에서 정차 yaw 풋프린트가 점유 칸에 안 닿는 자세가 있으면 목표까지 가장
  가까운 거리. 없으면 None (그 자리에서 도착 판정을 못 받는다)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detour import costmap, load_map  # noqa: E402

HALF_L, HALF_W, PAD = 0.565 / 2, 0.537 / 2, 0.01      # 서빙 로봇 footprint, footprint_padding
ROTATE_CLEARANCE = 0.05                                # DWA rotate_in_place_clearance (nav2_params.yaml)
XY_TOL = 0.25                                          # xy_goal_tolerance


def footprint_hits(lethal, res, origin, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    step = res * 0.5
    for lx in np.arange(-HALF_L, HALF_L + 1e-9, step):
        for ly in np.arange(-HALF_W, HALF_W + 1e-9, step):
            wx, wy = x + lx * c - ly * s, y + lx * s + ly * c
            col, row = int((wx - origin[0]) / res), int((wy - origin[1]) / res)
            if lethal[row, col]:
                return True
    return False


def check(name, pose, img, res, origin, cost, dist):
    x, y, yaw = pose
    col, row = int((x - origin[0]) / res), int((y - origin[1]) / res)
    lethal = img < 128
    circ = math.hypot(HALF_L, HALF_W)
    best = None
    for dx in np.arange(-XY_TOL, XY_TOL + 1e-9, 0.05):
        for dy in np.arange(-XY_TOL, XY_TOL + 1e-9, 0.05):
            if math.hypot(dx, dy) <= XY_TOL and not footprint_hits(lethal, res, origin, x + dx, y + dy, yaw):
                d = math.hypot(dx, dy)
                best = d if best is None else min(best, d)
    return {'station': name, 'xy': [x, y], 'yaw_deg': round(math.degrees(yaw)),
            'clear_m': round(float(dist[row, col]), 3),
            'center_cost': int(cost[row, col]), 'footprint_lethal': footprint_hits(lethal, res, origin, x, y, yaw),
            'turn_ok': bool(dist[row, col] > circ + PAD + ROTATE_CLEARANCE),
            'goal_reachable_m': None if best is None else round(best, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ws', required=True)
    ap.add_argument('--names', nargs='*')
    a = ap.parse_args()
    ws = Path(a.ws)
    img, res, origin = load_map(ws / 'localization/amr_mapping/maps/restaurant_gt.yaml')
    cost, dist = costmap(img, res)
    st = yaml.safe_load(open(ws / 'fleet/amr_fleet_manager/config/stations_restaurant.yaml'))['stations']
    names = a.names or sorted(st)
    print(json.dumps([check(n, tuple(st[n]), img, res, origin, cost, dist) for n in names], indent=1))


if __name__ == '__main__':
    main()
