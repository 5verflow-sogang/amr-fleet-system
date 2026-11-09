"""장면 시작 작업 경로가 같은 통로에 겹치는지: A* 경로(시작 자리 → 픽업 → 하역)끼리 가까운 구간 길이.

    python3 route_overlap.py --ws /ros2_ws/src --astar ./astar_cli --scene tools/harness/tasks/demo_s2b.yaml \\
        --robots src/bringup/amr_bringup/config/robots_restaurant.yaml [--steps 1]

시작 작업은 낸 순서대로 amr_05, 04, 03, 02, 01이 맡는다고 본다(장면 파일 머리말). 두 경로의 점이 near_m(0.8 m) 안에 있는
구간 길이를 재고, 그중 진행 방향이 반대(마주 옴)인 길이를 따로 낸다. 서빙 로봇 둘이 나란히 지나려면 약 1.07 m가 필요해
마주 오는 구간이 좁은 통로에 있으면 서로 막힌다 (11/8 2막 녹화: 1·2·3호가 윗줄 t03~t04 사이에 몰려 멈춤).
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detour import costmap, load_map, plan  # noqa: E402

ORDER = ['amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01']


def route(astar, cost, res, origin, pts, tmp):
    out = []
    for a, b in zip(pts, pts[1:]):
        st, path = plan(astar, cost, res, origin, a, b, tmp)
        if st != 0:
            return None
        out += path
    return np.array(out)


def overlap(p, q, near_m):
    """p 경로 점 중 q 경로와 near_m 안인 점들의 길이와, 그중 방향이 반대인 길이."""
    if p is None or q is None:
        return None, None
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dp = np.gradient(p, axis=0)
    dq = np.gradient(q, axis=0)
    close = opposite = 0.0
    for i in range(len(p) - 1):
        d = np.linalg.norm(q - p[i], axis=1)
        j = int(np.argmin(d))
        if d[j] <= near_m:
            close += seg[i]
            if np.dot(dp[i], dq[j]) < 0:
                opposite += seg[i]
    return round(close, 1), round(opposite, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ws', required=True)
    ap.add_argument('--astar', required=True)
    ap.add_argument('--scene', required=True)
    ap.add_argument('--robots', required=True)
    ap.add_argument('--steps', type=int, default=1, help='앞에서 몇 단계를 시작 작업으로 볼지 (첫 단계만이면 1)')
    ap.add_argument('--near-m', type=float, default=0.8)
    a = ap.parse_args()
    ws = Path(a.ws)
    img, res, origin = load_map(ws / 'localization/amr_mapping/maps/restaurant_gt.yaml')
    cost, _ = costmap(img, res)
    st = {k: tuple(v[:2]) for k, v in yaml.safe_load(
        open(ws / 'fleet/amr_fleet_manager/config/stations_restaurant.yaml'))['stations'].items()}
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(open(a.robots))['robots'].items()}
    steps = yaml.safe_load(open(a.scene, encoding='utf-8'))['steps'][:a.steps]
    tasks = [t for s in steps for t in s['tasks']]
    owners = [r for r in ORDER if r in robots][:len(tasks)]
    routes = {t['id']: route(a.astar, cost, res, origin, [robots[r], st[t['pickup']], st[t['dropoff']]], '/tmp/ro.bin')
              for r, t in zip(owners, tasks)}
    rows = []
    ids = list(routes)
    for i, x in enumerate(ids):
        for y in ids[i + 1:]:
            c, o = overlap(routes[x], routes[y], a.near_m)
            rows.append({'a': x, 'b': y, 'close_m': c, 'head_on_m': o})
    print(json.dumps(sorted(rows, key=lambda r: -(r['head_on_m'] or 0)), indent=1))


if __name__ == '__main__':
    main()
