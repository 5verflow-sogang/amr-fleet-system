"""식당 장면 설계 계산: restaurant_gt + 식당 프로필 costmap에 amr_planning_core A* (astar_cli) 경로 길이.

    python3 plan_tools.py start   --ws /ros2_ws/src --astar ./astar_cli --out out/   # 1-2 출발 자리 후보 (README)
    python3 plan_tools.py trips   --ws /ros2_ws/src --astar ./astar_cli --out out/   # 장면 2 시작 작업 길이
"""
import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detour import (KEEPOUT_COST, costmap, load_map, plan, summarize)  # noqa: E402


class World:
    def __init__(self, ws, astar, out):
        self.ws, self.astar, self.out = Path(ws), astar, Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.img, self.res, self.origin = load_map(self.ws / 'localization/amr_mapping/maps/restaurant_gt.yaml')
        self.base, self.dist = costmap(self.img, self.res)
        zones = yaml.safe_load(open(self.ws / 'fleet/amr_fleet_manager/config/congestion_zones_restaurant.yaml'))
        self.west = next(z['area'] for z in zones['zones'] if z['id'] == 'west_aisle')
        h, w = self.base.shape
        ys, xs = np.mgrid[0:h, 0:w]
        cx, cy = self.origin[0] + (xs + 0.5) * self.res, self.origin[1] + (ys + 0.5) * self.res
        x0, x1, y0, y1 = self.west
        inw = (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)
        self.keep = self.base.copy()
        self.keep[inw] = np.maximum(self.keep[inw], KEEPOUT_COST)
        st = yaml.safe_load(open(self.ws / 'fleet/amr_fleet_manager/config/stations_restaurant.yaml'))['stations']
        self.stations = {k: tuple(v[:2]) for k, v in st.items()}
        self._cache = {}

    def cost_at(self, xy):
        c = int((xy[0] - self.origin[0]) / self.res)
        r = int((xy[1] - self.origin[1]) / self.res)
        return int(self.base[r, c]), float(self.dist[r, c])

    def path(self, a, b, keepout=False):
        key = (round(a[0], 3), round(a[1], 3), round(b[0], 3), round(b[1], 3), keepout)
        if key not in self._cache:
            grid = self.keep if keepout else self.base
            st, pts = plan(self.astar, grid, self.res, self.origin, a, b, str(self.out / 'g.bin'))
            self._cache[key] = (st, pts)
        return self._cache[key]

    def length(self, a, b, keepout=False):
        st, pts = self.path(a, b, keepout)
        if st != 0 or not pts:
            return None
        return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))


def guest_clearance(xy, files):
    """xy에서 손님 경로(직선·곡선 = 꺾은선, 무작위 = 경유점 상자, 스폰 점)까지 가장 가까운 거리와 그 손님."""
    best = (1e9, None)
    for f in files:
        for o in yaml.safe_load(open(f))['obstacles']:
            pts = [(w['x'], w['y']) for w in o.get('waypoints', [])]
            sp = o.get('spawn')
            cands = [math.dist(xy, (sp['x'], sp['y']))] if sp else []
            if o['pattern'] == 'random' and pts:
                xs, ys = [p[0] for p in pts], [p[1] for p in pts]
                dx = max(min(xs) - xy[0], 0, xy[0] - max(xs))
                dy = max(min(ys) - xy[1], 0, xy[1] - max(ys))
                cands.append(math.hypot(dx, dy))
            else:
                for p, q in zip(pts, pts[1:]):
                    cands.append(seg_dist(xy, p, q))
            d = min(cands) if cands else 1e9
            if d < best[0]:
                best = (d, f'{Path(f).stem}:{o["id"]}')
    return best


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def map_free(w, xy, half=0.4):
    c = int((xy[0] - w.origin[0]) / w.res)
    r = int((xy[1] - w.origin[1]) / w.res)
    k = int(half / w.res)
    return bool((w.img[r - k:r + k + 1, c - k:c + k + 1] > 250).all())


def cmd_start(w, a):
    goal = w.stations['buffet_2']
    gz = w.ws / 'simulation/amr_gazebo/config'
    guest_files = [gz / 'dynamic_obstacles_restaurant_peak.yaml', gz / 'dynamic_obstacles_restaurant_peak_scene12.yaml',
                   gz / 'dynamic_obstacles_restaurant_staff.yaml']
    rows = []
    for x, y in itertools.product(np.round(np.arange(-11.2, -8.69, 0.1), 2), np.round(np.arange(-6.5, -5.39, 0.1), 2)):
        xy = (float(x), float(y))
        cost, clear = w.cost_at(xy)
        if cost >= 253 or not map_free(w, xy):
            continue
        gd, gid = guest_clearance(xy, guest_files)
        sta, pa = w.path(xy, goal, keepout=False)
        stb, pb = w.path(xy, goal, keepout=True)
        if sta != 0 or stb != 0:
            continue
        sa, sb = summarize(pa, w.dist, w.res, w.origin, w.west), summarize(pb, w.dist, w.res, w.origin, w.west)
        rows.append({'xy': xy, 'guest_m': round(gd, 2), 'nearest_guest': gid, 'clear_m': round(clear, 2),
                     'free_len_m': sa['length_m'], 'free_west_m': sa['in_west_aisle_m'],
                     'keep_len_m': sb['length_m'], 'keep_west_m': sb['in_west_aisle_m'],
                     'keep_detour': sb['uses_detour_x7275'], 'to_buffet2_m': round(math.dist(xy, goal), 2)})
    ok = [r for r in rows if r['free_west_m'] > 5 and r['free_len_m'] < r['keep_len_m'] and r['keep_west_m'] == 0
          and r['keep_detour'] and r['guest_m'] >= 1.0]
    ok.sort(key=lambda r: r['keep_len_m'])
    (w.out / 'start_candidates.json').write_text(json.dumps({'all': rows, 'ok': ok}, indent=1))
    print(json.dumps({'n_all': len(rows), 'n_ok': len(ok), 'best': ok[:12]}, indent=1))


def cmd_trips(w, a):
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(
        open(w.ws / 'bringup/amr_bringup/config/robots_restaurant.yaml'))['robots'].items()}
    pickups = ['stall_1', 'stall_2', 'stall_3', 'stall_4', 'buffet_1', 'buffet_2']
    drops = [k for k in w.stations if k[0] in 'tLb' or k.startswith('c0')]
    to_pick = {(r, p): w.length(robots[r], w.stations[p]) for r in robots for p in pickups}
    leg = {(p, d): w.length(w.stations[p], w.stations[d]) for p in pickups for d in drops}
    trips = {}
    for (r, p), l1 in to_pick.items():
        for (p2, d), l2 in leg.items():
            if p2 == p and l1 is not None and l2 is not None:
                trips[(r, p, d)] = round(l1 + l2, 1)
    out = {f'{r}|{p}|{d}': v for (r, p, d), v in trips.items()}
    (w.out / 'trips.json').write_text(json.dumps(out, indent=1))
    print(json.dumps({'n': len(out)}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['start', 'trips'])
    ap.add_argument('--ws', required=True)
    ap.add_argument('--astar', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    w = World(a.ws, a.astar, a.out)
    {'start': cmd_start, 'trips': cmd_trips}[a.cmd](w, a)


if __name__ == '__main__':
    main()
