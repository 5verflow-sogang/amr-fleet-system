"""1-2 우회로 확인: restaurant_gt + 식당 프로필 전역 costmap(정적 + inflation) + keepout(west_aisle 229)에 amr_planning_core A*.

    python3 detour.py --map <restaurant_gt.yaml> --zones <congestion_zones_restaurant.yaml>
        --stations <stations_restaurant.yaml> --astar ./astar_cli --out <dir>
        [--guests <dynamic_obstacles yaml>] [--start X Y] [--tag name]
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# nav2_params.yaml global_costmap + nav2_profile restaurant (dev):
# footprint 0.565 x 0.537, padding 0.01, inflation 0.55 / 3.0
INSCRIBED = 0.537 / 2 + 0.01
CIRCUMSCRIBED = math.hypot(0.565 / 2 + 0.01, 0.537 / 2 + 0.01)
INFLATION_RADIUS, COST_SCALING = 0.55, 3.0
KEEPOUT_COST = 229            # 혼잡도 1.0 → 마스크 90 → KeepoutFilter 비용 (90 / 100 * 254)
GUEST_R = 0.25
START = (-10.5, -5.6)
DETOUR_X = (-7.6, -6.95)      # x -7.275 세로 통로


def load_map(path):
    m = yaml.safe_load(open(path))
    img = np.array(Image.open(Path(path).parent / m['image']))[::-1]   # 행 0 = 아래쪽 (y 작은 쪽)
    return img, m['resolution'], m['origin'][:2]


def costmap(img, res, guests=()):
    lethal = img < 128
    ox_mask = np.zeros_like(lethal)
    if guests:
        h, w = img.shape
        ys, xs = np.mgrid[0:h, 0:w]
        for gx, gy, (ox, oy) in guests:
            ox_mask |= np.hypot((xs + 0.5) * res + ox - gx, (ys + 0.5) * res + oy - gy) < GUEST_R
    lethal = lethal | ox_mask
    d = ndimage.distance_transform_edt(~lethal) * res
    cost = np.zeros(img.shape, np.uint8)
    band = (d > INSCRIBED) & (d <= INFLATION_RADIUS)
    cost[band] = (252 * np.exp(-COST_SCALING * (d[band] - INSCRIBED))).astype(np.uint8)
    cost[(d <= INSCRIBED)] = 253
    cost[lethal] = 254
    cost[(img > 128) & (img < 250) & ~lethal] = 255    # 미관측(205)
    return cost, d


def to_cell(x, y, res, origin):
    return int((x - origin[0]) / res), int((y - origin[1]) / res)


def to_xy(c, r, res, origin):
    return origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res


def plan(astar, cost, res, origin, start, goal, tmp):
    h, w = cost.shape
    with open(tmp, 'wb') as f:
        f.write(np.array([w, h], np.uint32).tobytes() + np.array([res], np.float64).tobytes() + cost.tobytes())
    s, g = to_cell(*start, res, origin), to_cell(*goal, res, origin)
    out = subprocess.run([astar, tmp, *map(str, s), *map(str, g)], capture_output=True, text=True).stdout.split('\n')
    status = int(out[0].split()[1])
    path = [tuple(map(int, line.split())) for line in out[1:] if line.strip()]
    return status, [to_xy(c, r, res, origin) for c, r in path]


def summarize(path, dist, res, origin, west):
    if not path:
        return {'found': False}
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    x0, x1, y0, y1 = west
    inside = sum(math.dist(a, b) for a, b in zip(path, path[1:]) if x0 <= a[0] <= x1 and y0 <= a[1] <= y1)
    detour = [p for p in path if DETOUR_X[0] <= p[0] <= DETOUR_X[1] and -3.0 <= p[1] <= 2.5]
    clear = [dist[int((p[1] - origin[1]) / res), int((p[0] - origin[0]) / res)] for p in path]
    # 화분 칸막이(y -5.25) 넘는 곳
    cross = next((p for a, p in zip(path, path[1:]) if (a[1] + 5.25) * (p[1] + 5.25) <= 0), None)
    i = path.index(cross) if cross else None
    gap_clear = min(clear[max(0, i - 6):i + 7]) if i is not None else None
    return {'found': True, 'length_m': round(length, 2), 'in_west_aisle_m': round(inside, 2),
            'uses_detour_x7275': len(detour) > 10,
            'partition_cross_xy': [round(v, 2) for v in cross] if cross else None,
            'partition_gap_clearance_m': round(float(gap_clear), 3) if gap_clear is not None else None,
            'min_clearance_m': round(float(min(clear)), 3)}


def guests_from(path):
    out = []
    for o in yaml.safe_load(open(path))['obstacles']:
        if o['id'].startswith('peak_'):
            sp = o['spawn']
            out.append((o['id'], (sp['x'], sp['y']), [(w['x'], w['y']) for w in o.get('waypoints', [])], o['pattern']))
    return out


def draw(cost, res, origin, west, paths, guests, start, goal, out_png, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h, w = cost.shape
    ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    shade = np.where(cost >= 254, 0, np.where(cost == 253, 90, np.where(cost == 255, 200, 255 - cost * 0.6)))
    fig, ax = plt.subplots(figsize=(8, 10), facecolor='white')
    ax.imshow(shade, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=255)
    x0, x1, y0, y1 = west
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=True, color='#eb6834', alpha=0.18, lw=0))
    ax.text(x0 + 0.05, y1 - 0.35, 'west_aisle\nkeepout 229', color='#a8401a', fontsize=8)
    styles = {'a': ('#d1242f', '-', '(a) keepout on'), 'b': ('#2a78d6', '--', '(b) no congestion'),
              'a2': ('#1baf7a', '-', '(a) keepout + peak guests at spawn')}
    for key, pts in paths.items():
        if pts:
            c, ls, label = styles[key]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, ls=ls, lw=2, label=label)
    for gid, sp, wps, pattern in guests:
        if pattern == 'straight' and len(wps) >= 2:
            ax.plot([p[0] for p in wps], [p[1] for p in wps], color='#7a3db8', lw=1, alpha=0.6)
        elif pattern == 'random':
            xs, ys = [p[0] for p in wps], [p[1] for p in wps]
            ax.add_patch(plt.Rectangle((min(xs), min(ys)), max(xs) - min(xs), max(ys) - min(ys), fill=False,
                                       ec='#7a3db8', lw=1, ls=':'))
        ax.add_patch(plt.Circle(sp, GUEST_R, color='#7a3db8'))
        ax.text(sp[0] + 0.3, sp[1], gid, color='#7a3db8', fontsize=7)
    ax.plot(*start, 'o', color='black')
    ax.text(start[0] + 0.2, start[1] - 0.3, f'start {start}', fontsize=8)
    ax.plot(*goal, '*', color='black', ms=12)
    ax.text(goal[0] + 0.2, goal[1] + 0.15, 'buffet_2', fontsize=8)
    ax.axhline(-5.25, color='#1f8a3a', lw=0.8, ls=':')
    ax.text(-6.4, -5.15, 'planter partition y -5.25', color='#1f8a3a', fontsize=7)
    ax.set_xlim(-12.3, -4.0)
    ax.set_ylim(-7.6, 4.9)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=10, loc='left')
    ax.legend(loc='lower right', fontsize=8, frameon=True)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--zones', required=True)
    ap.add_argument('--stations', required=True)
    ap.add_argument('--astar', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--guests', help='손님 파일: peak_* 시작 자리를 장애물로 넣은 (a)도 계산한다')
    ap.add_argument('--tag', default='')
    ap.add_argument('--start', nargs=2, type=float, default=list(START))
    a = ap.parse_args()
    start = tuple(a.start)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    img, res, origin = load_map(a.map)
    west = next(z['area'] for z in yaml.safe_load(open(a.zones))['zones'] if z['id'] == 'west_aisle')
    goal = tuple(yaml.safe_load(open(a.stations))['stations']['buffet_2'][:2])
    base, dist = costmap(img, res)
    h, w = base.shape
    ys, xs = np.mgrid[0:h, 0:w]
    cx, cy = origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res
    in_west = (cx >= west[0]) & (cx <= west[1]) & (cy >= west[2]) & (cy <= west[3])
    keep = base.copy()
    keep[in_west] = np.maximum(keep[in_west], KEEPOUT_COST)
    result, paths = {}, {}
    for key, grid in (('a', keep), ('b', base)):
        st, pts = plan(a.astar, grid, res, origin, start, goal, str(out / f'grid_{key}.bin'))
        paths[key] = pts
        result[key] = {'status': st, **summarize(pts, dist, res, origin, west)}
    guests = []
    if a.guests:
        guests = guests_from(a.guests)
        gcost, gdist = costmap(img, res, [(sp[0], sp[1], origin) for _, sp, _, _ in guests])
        gcost[in_west] = np.maximum(gcost[in_west], KEEPOUT_COST)
        st, pts = plan(a.astar, gcost, res, origin, start, goal, str(out / 'grid_a2.bin'))
        paths['a2'] = pts
        result['a2_guests_at_spawn'] = {'status': st, **summarize(pts, gdist, res, origin, west)}
    result['inscribed_m'] = round(INSCRIBED, 4)
    result['circumscribed_m'] = round(CIRCUMSCRIBED, 4)
    title = f'1-2 A* (amr_planning_core): start {start} -> buffet_2 {goal}{a.tag}'
    draw(base if not a.guests else keep, res, origin, west, paths, guests, start, goal,
         str(out / f'detour{a.tag.replace(" ", "_")}.png'), title)
    print(json.dumps(result, indent=1))
    (out / f'detour{a.tag.replace(" ", "_")}.json').write_text(json.dumps(result, indent=1))


if __name__ == '__main__':
    main()
