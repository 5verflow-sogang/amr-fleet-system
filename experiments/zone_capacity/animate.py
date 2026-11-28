#!/usr/bin/env python3
"""zone_pass 통과 한 번을 위에서 본 애니메이션 (교착 확인용). bag_to_csv.py가 쓴 <run_dir>/video/*.csv를 읽는다.

    python3 experiments/zone_capacity/animate.py <run_dir> [--pair N] [--speed 4] --out deadlock.mp4

--pair를 안 주면 첫 시간 초과 통과. 왼쪽은 지도 위 로봇(파랑)·손님(주황)·경로, 오른쪽은 로봇·손님 속력과 안전 정지 구간.
손님 둘레 점선은 손님이 서는 거리(guest_keep_away_m 1.2 m)다.
"""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib import animation, font_manager  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402
from PIL import Image  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
MAP = REPO / 'src/localization/amr_mapping/maps/restaurant_gt.yaml'
ROBOT_L, ROBOT_W = 0.565, 0.537          # 서빙 몸체 (obstacle_motion yield_robot_size)
KEEP_AWAY_M = 1.2
BLUE, ORANGE, TEXT, TEXT2, GRID = '#257de0', '#f47230', '#191919', '#666666', '#e6e6e6'

for path in ('/System/Library/Fonts/AppleSDGothicNeo.ttc', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if Path(path).exists():
        font_manager.fontManager.addfont(path)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=path).get_name()
        break
plt.rcParams.update({'axes.unicode_minus': False, 'axes.spines.top': False, 'axes.spines.right': False})


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def pair_windows(run):
    by = defaultdict(list)
    for r in read(run / 'nav_kpi/gt_track.csv'):
        by[int(r['pair'])].append(float(r['time']))
    return {p: (min(ts), max(ts)) for p, ts in by.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--pair', type=int)
    ap.add_argument('--speed', type=float, default=4.0, help='재생 배속 (시뮬레이션 s / 영상 s)')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--title', default='')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    run = Path(a.run)
    pairs = {int(r['pair']): r for r in read(run / 'nav_kpi/nav_pairs.csv')}
    pair = a.pair or next(p for p, r in sorted(pairs.items()) if r['result'] == 'timeout')
    t0, t1 = pair_windows(run)[pair]
    result = pairs[pair]['result']

    rob = np.array([[float(r[k]) for k in ('t', 'x', 'y', 'yaw', 'v')] for r in read(run / 'video/robot.csv')])
    guests = defaultdict(list)
    for r in read(run / 'video/guests.csv'):
        guests[r['id']].append((float(r['t']), float(r['x']), float(r['y'])))
    guests = {g: np.array(v) for g, v in guests.items()}
    saf = np.array([(float(r['t']), int(r['level'])) for r in read(run / 'video/safety.csv')])
    plans = defaultdict(list)
    for r in read(run / 'video/plan.csv'):
        plans[float(r['t'])].append((float(r['x']), float(r['y'])))
    plan_ts = np.array(sorted(plans))

    def at(arr, t):
        i = min(max(np.searchsorted(arr[:, 0], t) - 1, 0), len(arr) - 1)
        return arr[i]

    def gspeed(arr, t):
        a_, b_ = at(arr, t - 0.5), at(arr, t)
        return math.hypot(b_[1] - a_[1], b_[2] - a_[2]) / 0.5

    # 지도와 보이는 범위 (이번 통과에서 로봇·손님이 지난 곳 + 여유)
    meta = yaml.safe_load(MAP.read_text())
    img = Image.open(MAP.parent / meta['image'])
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    extent = [ox, ox + img.width * res, oy, oy + img.height * res]
    sel = rob[(rob[:, 0] >= t0) & (rob[:, 0] <= t1)]
    xs = list(sel[:, 1]) + [p[1] for g in guests.values() for p in g if t0 <= p[0] <= t1]
    ys = list(sel[:, 2]) + [p[2] for g in guests.values() for p in g if t0 <= p[0] <= t1]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half_h = (max(ys) - min(ys)) / 2 + 1.2
    half_w = max((max(xs) - min(xs)) / 2 + 1.2, half_h * 0.45)

    # 시간축 자료 (통과 구간)
    ts = np.arange(t0, t1, 0.2)
    rv = np.array([at(rob, t)[4] for t in ts])
    gv = {g: np.array([gspeed(arr, t) for t in ts]) for g, arr in guests.items()}
    lvl = np.array([at(saf, t)[1] for t in ts]) if len(saf) else np.zeros(len(ts))

    fig = plt.figure(figsize=(10, 6.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.35], height_ratios=[1, 1])
    ax = fig.add_subplot(gs[:, 0])
    ax.imshow(np.asarray(img), cmap='gray', extent=extent, vmin=0, vmax=255, alpha=0.55)
    ax.set_xlim(cx - half_w, cx + half_w)
    ax.set_ylim(cy - half_h, cy + half_h)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    trail, = ax.plot([], [], color=BLUE, lw=1, alpha=0.5)
    plan_line, = ax.plot([], [], color=BLUE, lw=1.2, ls=':', alpha=0.9)
    robot = Polygon(np.zeros((4, 2)), closed=True, fc=BLUE, ec='white', lw=1.5, zorder=5)
    ax.add_patch(robot)
    heading, = ax.plot([], [], color='white', lw=2, zorder=6)
    gpatch, gring = {}, {}
    for g in guests:
        gpatch[g] = Circle((0, 0), 0.22, fc=ORANGE, ec='white', lw=1.5, zorder=5)
        gring[g] = Circle((0, 0), KEEP_AWAY_M, fill=False, ec=ORANGE, lw=1, ls='--', alpha=0.7, zorder=4)
        ax.add_patch(gpatch[g])
        ax.add_patch(gring[g])
    status = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', fontsize=10, color=TEXT,
                     bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=GRID))

    def timeline(axx, ys_, color, label, ylim):
        axx.fill_between(ts - t0, 0, ylim, where=lvl == 1, color='#f0f0f0', step='mid', lw=0, label='안전 노드: 감속')
        axx.fill_between(ts - t0, 0, ylim, where=lvl >= 2, color='#c8c8c8', step='mid', lw=0, label='안전 노드: 정지')
        axx.plot(ts - t0, ys_, color=color, lw=1.5, label=label)
        axx.set_xlim(0, t1 - t0)
        axx.set_ylim(0, ylim)
        axx.grid(axis='y', color=GRID, lw=0.8)
        axx.legend(frameon=False, fontsize=8.5, loc='upper right')
        return axx.axvline(0, color=TEXT, lw=1)

    a1 = fig.add_subplot(gs[0, 1])
    c1 = timeline(a1, rv, BLUE, '로봇 속력', max(0.6, rv.max() * 1.15))
    a1.set_ylabel('m/s')
    a1.set_title('로봇', fontsize=11, loc='left', fontweight='bold')
    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    g0 = next(iter(guests))
    c2 = timeline(a2, gv[g0], ORANGE, '손님 속력', 1.0)
    a2.set_ylabel('m/s')
    a2.set_xlabel('통과 시작부터 (시뮬레이션 s)')
    a2.set_title('손님', fontsize=11, loc='left', fontweight='bold')
    fig.suptitle(a.title or f'통과 {pair}번째 ({"120 s 안에 못 지나감" if result == "timeout" else result})',
                 fontsize=13, fontweight='bold', x=0.02, ha='left')
    fig.text(0.02, 0.01, f'{a.speed:g}배속. 파랑 = 로봇, 점선 = 로봇 경로, 주황 = 손님, 주황 점선 원 = 손님이 서는 거리 {KEEP_AWAY_M} m. '
             '옅은 회색 = 안전 노드 감속, 짙은 회색 = 정지', fontsize=8, color=TEXT2)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))

    frames = np.arange(t0, t1, a.speed / a.fps)

    def draw(t):
        r = at(rob, t)
        c, s_ = math.cos(r[3]), math.sin(r[3])
        corners = [(ROBOT_L / 2, ROBOT_W / 2), (-ROBOT_L / 2, ROBOT_W / 2), (-ROBOT_L / 2, -ROBOT_W / 2), (ROBOT_L / 2, -ROBOT_W / 2)]
        robot.set_xy([(r[1] + c * dx - s_ * dy, r[2] + s_ * dx + c * dy) for dx, dy in corners])
        heading.set_data([r[1], r[1] + c * 0.35], [r[2], r[2] + s_ * 0.35])
        m = (rob[:, 0] >= t0) & (rob[:, 0] <= t)
        trail.set_data(rob[m, 1], rob[m, 2])
        if len(plan_ts):
            i = np.searchsorted(plan_ts, t) - 1
            pts = plans[plan_ts[i]] if i >= 0 else []
            plan_line.set_data([p[0] for p in pts], [p[1] for p in pts])
        lines = []
        level = at(saf, t)[1] if len(saf) else 0
        move = '거의 멈춤' if r[4] < 0.05 else f'{r[4]:.2f} m/s'
        lines.append('로봇: ' + ('안전 정지' if level >= 2 else f'감속 · {move}' if level == 1 else ('주행' if r[4] > 0.05 else '멈춤')))
        for g, arr in guests.items():
            p = at(arr, t)
            gpatch[g].center = gring[g].center = (p[1], p[2])
            near = math.hypot(p[1] - r[1], p[2] - r[2])
            lines.append(f'손님: {"멈춤" if gspeed(arr, t) < 0.05 else "걷는 중"} (로봇과 {near:.1f} m)')
        lines.append(f'{t - t0:5.1f} s')
        status.set_text('\n'.join(lines))
        c1.set_xdata([t - t0])
        c2.set_xdata([t - t0])
        return []

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=False)
    anim.save(str(out), writer=animation.FFMpegWriter(fps=a.fps, bitrate=2400), dpi=120)
    draw(frames[len(frames) // 2])
    fig.savefig(out.with_suffix('.png'), dpi=120)
    print(f'썼다: {out} ({len(frames)} 프레임, 통과 {pair}, {result}, {t1 - t0:.0f} s)')


if __name__ == '__main__':
    main()
