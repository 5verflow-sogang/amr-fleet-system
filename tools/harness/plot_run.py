#!/usr/bin/env python3
"""실행 폴더 하나의 그림을 plots/*.png로 남긴다. ROS 불필요 (matplotlib). 시나리오 postprocess가 부른다.

    python3 tools/harness/plot_run.py <실행 폴더> --map <지도 yaml>

폴더에 있는 결과를 보고 그릴 것을 고른다.
- 위치추정 (loc_basic, eval_ekf·eval_amcl/localization.csv): loc_traj.png 지도 위 GT·EKF·AMCL 궤적,
  loc_error.png 시간에 따른 오차 (구간 색, KPI 선)
- Nav KPI (nav_kpi/): nav_map.png 지도 위 계획 경로·GT 궤적·웨이포인트(실패 쌍 빨강), nav_time.png 예측·실제 주행시간
- nav_compare (nav_series.csv): nav_compare.png 지도 위 로봇 궤적·장애물 (계획 경로·CTE 그림은 이진이 이 틀에 더한다)
좌표는 모두 map이다. 그림이 실패해도 실행 결과에는 영향이 없게 늘 0으로 끝난다 (실패는 출력만).
이미지에 한글 글꼴이 없어 그림 글자는 영어로 쓴다.
"""

import argparse
import csv
import json
import sys
import traceback
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

# 원 명세 위치 오차 KPI [m] (summary.py LOCALIZATION_KPI와 같은 값)
LOC_KPI = {'stationary': 0.03, 'straight': 0.05, 'rotation': 0.08}
SEGMENT_COLOR = {'stationary': 'tab:gray', 'straight': 'tab:blue', 'rotation': 'tab:orange'}


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding='utf-8') as f:
        return list(csv.DictReader(f))


def floats(rows, key):
    return np.array([float(r[key]) if r.get(key) not in (None, '') else np.nan for r in rows])


def load_map(yaml_path: Path):
    """(점유 격자 이미지, extent) - imshow(origin='lower', extent=...)로 map 좌표에 그린다."""
    meta = yaml.safe_load(yaml_path.read_text())
    raw = (yaml_path.parent / meta['image']).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:   # P5 PGM 헤더: 매직, 폭, 높이, 최대값
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
    img = np.frombuffer(raw[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)[::-1]
    res, (x0, y0, _) = float(meta['resolution']), meta['origin']
    return img, (x0, x0 + w * res, y0, y0 + h * res)


def draw_map(ax, map_yaml, xs, ys, margin=2.0):
    """지도를 그리고 궤적이 들어가게 자른다."""
    if map_yaml is not None:
        img, extent = load_map(map_yaml)
        ax.imshow(img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=255, interpolation='nearest')
    xs, ys = np.concatenate([x[np.isfinite(x)] for x in xs]), np.concatenate([y[np.isfinite(y)] for y in ys])
    if len(xs):
        ax.set_xlim(xs.min() - margin, xs.max() + margin)
        ax.set_ylim(ys.min() - margin, ys.max() + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map)')
    ax.set_ylabel('y [m] (map)')


def plot_localization(run: Path, map_yaml, out: Path) -> list[Path]:
    est = {name: read_csv(run / name / 'localization.csv') for name in ('eval_ekf', 'eval_amcl')
           if (run / name / 'localization.csv').exists()}
    if not est:
        return []
    ref = next(iter(est.values()))
    gx, gy = floats(ref, 'gt_x'), floats(ref, 'gt_y')
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_map(ax, map_yaml, [gx] + [floats(r, 'est_x') for r in est.values()],
             [gy] + [floats(r, 'est_y') for r in est.values()], margin=1.0)
    ax.plot(gx, gy, 'k-', lw=2, label='GT')
    for (name, rows), color in zip(est.items(), ('tab:red', 'tab:green')):
        ax.plot(floats(rows, 'est_x'), floats(rows, 'est_y'), '-', color=color, lw=1, label=name.replace('eval_', '').upper())
    ax.legend()
    ax.set_title(f'{run.name}: trajectory')
    paths = [out / 'loc_traj.png']
    fig.savefig(paths[0], dpi=120, bbox_inches='tight')
    plt.close(fig)

    fig, axes = plt.subplots(len(est), 1, figsize=(10, 3.2 * len(est)), sharex=True, squeeze=False)
    for ax, (name, rows) in zip(axes[:, 0], est.items()):
        t, e = floats(rows, 'timestamp'), floats(rows, 'error') * 100
        t = t - t[0] if len(t) else t
        seg = np.array([r['segment'] for r in rows])
        for s, color in SEGMENT_COLOR.items():
            m = seg == s
            ax.scatter(t[m], e[m], s=2, color=color, label=s)
            if s in LOC_KPI:
                ax.axhline(LOC_KPI[s] * 100, color=color, ls='--', lw=0.8)
        ax.set_ylabel(f'{name.replace("eval_", "").upper()} error [cm]')
        ax.legend(loc='upper right', markerscale=4, fontsize=8)
    axes[-1, 0].set_xlabel('time [s]')
    axes[0, 0].set_title(f'{run.name}: position error (dashed = KPI)')
    paths.append(out / 'loc_error.png')
    fig.savefig(paths[1], dpi=120, bbox_inches='tight')
    plt.close(fig)
    return paths


def plot_nav_kpi(run: Path, map_yaml, out: Path) -> list[Path]:
    d = run / 'nav_kpi'
    if not (d / 'nav_pairs.csv').exists():
        return []
    pairs = read_csv(d / 'nav_pairs.csv')
    paths_rows = read_csv(d / 'paths.csv') if (d / 'paths.csv').exists() else []
    track = read_csv(d / 'gt_track.csv') if (d / 'gt_track.csv').exists() else []
    tx, ty = floats(track, 'x'), floats(track, 'y')
    px, py = floats(paths_rows, 'x'), floats(paths_rows, 'y')
    fig, ax = plt.subplots(figsize=(12, 8))
    draw_map(ax, map_yaml, [tx, px], [ty, py], margin=1.0)
    by_pair = {}
    for r in paths_rows:
        by_pair.setdefault(r['pair'], []).append((float(r['x']), float(r['y'])))
    for i, pts in enumerate(by_pair.values()):
        a = np.array(pts)
        ax.plot(a[:, 0], a[:, 1], '-', color='tab:blue', lw=0.8, alpha=0.6, label='plan' if i == 0 else None)
    if len(tx):
        ax.plot(tx, ty, '-', color='black', lw=0.8, label='GT')
    ok = [p for p in pairs if p['nav_success'] == '1']
    bad = [p for p in pairs if p['nav_success'] != '1']
    # 목표는 world로 적혀 있다. 시작 자세(첫 쌍 출발)와 그 지점의 GT(map)를 맞춰 옮긴다
    shift = (0.0, 0.0)
    if track and pairs:
        shift = (float(track[0]['x']) - float(pairs[0]['start_x']), float(track[0]['y']) - float(pairs[0]['start_y']))
    for group, color, mark, label in ((ok, 'tab:green', 'o', 'goal reached'), (bad, 'tab:red', 'x', 'goal failed')):
        if group:
            ax.scatter([float(p['goal_x']) + shift[0] for p in group], [float(p['goal_y']) + shift[1] for p in group],
                       color=color, marker=mark, s=40, zorder=5, label=label)
    for p in bad:
        ax.annotate(p['pair'], (float(p['goal_x']) + shift[0], float(p['goal_y']) + shift[1]), color='tab:red', fontsize=8)
    ax.legend(loc='upper right')
    summary = json.loads((d / 'nav_kpi.json').read_text()) if (d / 'nav_kpi.json').exists() else {}
    ax.set_title(f'{run.name}: {len(ok)}/{len(pairs)} reached, verdict {summary.get("verdict", {})}', fontsize=8)
    paths = [out / 'nav_map.png']
    fig.savefig(paths[0], dpi=120, bbox_inches='tight')
    plt.close(fig)

    arrived = [p for p in pairs if p['actual_s']]
    if arrived:
        fig, ax = plt.subplots(figsize=(6, 6))
        pred, act = floats(arrived, 'predicted_s'), floats(arrived, 'actual_s')
        rec = np.array([int(p.get('recoveries') or 0) for p in arrived])
        ax.scatter(pred[rec == 0], act[rec == 0], s=15, label='no recovery')
        ax.scatter(pred[rec > 0], act[rec > 0], s=15, color='tab:red', label='with recovery')
        lim = max(pred.max(), act.max()) * 1.05
        xs = np.array([0, lim])
        ax.plot(xs, xs, 'k-', lw=0.8)
        ax.fill_between(xs, xs / 1.15, xs / 0.85, color='gray', alpha=0.2, label='|err| <= 15%')
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.set_xlabel('predicted [s]')
        ax.set_ylabel('actual [s]')
        ax.legend()
        ax.set_title(f'{run.name}: travel time')
        paths.append(out / 'nav_time.png')
        fig.savefig(paths[-1], dpi=120, bbox_inches='tight')
        plt.close(fig)
    return paths


def plot_nav_compare(run: Path, map_yaml, out: Path) -> list[Path]:
    path = run / 'nav_series.csv'
    if not path.exists():
        return []
    rows = read_csv(path)
    rx, ry, ox, oy = (floats(rows, k) for k in ('robot_x', 'robot_y', 'obstacle_x', 'obstacle_y'))
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_map(ax, map_yaml, [rx, ox], [ry, oy], margin=1.5)
    ax.plot(rx, ry, 'k-', lw=1.2, label='robot (GT)')
    if np.isfinite(ox).any():
        ax.plot(ox, oy, '-', color='tab:red', lw=0.8, alpha=0.7, label='obstacle')
    ax.legend()
    ax.set_title(f'{run.name}: trajectory')
    fig.savefig(out / 'nav_compare.png', dpi=120, bbox_inches='tight')
    plt.close(fig)
    return [out / 'nav_compare.png']


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='실행 폴더 그림 (plots/*.png)')
    ap.add_argument('run', help='results/<run_id>')
    ap.add_argument('--map', help='지도 yaml (없으면 지도 없이 궤적만)')
    args = ap.parse_args(argv)
    run = Path(args.run)
    out = run / 'plots'
    out.mkdir(parents=True, exist_ok=True)
    map_yaml = Path(args.map) if args.map and Path(args.map).exists() else None
    made = []
    for fn in (plot_localization, plot_nav_kpi, plot_nav_compare):
        try:
            made += fn(run, map_yaml, out)
        except Exception:  # noqa: BLE001 - 그림 실패가 실행을 실패로 만들지 않게 한다
            print(f'{fn.__name__} 실패:', file=sys.stderr)
            traceback.print_exc()
    print('그림:', ', '.join(str(p) for p in made) or '없음')
    return 0


if __name__ == '__main__':
    sys.exit(main())
