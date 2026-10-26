#!/usr/bin/env python3
"""8-3절 CTE 직선·곡선 평균을 위치 KPI 측정 구역(ZONE) 안의 점만으로 다시 계산한다 (10/16 결정, 8-5절 B안).

    python3 recompute_zone_cte.py <원자료 폴더>   # cte.csv·gt_track.csv가 있는 폴더

구역은 src/localization/amr_mapping/tools/draw_kpi_zone.py의 ZONE을 그대로 가져다 쓴다(복붙이 아니라 임포트) -
값이 바뀌면 이 스크립트도 따라 바뀐다. cte.csv에는 위치가 없어, 같은 pair의 gt_track.csv에서 시각이 가장
가까운 점의 (x, y)로 구역 안/밖을 판정한다(gt_track 5 Hz vs cte 10 Hz라 최악 ~0.05 s 어긋남 - 순항
0.5 m/s 기준 최대 ~2.5 cm 위치 오차, docs/algorithms/navigation.md 8-2절). pair_success(도착한 쌍)만
센다 - 기존 전체 평균(8-3절)도 같은 필터.
"""
import argparse
import csv
import importlib.util
from bisect import bisect_left
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_DRAW_KPI_ZONE = REPO_ROOT / 'src' / 'localization' / 'amr_mapping' / 'tools' / 'draw_kpi_zone.py'


def _load_zone() -> tuple[float, float, float, float]:
    spec = importlib.util.spec_from_file_location('draw_kpi_zone', _DRAW_KPI_ZONE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ZONE  # map x0, y0, x1, y1


def in_zone(x: float, y: float, zone: tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = zone
    return x0 <= x <= x1 and y0 <= y <= y1


def load_track(path: Path) -> dict[int, list[tuple[float, float, float]]]:
    track: dict[int, list[tuple[float, float, float]]] = {}
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            pair = int(row['pair'])
            track.setdefault(pair, []).append((float(row['time']), float(row['x']), float(row['y'])))
    for pts in track.values():
        pts.sort(key=lambda p: p[0])
    return track


def nearest_xy(times: list[float], pts: list[tuple[float, float, float]], t: float) -> tuple[float, float]:
    i = bisect_left(times, t)
    candidates = [p for p in (pts[i - 1] if i > 0 else None, pts[i] if i < len(pts) else None) if p is not None]
    best = min(candidates, key=lambda p: abs(p[0] - t))
    return best[1], best[2]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('data_dir', type=Path, help='cte.csv·gt_track.csv가 있는 폴더')
    args = ap.parse_args()

    zone = _load_zone()
    track = load_track(args.data_dir / 'gt_track.csv')
    times_by_pair = {pair: [p[0] for p in pts] for pair, pts in track.items()}

    totals = {'straight': [0.0, 0], 'curve': [0.0, 0]}
    zone_totals = {'straight': [0.0, 0], 'curve': [0.0, 0]}
    zone_pairs: set[int] = set()
    unmatched = 0

    with (args.data_dir / 'cte.csv').open(newline='') as f:
        for row in csv.DictReader(f):
            if row['pair_success'] not in ('1', 'True', 'true'):
                continue
            seg = row['segment']
            if seg not in totals:
                continue
            cte = float(row['cte_m'])
            totals[seg][0] += cte
            totals[seg][1] += 1

            pair = int(row['pair'])
            pts = track.get(pair)
            if not pts:
                unmatched += 1
                continue
            x, y = nearest_xy(times_by_pair[pair], pts, float(row['time']))
            if in_zone(x, y, zone):
                zone_totals[seg][0] += cte
                zone_totals[seg][1] += 1
                zone_pairs.add(pair)

    print(f'zone (map): x [{zone[0]}, {zone[2]}] m, y [{zone[1]}, {zone[3]}] m')
    print(f'unmatched pairs (gt_track에 없음): {unmatched}')
    print(f'구역에 걸치는 쌍 수: {len(zone_pairs)}')
    print()
    print(f'{"segment":10s} {"all_mean_cm":>12s} {"all_n":>8s} {"zone_mean_cm":>13s} {"zone_n":>8s}')
    for seg in ('straight', 'curve'):
        a_sum, a_n = totals[seg]
        z_sum, z_n = zone_totals[seg]
        a_mean = a_sum / a_n * 100 if a_n else float('nan')
        z_mean = z_sum / z_n * 100 if z_n else float('nan')
        print(f'{seg:10s} {a_mean:12.2f} {a_n:8d} {z_mean:13.2f} {z_n:8d}')


if __name__ == '__main__':
    main()
