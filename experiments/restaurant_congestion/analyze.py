"""식당 시연 1-1 켬/끔 결과 → 실행별 summary.csv + 조건별 results.md.

    python3 experiments/restaurant_congestion/analyze.py raw/ --out experiments/restaurant_congestion

raw/<조건 라벨>/<실행 폴더>/ 구조다(조건 라벨 예: off, on). 실행 폴더는 하네스 결과 폴더(task_outcomes.csv 등)다.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics as st
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from congestion_metrics import run_metrics  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
STATIONS = REPO / 'src/fleet/amr_fleet_manager/config/stations_restaurant.yaml'
ZONES = REPO / 'src/fleet/amr_fleet_manager/config/congestion_zones_restaurant.yaml'
# 피크 손님이 몰리는 구역 (amr_gazebo dynamic_obstacles_restaurant_peak.yaml)
CROWD = ('west_aisle', 'buffet_line')

COLUMNS = [
    ('wait_mean', '할당 대기 평균 (s)'),
    ('task_time_mean', '완료 시간 평균 (s)'),
    ('task_time_p95', '완료 시간 p95 (s)'),
    ('stopped_s', '작업 중 정지 합 (s)'),
    ('in_crowd_s', '혼잡 구역 체류 합 (s)'),
    ('choice_other_rate', '가까운 로봇 아닌 로봇 비율'),
]


def load_inputs(stations_path: Path, zones_path: Path, crowd_ids):
    stations = {k: (float(v[0]), float(v[1]))
                for k, v in yaml.safe_load(stations_path.read_text(encoding='utf-8'))['stations'].items()}
    zones = {z['id']: tuple(float(a) for a in z['area'])
             for z in yaml.safe_load(zones_path.read_text(encoding='utf-8'))['zones']}
    return stations, [zones[i] for i in crowd_ids]


def fmt(values):
    vals = [v for v in values if not math.isnan(v)]
    if not vals:
        return '—'
    if len(vals) == 1:
        return f'{vals[0]:.2f}'
    return f'{st.mean(vals):.2f} ± {st.stdev(vals):.2f}'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('raw', type=Path, help='raw/<조건>/<실행>/ 의 raw 폴더')
    ap.add_argument('--out', type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument('--stations', type=Path, default=STATIONS)
    ap.add_argument('--zones', type=Path, default=ZONES)
    ap.add_argument('--crowd', default=','.join(CROWD), help='혼잡 구역 id (쉼표)')
    args = ap.parse_args(argv)

    stations, crowd = load_inputs(args.stations, args.zones, args.crowd.split(','))
    rows = []
    for cond in sorted(p for p in args.raw.iterdir() if p.is_dir()):
        for run in sorted(p for p in cond.iterdir() if p.is_dir() and (p / 'task_outcomes.csv').exists()):
            rows.append({'condition': cond.name, 'run': run.name, **run_metrics(run, stations, crowd)})
    if not rows:
        print(f'결과 폴더가 없다: {args.raw}/<조건>/<실행>/task_outcomes.csv', file=sys.stderr)
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / 'summary.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    lines = ['| 조건 | 실행 수 | 완료 / 작업 | ' + ' | '.join(label for _, label in COLUMNS) + ' |',
             '| --- ' * (3 + len(COLUMNS)) + '|']
    for cond in dict.fromkeys(r['condition'] for r in rows):
        mine = [r for r in rows if r['condition'] == cond]
        done = f"{sum(r['completed'] for r in mine)} / {sum(r['tasks'] for r in mine)}"
        lines.append(f'| {cond} | {len(mine)} | {done} | '
                     + ' | '.join(fmt([r[key] for r in mine]) for key, _ in COLUMNS) + ' |')
    eligible = {c: sum(r['choice_eligible'] for r in rows if r['condition'] == c) for c in dict.fromkeys(
        r['condition'] for r in rows)}
    lines += ['', '값은 실행 평균 ± 표준편차. 시간은 시뮬레이션 시간이다.',
              '"가까운 로봇 아닌 로봇 비율"의 분모(놀고 있는 로봇이 2대 이상이던 할당 수): '
              + ', '.join(f'{c} {n}건' for c, n in eligible.items())]
    (args.out / 'results.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
