"""CTE 측정 결과를 구간별(직선·곡선)로 요약한다 (ROS 불필요).

    python3 analyze.py <결과 폴더>

- cte.csv: path_tracker_node 기록. 위치는 추종기가 쓴 odom, 구간은 노드의 곡률 기준 분류
- gt.csv: path_publisher.py가 기록한 GT 위치를 경로에 다시 투영해 CTE를 계산. 구간은 경로 정의 기준
- odom − GT: cte.csv의 위치(odom)와 같은 시각 GT의 거리. 추종기가 아니라 odom이 틀린 만큼이다
GT는 움직이는 동안(선속도 > 0.01 m/s)만 센다. 결과는 <결과 폴더>/cte_summary.json에도 쓴다.
"""

import bisect
import csv
import json
import math
import os
import sys

from path_def import build_path, leg_of, project

LEGS = ('straight_1', 'curve', 'straight_2')


def stats(vals):
    if not vals:
        return None
    return {
        'n': len(vals),
        'mean': sum(vals) / len(vals),
        'rmse': math.sqrt(sum(v * v for v in vals) / len(vals)),
        'max': max(vals),
    }


def group(rows, keys):
    return {k: stats([v for g, v in rows if g == k]) for k in keys if any(g == k for g, _ in rows)}


def main(argv):
    run_dir = argv[1]
    path = build_path()
    with open(os.path.join(run_dir, 'cte.csv')) as f:
        node = list(csv.DictReader(f))
    with open(os.path.join(run_dir, 'gt.csv')) as f:
        gt = list(csv.DictReader(f))

    node_rows = [(r['segment'], float(r['cte'])) for r in node]
    node_leg_rows = []
    gt_rows = []
    gt_leg_rows = []
    for r in gt:
        if float(r['v']) <= 0.01:
            continue
        x, y = float(r['x']), float(r['y'])
        _, _, cte, seg = project(path, x, y)
        gt_rows.append((seg, abs(cte)))
        gt_leg_rows.append((leg_of(seg, y), abs(cte)))

    gt_times = [float(r['timestamp']) for r in gt]
    drift_rows = []
    for r in node:
        x, y = float(r['actual_x']), float(r['actual_y'])
        _, _, _, seg = project(path, x, y)
        node_leg_rows.append((leg_of(seg, y), float(r['cte'])))
        i = bisect.bisect_left(gt_times, float(r['timestamp']))
        if i < len(gt):
            g = gt[i]
            drift_rows.append((leg_of(seg, y), math.hypot(x - float(g['x']), y - float(g['y']))))

    result = {
        'by_segment': {
            'node_odom': group(node_rows, ('straight', 'curve')),
            'ground_truth': group(gt_rows, ('straight', 'curve')),
        },
        'by_leg': {
            'node_odom': group(node_leg_rows, LEGS),
            'ground_truth': group(gt_leg_rows, LEGS),
            'odom_minus_gt': group(drift_rows, LEGS),
        },
    }
    with open(os.path.join(run_dir, 'cte_summary.json'), 'w') as f:
        json.dump(result, f, indent=2)

    def table(title, part, keys):
        print(f'\n{title}\n')
        print('| 기준 | 구간 | 샘플 | 평균 [cm] | RMSE [cm] | 최대 [cm] |')
        print('| --- | --- | --- | --- | --- | --- |')
        for name, label in keys:
            for seg, s in part.get(name, {}).items():
                print(f"| {label} | {seg} | {s['n']} | {s['mean'] * 100:.2f} | {s['rmse'] * 100:.2f} | "
                      f"{s['max'] * 100:.2f} |")

    table('구간별 CTE', result['by_segment'],
          (('node_odom', 'cte.csv (odom)'), ('ground_truth', 'GT')))
    table('경로 순서별 (직선 1 → 곡선 → 직선 2)', result['by_leg'],
          (('node_odom', 'CTE, odom'), ('ground_truth', 'CTE, GT'), ('odom_minus_gt', 'odom-GT 거리')))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
