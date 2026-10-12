#!/usr/bin/env python3
"""하네스 nav_compare 결과 폴더들을 컨트롤러 x 시나리오 표로 묶는다 (ROS 불필요).

    python3 experiments/teb_compare/analyze.py tools/harness/results/nav_compare-*_20261012-0[2-4]*

- 실행 폴더마다 meta.json(params.controller·scenario, outcome)과 nav_result.json을 읽는다
- 성공 = Nav2 결과 succeeded이고 충돌 없음(풋프린트-장애물 1 cm 미만이면 충돌, nav_measure.py와 같음). 하네스 단계에서 실패한 실행(기동 실패·timeout, nav_result.json 없음)도
  실패로 센다. _failN(기동 재시도) 폴더는 같은 반복의 앞 시도라 세지 않는다
- 지표는 평균(최소~최대). 도착 시간은 Nav2가 목표에 닿은(succeeded) 실행만, 최소 장애물 거리는 obstacle만
"""
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

# nav_measure.py COLLISION_GAP_M과 같은 값. 그 전에 잰 실행(정확히 0만 충돌로 기록)도 같은 기준으로 다시 판정한다
COLLISION_GAP_M = 0.01

METRICS = [
    ('arrival_s', '도착 시간 [s]', 1.0, '{:.1f}'),
    ('min_obstacle_m', '최소 장애물 거리 [cm]', 100.0, '{:.1f}'),
    ('path_length_m', '경로 길이 [m]', 1.0, '{:.2f}'),
    ('cte_mean_m', 'CTE 평균 [cm]', 100.0, '{:.1f}'),
    ('cte_max_m', 'CTE 최대 [cm]', 100.0, '{:.1f}'),
]


def load_runs(paths):
    runs = []
    for p in paths:
        p = Path(p)
        if not p.is_dir() or re.search(r'_fail\d+$', p.name) or not (p / 'meta.json').exists():
            continue
        meta = json.loads((p / 'meta.json').read_text(encoding='utf-8'))
        if meta.get('scenario') != 'nav_compare':
            continue
        nav_path = p / 'nav_result.json'
        nav = json.loads(nav_path.read_text(encoding='utf-8')) if nav_path.exists() else None
        if nav is not None:
            gap = nav.get('min_obstacle_m')
            nav['collision'] = isinstance(gap, (int, float)) and gap < COLLISION_GAP_M
            nav['success'] = nav.get('result') == 'succeeded' and not nav['collision']
        params = meta.get('params', {})
        runs.append({'run_id': p.name, 'controller': params.get('controller'), 'scenario': params.get('scenario'),
                     'outcome': meta.get('outcome'), 'nav': nav})
    return runs


def table(runs):
    groups = defaultdict(list)
    for r in runs:
        groups[(r['controller'], r['scenario'])].append(r)
    lines = ['| 컨트롤러 | 시나리오 | 성공/전체 | Nav2 도착 | 충돌 | ' + ' | '.join(m[1] for m in METRICS) + ' |',
             '| --- | --- | --- | --- | --- | ' + ' | '.join('---' for _ in METRICS) + ' |']
    for ctrl in ('dwa', 'teb'):
        for scen in ('clear', 'obstacle'):
            g = groups.get((ctrl, scen), [])
            if not g:
                continue
            ok = [r for r in g if r['nav'] and r['nav'].get('success')]
            arrived = [r for r in g if r['nav'] and r['nav'].get('result') == 'succeeded']
            hit = [r for r in g if r['nav'] and r['nav'].get('collision')]
            cells = []
            for key, _, scale, fmt in METRICS:
                # 도착 시간은 Nav2가 목표에 닿은 실행만 (충돌 여부와 따로 본다)
                pool = arrived if key == 'arrival_s' else [r for r in g if r['nav']]
                vals = [r['nav'][key] * scale for r in pool if isinstance(r['nav'].get(key), (int, float))]
                if not vals:
                    cells.append('—')
                    continue
                mean = sum(vals) / len(vals)
                cells.append(f"{fmt.format(mean)} ({fmt.format(min(vals))}~{fmt.format(max(vals))})")
            lines.append(f"| {ctrl.upper()} | {scen} | {len(ok)}/{len(g)} | {len(arrived)}/{len(g)} | "
                         f"{len(hit)}/{len(g)} | " + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def failures(runs):
    out = []
    for r in runs:
        if r['nav'] and r['nav'].get('success'):
            continue
        if r['nav']:
            reason = r['nav']['result'] + (' + 충돌(풋프린트가 장애물에 닿음)' if r['nav'].get('collision') else '')
        else:
            reason = f"하네스 {r['outcome']}"
        out.append(f"- `{r['run_id']}` ({r['controller']}/{r['scenario']}): {reason}")
    return '\n'.join(out)


def main(argv):
    runs = load_runs(argv[1:])
    print(table(runs))
    f = failures(runs)
    if f:
        print('\n실패한 실행\n')
        print(f)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
