"""v3 본 실험 정책과 DWA(B1·B2)를 같은 평가 목록끼리 비교한다 (보고 항목, 판정 아님. v3_plan.md 7절).

    uv run python scripts/compare_v3_dwa.py raw/v3_main_lists20-39 raw/v2_dwa

입력: <RL 폴더>/eval_<비교군>_s<시드>_list<k>/episodes.csv (정책 시드 s를 목록 s − 20으로 평가, 제안은 exact·unknown)
      <DWA 폴더>/list<k>_<b1·b2>_<정보 조건>/episodes.csv (민지 평가, research2_dwa_lists20-39_results.zip의 v2_dwa/)
충돌률·도착 시간 정의는 judge_v3_main.py와 같다. 통계 단위는 평가 목록이고, 차이는 같은 목록끼리 낸 뒤 목록 부트스트랩
(4000회, 난수 0) 95% CI를 붙인다. 결과는 <RL 폴더>/dwa_compare.md.
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from judge_v3_main import TRAIN, ci, coll, own_time  # noqa: E402

RL = (('R1', 'r1', 'exact'), ('R2', 'r2', 'exact'), ('제안 exact', 'proposed', 'exact'), ('제안 unknown', 'proposed', 'unknown'))
DWA = (('B1', 'b1', 'exact'), ('B2', 'b2', 'exact'), ('B2 unknown', 'b2', 'unknown'))
PAIRS = (('제안 exact', 'B2'), ('제안 exact', 'B1'), ('R2', 'B2'), ('R1', 'B1'), ('제안 exact', 'R2'), ('B2', 'B1'), ('제안 unknown', 'B2 unknown'))


def rate(rows, masses, outcome, scenario=None):
    sel = [r for r in rows if r['payload_kg'] in masses and (scenario is None or r['scenario'] == scenario)]
    return sum(r['outcome'] == outcome for r in sel) / len(sel) if sel else np.nan


def fmt(c, scale=1.0, unit=' s'):
    m, lo, hi, n = c
    if n == 0:
        return '– (짝지을 성공 판 없음)'
    return f'{m*scale:+.2f}{unit} ({lo*scale:+.2f} ~ {hi*scale:+.2f}, 목록 {n})'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rl', type=Path)
    ap.add_argument('dwa', type=Path)
    args = ap.parse_args()
    runs = {}
    for p in args.rl.glob('eval_*_s*_list*'):
        m = re.fullmatch(r'eval_(\w+?)_s(\d+)_list(\d+)', p.name)
        if m and (p / 'episodes.csv').exists():
            runs[(m[1], int(m[3]))] = p / 'episodes.csv'
    lists = sorted({k for _, k in runs})
    lists = [k for k in lists if all((v, k) in runs for _, v, _ in RL)
             and all((args.dwa / f'list{k}_{b}_{c}' / 'episodes.csv').exists() for _, b, c in DWA)]
    read = lambda f, cond: [r for r in csv.DictReader(open(f)) if r['mass_obs'] == cond]  # noqa: E731
    G = {name: {k: read(runs[(v, k)], c) for k in lists} for name, v, c in RL}
    G.update({name: {k: read(args.dwa / f'list{k}_{b}_{c}' / 'episodes.csv', c) for k in lists} for name, b, c in DWA})
    rng = np.random.default_rng(0)

    metrics = {
        '학습 질량 충돌': (lambda r: coll(r, TRAIN), 100, '%'), '0 kg 충돌': (lambda r: coll(r, ('0.0',)), 100, '%'),
        '10 kg 충돌': (lambda r: coll(r, ('10.0',)), 100, '%'), '25 kg 충돌': (lambda r: coll(r, ('25.0',)), 100, '%'),
        '25 kg 성공': (lambda r: rate(r, ('25.0',), 'success'), 100, '%'), '전체 성공': (lambda r: rate(r, TRAIN, 'success'), 100, '%'),
        'S2 성공': (lambda r: rate(r, TRAIN, 'success', 'S2'), 100, '%'), '지나침': (lambda r: rate(r, TRAIN, 'overshoot'), 100, '%'),
        '시간 초과': (lambda r: rate(r, TRAIN, 'timeout'), 100, '%'),
        '0 kg 시간': (lambda r: own_time(r, '0.0'), 1, ' s'), '25 kg 시간': (lambda r: own_time(r, '25.0'), 1, ' s'),
    }
    val = {g: {n: np.array([f(G[g][k]) for k in lists], dtype=float) for n, (f, _, _) in metrics.items()} for g in G}

    def cell(x, scale, unit):
        x = x[~np.isnan(x)]
        return f'{x.mean()*scale:.1f}{unit}' if len(x) else '–'
    L = ['# v3 정책 vs DWA (같은 평가 목록)', '',
         f'RL `{args.rl}`, DWA `{args.dwa}`, 목록 {lists[0]}~{lists[-1]} ({len(lists)}개). 학습 질량, 보고 항목(판정 아님, `v3_plan.md` 7절).', '',
         '## 비교군별 (목록 평균)', '', '| 비교군 | ' + ' | '.join(metrics) + ' |', '| --- |' + ' --- |' * len(metrics)]
    for g in G:
        L.append(f'| {g} | ' + ' | '.join(cell(val[g][n], s, u) for n, (_, s, u) in metrics.items()) + ' |')
    L += ['', '- 충돌률 분모는 돌발 장애물이 나타난 판, 시간은 성공한 판 평균 (성공 판이 없으면 –)', '',
          '## 짝지은 차이 (같은 목록, 95% CI)', '']
    for a, b in PAIRS:
        parts = [f'{n} {fmt(ci(val[a][n] - val[b][n], rng), s, "%p" if u == "%" else u)}' for n, (_, s, u) in metrics.items()
                 if n in ('학습 질량 충돌', '25 kg 충돌', '0 kg 시간', '25 kg 시간', '전체 성공')]
        L.append(f'- **{a} − {b}**: ' + ', '.join(parts))
    same = sum(all(x['outcome'] == y['outcome'] and x['time_s'] == y['time_s'] for x, y in zip(G['B1'][k], G['B2 unknown'][k]))
               and len(G['B1'][k]) == len(G['B2 unknown'][k]) for k in lists)
    L += ['', f'- 연결 확인: B2 unknown이 B1과 결과·시간이 모두 같은 목록 {same}/{len(lists)}개 (설계상 같아야 한다)']
    text = '\n'.join(L) + '\n'
    (args.rl / 'dwa_compare.md').write_text(text)
    print(text)


if __name__ == '__main__':
    main()
