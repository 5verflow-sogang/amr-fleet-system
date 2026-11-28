#!/usr/bin/env python3
"""zone_pass 실행 결과 → 통로별 통과 시간, BPR 정원, 그래프.

    python3 experiments/zone_capacity/analyze.py <결과 폴더>... --out experiments/zone_capacity

<결과 폴더> 아래(하위 폴더 포함)에서 meta.json의 scenario가 zone_pass인 실행을 모두 찾는다.
쌍(nav_kpi/nav_pairs.csv 한 줄)이 통과 한 번이다. result가 succeeded면 actual_s, 아니면 pass_timeout_s(검열값)로 넣는다.
쓰는 파일: results.md(표), results.json(실행·통과별 원값), pass_time.png(통로별 손님 수 - 통과 시간과 BPR 곡선)
"""

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bpr import bpr, doubling_point, fit, medians  # noqa: E402


def load(roots, zones=None):
    passes = []   # (zone, n, seed, run_id, pair, time_s, censored)
    for root in roots:
        for meta_path in sorted(Path(root).rglob('meta.json')):
            meta = json.loads(meta_path.read_text())
            if meta.get('scenario') != 'zone_pass':
                continue
            p = meta.get('params') or {}
            zone, n = p.get('zone'), int(p.get('guests_n', 0))
            if zones and zone not in zones:
                continue
            limit = float(p.get('pass_timeout_s', 120))
            pairs = meta_path.parent / 'nav_kpi' / 'nav_pairs.csv'
            if not pairs.exists():
                print(f'건너뜀 (nav_pairs.csv 없음): {meta_path.parent}', file=sys.stderr)
                continue
            for row in csv.DictReader(pairs.open()):
                if row.get('result') == 'no_plan':
                    continue   # 계획 실패는 통로 혼잡이 아니라 설정 문제라 뺀다 (results.md에 수를 적는다)
                ok = row.get('result') == 'succeeded' and row.get('actual_s')
                passes.append({'zone': zone, 'n': n, 'seed': meta.get('seed'), 'run': meta.get('run_id'),
                               'pair': int(row['pair']), 'time_s': float(row['actual_s']) if ok else limit,
                               'censored': not ok})
    return passes


def summarize(passes):
    by = defaultdict(lambda: defaultdict(list))
    for p in passes:
        by[p['zone']][p['n']].append(p['time_s'])
    out = {}
    for zone, samples in sorted(by.items()):
        meds = medians(samples)
        cens = {n: sum(1 for p in passes if p['zone'] == zone and p['n'] == n and p['censored']) for n in samples}
        ok = defaultdict(list)
        for p in passes:
            if p['zone'] == zone and not p['censored']:
                ok[p['n']].append(p['time_s'])
        res = {'medians': meds, 'counts': {n: len(v) for n, v in samples.items()}, 'censored': cens,
               'ok_medians': medians(ok),
               'doubling_n': doubling_point(meds)}
        try:
            res['fit'] = fit(meds)
        except ValueError as e:
            res['fit'] = None
            res['fit_error'] = str(e)
        out[zone] = res
    return out


def write_md(summary, path: Path):
    lines = ['# 구역 정원 측정 결과 (zone_pass)', '',
             '통과 시간은 시뮬레이션 s 중앙값. 괄호는 통과 수 / 시간 초과 수. 정원은 BPR 식 T = T0 × (1 + (n/C)^β) 맞춤값과,',
             '중앙값이 2 × T0를 넘는 두 인원 사이를 직선으로 이은 값(직접 읽기)을 함께 적는다.', '']
    ns = sorted({n for s in summary.values() for n in s['medians']})
    lines += ['| 통로 | ' + ' | '.join(f'손님 {n}명' for n in ns) + ' | 정원 C (BPR) | β | 맞춤 오차 | 정원 (직접 읽기) |',
              '| --- |' + ' --- |' * (len(ns) + 4)]
    for zone, s in summary.items():
        cells = []
        for n in ns:
            if n in s['medians']:
                cells.append(f"{s['medians'][n]:.1f} s ({s['counts'][n]}/{s['censored'][n]})")
            else:
                cells.append('-')
        f = s.get('fit')
        fc = f"{f['capacity']:.1f}명" + (' (격자 끝)' if f['at_edge'] else '') if f else '-'
        fb = f"{f['beta']:.2f}" if f else '-'
        fe = f"{f['rmse_rel'] * 100:.0f}%" if f else '-'
        d = s['doubling_n']
        dn = f'{d:.1f}명' if d is not None else f"{max(s['medians'])}명 넘음"
        lines.append(f'| {zone} | ' + ' | '.join(cells) + f' | {fc} | {fb} | {fe} | {dn} |')
    lines += ['', '## 시간 초과(교착)를 뺀 통과만', '',
              '시간 초과가 많으면 위 중앙값은 혼잡보다 교착(로봇 안전 정지와 손님 양보가 서로 기다림)을 잰다. 괄호는 지나간 통과 수.', '',
              '| 통로 | ' + ' | '.join(f'손님 {n}명' for n in ns) + ' |', '| --- |' + ' --- |' * len(ns)]
    for zone, s in summary.items():
        ok = s.get('ok_medians', {})
        t0 = ok.get(min(ns))
        cells = []
        for n in ns:
            v = ok.get(n)
            cells.append('-' if v is None else f"{v:.1f} s (×{v / t0:.1f}, {s['counts'][n] - s['censored'][n]})")
        lines.append(f'| {zone} | ' + ' | '.join(cells) + ' |')
    path.write_text('\n'.join(lines) + '\n')


def plot(summary, passes, path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    # 한글 글꼴: 맥(AppleGothic), 컨테이너(WenQuanYi Zen Hei) 중 있는 것
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in ('AppleGothic', 'Apple SD Gothic Neo', 'WenQuanYi Zen Hei', 'NanumGothic', 'Malgun Gothic'):
        if name in have:
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False
    zones = list(summary)
    fig, axes = plt.subplots(1, len(zones), figsize=(4.2 * len(zones), 3.6), squeeze=False)
    for ax, zone in zip(axes[0], zones):
        s = summary[zone]
        pts = [p for p in passes if p['zone'] == zone]
        ax.scatter([p['n'] for p in pts if not p['censored']], [p['time_s'] for p in pts if not p['censored']],
                   s=12, color='#9e9e9e', label='통과 한 번')
        ax.scatter([p['n'] for p in pts if p['censored']], [p['time_s'] for p in pts if p['censored']],
                   s=18, marker='^', color='#616161', label='시간 초과')
        ns = sorted(s['medians'])
        ax.plot(ns, [s['medians'][n] for n in ns], 'o', color='#000000', label='중앙값')
        f = s.get('fit')
        if f:
            xs = [i * max(ns) / 100 for i in range(101)]
            ax.plot(xs, [bpr(x, f['t0'], f['capacity'], f['beta']) for x in xs], '-', color='#000000',
                    label=f"BPR C={f['capacity']:.1f}, β={f['beta']:.2f}")
            ax.axhline(2 * f['t0'], ls='--', color='#9e9e9e', lw=1)
        ax.set_title(zone)
        ax.set_xlabel('구역 안 손님 수')
        ax.set_ylabel('통과 시간 (s)')
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('roots', nargs='+')
    ap.add_argument('--out', default=str(Path(__file__).resolve().parent))
    ap.add_argument('--zones', nargs='*', help='이 통로만 (예: --zones west_aisle aisle_3)')
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args()
    passes = load(a.roots, a.zones)
    if not passes:
        print('zone_pass 실행을 못 찾음', file=sys.stderr)
        return 1
    summary = summarize(passes)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    write_md(summary, out / 'results.md')
    (out / 'results.json').write_text(json.dumps({'summary': summary, 'passes': passes}, ensure_ascii=False, indent=1))
    if not a.no_plot:
        plot(summary, passes, out / 'pass_time.png')
    print((out / 'results.md').read_text())
    return 0


if __name__ == '__main__':
    sys.exit(main())
