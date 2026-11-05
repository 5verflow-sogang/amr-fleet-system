#!/usr/bin/env python3
"""식당 시연 2 리허설: aging 켬/끔 비교 표·CSV·그림.

    python3 experiments/fleet_collect_aging/analyze.py experiments/fleet_collect_aging/raw \\
        --out experiments/fleet_collect_aging

입력: <raw>/<라벨>/<실행 폴더>/ (하네스 결과 그대로, 라벨은 aging_off·aging_on). 출력:
- tasks.csv: 작업 1건마다 (조건, seed, 종류, 우선순위, 요청 시각 상대값, 할당 대기, 할당 여부, 완료 여부)
- summary.csv: 실행 1개마다 지표 (collect_aging.summarize)
- results.md: 조건별 표 (대기 평균은 작업 전체 평균, 최대는 실행들 중 최대, 미할당·처리량은 실행 평균)
- wait_cdf.png: 조건별 할당 대기 누적 분포 (일반 작업 / 회수 호출)
"""

import argparse
import csv
import math
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from collect_aging import load_run, summarize  # noqa: E402

CONDITIONS = [('aging_off', 'aging 끔 (0.0)'), ('aging_on', 'aging 켬 (1.667)')]
# dataviz 기본 범주 색 1·2 (검사 통과), 선 모양도 다르게 해 색만으로 구분하지 않는다
STYLE = {'aging_off': ('#2a78d6', '-'), 'aging_on': ('#eb6834', '--')}


def fmt(v, digits=1):
    return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{digits}f}'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('raw')
    ap.add_argument('--out', required=True)
    ap.add_argument('--font-file', help='한글 글꼴 파일 (.ttf). 컨테이너에 한글 글꼴이 없으면 준다 (예: Windows malgun.ttf)')
    a = ap.parse_args()
    raw, out = Path(a.raw), Path(a.out)

    runs = {}   # 라벨 -> [(seed, rows, cutoff)]
    for label, _ in CONDITIONS:
        for run in sorted(p for p in (raw / label).iterdir() if (p / 'meta.json').exists()):
            rows, cutoff, seed = load_run(run)
            runs.setdefault(label, []).append((seed, rows, cutoff))

    with (out / 'tasks.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['condition', 'seed', 'task_id', 'kind', 'priority', 'request_s', 'wait_s', 'assigned', 'completed'])
        for label, items in runs.items():
            for seed, rows, _ in items:
                t0 = min(r.request for r in rows)
                for r in rows:
                    w.writerow([label, seed, r.task_id, r.kind, r.priority, f'{r.request - t0:.3f}',
                                fmt(r.wait, 3) if r.wait is not None else '', int(r.assign is not None),
                                int(r.completed)])

    summaries = {label: [(seed, summarize(rows, cutoff)) for seed, rows, cutoff in items] for label, items in runs.items()}
    keys = list(next(iter(summaries.values()))[0][1])
    with (out / 'summary.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['condition', 'seed', *keys])
        for label, items in summaries.items():
            for seed, s in items:
                w.writerow([label, seed, *[fmt(s[k], 3) if isinstance(s[k], float) else s[k] for k in keys]])

    lines = ['| 조건 | 실행 | 회수 호출 대기 평균 / 최대 (s) | 일반 작업 대기 평균 / 최대 (s) | 끝까지 미할당 (회수 + 일반, 실행 평균) | '
             '처리량 (건/분) |', '|---|---|---|---|---|---|']
    for label, name in CONDITIONS:
        if label not in runs:
            continue
        all_rows = [r for _, rows, _ in runs[label] for r in rows]
        cw = [r.wait for r in all_rows if r.kind == 'collect' and r.wait is not None]
        nw = [r.wait for r in all_rows if r.kind == 'normal' and r.wait is not None]
        ss = [s for _, s in summaries[label]]
        lines.append(
            f'| {name} | {len(ss)} | {fmt(st.mean(cw))} / {fmt(max(cw))} | {fmt(st.mean(nw))} / {fmt(max(nw))} | '
            f'{st.mean(s["unassigned"] for s in ss):.1f} ({st.mean(s["collect_unassigned"] for s in ss):.1f} + '
            f'{st.mean(s["normal_unassigned"] for s in ss):.1f}) | {st.mean(s["throughput_per_min"] for s in ss):.2f} |')
    (out / 'results.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))

    plot(runs, out / 'wait_cdf.png', a.font_file)


def plot(runs, path, font_file=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if font_file:
        matplotlib.font_manager.fontManager.addfont(font_file)
        plt.rcParams['font.family'] = matplotlib.font_manager.FontProperties(fname=font_file).get_name()
    for font in ('NanumGothic', 'Noto Sans CJK KR', 'Malgun Gothic') if not font_file else ():
        if any(font in f.name for f in matplotlib.font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = font
            break
    plt.rcParams['axes.unicode_minus'] = False

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True, facecolor='#fcfcfb')
    for ax, (kind, title) in zip(axes, [('normal', '일반 작업 (priority 100)'), ('collect', '회수 호출 (priority 200)')]):
        ax.set_facecolor('#fcfcfb')
        for label, name in CONDITIONS:
            if label not in runs:
                continue
            waits = sorted(r.wait for _, rows, _ in runs[label] for r in rows if r.kind == kind and r.wait is not None)
            if not waits:
                continue
            ys = [(i + 1) / len(waits) for i in range(len(waits))]
            color, ls = STYLE[label]
            # 최대 대기는 범례에 적는다 (선 위에 쓰면 겹친다)
            ax.step(waits, ys, where='post', color=color, linestyle=ls, linewidth=2,
                    label=f'{name} · 최대 {waits[-1]:.0f} s')
        ax.set_title(title, fontsize=11, color='#0b0b0b', loc='left')
        ax.set_xlabel('할당까지 대기 (s)', color='#52514e')
        ax.grid(True, color='#e6e5e1', linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color('#c3c2b7')
        ax.tick_params(colors='#52514e')
        ax.set_xlim(left=0)
    axes[0].set_ylabel('누적 비율', color='#52514e')
    axes[0].set_ylim(0, 1.02)
    for ax in axes:
        ax.legend(loc='lower right', frameon=False, fontsize=9)
    fig.suptitle('할당 대기 시간 분포: aging 끔 / 켬 (가짜 로봇 3대, 할당받은 작업만)', fontsize=12, color='#0b0b0b', x=0.01,
                 ha='left')
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())


if __name__ == '__main__':
    main()
