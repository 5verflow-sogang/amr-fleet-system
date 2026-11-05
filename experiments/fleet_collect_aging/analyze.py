#!/usr/bin/env python3
"""식당 시연 2 리허설: 부하 × aging_rate 비교 표·CSV·그림.

    python3 experiments/fleet_collect_aging/analyze.py experiments/fleet_collect_aging/raw \\
        --out experiments/fleet_collect_aging --font-file /tmp/malgun.ttf

입력: <raw>/<라벨>/<실행 폴더>/ (하네스 결과 그대로). 조건(부하, aging_rate)은 라벨이 아니라 각 실행의 meta.json params로 가른다
(collect_aging.run_condition). 몰림 구간은 작업 시나리오(collect_aging.yaml loads)에서 읽는다. 출력:
- tasks.csv: 작업 1건마다 (부하, rate, seed, 종류, 우선순위, 요청 시각 상대값, 할당 대기, 할당 여부, 완료 여부)
- summary.csv: 실행 1개마다 지표 (collect_aging.summarize + 기아 시간)
- results.md: 부하·rate별 표 (대기 평균은 작업 전체 평균, 최대는 실행들 중 최대, 미할당·처리량·기아는 실행 평균, 기아는 최대도)
- wait_cdf.png: 과부하 조건 aging 끔/켬 할당 대기 누적 분포 (첫 측정)
- wait_cdf_loads.png: normal·collect_burst 부하의 rate별 할당 대기 누적 분포
"""

import argparse
import csv
import math
import statistics as st
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / 'src' / 'fleet' / 'amr_fleet_manager'))
from collect_aging import load_run, load_run_info, run_condition, starvation, summarize  # noqa: E402
from amr_fleet_manager.task_gen import select_load  # noqa: E402

CONFIG = REPO / 'src' / 'fleet' / 'amr_fleet_manager' / 'config'
LOADS = [('overload', '과부하 (처리량의 약 1.4배)'), ('normal', '시연 부하 (약 0.7배)'),
         ('collect_burst', '회수 몰림 (0.7배 + 120 s 회수 몰림)')]
# dataviz 기본 범주 색 1·2·3 (검사 통과, 3번은 대비 경고라 범례·표로 보완), 선 모양도 다르게 해 색만으로 구분하지 않는다
RATE_STYLE = [('#2a78d6', '-'), ('#eb6834', '--'), ('#1baf7a', ':')]


def fmt(v, digits=1):
    return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{digits}f}'


def rate_name(rate):
    return 'aging 끔 (0.0)' if rate == 0 else f'aging {rate:g} ({100 / rate:.0f} s에 따라잡음)'


def burst_window(load):
    """작업 시나리오의 몰림 구간 [시작, 끝] (일정 시각, s). 없으면 None."""
    scenario = yaml.safe_load((CONFIG / 'scenarios' / 'collect_aging.yaml').read_text(encoding='utf-8'))
    bursts = select_load(scenario, load).get('bursts') or []
    if not bursts:
        return None
    return (min(float(b['start_s']) for b in bursts),
            max(float(b['start_s']) + float(b['duration_s']) for b in bursts))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('raw')
    ap.add_argument('--out', required=True)
    ap.add_argument('--font-file', help='한글 글꼴 파일 (.ttf). 컨테이너에 한글 글꼴이 없으면 준다 (예: Windows malgun.ttf)')
    a = ap.parse_args()
    raw, out = Path(a.raw), Path(a.out)
    restaurant = yaml.safe_load((CONFIG / 'fleet_manager_restaurant.yaml').read_text(encoding='utf-8'))
    restaurant_rate = float(restaurant['fleet_manager']['ros__parameters']['assignment']['aging_rate'])

    runs = {}   # (부하, rate) -> [(seed, rows, cutoff, 기아 시간)]
    for run in sorted(p.parent for p in raw.glob('*/*/meta.json')):
        rows, cutoff, seed = load_run(run)
        params, offset = load_run_info(run)
        load, rate = run_condition(params, restaurant_rate)
        window = burst_window(load)
        starve = starvation(rows, (offset + window[0], offset + window[1]), cutoff) if window else float('nan')
        runs.setdefault((load, rate), []).append((seed, rows, cutoff, starve))
    conditions = sorted(runs, key=lambda c: ([n for n, _ in LOADS].index(c[0]), c[1]))

    with (out / 'tasks.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['load', 'aging_rate', 'seed', 'task_id', 'kind', 'priority', 'request_s', 'wait_s', 'assigned',
                    'completed'])
        for cond in conditions:
            for seed, rows, _, _ in sorted(runs[cond], key=lambda x: x[0]):
                t0 = min(r.request for r in rows)
                for r in rows:
                    w.writerow([*cond, seed, r.task_id, r.kind, r.priority, f'{r.request - t0:.3f}',
                                fmt(r.wait, 3) if r.wait is not None else '', int(r.assign is not None),
                                int(r.completed)])

    summaries = {cond: [(seed, {**summarize(rows, cutoff), 'starvation_s': starve})
                        for seed, rows, cutoff, starve in sorted(runs[cond], key=lambda x: x[0])]
                 for cond in conditions}
    keys = list(next(iter(summaries.values()))[0][1])
    with (out / 'summary.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['load', 'aging_rate', 'seed', *keys])
        for cond, items in summaries.items():
            for seed, s in items:
                w.writerow([*cond, seed, *[fmt(s[k], 3) if isinstance(s[k], float) else s[k] for k in keys]])

    lines = ['| 부하 | aging_rate | 실행 | 회수 호출 대기 평균 / 최대 (s) | 일반 작업 대기 평균 / 최대 (s) | '
             '끝까지 미할당 (회수 + 일반, 실행 평균) | 처리량 (건/분) | 기아 시간 평균 / 최대 (s) |',
             '|---|---|---|---|---|---|---|---|']
    for cond in conditions:
        load, rate = cond
        all_rows = [r for _, rows, _, _ in runs[cond] for r in rows]
        cw = [r.wait for r in all_rows if r.kind == 'collect' and r.wait is not None]
        nw = [r.wait for r in all_rows if r.kind == 'normal' and r.wait is not None]
        ss = [s for _, s in summaries[cond]]
        starves = [s['starvation_s'] for s in ss if not math.isnan(s['starvation_s'])]
        lines.append(
            f'| {dict(LOADS)[load]} | {rate:g} | {len(ss)} | {fmt(st.mean(cw))} / {fmt(max(cw))} | '
            f'{fmt(st.mean(nw))} / {fmt(max(nw))} | '
            f'{st.mean(s["unassigned"] for s in ss):.1f} ({st.mean(s["collect_unassigned"] for s in ss):.1f} + '
            f'{st.mean(s["normal_unassigned"] for s in ss):.1f}) | {st.mean(s["throughput_per_min"] for s in ss):.2f} | '
            f'{fmt(st.mean(starves)) + " / " + fmt(max(starves)) if starves else "-"} |')
    (out / 'results.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))

    plt = pyplot(a.font_file)
    overload = [c for c in conditions if c[0] == 'overload']
    if overload:
        fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True, facecolor='#fcfcfb')
        draw_row(axes, runs, overload)
        fig.suptitle('할당 대기 시간 분포: 과부하, aging 끔 / 켬 (가짜 로봇 3대, 할당받은 작업만)', fontsize=12,
                     color='#0b0b0b', x=0.01, ha='left')
        fig.tight_layout()
        fig.savefig(out / 'wait_cdf.png', dpi=150, facecolor=fig.get_facecolor())
    others = [load for load, _ in LOADS[1:] if any(c[0] == load for c in conditions)]
    if others:
        fig, axes = plt.subplots(len(others), 2, figsize=(10, 3.6 * len(others)), squeeze=False, facecolor='#fcfcfb')
        for row, load in zip(axes, others):
            draw_row(row, runs, [c for c in conditions if c[0] == load], prefix=f'{dict(LOADS)[load]} · ')
        fig.suptitle('할당 대기 시간 분포: 부하 × aging_rate (가짜 로봇 3대, seed 3개, 할당받은 작업만)', fontsize=12,
                     color='#0b0b0b', x=0.01, ha='left')
        fig.tight_layout()
        fig.savefig(out / 'wait_cdf_loads.png', dpi=150, facecolor=fig.get_facecolor())


def pyplot(font_file):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if font_file:
        matplotlib.font_manager.fontManager.addfont(font_file)
        plt.rcParams['font.family'] = matplotlib.font_manager.FontProperties(fname=font_file).get_name()
    else:
        for font in ('NanumGothic', 'Noto Sans CJK KR', 'Malgun Gothic'):
            if any(font in f.name for f in matplotlib.font_manager.fontManager.ttflist):
                plt.rcParams['font.family'] = font
                break
    plt.rcParams['axes.unicode_minus'] = False
    return plt


def draw_row(axes, runs, conds, prefix=''):
    """한 부하의 일반 작업 / 회수 호출 누적 분포 두 칸. 조건 순서대로 범주 색 1, 2, 3."""
    for ax, (kind, title) in zip(axes, [('normal', '일반 작업 (priority 100)'), ('collect', '회수 호출 (priority 200)')]):
        ax.set_facecolor('#fcfcfb')
        for (load, rate), (color, ls) in zip(conds, RATE_STYLE):
            waits = sorted(r.wait for _, rows, _, _ in runs[(load, rate)] for r in rows
                           if r.kind == kind and r.wait is not None)
            if not waits:
                continue
            ys = [(i + 1) / len(waits) for i in range(len(waits))]
            # 최대 대기는 범례에 적는다 (선 위에 쓰면 겹친다)
            ax.step(waits, ys, where='post', color=color, linestyle=ls, linewidth=2,
                    label=f'{rate_name(rate)} · 최대 {waits[-1]:.0f} s')
        ax.set_title(prefix + title, fontsize=11, color='#0b0b0b', loc='left')
        ax.set_xlabel('할당까지 대기 (s)', color='#52514e')
        ax.set_ylabel('누적 비율', color='#52514e')
        ax.set_ylim(0, 1.02)
        ax.set_xlim(left=0)
        ax.grid(True, color='#e6e5e1', linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color('#c3c2b7')
        ax.tick_params(colors='#52514e')
        ax.legend(loc='lower right', frameon=False, fontsize=8)


if __name__ == '__main__':
    main()
