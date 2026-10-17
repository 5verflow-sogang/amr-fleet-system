"""여러 학습 실행의 검증 곡선을 겹쳐 그리고 요약 지표로 비교한다 (README "학습 곡선 비교").

    uv run python scripts/plot_curves.py --out raw/curves/v1 \
        --group F raw/proposed_s100_20261011-182026 raw/proposed_s101_20261011-184621 \
        --group H raw/proposed_s100_20261012-164424 raw/proposed_s101_20261012-164429

그룹(후보)마다 시드 평균 곡선과 최소~최대 범위(음영), 시드별 곡선(가는 선)을 그린다.
--panels common이면 비교군 공통 검증(eval_common/*)을 그린다. 비교군끼리 비교할 때는 이쪽을 쓴다.
eval/*는 비교군마다 학습 질량으로 재서(R1은 0 kg만) 비교군 비교에 쓰지 않는다 (README "학습 곡선 비교").
결과: <out>/curves.png, curves.csv(모든 점), summary.csv와 화면 출력(실행·그룹별 요약 지표).
"""

import argparse
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_nav.curves import common_horizon, curve_summary, mean_band  # noqa: E402

# (TensorBoard 태그, 그래프 제목, 도달 기준값, 높을수록 좋은지, y축 위쪽). 없는 태그는 건너뛴다 (예전 실행에는 timeout이 없다).
# 충돌률·시간 초과율은 낮을수록 좋아서 "가장 좋은 값"이 최저, "나빠진 양"은 최종 − 최저다
PANEL_SETS = {
    # 체크포인트 선택용 검증 (비교군마다 학습 질량)
    'select': [('eval/success_rate', '검증 성공률', 0.8, True, 1.0),
               ('eval/collision_rate', '검증 충돌률', None, False, 1.0),
               ('eval/timeout_rate', '검증 시간 초과율', None, False, 1.0)],
    # 비교군 공통 검증 (학습 질량 고르게, 가림 없음). 속도는 m/s라 요약의 최고·나빠짐은 참고만
    'common': [('eval_common/success_rate', '공통 검증 성공률', 0.8, True, 1.0),
               ('eval_common/collision_rate', '공통 검증 충돌률', None, False, 1.0),
               ('eval_common/collision_25kg', '공통 검증 25 kg 충돌률', None, False, 1.0),
               ('eval_common/speed_0kg', '평균 속도 0 kg [m/s]', None, True, 1.5),
               ('eval_common/speed_25kg', '평균 속도 25 kg [m/s]', None, False, 1.5)],
}
PANELS = PANEL_SETS['select']
COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#8a63d2', '#c9a227', '#d6457a']


def read_run(run: Path, panels=PANELS) -> dict:
    tb = sorted((run / 'tb').glob('*/'))
    if not tb:
        raise FileNotFoundError(f'TensorBoard 기록이 없다: {run}')
    ea = EventAccumulator(str(tb[0]))
    ea.Reload()
    tags = set(ea.Tags()['scalars'])
    return {tag: ([e.step for e in ea.Scalars(tag)], [e.value for e in ea.Scalars(tag)])
            for tag, *_ in panels if tag in tags}


# 한글 글꼴 파일 (Windows·WSL, Linux, macOS 순). 이름만으로는 matplotlib이 못 찾는 경우가 있어 파일을 등록한다
KOREAN_FONTS = ['/mnt/c/Windows/Fonts/malgun.ttf', 'C:/Windows/Fonts/malgun.ttf',
                '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
                '/System/Library/Fonts/AppleSDGothicNeo.ttc', '/System/Library/Fonts/Supplemental/AppleGothic.ttf']


def use_korean_font():
    for path in KOREAN_FONTS:
        if Path(path).exists():
            matplotlib.font_manager.fontManager.addfont(path)
            plt.rcParams['font.family'] = matplotlib.font_manager.FontProperties(fname=path).get_name()
            plt.rcParams['axes.unicode_minus'] = False
            return


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--group', nargs='+', action='append', required=True, metavar=('NAME', 'RUN'),
                   help='후보 이름과 실행 폴더들 (여러 번)')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--until', type=float, default=None,
                   help='요약에 쓸 마지막 스텝 (백만 단위). 기본은 모든 실행에 공통으로 있는 마지막 스텝. 0이면 실행마다 전체')
    p.add_argument('--panels', choices=sorted(PANEL_SETS), default='select',
                   help='select: 체크포인트 선택용 검증 eval/* (기본), common: 비교군 공통 검증 eval_common/*')
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    panel_set = PANEL_SETS[args.panels]

    groups = {g[0]: [Path(r) for r in g[1:]] for g in args.group}
    data = {name: {run.name: read_run(run, panel_set) for run in runs} for name, runs in groups.items()}
    panels = [pn for pn in panel_set if any(pn[0] in d for runs in data.values() for d in runs.values())]

    # 실행마다 학습량이 다르면 "최종"이 서로 다른 구간의 평균이 된다. 요약은 공통 스텝까지로 자른다
    horizons = {}
    for tag, *_ in panels:
        curves = [d[tag] for runs in data.values() for d in runs.values() if tag in d]
        if args.until is None:
            horizons[tag] = common_horizon(curves)
        else:
            horizons[tag] = None if args.until == 0 else args.until * 1e6

    use_korean_font()
    fig, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 4.2), squeeze=False)
    rows, summary = [], []
    for gi, (name, runs) in enumerate(data.items()):
        color = COLORS[gi % len(COLORS)]
        for ax, (tag, title, *_) in zip(axes[0], panels):
            curves = [(np.asarray(d[tag][0]) / 1e6, d[tag][1]) for d in runs.values() if tag in d]
            if not curves:
                continue
            for s, v in curves:
                ax.plot(s, v, color=color, lw=0.8, alpha=0.35)
            grid, mean, low, high = mean_band(curves)
            ax.fill_between(grid, low, high, color=color, alpha=0.12, lw=0)
            ax.plot(grid, mean, color=color, lw=2.2, label=f'{name} (시드 {len(curves)}개)')
            ax.set_title(title)
        for run_name, d in runs.items():
            for tag, title, threshold, higher, _ in panels:
                if tag not in d:
                    continue
                for step, value in zip(*d[tag]):
                    rows.append({'group': name, 'run': run_name, 'tag': tag, 'step': step, 'value': value})
                sm = curve_summary(*d[tag], threshold=threshold, higher_is_better=higher, until=horizons[tag])
                summary.append({'group': name, 'run': run_name, 'tag': tag, 'higher_is_better': higher, **sm})

    for ax, (tag, *_, ymax) in zip(axes[0], panels):
        if horizons[tag] is not None:   # 요약에 쓴 구간의 끝
            ax.axvline(horizons[tag] / 1e6, color='#888888', lw=1, ls='--')
        ax.set_xlabel('학습 스텝 (백만)')
        ax.set_ylim(0, ymax)
        ax.grid(alpha=0.3)
    axes[0][0].legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out / 'curves.png', dpi=150)

    with open(args.out / 'curves.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['group', 'run', 'tag', 'step', 'value'])
        w.writeheader()
        w.writerows(rows)
    keys = ['group', 'run', 'tag', 'higher_is_better', 'points', 'last_step', 'auc', 'final', 'best', 'worsen',
            'steps_to']
    with open(args.out / 'summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(summary)

    pct = lambda x: '-' if x is None else f'{100 * x:.0f}%'  # noqa: E731
    mps = lambda x: '-' if x is None else f'{x:.2f}'  # noqa: E731
    for tag, *_ in panels:
        h = horizons[tag]
        print(f'{tag}: 요약 구간 ' + ('실행마다 전체' if h is None else f'~{h / 1e6:.2f}M 스텝 (공통)'))
    print(f'{"후보":6s} {"실행":34s} {"지표":22s} {"점":>3s} {"끝(M)":>6s} {"AUC":>5s} {"최종":>5s} '
          f'{"최고/최저":>7s} {"나빠짐":>6s} {"80%도달(M)":>9s}')
    for r in summary:
        to = '-' if r['steps_to'] is None else f'{r["steps_to"] / 1e6:.2f}'
        fmt = mps if '/speed_' in r['tag'] else pct      # 속도는 m/s
        best = ('최고 ' if r['higher_is_better'] else '최저 ') + fmt(r['best'])
        print(f'{r["group"]:6s} {r["run"]:34s} {r["tag"]:22s} {r["points"]:3d} {r["last_step"] / 1e6:6.2f} '
              f'{fmt(r["auc"]):>5s} {fmt(r["final"]):>5s} {best:>7s} {fmt(r["worsen"]):>6s} {to:>9s}')
    print(f'저장: {args.out}/curves.png, curves.csv, summary.csv')


if __name__ == '__main__':
    main()
