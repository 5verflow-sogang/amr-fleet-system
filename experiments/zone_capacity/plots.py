#!/usr/bin/env python3
"""zone_capacity 그래프 여러 버전 (analyze.py가 쓴 results.json을 읽는다).

    python3 experiments/zone_capacity/plots.py experiments/zone_capacity/results.json --out <폴더>

그래프 (각각 컬러 _color.png, 흑백 _mono.png):
  A_pass_time     손님 수 → 통과 시간 중앙값 (중앙값의 95% 부트스트랩 구간) + BPR 맞춤 곡선
  B_relative      손님 수 → 상대 통과 시간 (손님 0명 = 1) + "2배 = 정원" 기준선. 두 통로를 한 축에
  C_distribution  통과 한 번씩 (시간 초과는 세모로 제한 시간에) + 중앙값 막대
  D_within60      60 s 안에 지나간 비율
  E_capacity      정원: 가정값(통로 길이) vs 측정값(통과 시간 2배)
  poster          B + E 한 장 (흑백)
"""

import argparse
import json
import random
import sys
from pathlib import Path
from statistics import median

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bpr import bpr  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for path in ('/System/Library/Fonts/AppleSDGothicNeo.ttc', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if Path(path).exists():
        font_manager.fontManager.addfont(path)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=path).get_name()
        break
plt.rcParams.update({'axes.unicode_minus': False, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.edgecolor': '#666666', 'axes.labelcolor': '#191919', 'xtick.color': '#666666',
                     'ytick.color': '#666666', 'axes.titleweight': 'bold', 'axes.titlesize': 12,
                     'axes.titlelocation': 'left'})

NAMES = {'west_aisle': '서쪽 통로 (폭 1.6 m)', 'aisle_3': '세로 통로 (폭 1.37 m)'}
ASSUMED = {'west_aisle': 3, 'aisle_3': 3}       # 7·12구역의 가정값 (통로 길이 2.3 m당 1명)
STYLE = {   # 통로별 고정 순서: 컬러는 기본 팔레트 1·2번, 흑백은 검정·회색 + 마커 모양
    'color': {'west_aisle': dict(c='#257de0', m='o', ls='-'), 'aisle_3': dict(c='#f47230', m='s', ls='-')},
    'mono': {'west_aisle': dict(c='#191919', m='o', ls='-'), 'aisle_3': dict(c='#8a8a8a', m='s', ls='--')},
}
TEXT2 = '#666666'
GRID = '#e6e6e6'


def boot_ci(xs, n=2000, seed=1):
    rng = random.Random(seed)
    meds = sorted(median(rng.choices(xs, k=len(xs))) for _ in range(n))
    return meds[int(0.025 * n)], meds[int(0.975 * n)]


def stats(data):
    passes, summary = data['passes'], data['summary']
    zones = [z for z in NAMES if z in summary]
    out = {}
    for z in zones:
        ns = sorted({p['n'] for p in passes if p['zone'] == z})
        rows = []
        for n in ns:
            sel = [p for p in passes if p['zone'] == z and p['n'] == n]
            xs = [p['time_s'] for p in sel]
            cens = sum(1 for p in sel if p['censored'])
            lo, hi = boot_ci(xs)
            rows.append({'n': n, 'xs': xs, 'med': median(xs), 'lo': lo, 'hi': hi, 'k': len(xs), 'cens': cens,
                         'within60': sum(1 for p in sel if not p['censored'] and p['time_s'] <= 60) / len(sel)})
        t0 = rows[0]['med']
        out[z] = {'rows': rows, 't0': t0, 'fit': summary[z].get('fit'), 'dbl': summary[z].get('doubling_n')}
    return zones, out


def grid(ax, axis='y'):
    ax.grid(axis=axis, color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def note(fig, text):
    fig.text(0.01, 0.005, text, fontsize=7.5, color=TEXT2, ha='left', va='bottom')


FOOT = '식당 Gazebo, 서빙 로봇 1대, Nav2 + 안전 노드. 조건마다 통과 {k}번. 120 s 넘으면 120 s로 넣음 (시간 초과)'


def plot_a(zones, st, mode, out):
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    for z in zones:
        s, rows = STYLE[mode][z], st[z]['rows']
        ns = [r['n'] for r in rows]
        ax.errorbar(ns, [r['med'] for r in rows], yerr=[[r['med'] - r['lo'] for r in rows], [r['hi'] - r['med'] for r in rows]],
                    fmt=s['m'], color=s['c'], ms=7, capsize=3, lw=1.2, label=NAMES[z])
        f = st[z]['fit']
        if f and not f.get('at_edge'):
            xs = [i * max(ns) / 100 for i in range(101)]
            ax.plot(xs, [bpr(x, f['t0'], f['capacity'], f['beta']) for x in xs], ls=s['ls'], color=s['c'], lw=1.5, alpha=0.8)
        else:
            ax.plot(ns, [r['med'] for r in rows], ls=s['ls'], color=s['c'], lw=1.5, alpha=0.8)
    ax.axhline(120, color=TEXT2, lw=0.8, ls=':')
    ax.text(max(r['n'] for r in st[zones[0]]['rows']), 121, '시간 초과 (120 s)', fontsize=8, color=TEXT2, ha='right', va='bottom')
    ax.set_xlabel('통로 안 손님 수 (명)')
    ax.set_ylabel('통과 시간 중앙값 (s)')
    ax.set_title('통로 안 손님 수와 통과 시간')
    ax.set_xticks(sorted({r['n'] for z in zones for r in st[z]['rows']}))
    ax.set_ylim(0, 130)
    grid(ax)
    ax.legend(frameon=False, fontsize=9, loc='lower right', bbox_to_anchor=(0.86, 0.02))
    note(fig, FOOT.format(k=st[zones[0]]['rows'][0]['k']) + '. 막대 = 중앙값의 95% 구간')
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, dpi=200)
    plt.close(fig)


def plot_b(zones, st, mode, out, ax=None):
    own = ax is None
    if own:
        fig, ax = plt.subplots(figsize=(6.4, 4.0))
    top = 1
    for z in zones:
        s, rows, t0 = STYLE[mode][z], st[z]['rows'], st[z]['t0']
        ns = [r['n'] for r in rows]
        ys = [r['med'] / t0 for r in rows]
        top = max(top, max(ys))
        ax.plot(ns, ys, marker=s['m'], ls=s['ls'], color=s['c'], lw=2, ms=8, label=NAMES[z])
        d = st[z]['dbl']
        if d is not None:
            ax.plot([d], [2.0], marker='|', color=s['c'], ms=16, mew=2)
            first = z == zones[0]
            ax.annotate(f'정원 ≈ {d:.1f}명', (d, 2.0), xytext=(-62, 22) if first else (14, -26), textcoords='offset points',
                        fontsize=9, color='#191919', arrowprops=dict(arrowstyle='-', color=s['c'], lw=1),
                        bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='none'))
    ax.axhline(2.0, color='#191919', lw=1, ls='--')
    xmax = max(r['n'] for z in zones for r in st[z]['rows'])
    ax.text(xmax, 1.93, '2배 = 정원 기준', fontsize=8.5, color='#191919', va='top', ha='right')
    ax.axhline(1.0, color=GRID, lw=1)
    ax.set_xlabel('통로 안 손님 수 (명)')
    ax.set_ylabel('통과 시간 (손님 0명 = 1)')
    ax.set_title('통과 시간이 2배가 되는 손님 수 = 정원')
    ax.set_xticks(sorted({r['n'] for z in zones for r in st[z]['rows']}))
    ax.set_ylim(0, max(4.0, top * 1.1))
    grid(ax)
    ax.legend(frameon=False, fontsize=9, loc='lower right')
    if own:
        note(fig, FOOT.format(k=st[zones[0]]['rows'][0]['k']) + '. 점 = 중앙값')
        fig.tight_layout(rect=(0, 0.03, 1, 1))
        fig.savefig(out, dpi=200)
        plt.close(fig)


def plot_c(zones, st, mode, out):
    fig, axes = plt.subplots(1, len(zones), figsize=(4.0 * len(zones), 3.8), sharey=True)
    axes = axes if len(zones) > 1 else [axes]
    for ax, z in zip(axes, zones):
        s = STYLE[mode][z]
        rng = random.Random(3)
        for r in st[z]['rows']:
            for x in r['xs']:
                cens = x >= 120
                ax.plot(r['n'] + rng.uniform(-0.18, 0.18), x, marker='^' if cens else s['m'], ls='none', ms=5,
                        color=s['c'], alpha=0.55 if not cens else 0.9, mfc='white' if cens else s['c'])
            ax.plot([r['n'] - 0.3, r['n'] + 0.3], [r['med']] * 2, color='#191919', lw=2)
            ax.text(r['n'], 126, f"{r['cens']}/{r['k']}", ha='center', fontsize=8, color=TEXT2)
        ax.set_title(NAMES[z])
        ax.set_xticks([r['n'] for r in st[z]['rows']])
        ax.set_xlabel('통로 안 손님 수 (명)')
        ax.set_ylim(0, 132)
        grid(ax)
    axes[0].set_ylabel('통과 한 번 (s)')
    fig.suptitle('통과 한 번씩: 검은 막대 = 중앙값, 빈 세모 = 시간 초과(120 s), 위 숫자 = 시간 초과 수', fontsize=9.5,
                 x=0.01, ha='left', color='#191919')
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=200)
    plt.close(fig)


def plot_d(zones, st, mode, out):
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ns = sorted({r['n'] for z in zones for r in st[z]['rows']})
    w = 0.36
    for i, z in enumerate(zones):
        s = STYLE[mode][z]
        rows = {r['n']: r for r in st[z]['rows']}
        xs = [n + (i - (len(zones) - 1) / 2) * (w + 0.03) for n in ns]
        ys = [rows[n]['within60'] * 100 if n in rows else 0 for n in ns]
        ax.bar(xs, ys, width=w, color=s['c'], label=NAMES[z], hatch=None if mode == 'color' or i == 0 else '///',
               edgecolor='white', lw=0)
        for x, y in zip(xs, ys):
            ax.text(x, y + 2, f'{y:.0f}%', ha='center', fontsize=8, color='#191919')
    ax.set_xticks(ns)
    ax.set_xlabel('통로 안 손님 수 (명)')
    ax.set_ylabel('60 s 안에 지나간 비율 (%)')
    ax.set_title('60 s 안에 통로를 지나간 비율')
    ax.set_ylim(0, 110)
    grid(ax)
    ax.legend(frameon=False, fontsize=9, loc='upper right')
    note(fig, FOOT.format(k=st[zones[0]]['rows'][0]['k']))
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, dpi=200)
    plt.close(fig)


def plot_e(zones, st, mode, out, ax=None):
    own = ax is None
    if own:
        fig, ax = plt.subplots(figsize=(5.6, 3.2))
    ys = list(range(len(zones)))[::-1]
    for y, z in zip(ys, zones):
        s = STYLE[mode][z]
        a, m = ASSUMED[z], st[z]['dbl']
        ax.plot([m if m is not None else 0, a], [y, y], color=GRID, lw=6, solid_capstyle='round')
        ax.plot([a], [y], marker='o', ms=11, mfc='white', mec=TEXT2, mew=2, ls='none')
        ax.text(a, y + 0.22, f'가정 {a}명', ha='center', fontsize=9, color=TEXT2)
        if m is not None:
            ax.plot([m], [y], marker=s['m'], ms=11, color=s['c'], ls='none')
            ax.text(m, y + 0.22, f'측정 {m:.1f}명', ha='center', fontsize=9, color='#191919', fontweight='bold')
    ax.set_yticks(ys)
    ax.set_yticklabels([NAMES[z] for z in zones])
    ax.set_xlim(0, max(ASSUMED.values()) + 0.8)
    ax.set_ylim(-0.6, len(zones) - 0.3)
    ax.set_xlabel('정원 (명): 통과 시간이 2배가 되는 손님 수')
    ax.set_title('정원: 가정값과 측정값')
    ax.spines['left'].set_visible(False)
    ax.tick_params(axis='y', length=0)
    grid(ax, 'x')
    if own:
        fig.tight_layout()
        fig.savefig(out, dpi=200)
        plt.close(fig)


def plot_poster(zones, st, out):
    plt.rcParams.update({'font.size': 12, 'axes.titlesize': 14, 'axes.labelsize': 12})
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={'width_ratios': [1.4, 1]})
    plot_b(zones, st, 'mono', None, ax=a1)
    plot_e(zones, st, 'mono', None, ax=a2)
    note(fig, FOOT.format(k=st[zones[0]]['rows'][0]['k']))
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=220)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('results')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    data = json.loads(Path(a.results).read_text())
    data['summary'] = {z: s for z, s in data['summary'].items()}
    zones, st = stats(data)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for mode in ('color', 'mono'):
        plot_a(zones, st, mode, out / f'A_pass_time_{mode}.png')
        plot_b(zones, st, mode, out / f'B_relative_{mode}.png')
        plot_c(zones, st, mode, out / f'C_distribution_{mode}.png')
        plot_d(zones, st, mode, out / f'D_within60_{mode}.png')
        plot_e(zones, st, mode, out / f'E_capacity_{mode}.png')
    plot_poster(zones, st, out / 'poster_mono.png')
    for z in zones:
        rows = st[z]['rows']
        print(z, ' '.join(f"n{r['n']}: 중앙값 {r['med']:.1f} [{r['lo']:.1f}, {r['hi']:.1f}] 초과 {r['cens']}/{r['k']} 60s안 {r['within60']:.0%}"
                          for r in rows), '정원', st[z]['dbl'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
