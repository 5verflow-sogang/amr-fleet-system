"""규모 곡선 그림 (scale_curve.md 8절): 패널 3개, 색=작업 부하 안(A·B), 선 모양=지연(0 실선, 100 점선), 95% CI 띠.

    python3 experiments/research1/scale_curve/plot.py <data>/summary.csv experiments/research1/scale_curve/scale_curve.png
"""
import csv, sys
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

src, out = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(src)))
data = defaultdict(dict)  # (metric, plan, d) -> n -> (mean, lo, hi)
for r in rows:
    data[(r['metric'], r['plan'], int(r['delay_ms']))][int(r['n'])] = tuple(float(r[k]) for k in ('mean', 'ci_low', 'ci_high'))

INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e6e5e1', '#fcfcfb'
COLOR = {'a': '#2a78d6', 'b': '#eb6834'}
NAME = {'a': 'A (total load fixed)', 'b': 'B (per-robot load fixed)'}
# (metric, title, injected delay subtracted?)
PANELS = [('fleet.ack_rtt_ms.mean', 'Ack round trip − injected delay (ms)', True),
          ('fleet.latency_ms.p99', 'Link latency P99 − injected delay (ms)', True),
          ('fleet.throughput_per_min', 'Throughput (tasks/min)', False)]
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'font.family': 'DejaVu Sans'})
fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.3), facecolor=SURF)
for ax, (metric, title, sub) in zip(axes, PANELS):
    ax.set_facecolor(SURF)
    for plan in ('a', 'b'):
        for d, ls in ((0, '-'), (100, '--')):
            series = data[(metric, plan, d)]
            ns = sorted(series)
            off = d if sub else 0
            m = [series[n][0] - off for n in ns]; lo = [series[n][1] - off for n in ns]; hi = [series[n][2] - off for n in ns]
            ax.fill_between(ns, lo, hi, color=COLOR[plan], alpha=0.15, linewidth=0)
            ax.plot(ns, m, ls, color=COLOR[plan], linewidth=2, marker='o', markersize=4.5,
                    markeredgecolor=SURF, markeredgewidth=1.5, label=f'{NAME[plan]}, delay {d} ms')
    ax.set_title(title, fontsize=9.5, color=INK, loc='left')
    ax.set_xticks([3, 5, 7, 10]); ax.set_xlabel('Robots')
    ax.set_ylim(bottom=0)
    ax.grid(axis='y', color=GRID, linewidth=0.8); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
h, l = axes[0].get_legend_handles_labels()
fig.legend(h, l, loc='lower center', ncol=4, frameon=False, fontsize=8.5, labelcolor=INK2)
fig.tight_layout(rect=(0, 0.1, 1, 1))
fig.savefig(out, dpi=200, facecolor=SURF)
print(out)
