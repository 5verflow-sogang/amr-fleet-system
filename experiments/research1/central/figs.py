"""연구 1 KSC 그림. 사용: python3 figs.py data figures   (data는 fetch.sh로 받은 폴더)
A: 조건별 처리량 변화 (중앙, 포화)   B: 예측 vs 실측 (추가 대기, 처리량)   C: 단절 로봇 정책
단 폭 88 mm(한 단), B·C는 두 단 폭 180 mm 판도 같이 만든다."""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

D, OUT = Path(sys.argv[1]), Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.family': 'NanumGothic', 'font.size': 7, 'axes.linewidth': 0.6, 'pdf.fonttype': 42,
                     'axes.unicode_minus': False, 'xtick.major.width': 0.6, 'ytick.major.width': 0.6})
MM = 1 / 25.4
rng = np.random.default_rng(0)


def rows(p):
    if not p.exists():
        return []
    with open(p, newline='') as f:
        return list(csv.DictReader(f))


_cache = {}


def run_stats(run):
    if run in _cache:
        return _cache[run]
    tasks = rows(run / 'task_outcomes.csv')
    acks = {}
    for r in rows(run / 'response_time.csv'):
        if r['ack_time']:
            acks.setdefault((r['task_id'], r['robot']), float(r['ack_time']))
    done = [t for t in tasks if t['final_state'] == 'COMPLETED']
    ends, ak = defaultdict(list), defaultdict(list)
    for t in done:
        ends[t['robots'].split()[-1]].append(float(t['end_time']))
    for (_, rb), v in acks.items():
        ak[rb].append(v)
    gaps = []
    for rb, es in ends.items():
        a = sorted(ak[rb])
        for e in es:
            nxt = [x for x in a if x > e]
            if nxt:
                gaps.append(nxt[0] - e)
    exec_t = [float(t['end_time']) - acks[(t['task_id'], t['robots'].split()[-1])] for t in done
              if (t['task_id'], t['robots'].split()[-1]) in acks]
    a3 = 0
    for t in done:
        who = (t.get('runners') or t['robots']).split()
        a3 += bool(who) and who[-1] == 'amr_03'
    s = {'seed': json.loads((run / 'meta.json').read_text())['seed'], 'completed': len(done),
         'task_time': {t['task_id']: float(t['task_time_s']) for t in done},
         'gap': float(np.mean(gaps)), 'T': float(np.mean(exec_t)), 'a3': a3}
    _cache[run] = s
    return s


def load(cond):
    d = D / cond
    return {s['seed']: s for s in (run_stats(r) for r in sorted(d.iterdir()))} if d.exists() else {}


def boot(x):
    x = np.asarray(x, float)
    m = rng.choice(x, (10000, len(x))).mean(axis=1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


def paired(base, cond, fn):
    a, b = load(base), load(cond)
    s = sorted(set(a) & set(b))
    return boot([fn(a[k], b[k]) for k in s])


def comp_change(a, b):
    return (b['completed'] / a['completed'] - 1) * 100


def time_ratio(a, b):
    common = sorted(set(a['task_time']) & set(b['task_time']))
    return (np.mean([b['task_time'][k] for k in common]) / np.mean([a['task_time'][k] for k in common]) - 1) * 100


def save(fig, name):
    fig.tight_layout(pad=0.3)
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'{name}.{ext}', dpi=300)
    plt.close(fig)


# ---------- 그림 A: 조건별 처리량 변화 (중앙 관제, 포화, 60초 작업) ----------
conds = [('지연 100 ms', 'central_d100'), ('지연 200 ms', 'central_d200'), ('지연 500 ms', 'central_d500'),
         ('손실 30%', 'central_loss30'), ('손실 50%', 'central_loss50'), ('손실 70%', 'central_loss70'),
         ('단절 3초', 'central_bo3_r3'), ('단절 10초', 'central_bo10_r3'), ('단절 30초', 'central_bo30_r3'),
         ('25초 작업\n+ 손실 70%', 'short_loss70'), ('단절 30초\n+ 멈춤 정책', 'central_bo30_r3_pause')]
fig, ax = plt.subplots(figsize=(88 * MM, 66 * MM))
ys = np.arange(len(conds))[::-1]
for y, (label, c) in zip(ys, conds):
    m, lo, hi = paired('short_d0' if c.startswith('short') else 'central_d0', c, time_ratio)
    sig = lo > 0 or hi < 0
    color = 'black' if sig else '0.6'
    marker = 's' if 'pause' in c else ('D' if c.startswith('short') else 'o')
    ax.errorbar(m, y, xerr=[[m - lo], [hi - m]], fmt=marker, ms=3.2, color=color, mfc=color, elinewidth=0.7,
                capsize=1.5)
    ax.text(hi + 1.0, y, f'{m:+.1f}%', va='center', fontsize=5.8, color=color,
            bbox={'fc': 'white', 'ec': 'none', 'pad': 0.4}, zorder=4)
ax.axvline(0, color='0.3', lw=0.6)
ax.axvline(10, color='black', lw=0.7, ls='--')
ax.text(10.4, ys[0] + 0.2, '급락 기준 +10%', fontsize=5.8, va='center')
for yb in (ys[2] - 0.5, ys[5] - 0.5, ys[8] - 0.5, ys[9] - 0.5):
    ax.axhline(yb, color='0.85', lw=0.5)
ax.set_yticks(ys)
ax.set_yticklabels([c[0] for c in conds], fontsize=6.2)
ax.set_xlabel('평균 작업 완료 시간 변화 (장애 없음 대비, %)')
ax.set_xlim(-3, 28)
ax.spines[['top', 'right']].set_visible(False)
save(fig, 'figA_conditions')


# ---------- 그림 B: 예측 vs 실측 ----------
def model_points():
    """(그룹, p, 예측 W, 실측 추가 대기, 예측 처리량 변화 %, 실측 처리량 변화 CI)"""
    post = [(0.3, 'central_loss30'), (0.5, 'central_loss50'), (0.7, 'central_loss70')]
    resend = [(0.5, 'central_loss50_k0'), (0.5, 'central_loss50_k1')]
    groups = [('60초 작업 (사후 적합)', 'central_d0', post, 'o', 'black'),
              ('60초, 재전송 0·1번 (사전 예측)', 'central_d0', resend, 's', 'white'),
              ('25초 작업 (사전 예측)', 'short_d0', [(0.5, 'short_loss50'), (0.7, 'short_loss70')], '^', 'white')]
    out = []
    for name, base, pts, marker, face in groups:
        b = load(base)
        T = np.mean([v['T'] for v in b.values()])
        g0 = np.mean([v['gap'] for v in b.values()])
        for p, c in pts:
            W = p / (1 - p)
            gap = paired(base, c, lambda a, x: x['gap'] - a['gap'])
            thr = paired(base, c, comp_change)
            out.append((name, marker, face, p, W, gap, ((T + g0) / (T + g0 + W) - 1) * 100, thr))
    return out


pts = model_points()


def panel_wait(ax):
    lim = 2.7
    ax.plot([0, lim], [0, lim], color='0.6', lw=0.6, ls='--', zorder=0)
    ax.text(1.85, 1.62, '예측 = 실측', color='0.45', fontsize=5.8, rotation=35)
    seen = set()
    for name, marker, face, p, W, gap, _, _ in pts:
        ax.errorbar(W, gap[0], yerr=[[gap[0] - gap[1]], [gap[2] - gap[0]]], fmt=marker, ms=3.4, mfc=face,
                    mec='black', ecolor='black', elinewidth=0.6, capsize=1.5,
                    label=None if name in seen else name, zorder=3 if face == 'white' else 2)
        seen.add(name)
    for p in (0.3, 0.5, 0.7):
        ax.annotate(f'손실 {p:.0%}', (p / (1 - p), p / (1 - p)), xytext=(-5, 7), textcoords='offset points',
                    ha='right', fontsize=5.5, color='0.35')
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel('예측 추가 대기 W = r·p/(1-p) (초)')
    ax.set_ylabel('실측 추가 대기 (초)')
    ax.legend(frameon=False, fontsize=5.5, loc='upper left', handletextpad=0.3)


def panel_thr(ax):
    lim = 11
    ax.plot([0, lim], [0, lim], color='0.6', lw=0.6, ls='--', zorder=0)
    for name, marker, face, p, W, gap, pred, thr in pts:
        ax.errorbar(-pred, -thr[0], yerr=[[thr[2] - thr[0]], [thr[0] - thr[1]]], fmt=marker, ms=3.4, mfc=face,
                    mec='black', ecolor='black', elinewidth=0.6, capsize=1.5, zorder=3 if face == 'white' else 2)
        if '재전송' in name:
            continue
        short = '25초' in name
        ax.annotate(f'{"25초" if short else "60초"} 작업, 손실 {p:.0%}', (-pred, -thr[0]),
                    xytext=(-6, 4) if short else (7, -3), textcoords='offset points',
                    ha='right' if short else 'left', fontsize=5.3, color='0.35')
    ax.set_xlim(-0.3, lim)
    ax.set_ylim(-0.3, lim)
    ax.set_xlabel('예측 처리량 감소 (%)')
    ax.set_ylabel('실측 처리량 감소 (%)')


fig, (a1, a2) = plt.subplots(1, 2, figsize=(180 * MM, 60 * MM))
panel_wait(a1)
panel_thr(a2)
a1.set_title('(a) 명령을 받기까지 추가 대기', fontsize=7)
a2.set_title('(b) 처리량 감소 (완료 작업 수)', fontsize=7)
save(fig, 'figB_prediction_wide')
fig, ax = plt.subplots(figsize=(88 * MM, 60 * MM))
panel_wait(ax)
save(fig, 'figB_prediction')


# ---------- 그림 C: 단절 로봇 정책 ----------
policies = [('① 끊기면\n멈춤', 'central_bo30_r3_pause', '0.35', ''), ('② 받은 일\n계속', 'central_bo30_r3', '0.6', ''),
            ('③ 새 일도\n스스로 (상한)', 'hybrid_bo30_r3', '0.9', '////')]


def panel_a3(ax):
    base = boot([v['a3'] for v in load('central_d0').values()])
    for i, (label, c, color, hatch) in enumerate(policies):
        m = boot([v['a3'] for v in load(c).values()])
        ax.bar(i, m[0], width=0.62, color=color, edgecolor='black', lw=0.6, hatch=hatch,
               yerr=[[m[0] - m[1]], [m[2] - m[0]]], error_kw={'elinewidth': 0.6, 'capsize': 2})
        ax.text(i, m[2] + 0.2, f'{m[0]:.1f}', ha='center', va='bottom', fontsize=6)
    ax.axhline(base[0], color='black', lw=0.6, ls=':')
    ax.text(-0.42, base[0] + 0.15, f'단절 없음 {base[0]:.1f}', fontsize=5.6, va='bottom')
    ax.set_xticks(range(len(policies)))
    ax.set_xticklabels([p[0] for p in policies], fontsize=6)
    ax.set_ylabel('단절 로봇의 완료 작업 (건)')
    ax.set_ylim(0, 9.5)
    ax.spines[['top', 'right']].set_visible(False)


def panel_time(ax):
    for i, (label, c, color, hatch) in enumerate(policies):
        m = paired('central_d0', c, time_ratio)
        ax.bar(i, m[0], width=0.62, color=color, edgecolor='black', lw=0.6, hatch=hatch,
               yerr=[[m[0] - m[1]], [m[2] - m[0]]], error_kw={'elinewidth': 0.6, 'capsize': 2})
        ax.text(i, max(m[2], 0) + 0.6, f'{m[0]:+.1f}%', ha='center', va='bottom', fontsize=6)
    ax.axhline(10, color='black', lw=0.7, ls='--')
    ax.text(2.42, 10.5, '급락 기준 +10%', fontsize=5.6, ha='right')
    ax.axhline(0, color='0.3', lw=0.6)
    ax.set_xticks(range(len(policies)))
    ax.set_xticklabels([p[0] for p in policies], fontsize=6)
    ax.set_ylabel('전체 평균 완료 시간 변화 (%)')
    ax.set_ylim(-6, 28)
    ax.spines[['top', 'right']].set_visible(False)


fig, (c1, c2) = plt.subplots(1, 2, figsize=(180 * MM, 58 * MM))
panel_a3(c1)
panel_time(c2)
c1.set_title('(a) 단절 로봇(amr_03)의 처리량', fontsize=7)
c2.set_title('(b) 함대 전체 완료 시간 (단절 없음 대비)', fontsize=7)
save(fig, 'figC_policy_wide')
fig, (c1, c2) = plt.subplots(2, 1, figsize=(88 * MM, 100 * MM))
panel_a3(c1)
panel_time(c2)
c1.set_title('(a) 단절 로봇(amr_03)의 처리량', fontsize=7)
c2.set_title('(b) 함대 전체 완료 시간 (단절 없음 대비)', fontsize=7)
save(fig, 'figC_policy')
print('ok')
