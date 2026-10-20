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


def save(fig, name, **layout):
    fig.tight_layout(pad=0.3, **layout)
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'{name}.{ext}', dpi=300)
    plt.close(fig)


# ---------- 그림 A: 조건별 처리량 변화 (중앙 관제, 포화, 60초 작업) ----------
conds = [('지연 100 ms', 'central_d100'), ('지연 200 ms', 'central_d200'), ('지연 500 ms', 'central_d500'),
         ('손실 30%', 'central_loss30'), ('손실 50%', 'central_loss50'), ('손실 70%', 'central_loss70'),
         ('단절 3초', 'central_bo3_r3'), ('단절 10초', 'central_bo10_r3'), ('단절 30초', 'central_bo30_r3'),
         ('25초 작업\n+ 손실 70%', 'short_loss70'), ('단절 30초\n+ 정지 정책', 'central_bo30_r3_pause')]
# 무리(그룹) 경계와 이름: 같은 무리끼리 가는 선으로 나누고 오른쪽 끝에 이름을 단다
GROUPS_A = [('지연', 0, 2), ('손실', 3, 5), ('단절\n(1대)', 6, 8), ('취약\n조건', 9, 10)]
fig, ax = plt.subplots(figsize=(88 * MM, 74 * MM))
ys = np.arange(len(conds))[::-1]
XMAX = 30
for y, (label, c) in zip(ys, conds):
    m, lo, hi = paired('short_d0' if c.startswith('short') else 'central_d0', c, time_ratio)
    sig = lo > 0 or hi < 0
    color = 'black' if sig else '0.6'
    marker = 's' if 'pause' in c else ('D' if c.startswith('short') else 'o')
    ax.errorbar(m, y, xerr=[[m - lo], [hi - m]], fmt=marker, ms=3.2, color=color, mfc=color, elinewidth=0.7,
                capsize=1.5, zorder=3)
    # 값은 신뢰구간 끝에서 띄워 오차 막대와 겹치지 않게
    ax.text(hi + 0.9, y, f'{m:+.1f}%', va='center', ha='left', fontsize=5.8, color=color, zorder=4,
            bbox={'fc': 'white', 'ec': 'none', 'pad': 0.3})
ax.axvline(0, color='0.3', lw=0.6, zorder=1)
ax.axvline(10, color='black', lw=0.7, ls='--', zorder=1)
ax.text(10.4, ys[0] + 0.45, '급락 기준 +10%', fontsize=5.6, va='bottom')
for name, i0, i1 in GROUPS_A:
    if i1 < len(conds) - 1:
        ax.axhline(ys[i1] - 0.5, color='0.85', lw=0.5, zorder=0)
ax.set_yticks(ys)
ax.set_yticklabels([c[0] for c in conds], fontsize=6.2)
ax.set_xlabel('평균 작업 완료 시간 변화 (%)\n장애 없음 대비, 같은 seed 두 실행 모두 끝난 작업끼리')
ax.set_xlim(-3, XMAX)
ax.set_ylim(ys[-1] - 0.6, ys[0] + 0.9)
ax.spines[['top', 'right']].set_visible(False)
# 범례: 회색은 차이 없음, 모양은 조건 종류 (색만으로 구분하지 않게)
from matplotlib.lines import Line2D  # noqa: E402
handles = [Line2D([], [], marker='o', ls='', color='black', ms=3.2, label='95% CI가 0 밖 (차이 있음)'),
           Line2D([], [], marker='o', ls='', color='0.6', ms=3.2, label='95% CI가 0 포함 (차이 없음)')]
ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(1.0, 0.93), frameon=False, fontsize=5.4,
          handletextpad=0.2, borderaxespad=0.2)
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
            out.append((name, marker, face, p, W, gap, ((T + g0) / (T + g0 + W) - 1) * 100, thr, c))
    return out


pts = model_points()
# 같은 예측값에 점이 여럿 겹치면(손실 50%: 사후 적합·재전송 0번·1번·25초 작업) 가로로 조금 띄운다.
# 띄운 만큼은 값이 아니라 보기용이라 그림 설명에 적는다
DODGE = {'central_loss50': 0.0, 'central_loss70': 0.0, 'central_loss30': 0.0,
         'central_loss50_k0': -1, 'central_loss50_k1': 1, 'short_loss50': 2, 'short_loss70': 2}
LEGEND_B = []


def _wait_points(ax, step, collect):
    seen = set()
    for name, marker, face, p, W, gap, _, _, c in pts:
        x = W + DODGE[c] * step   # 같은 예측값의 점을 조금만 띄운다 (값이 틀려 보이지 않을 만큼)
        h = ax.errorbar(x, gap[0], yerr=[[gap[0] - gap[1]], [gap[2] - gap[0]]], fmt=marker, ms=3.0, mfc=face,
                        mec='black', ecolor='black', elinewidth=0.6, capsize=1.5,
                        label=None if name in seen else name, zorder=3 if face == 'white' else 2)
        if collect and name not in seen and len(LEGEND_B) < 3:
            LEGEND_B.append((h, name))
        seen.add(name)


def panel_wait(ax, legend=True, inset=False):
    lim = 2.7
    ax.plot([0, lim], [0, lim], color='0.6', lw=0.6, ls='--', zorder=0)
    ax.text(1.72, 1.52, '예측 = 실측', color='0.45', fontsize=5.6, rotation=35, ha='center', va='top')
    _wait_points(ax, 0.03 if inset else 0.045, collect=True)
    # 손실률 이름은 점 묶음 아래 오른쪽에 (오차 막대·대각선과 안 겹치게)
    for p, dx, dy in ((0.3, 6, -10), (0.5, 8, -16), (0.7, -10, 10)):
        ax.annotate(f'손실 {p:.0%}', (p / (1 - p), p / (1 - p)), xytext=(dx, dy), textcoords='offset points',
                    ha='left' if dx > 0 else 'right', fontsize=5.5, color='0.35')
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel('예측 추가 대기 W = r·p/(1-p) (초, r = 1초)')
    ax.set_ylabel('실측 추가 대기 (초)')
    if inset:
        # 손실 50%는 네 조건의 예측이 모두 1.0초라 점이 겹친다. 확대 창에서는 조건을 가로로 나란히 놓고
        # 예측 1.0초를 가로 점선으로 그려, 옆으로 띄워도 값이 틀려 보이지 않게 한다
        ins = ax.inset_axes([0.08, 0.6, 0.44, 0.34])
        order = [('central_loss50', '60초'), ('central_loss50_k1', '재전송\n1회'),
                 ('central_loss50_k0', '재전송\n0회'), ('short_loss50', '25초')]
        by = {c: (marker, face, gap) for _, marker, face, _, _, gap, _, _, c in pts}
        for i, (c, _) in enumerate(order):
            marker, face, gap = by[c]
            ins.errorbar(i, gap[0], yerr=[[gap[0] - gap[1]], [gap[2] - gap[0]]], fmt=marker, ms=3.0, mfc=face,
                         mec='black', ecolor='black', elinewidth=0.6, capsize=1.5)
        ins.axhline(1.0, color='0.6', lw=0.6, ls='--', zorder=0)
        ins.text(4.35, 1.012, '예측 1.0초', fontsize=5, color='0.45', va='bottom', ha='right')
        ins.set_xticks(range(len(order)))
        ins.set_xticklabels([n for _, n in order], fontsize=5)
        ins.set_xlim(-0.5, 4.4)
        ins.set_ylim(0.74, 1.26)
        ins.tick_params(labelsize=5, length=2, pad=1)
        ins.set_title('손실 50% (예측이 같은 네 조건)', fontsize=5.3, pad=2, color='0.3')
        ins.spines[['top', 'right']].set_visible(False)
        # 본 그림의 해당 부분에 상자를 그려 어디를 늘린 것인지 표시
        from matplotlib.patches import Rectangle
        ax.add_patch(Rectangle((0.9, 0.74), 0.24, 0.52, fill=False, ec='0.55', lw=0.5, zorder=1))
    if legend:
        ax.legend(frameon=False, fontsize=5.5, loc='lower right' if inset else 'upper left', handletextpad=0.3)


# (b) 점 이름: (x 오프셋, y 오프셋, 정렬). 오차 막대 옆 빈 곳으로
LABEL_THR = {'central_loss30': ('60초, 손실 30%', (2.6, 0.2)), 'central_loss50': ('60초, 손실 50%', (2.6, 0.95)),
             'central_loss70': ('60초, 손실 70%', (4.6, 2.3)), 'short_loss50': ('25초, 손실 50%', (5.2, 3.9)),
             'short_loss70': ('25초, 손실 70%', (6.0, 9.4)), 'central_loss50_k0': ('재전송 0·1번', (0.2, 3.0)),
             'central_loss50_k1': ('재전송 0·1번', (0.2, 3.0))}


def panel_thr(ax):
    lim = 11
    ax.plot([0, lim], [0, lim], color='0.6', lw=0.6, ls='--', zorder=0)
    for name, marker, face, p, W, gap, pred, thr, c in pts:
        x = -pred + (DODGE[c] * 0.15 if '_k' in c else 0)   # (b)는 예측이 같은 재전송 0·1번만 띄운다
        ax.errorbar(x, -thr[0], yerr=[[thr[2] - thr[0]], [thr[0] - thr[1]]], fmt=marker, ms=3.0, mfc=face,
                    mec='black', ecolor='black', elinewidth=0.6, capsize=1.5, zorder=3 if face == 'white' else 2)
        if c in LABEL_THR:
            text, (tx, ty) = LABEL_THR[c]
            ax.annotate(text, (x, -thr[0]), xytext=(tx, ty), ha='left', va='center', fontsize=5.3, color='0.35',
                        arrowprops={'arrowstyle': '-', 'color': '0.65', 'lw': 0.4, 'shrinkA': 1, 'shrinkB': 3})
    ax.set_xlim(-0.3, lim)
    ax.set_ylim(-0.3, lim)
    ax.set_xlabel('예측 처리량 감소 (%)')
    ax.set_ylabel('실측 처리량 감소 (%)')


fig, (a1, a2) = plt.subplots(1, 2, figsize=(180 * MM, 64 * MM))
panel_wait(a1, legend=False)
panel_thr(a2)
a1.set_title('(a) 명령을 받기까지 추가 대기', fontsize=7)
a2.set_title('(b) 처리량 감소 (완료 작업 수)', fontsize=7)
# 두 패널이 같은 기호를 쓰므로 범례는 위에 하나만. 검은 점 = 사후 적합, 흰 기호 = 실행 전에 등록한 예측
fig.legend([h for h, _ in LEGEND_B], [n for _, n in LEGEND_B], loc='upper center', ncol=3, frameon=False,
           fontsize=6, handletextpad=0.3, columnspacing=1.6, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(pad=0.3, rect=(0, 0, 1, 0.92))
for ext in ('pdf', 'png'):
    fig.savefig(OUT / f'figB_prediction_wide.{ext}', dpi=300)
plt.close(fig)
# 한 단 판: (a)·(b)를 따로 한 장씩 (원고에서 그림 2·3으로 나눠 넣는다)
fig, ax = plt.subplots(figsize=(88 * MM, 72 * MM))
panel_wait(ax, inset=True)
save(fig, 'figB_wait')
fig, ax = plt.subplots(figsize=(88 * MM, 72 * MM))
panel_thr(ax)
ax.legend([h for h, _ in LEGEND_B], [n for _, n in LEGEND_B], frameon=False, fontsize=5.5, loc='upper left',
          handletextpad=0.3)
save(fig, 'figB_throughput')



# 한 단 위아래 합본: (a) 추가 대기(확대 창 포함) + (b) 처리량 감소, 범례는 맨 위에 하나
fig, (b1, b2) = plt.subplots(2, 1, figsize=(88 * MM, 128 * MM))
LEGEND_B.clear()
panel_wait(b1, legend=False, inset=True)
panel_thr(b2)
b1.set_title('(a) 명령을 받기까지 추가 대기', fontsize=7)
b2.set_title('(b) 처리량 감소', fontsize=7)
# 범례는 (b) 왼쪽 위 빈 곳에 하나 (두 패널이 같은 기호)
b2.legend([h for h, _ in LEGEND_B], [n for _, n in LEGEND_B], loc='upper left', frameon=False, fontsize=5.6,
          handletextpad=0.3)
fig.set_size_inches(88 * MM, 122 * MM)
save(fig, 'figB_stacked', h_pad=1.4)

# 한 단 점 그림: 조건마다 한 줄, 예측(◇)과 실측(● + 95% CI)을 나란히. 대각선 그림보다 읽기 쉽다
DOT_LABEL = {'central_loss30': '손실 30%', 'central_loss50': '손실 50%', 'central_loss70': '손실 70%',
             'central_loss50_k1': '손실 50%, 재전송 1회', 'central_loss50_k0': '손실 50%, 재전송 0회',
             'short_loss50': '25초 작업, 손실 50%', 'short_loss70': '25초 작업, 손실 70%'}
DOT_ORDER = ['central_loss30', 'central_loss50', 'central_loss70',
             'central_loss50_k1', 'central_loss50_k0', 'short_loss50', 'short_loss70']


def dot_panel(ax, kind):
    by = {c: (W, gap, pred, thr) for _, _, _, _, W, gap, pred, thr, c in pts}
    ys = np.arange(len(DOT_ORDER))[::-1]
    for y, c in zip(ys, DOT_ORDER):
        W, gap, pred, thr = by[c]
        if kind == 'wait':
            pv, m, lo, hi = W, gap[0], gap[1], gap[2]
        else:   # 처리량 감소 (%): 부호를 바꿔 양수로
            pv, m, lo, hi = -pred, -thr[0], -thr[2], -thr[1]
        ax.plot([pv, m], [y, y], color='0.8', lw=1.2, zorder=1)
        ax.errorbar(m, y, xerr=[[m - lo], [hi - m]], fmt='o', ms=3.4, color='black', elinewidth=0.7, capsize=1.5,
                    zorder=3)
        ax.plot(pv, y, 'D', ms=4.2, mfc='white', mec='black', mew=0.8, zorder=4)
    # 사후 적합(위 3줄)과 사전 예측(아래 4줄) 사이 구분선과 이름
    yb = ys[2] - 0.5
    ax.axhline(yb, color='0.75', lw=0.6, ls='--')
    ax.text(0.995, yb + 0.06, '↑ 사후 적합', transform=ax.get_yaxis_transform(), ha='right', va='bottom',
            fontsize=5.4, color='0.4')
    ax.text(0.995, yb - 0.06, '↓ 사전 예측', transform=ax.get_yaxis_transform(), ha='right', va='top',
            fontsize=5.4, color='0.4')
    ax.set_yticks(ys)
    ax.set_yticklabels([DOT_LABEL[c] for c in DOT_ORDER], fontsize=6.2)
    ax.set_ylim(ys[-1] - 0.6, ys[0] + 0.6)
    ax.set_xlim(left=0)
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='x', color='0.92', lw=0.5)
    ax.set_axisbelow(True)
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], marker='D', ls='', mfc='white', mec='black', ms=4.2, label='예측 (식)'),
                       Line2D([], [], marker='o', ls='', color='black', ms=3.4, label='실측 (95% CI)')],
              loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=5.8,
              handletextpad=0.2, columnspacing=1.5, borderaxespad=0.1)


fig, ax = plt.subplots(figsize=(88 * MM, 62 * MM))
dot_panel(ax, 'wait')
ax.set_xlabel('명령을 받기까지 추가 대기 (초)')
save(fig, 'figB_wait_dot')
fig, ax = plt.subplots(figsize=(88 * MM, 62 * MM))
dot_panel(ax, 'thr')
ax.set_xlabel('처리량 감소 (%)')
save(fig, 'figB_throughput_dot')


# 한 단 막대판: 조건마다 예측(흰 막대)과 실측(회색 막대 + 95% CI)을 나란히
def bar_panel(ax, kind):
    by = {c: (W, gap, pred, thr) for _, _, _, _, W, gap, pred, thr, c in pts}
    ys = np.arange(len(DOT_ORDER))[::-1] * 1.0
    h = 0.36
    xr = 0
    for y, c in zip(ys, DOT_ORDER):
        W, gap, pred, thr = by[c]
        if kind == 'wait':
            pv, m, lo, hi = W, gap[0], gap[1], gap[2]
        else:
            pv, m, lo, hi = -pred, -thr[0], -thr[2], -thr[1]
        xr = max(xr, pv, hi)
        ax.barh(y + h / 2, pv, height=h, color='white', edgecolor='black', lw=0.6)
        ax.barh(y - h / 2, m, height=h, color='0.6', edgecolor='black', lw=0.6,
                xerr=[[m - lo], [hi - m]], error_kw={'elinewidth': 0.6, 'capsize': 1.5})
    yb = ys[2] - 0.5
    ax.axhline(yb, color='0.75', lw=0.6, ls='--')
    ax.text(0.995, yb + 0.06, '↑ 사후 적합', transform=ax.get_yaxis_transform(), ha='right', va='bottom',
            fontsize=5.4, color='0.4')
    ax.text(0.995, yb - 0.06, '↓ 사전 예측', transform=ax.get_yaxis_transform(), ha='right', va='top',
            fontsize=5.4, color='0.4')
    ax.set_yticks(ys)
    ax.set_yticklabels([DOT_LABEL[c] for c in DOT_ORDER], fontsize=6.2)
    ax.set_ylim(ys[-1] - 0.6, ys[0] + 0.6)
    ax.set_xlim(0, xr * 1.22)   # 오른쪽 끝에 '사후 적합/사전 예측' 글씨 자리
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='x', color='0.92', lw=0.5)
    ax.set_axisbelow(True)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor='white', edgecolor='black', lw=0.6, label='예측 (식)'),
                       Patch(facecolor='0.6', edgecolor='black', lw=0.6, label='실측 (95% CI)')],
              loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=5.8,
              handletextpad=0.3, columnspacing=1.5, borderaxespad=0.1)


fig, ax = plt.subplots(figsize=(88 * MM, 70 * MM))
bar_panel(ax, 'wait')
ax.set_xlabel('명령을 받기까지 추가 대기 (초)')
save(fig, 'figB_wait_bar')
fig, ax = plt.subplots(figsize=(88 * MM, 70 * MM))
bar_panel(ax, 'thr')
ax.set_xlabel('처리량 감소 (%)')
save(fig, 'figB_throughput_bar')

# ---------- 그림 C: 단절 로봇 정책 ----------
# 원고 3장 번호·이름과 같게: ① 정지 ② 허가 지점 대기 ③ 할당 작업 완수 ④ 자율 재할당(분산 경매, 상한).
# 결과가 아직 없는 정책(data/<라벨>이 없음)은 빼고 그린다
policies = [p for p in [('① 정지', 'central_bo30_r3_pause', '0.3', ''),
                        ('② 허가 지점\n대기', 'central_bo30_r3_leg', '0.5', ''),
                        ('③ 할당 작업\n완수', 'central_bo30_r3', '0.7', ''),
                        ('④ 자율 재할당\n(상한)', 'hybrid_bo30_r3', '0.92', '////')] if (D / p[1]).exists()]


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
        # 그림 A와 같게: 95% CI가 0을 포함하면 회색
        ax.text(i, max(m[2], 0) + 0.6, f'{m[0]:+.1f}%', ha='center', va='bottom', fontsize=6,
                color='black' if (m[1] > 0 or m[2] < 0) else '0.55')
    ax.axhline(10, color='black', lw=0.7, ls='--')
    ax.text(len(policies) - 0.58, 10.5, '급락 기준 +10%', fontsize=5.6, ha='right')
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
c2.set_title('(b) 전체 로봇의 평균 완료 시간 변화 (단절 없음 대비)', fontsize=7)
save(fig, 'figC_policy_wide')
fig, (c1, c2) = plt.subplots(2, 1, figsize=(88 * MM, 106 * MM))
panel_a3(c1)
panel_time(c2)
c1.set_title('(a) 단절 로봇(amr_03)의 처리량', fontsize=7)
c2.set_title('(b) 전체 로봇의 평균 완료 시간 변화 (단절 없음 대비)', fontsize=7)
save(fig, 'figC_policy', h_pad=1.6)
print('ok')
