"""연구 1 중앙집중 결과 분석. 사용: python3 analysis.py data
 (statistical_design.md 2·5절): seed끼리 짝지은 차이, 부트스트랩 CI, Wilcoxon, Holm."""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

D = Path(sys.argv[1])
BASE = 'central_d0'
ORDER = ['central_d0', 'central_d100', 'central_d200', 'central_d500', 'central_loss30', 'central_loss50',
         'central_loss70', 'central_bo3_r3', 'central_bo10_r3', 'central_bo30_r3']
rng = np.random.default_rng(0)


def rows(p):
    with open(p, newline='') as f:
        return list(csv.DictReader(f))


def run_metrics(run):
    meta = json.loads((run / 'meta.json').read_text())
    tasks = rows(run / 'task_outcomes.csv')
    resp = rows(run / 'response_time.csv')
    states = rows(run / 'robot_states.csv')
    done = {t['task_id']: t for t in tasks if t['final_state'] == 'COMPLETED'}
    # 배정 확정 지연: 요청 → 최종 담당 로봇의 첫 Ack (Ack 없는 명령은 닿지 않은 것)
    acks = defaultdict(dict)
    for r in resp:
        if r['ack_time']:
            acks[r['task_id']].setdefault(r['robot'], float(r['ack_time']))
    dispatch = {}
    for t in tasks:
        final = t['robots'].split()[-1] if t['robots'] else ''
        if final in acks.get(t['task_id'], {}) and t['request_time']:
            dispatch[t['task_id']] = acks[t['task_id']][final] - float(t['request_time'])
    # 로봇별 BUSY 비율 (RobotState mode 1)
    busy = defaultdict(lambda: [0, 0])
    for s in states:
        b = busy[s['robot']]
        b[1] += 1
        b[0] += s['mode'] == '1'
    by_robot = defaultdict(int)
    for t in done.values():
        by_robot[t['robots'].split()[-1]] += 1
    return {
        'seed': meta['seed'],
        'completed': len(done),
        'task_time': {k: float(v['task_time_s']) for k, v in done.items()},
        'dispatch': dispatch,
        'reassigned': sum(len(t['robots'].split()) > 1 for t in tasks),
        'busy': {r: b[0] / b[1] for r, b in busy.items()},
        'done_by': dict(by_robot),
        'ack_p95': float(np.percentile([float(r['ack_rtt_ms']) for r in resp if r['ack_rtt_ms']], 95)),
    }


def boot_ci(x, n=10000):
    x = np.asarray(x, float)
    means = rng.choice(x, (n, len(x))).mean(axis=1)
    return x.mean(), np.percentile(means, 2.5), np.percentile(means, 97.5)


def holm(ps):
    order = np.argsort(ps)
    adj = np.empty(len(ps))
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, (len(ps) - rank) * ps[i])
        adj[i] = min(1.0, running)
    return adj


data = {c: {m['seed']: m for m in (run_metrics(r) for r in sorted((D / c).iterdir()))} for c in ORDER}
base = data[BASE]

print('## 조건별 (실행 단위 평균, 괄호는 부트스트랩 95% CI)')
print('| 조건 | 완료 | 평균 완료 시간 s | 배정 확정 지연 평균 s | 배정 확정 지연 P95 s | 재배정 작업 | Ack P95 ms |')
print('| --- | --- | --- | --- | --- | --- | --- |')
for c in ORDER:
    runs = list(data[c].values())
    comp = boot_ci([r['completed'] for r in runs])
    tt = boot_ci([np.mean(list(r['task_time'].values())) for r in runs])
    dm = boot_ci([np.mean(list(r['dispatch'].values())) for r in runs])
    dp = boot_ci([np.percentile(list(r['dispatch'].values()), 95) for r in runs])
    ra = np.mean([r['reassigned'] for r in runs])
    ack = np.mean([r['ack_p95'] for r in runs])
    print(f'| {c} | {comp[0]:.1f} ({comp[1]:.1f}–{comp[2]:.1f}) | {tt[0]:.1f} ({tt[1]:.1f}–{tt[2]:.1f}) | '
          f'{dm[0]:.2f} ({dm[1]:.2f}–{dm[2]:.2f}) | {dp[0]:.2f} ({dp[1]:.2f}–{dp[2]:.2f}) | {ra:.1f} | {ack:.0f} |')

print()
print('## 지연 0 대비 짝지은 차이 (같은 seed, 둘 다 완료한 작업만으로 평균 완료 시간 비율)')
print('| 조건 | 완료 차이 | 완료 시간 비율 | 배정 확정 지연 차이 s | p(완료 시간, Holm) | +10% 급락? |')
print('| --- | --- | --- | --- | --- | --- |')
results = []
for c in ORDER[1:]:
    dc, ratio, ddisp = [], [], []
    for seed, r in data[c].items():
        b = base[seed]
        dc.append(r['completed'] - b['completed'])
        common = sorted(set(r['task_time']) & set(b['task_time']))
        ratio.append(np.mean([r['task_time'][k] for k in common]) / np.mean([b['task_time'][k] for k in common]))
        cd = sorted(set(r['dispatch']) & set(b['dispatch']))
        ddisp.append(np.mean([r['dispatch'][k] - b['dispatch'][k] for k in cd]))
    p = wilcoxon(np.asarray(ratio) - 1.0).pvalue if np.any(np.asarray(ratio) != 1.0) else 1.0
    results.append((c, boot_ci(dc), boot_ci(ratio), boot_ci(ddisp), p))
adj = holm([r[4] for r in results])
for (c, dc, ra, dd, _), pa in zip(results, adj):
    flag = '예' if ra[1] > 1.10 else '아니오'
    print(f'| {c} | {dc[0]:+.1f} ({dc[1]:+.1f}–{dc[2]:+.1f}) | {ra[0]:.3f} ({ra[1]:.3f}–{ra[2]:.3f}) | '
          f'{dd[0]:+.2f} ({dd[1]:+.2f}–{dd[2]:+.2f}) | {pa:.3f} | {flag} |')

print()
print('## 끊긴 로봇(amr_03)의 기여 (끊김 조건, 실행 평균)')
print('| 조건 | amr_03 완료 작업 | 다른 로봇 1대 평균 | amr_03 BUSY 비율 | 다른 로봇 BUSY 비율 |')
print('| --- | --- | --- | --- | --- |')
for c in ['central_d0', 'central_bo3_r3', 'central_bo10_r3', 'central_bo30_r3']:
    runs = list(data[c].values())
    a = np.mean([r['done_by'].get('amr_03', 0) for r in runs])
    o = np.mean([np.mean([v for k, v in r['done_by'].items() if k != 'amr_03'] or [0]) for r in runs])
    ba = np.mean([r['busy'].get('amr_03', 0) for r in runs])
    bo = np.mean([np.mean([v for k, v in r['busy'].items() if k != 'amr_03']) for r in runs])
    print(f'| {c} | {a:.1f} | {o:.1f} | {ba:.2f} | {bo:.2f} |')
