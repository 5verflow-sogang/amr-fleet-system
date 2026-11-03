"""두 조건을 같은 seed끼리 짝지어 비교. 사용: python3 compare.py data central_bo30_r3 hybrid_bo30_r3"""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

D = Path(sys.argv[1])
A, B = sys.argv[2], sys.argv[3]
rng = np.random.default_rng(0)


def rows(p):
    if not p.exists():
        return []
    with open(p, newline='') as f:
        return list(csv.DictReader(f))


def metrics(run):
    meta = json.loads((run / 'meta.json').read_text())
    tasks = rows(run / 'task_outcomes.csv')
    resp = rows(run / 'response_time.csv')
    done = {t['task_id']: float(t['task_time_s']) for t in tasks if t['final_state'] == 'COMPLETED'}
    by = defaultdict(int)
    for t in tasks:
        if t['final_state'] == 'COMPLETED':
            # 실제로 끝낸 로봇: IN_PROGRESS 보고(runners) 중 마지막, 없으면 마지막 배정
            who = (t.get('runners') or t['robots']).split()
            by[who[-1] if who else ''] += 1
    busy = defaultdict(lambda: [0, 0])
    for s in rows(run / 'robot_states.csv'):
        busy[s['robot']][1] += 1
        busy[s['robot']][0] += s['mode'] == '1'
    # amr_03 작업 사이 노는 시간: 끝낸 시각 → 다음 작업 IN_PROGRESS가 아니라 다음 BUSY 시작으로 잰다 (출처 무관)
    st = [s for s in rows(run / 'robot_states.csv') if s['robot'] == 'amr_03']
    gaps, idle_start, started = [], None, False
    for s in st:
        b = s['mode'] == '1'
        started = started or b
        if started and not b and idle_start is None:
            idle_start = float(s['time'])
        elif b and idle_start is not None:
            gaps.append(float(s['time']) - idle_start)
            idle_start = None
    # 논문 3.3의 작업 사이 대기: amr_03이 작업을 끝낸 시각 → 다음 명령의 Ack 시각 (a3_gap과 다른 정의)
    a3_acks = sorted(float(r['ack_time']) for r in resp if r['robot'] == 'amr_03' and r['ack_time'])
    a3_ends = [float(t['end_time']) for t in tasks if t['final_state'] == 'COMPLETED'
               and (t.get('runners') or t['robots']).split()[-1:] == ['amr_03']]
    waits = [min(x for x in a3_acks if x > e) - e for e in a3_ends if any(x > e for x in a3_acks)]
    modes = rows(run / 'fleet_modes.csv')
    return {
        'a3_wait': np.mean(waits) if waits else 0.0,
        'seed': meta['seed'], 'completed': len(done), 'task_time': done,
        # runners 열이 생기기 전 실행(central_d0·d100·d200·d500)은 중복을 셀 수 없어 nan으로 둔다 (0이 아님)
        'dup': sum(len((t.get('runners') or '').split()) > 1 for t in tasks) if tasks and 'runners' in tasks[0] else np.nan,
        'a3_done': by.get('amr_03', 0),
        'a3_busy': busy['amr_03'][0] / max(busy['amr_03'][1], 1),
        'others_busy': np.mean([v[0] / v[1] for k, v in busy.items() if k != 'amr_03']),
        'a3_gap': np.mean(gaps) if gaps else 0.0,
        'fallbacks': sum(m['mode'] == 'decentral' for m in modes),
        'decentral_s': sum(float(m['duration_s']) for m in modes if m['mode'] == 'decentral'),
        'ack_p95': float(np.percentile([float(r['ack_rtt_ms']) for r in resp if r['ack_rtt_ms']] or [0], 95)),
    }


def load(c):
    return {m['seed']: m for m in (metrics(r) for r in sorted((D / c).iterdir()))}


def ci(x):
    x = np.asarray(x, float)
    m = rng.choice(x, (10000, len(x))).mean(axis=1)
    return f'{x.mean():.2f} [{np.percentile(m, 2.5):.2f}, {np.percentile(m, 97.5):.2f}]'


a, b = load(A), load(B)
seeds = sorted(set(a) & set(b))
print(f'{A} vs {B} (seed {len(seeds)}개)')
for key in ['completed', 'a3_done', 'a3_busy', 'others_busy', 'a3_wait', 'a3_gap', 'dup', 'fallbacks', 'decentral_s', 'ack_p95']:
    print(f'  {key:12s} {A}: {ci([a[s][key] for s in seeds])}   {B}: {ci([b[s][key] for s in seeds])}   '
          f'차이(B-A): {ci([b[s][key] - a[s][key] for s in seeds])}')
ratio = []
for s in seeds:
    common = sorted(set(a[s]['task_time']) & set(b[s]['task_time']))
    ratio.append(np.mean([b[s]['task_time'][k] for k in common]) / np.mean([a[s]['task_time'][k] for k in common]))
print(f'  완료 시간 비율(B/A, 둘 다 끝낸 작업): {ci(ratio)}')
