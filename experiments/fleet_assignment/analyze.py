"""운용 실험(가짜 로봇)에서 세 할당 알고리즘 비교. 사용: python3 analyze.py data
같은 seed끼리 최소거리(nearest) 대비 차이를 부트스트랩 95% CI(10,000회)로 낸다. 표준 라이브러리만 쓴다."""
import csv
import json
import random
import statistics as st
import sys
from collections import Counter
from pathlib import Path

D = Path(sys.argv[1])
random.seed(0)


def nums(xs):
    return [float(v) for v in xs if v]


def run_stats(run):
    tasks = list(csv.DictReader(open(run / 'task_outcomes.csv')))
    assigns = [r for r in csv.DictReader(open(run / 'assignments.csv')) if r['robot_id']]   # 짝지은 행만
    done = [t for t in tasks if t['final_state'] == 'COMPLETED']
    per_call = Counter(r['call_id'] for r in assigns)
    return {'seed': json.loads((run / 'meta.json').read_text())['seed'], 'done': len(done),
            'dist': st.mean(nums(t['distance_m'] for t in done)), 'time': st.mean(nums(t['task_time_s'] for t in done)),
            'multi': sum(n > 1 for n in per_call.values()), 'calls': len(per_call)}


def load(cond):
    return {s['seed']: s for s in (run_stats(r) for r in sorted((D / cond).iterdir()))}


def ci(v):
    ms = sorted(st.mean(random.choices(v, k=len(v))) for _ in range(10000))
    return f'{st.mean(v):.2f} [{ms[249]:.2f}, {ms[9749]:.2f}]'


for sc in ('base', 'batch'):
    ref = load(f'alg_{sc}_nearest')
    print(f'== {sc}')
    for alg in ('nearest', 'load_balance', 'hungarian'):
        x = load(f'alg_{sc}_{alg}')
        ks = sorted(set(ref) & set(x))
        print(f'{alg:13s} 완료 {ci([x[s]["done"] for s in ks])}  작업당 이동 {ci([x[s]["dist"] for s in ks])} m  '
              f'완료 시간 {ci([x[s]["time"] for s in ks])} s  2건 이상 짝지은 호출 {sum(x[s]["multi"] for s in ks)}'
              f'/{sum(x[s]["calls"] for s in ks)}')
        if alg != 'nearest':
            print(f'{"":13s} 최소거리 대비 Δ이동 {ci([x[s]["dist"] - ref[s]["dist"] for s in ks])} m  '
                  f'Δ완료 시간 {ci([x[s]["time"] - ref[s]["time"] for s in ks])} s')
