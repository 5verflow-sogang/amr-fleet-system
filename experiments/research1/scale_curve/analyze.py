"""연구 1 규모 곡선 분석 (scale_curve.md 8절). 조건별 실행 단위 평균·부트스트랩 95% CI, N=5 대비 같은 seed 짝 비율.

    gsutil -m rsync -r -x '.*rosbag.*|.*/logs/.*|.*\\.png$' \\
        gs://amr-fleet-results/runs/fleet_fake-<라벨>/<날짜>_<run_id>/ <data>/<라벨>/     # runs.txt의 14개
    python3 experiments/research1/scale_curve/analyze.py <data>        # → <data>/summary.csv

<data>/<라벨>/ 아래 실행 폴더(fleet_fake-<라벨>_<시각>_NN)를 읽는다. 실행이 completed가 아니거나 러너 기종이
c2d-standard-8이 아니면 빼고 알린다(같은 기종 규칙). 5대는 A·B가 같은 조건이라 A의 값을 B에도 쓴다.
"""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools' / 'harness'))
from harness.stats import bootstrap_mean_ci, summarize_runs  # noqa: E402
from harness.summary import _floats, _read_csv, fleet_metrics  # noqa: E402

MACHINE = 'c2d-standard-8'
METRICS = ['fleet.latency_ms.mean', 'fleet.latency_ms.p95', 'fleet.latency_ms.p99', 'fleet.latency_ms.max',
           'fleet.loss_rate', 'fleet.ack_rtt_ms.mean', 'fleet.ack_rtt_ms.p95', 'fleet.ack_rtt_ms.max',
           'fleet.resends.mean', 'fleet.commands', 'fleet.ack_rate',
           'fleet.tasks_requested', 'fleet.tasks_completed', 'fleet.task_completion_rate',
           'fleet.task_time_s.mean', 'fleet.throughput_per_min', 'fleet.distance_m.mean', 'rtf']


def throughput_per_min(run_dir: Path):
    """완료 작업 수 / 첫 요청~마지막 완료(분). 하네스 fleet.throughput_per_min(#208)과 같은 정의."""
    tasks = _read_csv(run_dir / 'task_outcomes.csv')
    ends = _floats([r for r in tasks if r['final_state'] == 'COMPLETED'], 'end_time')
    requested = _floats(tasks, 'request_time')
    if requested and ends and max(ends) > min(requested):
        return len(ends) / ((max(ends) - min(requested)) / 60.0)
    return None


def load(data: Path):
    runs = defaultdict(dict)   # (plan, n, delay) -> seed -> metrics
    skipped = []
    for cond in sorted(p for p in data.iterdir() if p.is_dir() and p.name.startswith('scale_')):
        _, plan, n, d = cond.name.split('_')
        key = (plan, int(n[1:]), int(d[1:]))
        for r in sorted(cond.glob('fleet_fake-*_[0-9][0-9]')):
            meta = json.loads((r / 'meta.json').read_text())
            machine = (meta.get('host') or {}).get('runner_machine')
            if meta['outcome'] != 'completed' or machine != MACHINE:
                skipped.append((r.name, meta['outcome'], machine))
                continue
            m = fleet_metrics(r)
            m.setdefault('fleet.throughput_per_min', throughput_per_min(r))
            m['rtf'] = meta.get('real_time_factor')
            runs[key][meta['seed']] = m
    for d in (0, 100):
        runs[('b', 5, d)] = runs[('a', 5, d)]
    return runs, skipped


def summarize(runs):
    rows = []
    for (plan, n, d), by_seed in sorted(runs.items()):
        base = runs[(plan, 5, d)]
        for metric in METRICS:
            iv = summarize_runs([m[metric] for m in by_seed.values() if m.get(metric) is not None])
            if iv is None:
                continue
            ratios = [by_seed[s][metric] / base[s][metric] for s in by_seed
                      if by_seed[s].get(metric) is not None and base.get(s, {}).get(metric)]
            rlo, rhi = bootstrap_mean_ci(ratios) if len(ratios) > 1 else (None, None)
            rows.append({'plan': plan, 'n': n, 'delay_ms': d, 'metric': metric, 'runs': iv.n, 'mean': iv.mean,
                         'std': iv.std, 'ci_low': iv.ci_low, 'ci_high': iv.ci_high, 'min': iv.min, 'max': iv.max,
                         'ratio_vs_n5': sum(ratios) / len(ratios) if ratios else None,
                         'ratio_ci_low': rlo, 'ratio_ci_high': rhi})
    return rows


def main():
    data = Path(sys.argv[1])
    runs, skipped = load(data)
    rows = summarize(runs)
    out = data / 'summary.csv'
    with out.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('뺀 실행:', skipped or '없음')
    print('조건별 실행 수:', {f'{p}{n}_d{d}': len(v) for (p, n, d), v in sorted(runs.items())})
    print('저장:', out)


if __name__ == '__main__':
    main()
