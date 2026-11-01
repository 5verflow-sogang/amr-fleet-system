#!/usr/bin/env python3
"""본 측정 결과 → perception.md 1-7 결과 표 (마크다운). 숫자를 손으로 옮기지 않으려고 쓴다.

    python3 make_tables.py <결과 폴더> <요약 JSON>

- 결과 폴더: fetch_runs.py --out으로 받은 폴더 (장면별 하위 폴더)
- 요약 JSON: tools/harness/summarize.py --json 출력 (판정에 쓴 실행만 넣어 만든 것, README)
- 표 다섯 개: 판정, 보고 지표, 동적 판정 지연, 접근 판정, 가짜 동적 트랙
"""
import csv
import json
import statistics
import sys
from pathlib import Path

GROUP = {'A': 'tracking_kpi_a/main', 'B': 'tracking_kpi_b/main_rerun', 'C05': 'tracking_kpi_c05/main',
         'C10': 'tracking_kpi_c10/main', 'C15': 'tracking_kpi_c15/main', 'D': 'tracking_kpi_d/main',
         'E': 'tracking_kpi_e/main'}
FOLDER = {'A': 'tracking_kpi_a-main', 'B': 'tracking_kpi_b-main_rerun', 'C05': 'tracking_kpi_c05-main',
          'C10': 'tracking_kpi_c10-main', 'C15': 'tracking_kpi_c15-main', 'D': 'tracking_kpi_d-main',
          'E': 'tracking_kpi_e-main'}
NAME = {'A': 'A 메인 통로', 'B': 'B B–C 통로', 'C05': 'C 정면 0.5 m/s', 'C10': 'C 정면 1.0 m/s',
        'C15': 'C 정면 1.5 m/s', 'D': 'D 가려짐', 'E': 'E 로봇 주행'}
BINS = {'<0.6': '< 0.6 m/s', '0.6-1.0': '0.6~1.0 m/s', '>=1.0': '≥ 1.0 m/s'}


class Results:
    def __init__(self, root: Path, summary: dict):
        self.root, self.summary = root, summary

    def metric(self, scene, name):
        return self.summary[GROUP[scene]]['metrics'][name]

    def mean(self, scene, name):
        return self.metric(scene, name)['mean']

    def ci(self, scene, name, digits=2):
        x = self.metric(scene, name)
        return f"{x['mean']:.{digits}f} [{x['ci_low']:.{digits}f}, {x['ci_high']:.{digits}f}]"

    def runs(self, scene):
        return sorted((self.root / FOLDER[scene]).glob('*/*_0[0-9]'))

    def run_summary(self, run):
        return json.loads((run / 'tracking/summary.json').read_text())


def verdict_table(r):
    print('판정\n')
    print('| 기준 | 장면 | 값: 평균 [95% CI] | 통과 조건 | 판정 |\n| --- | --- | --- | --- | --- |')
    for k in 'ABDE':
        ok = r.metric(k, 'track.pred_gain_1s')['ci_low'] > 0
        print(f"| P1 예측 이득 (+1 s) [m] | {NAME[k]} | {r.ci(k, 'track.pred_gain_1s')} | CI 하한 > 0 | "
              f"{'통과' if ok else '**실패**'} |")
    for k in 'ABDE':
        ok = r.metric(k, 'track.vel.rmse')['ci_high'] <= 0.2
        print(f"| P2 속도 RMSE [m/s] | {NAME[k]} | {r.ci(k, 'track.vel.rmse', 3)} | CI 상한 ≤ 0.2 | "
              f"{'통과' if ok else '**실패**'} |")
    for k in ('C10', 'C15'):
        x = r.metric(k, 'track.d1_ttc_s')
        ok = x['min'] >= 3 and x['n'] == 10
        print(f"| D1 판정 순간 TTC [s] | {NAME[k]} | 최소 {x['min']:.2f} (평균 {x['mean']:.2f}, {x['n']}회) | "
              f"모든 실행 ≥ 3 | {'통과' if ok else '**실패**'} |")


def report_table(r):
    print('\n보고 지표 (실행 10회 평균)\n')
    print('| 장면 | 범위 안 물체 | 범위 안 짝 비율 | 위치 RMSE [m] | 센서 방향 편향 [m] | 속도 RMSE [m/s] | 방향 오차 [°] '
          '| +1 s / +2 s 예측 RMSE [m] (정지 가정) | NEES 위치 평균 | ID 전환 [회/실행] |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for k in GROUP:
        objs, rates = set(), []
        for run in r.runs(k):
            per = r.run_summary(run)['per_object']
            objs |= {g for g, v in per.items() if v.get('frames_in_range', 0) > 0}
            # 정의대로: 12 m 안 GT 프레임 중 짝 (하네스 track.match_rate는 분자에 12 m 밖 짝까지 들어간다)
            with open(run / 'tracking/frames.csv') as f:
                rows = [x for x in csv.DictReader(f) if x['kind'] in ('match', 'miss') and x['in_range'] == '1']
            rates.append(sum(x['kind'] == 'match' for x in rows) / len(rows))
        mean = lambda name: r.mean(k, name)  # noqa: E731
        print(f"| {NAME[k]} | {', '.join(f'`{o}`' for o in sorted(objs))} | {statistics.mean(rates):.0%} | "
              f"{mean('track.pos.rmse'):.2f} | {mean('track.radial.mean'):+.2f} | {mean('track.vel.rmse'):.2f} | "
              f"{mean('track.heading.mean'):.1f} | {mean('track.pred_1s.rmse'):.2f} / {mean('track.pred_2s.rmse'):.2f} "
              f"({mean('track.hold_1s.rmse'):.2f} / {mean('track.hold_2s.rmse'):.2f}) | "
              f"{mean('track.nees_pos.mean'):.1f} | {mean('track.id_switches'):.1f} |")


def latency_table(r):
    print('\n동적 판정 지연 (A·B·D·E 합침, 구간 전부터 동적·기록 전부터 구간은 뺌)\n')
    print('| GT 속력 | 동적 판정 / 보인 구간 (전체 구간) | 지연 평균 (최대) [s] | 판정 시 거리 중앙 [m] |\n'
          '| --- | --- | --- | --- |')
    for b, label in BINS.items():
        n = judged = seen = 0
        latency, dist = [], []
        for k in 'ABDE':
            for run in r.runs(k):
                with open(run / 'tracking/episodes.csv') as f:
                    for e in csv.DictReader(f):
                        if e['speed_bin'] != b or e['censored'] == '1' or e['carried'] == '1':
                            continue
                        n += 1
                        seen += e['t_seen'] != ''
                        if e['latency_from_seen'] != '':
                            judged += 1
                            latency.append(float(e['latency_from_seen']))
                            dist.append(float(e['dynamic_dist']))
        print(f'| {label} | {judged} / {seen} ({n}) | {statistics.mean(latency):.2f} ({max(latency):.2f}) | '
              f'{statistics.median(dist):.1f} |')


def approach_table(r):
    print('\n접근 판정 (장면 C, 실행 10회)\n')
    print('| 작업자 속력 | 판정 순간 TTC 최소 (평균) [s] | 판정 시 거리 평균 [m] | 지연 평균 [s] |\n| --- | --- | --- | --- |')
    for k, label in (('C05', '0.5 m/s (판정 안 함)'), ('C10', '1.0 m/s'), ('C15', '1.5 m/s')):
        ttc = r.metric(k, 'track.d1_ttc_s')
        print(f"| {label} | {ttc['min']:.2f} ({ttc['mean']:.2f}) | {r.mean(k, 'track.d1_dist_m'):.2f} | "
              f"{r.mean(k, 'track.d1_latency_s'):.2f} |")


def false_table(r):
    print('\n가짜 동적 트랙 (실행 10회 평균, 정지 횟수는 10회 합)\n')
    print('| 장면 | 가짜 동적 트랙 / 프레임 | GT 옆 중복 동적 / 프레임 | 가짜 때문에 감속·정지 (그중 정지) [%] '
          '| 가짜 때문에 정지 [회] |\n| --- | --- | --- | --- | --- |')
    for k in GROUP:
        stops = sum(r.run_summary(run)['false_dynamic_safety']['stop_events'] for run in r.runs(k))
        print(f"| {NAME[k]} | {r.mean(k, 'track.false_dynamic_per_frame'):.3f} | "
              f"{r.mean(k, 'track.duplicate_dynamic_per_frame'):.3f} | {r.mean(k, 'track.false_limited_frac'):.1%} "
              f"({r.mean(k, 'track.false_stop_frac'):.1%}) | {stops} |")


def main(argv):
    r = Results(Path(argv[1]).expanduser(), json.loads(Path(argv[2]).expanduser().read_text()))
    verdict_table(r)
    report_table(r)
    latency_table(r)
    approach_table(r)
    false_table(r)
    metas = [json.loads((run / 'meta.json').read_text()) for k in GROUP for run in r.runs(k)]
    rtf = [m['real_time_factor'] for m in metas]
    print(f"\n실행 {len(metas)}개, 커밋 {sorted({m['git_sha'][:7] for m in metas})}, RTF {min(rtf):.3f}~{max(rtf):.3f}")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
