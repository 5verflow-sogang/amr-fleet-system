"""도킹 KPI 실험 결과(results.csv) 요약: 스테이션 종류별 성공률과 GT 오차 분포.

    python3 analyze.py results.csv
"""

import csv
import math
import statistics as st
import sys

POS_KPI_CM = 2.0      # 명세: 도킹 2 cm
YAW_KPI_DEG = 1.0     # 명세: 1도


def quantile(values, p):
    v = sorted(values)
    k = (len(v) - 1) * p
    lo = math.floor(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def kind(station):
    return station.split('_')[0]   # rack / charger / dock


def summarize(rows, label):
    ok = [r for r in rows if r['success'] == '1']
    pos = [float(r['gt_pos_cm']) for r in ok]
    yaw = [abs(float(r['gt_yaw_deg'])) for r in ok]
    secs = [float(r['wall_s']) for r in ok]
    within = [r for r in ok if float(r['gt_pos_cm']) <= POS_KPI_CM and abs(float(r['gt_yaw_deg'])) <= YAW_KPI_DEG]
    print(f'| {label} | {len(rows)} | {len(ok)} | {len(within)} | '
          + (f'{st.median(pos):.2f} / {quantile(pos, 0.9):.2f} / {max(pos):.2f} | '
             f'{st.median(yaw):.2f} / {quantile(yaw, 0.9):.2f} / {max(yaw):.2f} | {st.median(secs):.0f} |'
             if ok else '- | - | - |'))


def main(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    print('| 스테이션 | 시도 | 성공 | KPI 안 (2 cm·1°) | 위치 오차 cm 중앙값 / p90 / 최대 | 방향 오차 ° 중앙값 / p90 / 최대 | 걸린 시간 s (중앙값) |')
    print('| --- | --- | --- | --- | --- | --- | --- |')
    for k in ('rack', 'charger', 'dock'):
        sel = [r for r in rows if kind(r['station']) == k]
        if sel:
            summarize(sel, k)
    summarize(rows, '전체')
    failed = [r for r in rows if r['success'] != '1']
    for r in failed:
        print(f'- 실패: {r["station"]} #{r["trial"]} code {r["error_code"]} {r["message"]}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'results.csv')
