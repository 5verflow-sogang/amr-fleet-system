"""드리프트 실험 결과(results.csv) 요약: 위치·방향 오차 분포, 보고 공분산과 비교, 직진·회전 성분 분리.

    python3 analyze.py results.csv [results_d9_before.csv ...]
"""

import csv
import math
import statistics as st
import sys


def quantile(values, p):
    v = sorted(values)
    k = (len(v) - 1) * p
    lo = math.floor(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def seed_of(row):
    return row['label'].split(';')[1].split('=')[1]


def summarize(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    base = [r for r in rows if 'noise=false' in r['label']]
    noisy = [r for r in rows if 'noise=true' in r['label']]
    pos = [float(r['err_pos_cm']) for r in noisy]
    pct = [float(r['err_pos_pct']) for r in noisy]
    yaw = [abs(float(r['err_yaw_deg'])) for r in noisy]
    print(f'== {path}: 노이즈 켬 {len(noisy)}회, 주행 {float(noisy[0]["path_m"]):.1f} m')
    for r in base:
        print(f'  노이즈 끔 ({r["dir"]}): 위치 {r["err_pos_cm"]} cm ({r["err_pos_pct"]}%), 방향 {r["err_yaw_deg"]}°')
    print('  | 통계 | 위치 cm | 이동 거리 대비 % | 방향 ° |')
    for name, f in (('중앙값', st.median), ('평균', st.mean), ('상위 10% (p90)', lambda v: quantile(v, 0.9)),
                    ('최대', max)):
        print(f'  | {name} | {f(pos):.0f} | {f(pct):.2f} | {f(yaw):.1f} |')

    # 보고 공분산 대비 실제 오차 (NEES): 위치 2자유도 기대 2, 방향 기대 1
    nees_pos = [(float(r['err_x_cm']) / float(r['rep_sx_cm'])) ** 2
                + (float(r['err_y_cm']) / float(r['rep_sy_cm'])) ** 2 for r in noisy]
    nees_yaw = [(float(r['err_yaw_deg']) / float(r['rep_syaw_deg'])) ** 2 for r in noisy]
    print(f'  보고 1σ: 위치 x·y 중앙값 {st.median([float(r["rep_sx_cm"]) for r in noisy]):.0f}·'
          f'{st.median([float(r["rep_sy_cm"]) for r in noisy]):.0f} cm, 방향 {st.median([float(r["rep_syaw_deg"]) for r in noisy]):.1f}°')
    print(f'  NEES 위치 (기대 2): 중앙값 {st.median(nees_pos):.2f}, 평균 {st.mean(nees_pos):.2f} / '
          f'방향 (기대 1): 중앙값 {st.median(nees_yaw):.2f}, 평균 {st.mean(nees_yaw):.2f}')

    # UMBmark: 같은 seed의 반시계·시계 방향 오차 합/2 = 직진 중 쌓인 성분(바퀴 반지름 차이), 차/2 = 회전량 성분
    by = {}
    for r in noisy:
        by.setdefault(seed_of(r), {})[r['dir']] = float(r['err_yaw_deg'])
    straight = [abs(v['ccw'] + v['cw']) / 2 for v in by.values() if len(v) == 2]
    turn = [abs(v['ccw'] - v['cw']) / 2 for v in by.values() if len(v) == 2]
    if straight:
        print(f'  방향 오차 성분 (seed {len(straight)}개): 직진 중 쌓인 것 중앙값 {st.median(straight):.1f}°, '
              f'회전 중 쌓인 것 중앙값 {st.median(turn):.1f}°')
    return {'pos': pos, 'pct': pct, 'yaw': yaw}


if __name__ == '__main__':
    for p in sys.argv[1:] or ['results.csv']:
        summarize(p)
        print()
