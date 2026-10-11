"""Gazebo 평가 결과와 같은 정책의 2D 결과를 (질량, 시나리오, 에피소드)로 짝지어 비교한다.

    python3 compare.py gazebo.csv 2d.csv
"""

import csv
import math
import statistics as st
import sys


def key(r):
    return (round(float(r['payload_kg']), 1), r['scenario'], int(r['episode']))


def num(v):
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def main(gz_path, d2_path):
    gz = {key(r): r for r in csv.DictReader(open(gz_path, encoding='utf-8'))}
    d2 = {key(r): r for r in csv.DictReader(open(d2_path, encoding='utf-8'))}
    pairs = [(gz[k], d2[k]) for k in sorted(gz) if k in d2]
    print(f'짝 {len(pairs)}개 (Gazebo {len(gz)}, 2D {len(d2)})\n')

    print('| 질량 kg | 시나리오 | 에피소드 | 결과 같음 | 성공 Gazebo / 2D | 충돌 Gazebo / 2D | 소요 시간 차 s (Gazebo − 2D, 중앙값) |')
    print('| --- | --- | --- | --- | --- | --- | --- |')
    for payload in sorted({k[0] for k in gz}):
        for scen in ('S1', 'S2'):
            sel = [(g, d) for g, d in pairs if key(g)[0] == payload and g['scenario'] == scen]
            if not sel:
                continue
            same = sum(g['outcome'] == d['outcome'] for g, d in sel)
            succ = (sum(g['outcome'] == 'success' for g, _ in sel), sum(d['outcome'] == 'success' for _, d in sel))
            coll = (sum(g['outcome'] == 'collision' for g, _ in sel), sum(d['outcome'] == 'collision' for _, d in sel))
            dt = [num(g['time_s']) - num(d['time_s']) for g, d in sel
                  if g['outcome'] == d['outcome'] == 'success']
            print(f'| {payload:g} | {scen} | {len(sel)} | {same}/{len(sel)} | {succ[0]} / {succ[1]} | '
                  f'{coll[0]} / {coll[1]} | {st.median(dt):+.2f} |' if dt else
                  f'| {payload:g} | {scen} | {len(sel)} | {same}/{len(sel)} | {succ[0]} / {succ[1]} | '
                  f'{coll[0]} / {coll[1]} | - |')

    print('\n돌발 장애물 앞에서 멈춘 에피소드 (둘 다 stopped): 제동 거리 (나타난 순간 → 멈춤)\n')
    print('| 질량 kg | 짝 | 제동 거리 Gazebo m (평균) | 2D m (평균) | 차 m (Gazebo − 2D, 평균 / 최대) | 간격 차 m (실제 − 시나리오, 최대) |')
    print('| --- | --- | --- | --- | --- | --- |')
    for payload in sorted({k[0] for k in gz}):
        sel = [(g, d) for g, d in pairs if key(g)[0] == payload and g['popout_result'] == d['popout_result'] == 'stopped']
        if not sel:
            print(f'| {payload:g} | 0 | - | - | - | - |')
            continue
        gs = [num(g['popout_stop_distance']) for g, _ in sel]
        ds = [num(d['popout_stop_distance']) for _, d in sel]
        diff = [a - b for a, b in zip(gs, ds)]
        gap = [abs(num(g['popout_gap_actual']) - num(g['popout_gap'])) for g, _ in sel]
        print(f'| {payload:g} | {len(sel)} | {st.mean(gs):.3f} | {st.mean(ds):.3f} | '
              f'{st.mean(diff):+.3f} / {max(diff, key=abs):+.3f} | {max(gap):.3f} |')

    differ = [(g, d) for g, d in pairs if g['outcome'] != d['outcome']]
    for g, d in differ:
        print(f'- 다름: {key(g)} Gazebo {g["outcome"]}({g["popout_result"]}) / 2D {d["outcome"]}({d["popout_result"]}), '
              f'설 수 있었나 {g["popout_stoppable"]}, 막힘 {g["popout_blocked"]}')


if __name__ == '__main__':
    main(*sys.argv[1:3])
