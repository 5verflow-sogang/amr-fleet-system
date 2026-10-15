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
    gz_rows = list(csv.DictReader(open(gz_path, encoding='utf-8')))
    # 장애물 배치가 틀린 에피소드(valid False, 다시 해도 안 된 것)는 뺀다. valid 열이 없는 예전 CSV는 모두 쓴다
    invalid = [r for r in gz_rows if r.get('valid', 'True') != 'True']
    gz = {key(r): r for r in gz_rows if r.get('valid', 'True') == 'True'}
    d2 = {key(r): r for r in csv.DictReader(open(d2_path, encoding='utf-8'))}
    pairs = [(gz[k], d2[k]) for k in sorted(gz) if k in d2]
    retried = sum(int(r.get('attempts') or 1) > 1 for r in gz_rows)
    print(f'짝 {len(pairs)}개 (Gazebo {len(gz)}, 2D {len(d2)}), 무효로 뺀 Gazebo {len(invalid)}개, 다시 한 에피소드 {retried}개\n')

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

    # 짝마다 (#144 리뷰 3: 결과 일치 수만으로는 약하다. 둘 다 충돌하는 에피소드는 어차피 같게 나온다)
    stopped = [(g, d) for g, d in pairs if g['popout_result'] == d['popout_result'] == 'stopped']
    print('\n둘 다 멈춘 에피소드, 짝마다 (제동 거리 = 나타난 순간 → 멈춤)\n')
    print('| 질량 kg | 에피소드 | 출발 속도 m/s | 간격 m (시나리오 / Gazebo 실제) | 제동 거리 Gazebo / 2D m | 차 m | 남은 여유 Gazebo / 2D m |')
    print('| --- | --- | --- | --- | --- | --- | --- |')
    for g, d in stopped:
        print(f'| {key(g)[0]:g} | {key(g)[2]} | {num(g["initial_speed"]):.2f} | '
              f'{num(g["popout_gap"]):.3f} / {num(g["popout_gap_actual"]):.3f} | '
              f'{num(g["popout_stop_distance"]):.3f} / {num(d["popout_stop_distance"]):.3f} | '
              f'{num(g["popout_stop_distance"]) - num(d["popout_stop_distance"]):+.3f} | '
              f'{num(g["popout_stop_margin"]):.3f} / {num(d["popout_stop_margin"]):.3f} |')

    both_hit = [(g, d) for g, d in pairs if g['outcome'] == d['outcome'] == 'collision'
                and num(g['impact_speed']) is not None and num(d['impact_speed']) is not None]
    print('\n둘 다 충돌한 에피소드: 충돌 속도 (같은 결과라도 얼마나 세게 부딪혔나)\n')
    print('| 질량 kg | 짝 | 충돌 속도 Gazebo m/s (평균) | 2D m/s (평균) | 차 m/s (평균 / 최대) |')
    print('| --- | --- | --- | --- | --- |')
    for payload in sorted({k[0] for k in gz}):
        sel = [(g, d) for g, d in both_hit if key(g)[0] == payload]
        if not sel:
            continue
        diff = [num(g['impact_speed']) - num(d['impact_speed']) for g, d in sel]
        print(f'| {payload:g} | {len(sel)} | {st.mean(num(g["impact_speed"]) for g, _ in sel):.3f} | '
              f'{st.mean(num(d["impact_speed"]) for _, d in sel):.3f} | '
              f'{st.mean(diff):+.3f} / {max(diff, key=abs):+.3f} |')

    print()
    differ = [(g, d) for g, d in pairs if g['outcome'] != d['outcome']]
    for g, d in differ:
        print(f'- 다름: {key(g)} Gazebo {g["outcome"]}({g["popout_result"]}) / 2D {d["outcome"]}({d["popout_result"]}), '
              f'설 수 있었나 {g["popout_stoppable"]}, 막힘 {g["popout_blocked"]}')


if __name__ == '__main__':
    main(*sys.argv[1:3])
