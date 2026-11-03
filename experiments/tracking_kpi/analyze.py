#!/usr/bin/env python3
"""추적 KPI 본 측정의 결과 뒤 분석 (perception.md 1-7 "P2 실패 분석"과 보고 지표 설명의 숫자). 판정에는 쓰지 않는다.

    python3 analyze.py <결과 폴더>    # fetch_runs.py --out으로 받은 폴더 (장면별 하위 폴더)

1. P2 원인 분해: 속도 오차 중앙값·95%, 오차² 몫(방향 전환 ±1 s / 트랙 처음 짝 뒤 1 s / 나머지), 나머지만의 RMSE,
   12 m 밖 짝 비율과 12 m 안 짝만의 RMSE, 물체별 RMSE
2. 물체별 ID 전환·NEES (B·D·E)
3. 장면 C: 출발 거리 → 처음 짝지어진 거리 → 판정 거리
4. 장면 C 가짜 동적 트랙: 반환 뒤 3 s 안 비율, 그전에 작업자와 짝지어졌던 트랙 비율, GT·차체 거리
5. 판정 안 된 구간: 한 번도 안 보임 / 보였는데 판정 안 됨 / 판정 (A·B·D·E)
"""
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SCENES = {'A': 'tracking_kpi_a-main', 'B': 'tracking_kpi_b-main_rerun', 'D': 'tracking_kpi_d-main',
          'E': 'tracking_kpi_e-main', 'C05': 'tracking_kpi_c05-main', 'C10': 'tracking_kpi_c10-main',
          'C15': 'tracking_kpi_c15-main'}
TURN_WINDOW_S = 1.0       # 방향 전환 앞뒤
TURN_LAG_ROWS = 5         # GT 속도 비교 간격 (프레임 10 Hz → 0.5 s)
TURN_SPEED_CHANGE = 0.3   # m/s
TURN_ANGLE_DEG = 60.0
YOUNG_S = 1.0             # 트랙이 처음 짝지어진 뒤


def runs(root, scene):
    return sorted((root / SCENES[scene]).glob('*/*_0[0-9]'))


def frames(run):
    with open(run / 'tracking/frames.csv') as f:
        return list(csv.DictReader(f))


def angle(a, b):
    d = math.atan2(a[1], a[0]) - math.atan2(b[1], b[0])
    return abs((d + math.pi) % (2 * math.pi) - math.pi)


def turn_events(rows):
    """GT마다 속력이 0.3 m/s 넘게 또는 방향이 60° 넘게 바뀐 시각 (0.5 s 간격 비교)."""
    seqs = defaultdict(list)
    for x in rows:
        if x['kind'] in ('match', 'miss') and x['gt_vx'] != '':
            seqs[x['gt_id']].append((float(x['t']), float(x['gt_vx']), float(x['gt_vy'])))
    events = defaultdict(list)
    for g, seq in seqs.items():
        seq.sort()
        for (t0, vx0, vy0), (t1, vx1, vy1) in zip(seq, seq[TURN_LAG_ROWS:]):
            s0, s1 = math.hypot(vx0, vy0), math.hypot(vx1, vy1)
            if abs(s1 - s0) > TURN_SPEED_CHANGE or (
                    min(s0, s1) > 0.1 and angle((vx0, vy0), (vx1, vy1)) > math.radians(TURN_ANGLE_DEG)):
                events[g].append((t0 + t1) / 2)
    return events


def p2_breakdown(root):
    print('## 1. P2 원인 분해 (오차² 몫)')
    print('장면 | RMSE | 중앙값 | 95% | 방향 전환 ±1 s (시점 비율) | 처음 짝 뒤 1 s (시점 비율) | 나머지 | 나머지만 RMSE '
          '| 12 m 밖 짝 | 12 m 안만 RMSE | 물체별 RMSE')
    for scene in SCENES:
        err, turn, young, inside, by_obj = [], [], [], [], defaultdict(list)
        out_share, in_rmse = [], []   # 실행마다 내서 평균한다 (P2 판정과 같은 방식)
        for run in runs(root, scene):
            rows = frames(run)
            events = turn_events(rows)
            match = [x for x in rows if x['kind'] == 'match' and x['vel_err'] != '']
            first = {}
            for x in match:
                first.setdefault(x['track_id'], float(x['t']))
            out_share.append(sum(x['in_range'] != '1' for x in match) / len(match))
            in_rmse.append(math.sqrt(np.mean([float(x['vel_err']) ** 2 for x in match if x['in_range'] == '1'])))
            for x in match:
                t, e, g = float(x['t']), float(x['vel_err']), x['gt_id']
                err.append(e)
                by_obj[g].append(e)
                turn.append(any(abs(t - te) <= TURN_WINDOW_S for te in events[g]))
                young.append(t - first[x['track_id']] < YOUNG_S)
                inside.append(x['in_range'] == '1')
        e = np.array(err)
        turn = np.array(turn)
        young = np.array(young) & ~turn
        rest = ~turn & ~young
        sq = e ** 2
        tot = sq.sum()
        objs = ', '.join(f'{g} {math.sqrt(np.mean(np.square(v))):.2f}' for g, v in sorted(by_obj.items()))
        print(f'{scene} | {math.sqrt(sq.mean()):.3f} | {np.median(e):.3f} | {np.percentile(e, 95):.2f} | '
              f'{sq[turn].sum() / tot:.0%} ({turn.mean():.0%}) | {sq[young].sum() / tot:.0%} ({young.mean():.0%}) | '
              f'{sq[rest].sum() / tot:.0%} | {math.sqrt(sq[rest].mean()):.3f} | {np.mean(out_share):.0%} | '
              f'{np.mean(in_rmse):.3f} | {objs}')
    print('(오차² 몫·나머지만 RMSE·물체별 RMSE는 장면 안 모든 짝을 모아 냈고, 12 m 밖 짝 비율·12 m 안만 RMSE는 '
          '실행마다 내서 평균했다 - P2 판정과 같은 방식)')


def objects(root):
    print('\n## 2. 물체별 ID 전환 [회/실행]·NEES 위치 평균')
    for scene in ('B', 'D', 'E'):
        switches, nees = defaultdict(list), defaultdict(list)
        for run in runs(root, scene):
            per = json.loads((run / 'tracking/summary.json').read_text())['per_object']
            for g, v in per.items():
                switches[g].append(v['id_switches'])
                if v.get('nees_pos') and v['nees_pos'].get('mean') is not None:
                    nees[g].append(v['nees_pos']['mean'])
        n = len(runs(root, scene))
        sw = ', '.join(f'{g}: {sum(v) / n:.1f}' for g, v in sorted(switches.items()))
        print(f"{scene}: ID 전환 {{{sw}}} "
              f"| NEES {{{', '.join(f'{g}: {np.mean(v):.1f}' for g, v in sorted(nees.items()))}}}")


def detection(root):
    print('\n## 3. 장면 C: 출발 거리 → 처음 짝지어진 거리 → 판정 거리 (실행 10회 범위)')
    for scene in ('C05', 'C10', 'C15'):
        vals = []
        for run in runs(root, scene):
            ap = json.loads((run / 'tracking/summary.json').read_text())['approach']
            seen_dist = ap['dynamic_dist'] + (ap['dynamic_speed'] or 0.0) * (ap['t_dynamic'] - ap['t_seen'])
            vals.append((ap['start_dist'], seen_dist, ap['dynamic_dist'], ap['t_seen'] - ap['t_start']))
        rng = [f'{min(v[i] for v in vals):.2f}~{max(v[i] for v in vals):.2f}' for i in range(4)]
        print(f'{scene}: 출발 {rng[0]} m → 처음 짝 {rng[1]} m → 판정 {rng[2]} m | 출발부터 처음 짝까지 {rng[3]} s')


def ghosts(root):
    print('\n## 4. 장면 C 가짜 동적 트랙')
    for scene in ('C10', 'C15'):
        since, gdist, clear, old, new = [], [], [], 0, 0
        for run in runs(root, scene):
            rows = frames(run)
            gt = {float(x['t']): x for x in rows if x['kind'] in ('match', 'miss')}
            seq = sorted((t, float(x['gt_vx'])) for t, x in gt.items())
            reversals = [b[0] for a, b in zip(seq, seq[1:]) if a[1] * b[1] < 0]
            first_match = {}
            for x in rows:
                t = float(x['t'])
                if x['kind'] == 'match':
                    first_match.setdefault(x['track_id'], t)
                elif x['kind'] == 'track_only' and x['is_dynamic'] == '1' and x['near_gt'] == '0':
                    if x['track_id'] in first_match and first_match[x['track_id']] < t:
                        old += 1
                    else:
                        new += 1
                    g = gt.get(t)
                    if g:
                        dx, dy = float(x['tr_x']) - float(g['gt_x']), float(x['tr_y']) - float(g['gt_y'])
                        gdist.append(math.hypot(dx, dy))
                    past = [t - rv for rv in reversals if rv <= t]
                    since.append(min(past) if past else math.inf)
                    if x.get('clearance'):
                        clear.append(float(x['clearance']))
        since = np.array(since)
        print(f'{scene}: 가짜 행 {len(since)} | 반환 뒤 3 s 안 {np.mean(since <= 3):.0%} | '
              f'그전에 작업자와 짝지어졌던 트랙 {old / (old + new):.0%} | GT와 거리 중앙 {np.median(gdist):.2f} m | '
              f'차체 거리 중앙 {np.median(clear):.2f} m')


def unjudged(root):
    print('\n## 5. 판정 지연 구간 (A·B·D·E, 구간 전부터 동적·기록 전부터 구간 뺌): 안 보임 / 보였는데 판정 안 됨 / 판정')
    count = defaultdict(lambda: [0, 0, 0])
    for scene in 'ABDE':
        for run in runs(root, scene):
            with open(run / 'tracking/episodes.csv') as f:
                for e in csv.DictReader(f):
                    if e['censored'] == '1' or e['carried'] == '1':
                        continue
                    c = count[e['speed_bin']]
                    c[0 if e['t_seen'] == '' else 1 if e['t_dynamic'] == '' else 2] += 1
    for b in ('<0.6', '0.6-1.0', '>=1.0'):
        print(f'{b}: {count[b][0]} / {count[b][1]} / {count[b][2]}')


def main(argv):
    root = Path(argv[1]).expanduser()
    p2_breakdown(root)
    objects(root)
    detection(root)
    ghosts(root)
    unjudged(root)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
