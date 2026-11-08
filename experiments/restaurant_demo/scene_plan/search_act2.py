"""장면 2 2막 시작 작업 두 바퀴 고르기 (무작위 탐색): python3 search_act2.py <pairs.json> <stations_restaurant.yaml>

조건 (tools/harness/tasks/demo_s2b.yaml 머리말):
- 4호 1바퀴가 다른 로봇 1바퀴보다 10 m 넘게 짧다 (먼저 끝나 2바퀴 첫 작업을 가져간다)
- 4호 두 바퀴 합이 최고 속도(V)로도 0초(T0) + 10 s 뒤에 끝난다 (픽업·하역 4번 머무름 DW)
- 4호 두 바퀴가 다른 로봇 1바퀴보다 5 m 넘게 길다 (2바퀴 나머지 넷을 다른 로봇이 가져간다)
- 다른 로봇은 2바퀴 어느 작업을 받아도 두 바퀴 합이 4호보다 10 m 넘게 길다 (여유 slack을 크게)
- 핵심 요청 자리(회수 5곳, return_1·2, c01)와 2인석 배달 픽업(stall_3)에서 1 m 안은 쓰지 않는다. 한 바퀴에서 한 픽업은 둘까지,
  4호 픽업은 혼자. 한 바퀴 안 하역은 서로 다르다
"""
import json
import math
import random
import sys

import yaml

PICK = ['stall_1', 'stall_2', 'stall_3', 'stall_4', 'buffet_1', 'buffet_2']
OTH = ['amr_05', 'amr_03', 'amr_02', 'amr_01']
CORE = ['t01', 't06', 't08', 't12', 't14', 'return_1', 'return_2', 'c01', 'stall_3']
V, DW, T0 = 0.8, 2.0, 66.0


def main(iterations=1500000, seed=1):
    lengths = {tuple(k.split('|')): v for k, v in json.load(open(sys.argv[1])).items()}
    st = {k: tuple(v[:2]) for k, v in yaml.safe_load(open(sys.argv[2], encoding='utf-8'))['stations'].items()}
    drops_all = sorted({b for (_, b) in lengths if b[0] in 'tL' or b.startswith(('b0', 'c0'))})

    def ok(name):
        return all(math.dist(st[name], st[c]) >= 1.0 for c in CORE)

    picks = [p for p in PICK if ok(p)]
    drops = [d for d in drops_all if ok(d)]
    round1 = {r: [(p, d, lengths[(r, p)] + lengths[(p, d)]) for p in picks for d in drops]
              for r in ['amr_04'] + OTH}
    legs = [(p, d, lengths[(p, d)]) for p in picks for d in drops]
    rnd = random.Random(seed)
    best = None
    for _ in range(iterations):
        p4, d4, b4 = rnd.choice(round1['amr_04'])
        if b4 > 28:
            continue
        p0, d0, l0 = rnd.choice(legs)
        if d0 == d4:
            continue
        total4 = b4 + lengths[(d4, p0)] + l0
        if total4 / V + 4 * DW < T0 + 10:
            continue
        b1 = {}
        for r in OTH:
            c = [x for x in round1[r] if b4 + 10 <= x[2] <= total4 - 5 and x[1] not in (d4, d0)]
            if not c:
                break
            b1[r] = rnd.choice(c)
        if len(b1) < 4:
            continue
        if len({b1[r][1] for r in OTH} | {d4}) < 5:
            continue
        p1s = [b1[r][0] for r in OTH] + [p4]
        if max(p1s.count(p) for p in p1s) > 2 or p1s.count(p4) > 1:
            continue
        more = [rnd.choice(legs) for _ in range(4)]
        if len({x[1] for x in more} | {d0}) < 5:
            continue
        p2s = [x[0] for x in more] + [p0]
        if max(p2s.count(p) for p in p2s) > 2 or p2s.count(p0) > 1:
            continue
        slack = min(b1[r][2] + lengths[(b1[r][1], x[0])] + x[2] - (total4 + 10) for r in OTH for x in more)
        if slack < 0:
            continue
        gap1 = min(b1[r][2] for r in OTH) - b4
        score = min(slack, 12) * 2 + min(gap1 - 10, 8) + min(total4 / V + 4 * DW - T0 - 10, 10)
        if best is None or score > best[0]:
            best = (score, {'round1_amr_04': (p4, d4, round(b4, 1)), 'round2_first': (p0, d0),
                            'total_amr_04': round(total4, 1), 'fast_s': round(total4 / V + 4 * DW, 1),
                            'round1': {r: (b1[r][0], b1[r][1], round(b1[r][2], 1)) for r in OTH},
                            'round2_rest': [(x[0], x[1]) for x in more], 'slack': round(slack, 1),
                            'gap1': round(gap1, 1)})
    print(json.dumps(best, indent=1))


if __name__ == '__main__':
    main()
