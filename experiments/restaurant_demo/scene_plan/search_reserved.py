"""장면 2 시작 작업 고르기 - 서버 스테이션 예약을 지키는 판 (무작위 탐색).

    python3 search_reserved.py act1 <pairs.json> <stations_restaurant.yaml> <robots_restaurant.yaml>
    python3 search_reserved.py act2 <pairs.json> <stations_restaurant.yaml> <robots_restaurant.yaml>

Fleet 서버는 맡긴 작업의 픽업·하역 자리를 그 작업이 끝날 때까지 잡아 둔다(assignment.station_clearance_m 1 m). 그 안을 쓰는
다른 작업은 큐에서 기다리고, 큐 안에서 겹치면 하나만 맡긴다. 그래서 같이 잡혀 있을 수 있는 작업은 자리가 모두 달라야 한다.
- 1막: 시작 작업 5건 + 대기 배달 4건(9·15·4·2번 테이블, 픽업 stall_3·4·1·2 그대로) + 회수(t03 → return_1)가 모두 다른 자리
- 2막: 1바퀴 5건 + 2바퀴 5건 + 2인석 배달(stall_3 → c01)이 모두 다른 자리. 1-2·2막 핵심 자리(buffet_2, t11, c01, 회수 대상
  t01·t06·t08·t12·t14, 퇴식구 return_1·2)는 시작 작업에 쓰지 않는다
누가 맡나는 서버와 같이 낸 순서대로 남은 로봇 중 픽업까지 직선 거리가 가장 짧은 로봇(nearest 탐욕)으로 흉내 내고, 그 순서가
amr_05, 04, 03, 02, 01인 것만 받는다. 길이는 pairs.json(A*, pairs.py).
"""
import json
import math
import random
import sys

import yaml

ORDER = ['amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01']
CLEARANCE = 1.0
SPEED, DWELL, T0_ACT2 = 0.8, 2.0, 66.0
TOTAL_GAP = 5.0     # 2막: 다른 로봇 두 바퀴 합이 4호보다 이만큼 길다 (자리가 모두 달라야 해 10 m는 못 맞춘다)


def load(argv):
    lengths = {tuple(k.split('|')): v for k, v in json.load(open(argv[0])).items()}
    st = {k: tuple(v[:2]) for k, v in yaml.safe_load(open(argv[1], encoding='utf-8'))['stations'].items()}
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(open(argv[2], encoding='utf-8'))['robots'].items()}
    return lengths, st, robots


def greedy_owners(robots, st, pickups):
    """낸 순서대로 남은 로봇 중 픽업에 가장 가까운 로봇 (직선, 같으면 이름순)."""
    idle, out = dict(robots), []
    for p in pickups:
        r = min(idle, key=lambda k: (math.dist(idle[k], st[p]), k))
        out.append(r)
        idle.pop(r)
    return out


def allowed_stations(st, lengths, forbid):
    names = sorted({b for (_, b) in lengths})
    return [n for n in names if all(math.dist(st[n], st[f]) > CLEARANCE for f in forbid)]


def act1(lengths, st, robots, iterations=400000, seed=1, gap=12.0):
    queue = [('stall_3', 't09'), ('stall_4', 't15'), ('stall_1', 't04'), ('stall_2', 't02')]
    forbid = ['t03', 'return_1'] + [s for q in queue for s in q]
    pool = allowed_stations(st, lengths, forbid)
    rnd = random.Random(seed)
    best = None
    for _ in range(iterations):
        pick = rnd.sample(pool, 10)
        tasks = list(zip(pick[0::2], pick[1::2]))
        if greedy_owners(robots, st, [p for p, _ in tasks]) != ORDER:
            continue
        length = {r: lengths[(r, p)] + lengths[(p, d)] for r, (p, d) in zip(ORDER, tasks)}
        others = [v for r, v in length.items() if r != 'amr_02']
        if not 12 <= length["amr_02"] <= 22 or min(others) < length["amr_02"] + gap:
            continue
        score = sum(others)                       # 짧을수록 좋다 (끔에서 회수가 녹화 안에 끝나게)
        if best is None or score < best[0]:
            best = (score, [(r, p, d, round(length[r], 1)) for r, (p, d) in zip(ORDER, tasks)])
    return best


def act2(lengths, st, robots, iterations=300000, seed=1):
    """1바퀴 픽업을 로봇 줄보다 북쪽(y >= 가장 북쪽 로봇)에서 고르면 낸 순서대로 05, 04, 03, 02, 01이 맡는다.
    조건을 만족하는 후보 목록에서 뽑고, 자리 20곳이 모두 다른지 본다."""
    forbid = ['buffet_2', 't11', 'c01', 't01', 't06', 't08', 't12', 't14', 'return_1', 'return_2', 'stall_3']
    pool = allowed_stations(st, lengths, forbid)
    north = max(y for _, y in robots.values())
    picks1 = [n for n in pool if st[n][1] >= north]
    legs = [(p, d, lengths[(p, d)]) for p in pool for d in pool if p != d]
    r1 = {r: [(p, d, lengths[(r, p)] + lengths[(p, d)]) for p in picks1 for d in pool if p != d] for r in ORDER}
    rnd = random.Random(seed)
    first = [x for x in r1['amr_04'] if x[2] <= 32]
    best = None
    for _ in range(iterations):
        p4, d4, b4 = rnd.choice(first)
        p0, d0, l0 = rnd.choice(legs)
        used = {p4, d4, p0, d0}
        if len(used) < 4:
            continue
        total4 = b4 + lengths[(d4, p0)] + l0
        if total4 / SPEED + 4 * DWELL < T0_ACT2 + 10:
            continue
        b, ok = {'amr_04': (p4, d4, b4)}, True
        for r in rnd.sample(['amr_05', 'amr_03', 'amr_02', 'amr_01'], 4):
            c = sorted([x for x in r1[r] if b4 + 10 <= x[2] <= total4 - 5 and x[0] not in used and x[1] not in used],
                       key=lambda x: -x[2])[:6]
            if not c:
                ok = False
                break
            b[r] = rnd.choice(c)
            used |= {b[r][0], b[r][1]}
        if not ok:
            continue
        # 2바퀴 나머지 넷: 남은 자리로 만들 수 있는 가장 긴 다리부터 (위쪽 몇 개 중 무작위)
        rest = []
        for _ in range(4):
            c = sorted([x for x in legs if x[0] not in used and x[1] not in used], key=lambda x: -x[2])[:4]
            if not c:
                ok = False
                break
            x = rnd.choice(c)
            rest.append(x)
            used |= {x[0], x[1]}
        if not ok:
            continue
        if greedy_owners(robots, st, [b[r][0] for r in ORDER]) != ORDER:
            continue
        slack = min(b[r][2] + lengths[(b[r][1], x[0])] + x[2] - (total4 + TOTAL_GAP)
                    for r in ORDER if r != 'amr_04' for x in rest)
        if slack < 0:
            continue
        gap1 = min(b[r][2] for r in ORDER if r != 'amr_04') - b4
        score = min(slack, 12) * 2 + min(gap1 - 10, 8) + min(total4 / SPEED + 4 * DWELL - T0_ACT2 - 10, 10)
        if best is None or score > best[0]:
            best = (score, {'round1': [(r, b[r][0], b[r][1], round(b[r][2], 1)) for r in ORDER],
                            'round2': [(p0, d0, round(l0, 1))] + [(x[0], x[1], round(x[2], 1)) for x in rest],
                            'first_total_m': round(total4, 1),
                            'others_min_total_m': round(total4 + TOTAL_GAP + slack, 1),
                            'fast_s': round(total4 / SPEED + 4 * DWELL, 1), 'gap1_m': round(gap1, 1),
                            'slack_m': round(slack, 1)})
    return best


def main():
    which, argv = sys.argv[1], sys.argv[2:]
    lengths, st, robots = load(argv)
    print(json.dumps({'act1': act1, 'act2': act2}[which](lengths, st, robots), indent=1))


if __name__ == '__main__':
    main()
