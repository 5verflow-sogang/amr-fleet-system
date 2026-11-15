"""장면 2 1막 시작 작업 고르기: python3 search_act1.py <pairs.json> <stations_restaurant.yaml>

2호(amr_02) 작업이 12~20 m로 가장 짧고, 나머지 넷은 2호보다 11 m 넘게 길며 픽업·하역이 모두 다른 조합 중 길이 합이 가장 작은 것.
핵심 요청 자리(t03, return_1)와 대기 배달 하역(t09·t15·t04·t02)에서 1 m 안은 쓰지 않는다.
"""
import itertools
import json
import math
import sys

import yaml

PICK = ['stall_1', 'stall_2', 'stall_3', 'stall_4', 'buffet_1', 'buffet_2']
CORE = ['t03', 'return_1', 't09', 't15', 't04', 't02']
ORDER = ['amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01']


def main():
    lengths = {tuple(k.split('|')): v for k, v in json.load(open(sys.argv[1])).items()}
    st = {k: tuple(v[:2]) for k, v in yaml.safe_load(open(sys.argv[2], encoding='utf-8'))['stations'].items()}
    drops = sorted({b for (_, b) in lengths if b[0] in 'tL' or b.startswith(('b0', 'c0'))})

    def ok(name):
        return all(math.dist(st[name], st[c]) >= 1.0 for c in CORE)

    opts = {r: sorted([(p, d, round(lengths[(r, p)] + lengths[(p, d)], 1)) for p in PICK for d in drops
                       if ok(p) and ok(d)], key=lambda x: x[2]) for r in ORDER}
    best = None
    others = [r for r in ORDER if r != 'amr_02']
    for key in opts['amr_02']:
        if not 12 <= key[2] <= 20:
            continue
        lists = [[o for o in opts[r] if o[2] >= key[2] + 11 and o[0] != key[0] and o[1] != key[1]][:10]
                 for r in others]
        for combo in itertools.product(*lists):
            if len({o[0] for o in combo}) < 4 or len({o[1] for o in combo}) < 4:
                continue
            total = sum(o[2] for o in combo)
            if best is None or total < best[0]:
                best = (total, key, dict(zip(others, combo)))
    print(best)


if __name__ == '__main__':
    main()
