"""탐색 실험(질량 정보 값어치를 키운 보상) 판정. 공유 문서 "탐색 실험" 절의 기준을 그대로 계산한다.

    uv run python scripts/judge_info_pilot.py raw/info_pilot --cand info --base base

주 지표는 제안 − R2 차이다 (같은 시드끼리). 기준 두 개를 모두 만족해야 "질량 정보 효과가 커졌다":
  1. 25 kg 충돌률의 (제안 − R2)가 기준선보다 2%p 이상 더 벌어진다 (더 음수)
  2. 제안의 감속 폭(0 → 25 kg, 같은 시나리오를 둘 다 성공한 판끼리)이 기준선보다 5%p 이상 커진다 (더 음수)
안전장치: 제안 시간 초과 +5%p 이내, R2 성공 −5%p 이내.
참고: 0 kg 도착 시간 (제안 − R2, 같은 시나리오를 둘 다 성공한 판끼리), 15·30 kg 충돌(eval15_* 폴더가 있으면).
입력: <폴더>/eval_<후보>_<비교군>_s<시드>/episodes.csv, 있으면 eval15_<후보>_<비교군>_s<시드>/episodes.csv
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

GAP_WIDEN = -0.02
DECEL_GROW = -0.05
TIMEOUT_RISE = 0.05
R2_SUCCESS_DROP = -0.05


def truthy(v) -> bool:
    return str(v).strip().lower() in ('true', '1')


def load(path: Path) -> list[dict]:
    with path.open() as f:
        return [{'scenario': r['scenario'], 'seed': int(r['scenario_seed']), 'kg': float(r['payload_kg']),
                 'outcome': r['outcome'], 'success': truthy(r['success']), 'collision': truthy(r['collision']),
                 'appeared': truthy(r['popout_appeared']), 'speed': float(r['mean_speed']), 'time': float(r['time_s'])}
                for r in csv.DictReader(f) if r['mass_obs'] in ('exact', 'true')]


def coll(rows, kg):
    den = [r for r in rows if r['kg'] == kg and r['appeared']]
    return float(np.mean([r['collision'] for r in den])) if den else float('nan')


def decel(rows):
    ok = {(r['scenario'], r['seed'], r['kg']): r['speed'] for r in rows if r['success']}
    pairs = [(ok[(s, n, 0.0)], ok[(s, n, 25.0)]) for (s, n, kg) in ok if kg == 0.0 and (s, n, 25.0) in ok]
    return np.mean([b for _, b in pairs]) / np.mean([a for a, _ in pairs]) - 1


def arrival_gap(prop, r2, kg=0.0):
    """같은 시나리오를 제안·R2가 둘 다 성공한 판끼리 도착 시간 차 (제안 − R2) [s]."""
    a = {(r['scenario'], r['seed']): r['time'] for r in prop if r['kg'] == kg and r['success']}
    b = {(r['scenario'], r['seed']): r['time'] for r in r2 if r['kg'] == kg and r['success']}
    keys = a.keys() & b.keys()
    return float(np.mean([a[k] - b[k] for k in keys])) if keys else float('nan')


def boot_ci(d, n=10_000, seed=0):
    d = np.asarray(d)
    means = d[np.random.default_rng(seed).integers(0, len(d), size=(n, len(d)))].mean(axis=1)
    return np.percentile(means, 2.5), np.percentile(means, 97.5)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--cand', default='info')
    p.add_argument('--base', default='base')
    args = p.parse_args()

    data = defaultdict(dict)    # (prefix, 후보, 비교군) -> 시드 -> rows
    for f in sorted(args.root.glob('eval*_s*/episodes.csv')):
        prefix, rest = f.parent.name.split('_', 1)
        cand, variant, seed = rest.rsplit('_', 2)
        data[(prefix, cand, variant)][int(seed[1:])] = load(f)

    per = {}   # 후보 -> 시드 -> 지표
    for c in (args.base, args.cand):
        P, R = data[('eval', c, 'proposed')], data[('eval', c, 'r2')]
        per[c] = {}
        for s in sorted(P.keys() & R.keys()):
            per[c][s] = {
                'gap25': coll(P[s], 25.0) - coll(R[s], 25.0), 'p_coll25': coll(P[s], 25.0), 'r_coll25': coll(R[s], 25.0),
                'decel': decel(P[s]), 'r_decel': decel(R[s]),
                'timeout': np.mean([r['outcome'] == 'timeout' for r in P[s]]),
                'r_success': np.mean([r['success'] for r in R[s]]),
                'arr0': arrival_gap(P[s], R[s], 0.0)}
            for kg in (15.0, 30.0):
                P15, R15 = data[('eval15', c, 'proposed')].get(s), data[('eval15', c, 'r2')].get(s)
                if P15 and R15:
                    per[c][s][f'p_coll{kg:g}'] = coll(P15, kg)
                    per[c][s][f'gap{kg:g}'] = coll(P15, kg) - coll(R15, kg)
    seeds = sorted(per[args.base].keys() & per[args.cand].keys())
    d = lambda k: np.array([per[args.cand][s][k] - per[args.base][s][k] for s in seeds])
    m = lambda c, k: np.mean([per[c][s][k] for s in seeds if k in per[c][s]])

    c1, c2 = d('gap25').mean() <= GAP_WIDEN, d('decel').mean() <= DECEL_GROW
    s1, s2 = d('timeout').mean() <= TIMEOUT_RISE, d('r_success').mean() >= R2_SUCCESS_DROP
    fmt = lambda x: f'{x*100:+.1f}%p'
    ci = lambda k: '({:+.1f} ~ {:+.1f})'.format(*[v * 100 for v in boot_ci(d(k))])
    out = [f'# 탐색 실험 판정 ({args.cand} vs {args.base}, 시드 {len(seeds)}개)', '',
           '| 지표 | 기준선 | 후보 | 변화 (95% CI) | 기준 | 결과 |', '| --- | --- | --- | --- | --- | --- |',
           f'| ① 25 kg 충돌 제안 − R2 | {fmt(m(args.base, "gap25"))} | {fmt(m(args.cand, "gap25"))} | '
           f'{fmt(d("gap25").mean())} {ci("gap25")} | −2%p 이하 | {"만족" if c1 else "**못 미침**"} |',
           f'| ② 제안 감속 (0 → 25 kg) | {m(args.base, "decel")*100:+.1f}% | {m(args.cand, "decel")*100:+.1f}% | '
           f'{fmt(d("decel").mean())} {ci("decel")} | −5%p 이하 | {"만족" if c2 else "**못 미침**"} |',
           f'| 안전: 제안 시간 초과 | {m(args.base, "timeout")*100:.1f}% | {m(args.cand, "timeout")*100:.1f}% | '
           f'{fmt(d("timeout").mean())} | +5%p 이내 | {"통과" if s1 else "**어김**"} |',
           f'| 안전: R2 성공 | {m(args.base, "r_success")*100:.1f}% | {m(args.cand, "r_success")*100:.1f}% | '
           f'{fmt(d("r_success").mean())} | −5%p 이내 | {"통과" if s2 else "**어김**"} |', '',
           f'**결론: {"질량 정보 효과가 커졌다" if c1 and c2 and s1 and s2 else "기준을 모두 만족하지 못했다"}**', '',
           '## 참고', '',
           '| 지표 | 기준선 | 후보 |', '| --- | --- | --- |',
           f'| 제안 25 kg 충돌 | {m(args.base, "p_coll25")*100:.1f}% | {m(args.cand, "p_coll25")*100:.1f}% |',
           f'| R2 25 kg 충돌 | {m(args.base, "r_coll25")*100:.1f}% | {m(args.cand, "r_coll25")*100:.1f}% |',
           f'| R2 감속 (0 → 25 kg) | {m(args.base, "r_decel")*100:+.1f}% | {m(args.cand, "r_decel")*100:+.1f}% |',
           f'| 0 kg 도착 시간 제안 − R2 [s] | {m(args.base, "arr0"):+.2f} | {m(args.cand, "arr0"):+.2f} |']
    for kg in (15, 30):
        if all(f'gap{kg}' in per[c][s] for c in (args.base, args.cand) for s in seeds):
            out.append(f'| {kg} kg 충돌 제안 / 제안 − R2 (처음 보는 질량) | {m(args.base, f"p_coll{kg}")*100:.1f}% / '
                       f'{fmt(m(args.base, f"gap{kg}"))} | {m(args.cand, f"p_coll{kg}")*100:.1f}% / {fmt(m(args.cand, f"gap{kg}"))} |')
    out += ['', '- 충돌률 분모는 돌발 장애물이 나타난 판. 감속·도착 시간은 같은 시나리오를 둘 다 성공한 판끼리',
            f'- 기준선은 `{args.base}` (같은 맥·학습 코드·시드). 판정 기준은 실행 전에 공유 문서에 적었다']
    text = '\n'.join(out)
    (args.root / 'judge_info.md').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
