"""v2 튜닝 채택 판정 (공유 문서 "정한 것 (10/17 확정)" 3절). 후보마다 같은 시드의 기준선과 짝지어 비교한다.

    uv run python scripts/judge_v2_tuning.py raw/v2_track_b --cands b1 b2
    uv run python scripts/judge_v2_tuning.py raw/v2_track_a --cands a1 a2

입력: <폴더>/eval_<후보>_<비교군>_s<시드>/episodes.csv (run_v2_track_b.sh 출력 형식). 기준선 후보 이름은 --base(기본 base).
평가는 학습 질량(0·2·10·25 kg), 질량 정보 정확, S1·S2다.

채택 조건 (모두 만족, 시드 평균, 후보 − 기준선):
  ① 제안 25 kg 충돌률(분모 = 돌발 장애물이 나타난 에피소드, analyze.py와 같음) 2%p 이상 감소
  ② 제안 시간 초과율(학습 질량 전체, outcome == timeout) 증가 +5%p 이내
  ③ R2 성공률(학습 질량 전체) 하락 5%p 이내
둘 이상 통과하면 ①이 가장 낮은 후보. 모두 탈락이면 기준선 유지.
95% CI(시드 짝지은 부트스트랩)·감속 폭·충돌 원인은 참고로만 보고한다.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

HEAVY = 25.0
COLLISION_DROP = -0.02     # ①
TIMEOUT_RISE = 0.05        # ②
R2_SUCCESS_DROP = -0.05    # ③


def truthy(v) -> bool:
    return str(v).strip().lower() in ('true', '1')


def load(path: Path) -> list[dict]:
    rows = []
    with path.open() as f:
        for r in csv.DictReader(f):
            if r['mass_obs'] not in ('exact', 'true'):
                continue
            rows.append({'scenario': r['scenario'], 'seed': int(r['scenario_seed']),
                         'kg': float(r['payload_kg']), 'outcome': r['outcome'],
                         'success': truthy(r['success']), 'collision': truthy(r['collision']),
                         'appeared': truthy(r['popout_appeared']), 'stoppable': truthy(r['popout_stoppable']),
                         'speed': float(r['mean_speed'])})
    return rows


def rate(xs) -> float:
    xs = list(xs)
    return float(np.mean(xs)) if xs else float('nan')


def metrics(rows: list[dict]) -> dict:
    heavy_app = [r for r in rows if r['kg'] == HEAVY and r['appeared']]
    m = {'coll25': rate(r['collision'] for r in heavy_app),
         'timeout': rate(r['outcome'] == 'timeout' for r in rows),
         'success': rate(r['success'] for r in rows),
         'too_fast': sum(r['collision'] and not r['stoppable'] for r in heavy_app),
         'stoppable': sum(r['collision'] and r['stoppable'] for r in heavy_app)}
    for kg in (2.0, 10.0):
        m[f'coll{kg:g}'] = rate(r['collision'] for r in rows if r['kg'] == kg and r['appeared'])
    m['s2_success'] = rate(r['success'] for r in rows if r['scenario'] == 'S2')
    # 감속 폭: 같은 시나리오를 0 kg과 25 kg에서 둘 다 성공한 것끼리 평균 속도 비 (성공 평균의 편향을 줄인다)
    ok = {(r['scenario'], r['seed'], r['kg']): r['speed'] for r in rows if r['success']}
    pairs = [(ok[(s, n, 0.0)], ok[(s, n, HEAVY)]) for (s, n, kg) in ok if kg == 0.0 and (s, n, HEAVY) in ok]
    m['decel'] = (np.mean([b for _, b in pairs]) / np.mean([a for a, _ in pairs]) - 1) if pairs else float('nan')
    return m


def boot_ci(d: np.ndarray, n: int = 10_000, seed: int = 0) -> tuple[float, float]:
    rng = np.random.default_rng(seed)
    means = d[rng.integers(0, len(d), size=(n, len(d)))].mean(axis=1)
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--cands', nargs='+', required=True)
    p.add_argument('--base', default='base')
    args = p.parse_args()

    per = defaultdict(dict)    # (후보, 비교군) -> 시드 -> metrics
    for f in sorted(args.root.glob('eval_*_s*/episodes.csv')):
        cand, variant, seed = f.parent.name[len('eval_'):].rsplit('_', 2)
        per[(cand, variant)][int(seed[1:])] = metrics(load(f))

    def delta(cand, variant, key):
        base, c = per[(args.base, variant)], per[(cand, variant)]
        seeds = sorted(set(base) & set(c))
        return np.array([c[s][key] - base[s][key] for s in seeds]), seeds

    out = ['# v2 튜닝 판정', '',
           f'폴더 `{args.root}`, 기준선 `{args.base}`. 값은 시드 평균, Δ = 후보 − 기준선 (같은 시드끼리), 괄호는 95% CI (참고).', '',
           '| 후보 | 시드 | ① 제안 25 kg 충돌 Δ | ② 제안 시간 초과 Δ | ③ R2 성공 Δ | 판정 |',
           '| --- | --- | --- | --- | --- | --- |']
    passed = []
    for cand in args.cands:
        d1, seeds = delta(cand, 'proposed', 'coll25')
        d2, _ = delta(cand, 'proposed', 'timeout')
        d3, _ = delta(cand, 'r2', 'success')
        ok = (d1.mean() <= COLLISION_DROP, d2.mean() <= TIMEOUT_RISE, d3.mean() >= R2_SUCCESS_DROP)
        if all(ok):
            passed.append((d1.mean(), cand))
        cell = lambda d, good: (f'{d.mean()*100:+.1f}%p ({boot_ci(d)[0]*100:+.1f} ~ {boot_ci(d)[1]*100:+.1f}) '
                                f'{"통과" if good else "**탈락**"}')
        out.append(f'| {cand} | {len(seeds)} | {cell(d1, ok[0])} | {cell(d2, ok[1])} | {cell(d3, ok[2])} | '
                   f'{"채택 후보" if all(ok) else "탈락"} |')
    choice = min(passed)[1] if passed else f'없음 → {args.base} 유지'
    out += ['', f'**선택: {choice}** (통과한 후보 중 ① 25 kg 충돌이 가장 낮은 것)', '',
            '## 참고 (판정에 안 씀)', '',
            '| 후보 | 비교군 | 25 kg 충돌 | 2 kg 충돌 | 10 kg 충돌 | 시간 초과 | 성공 | S2 성공 | 감속 (0→25 kg, 짝지음) | 25 kg 충돌: 못 섬 / 설 수 있었음 |',
            '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for variant in ('proposed', 'r2'):
        for cand in [args.base] + args.cands:
            ms = list(per[(cand, variant)].values())
            if not ms:
                continue
            avg = lambda k: np.mean([m[k] for m in ms])
            out.append(f'| {cand} | {variant} | {avg("coll25")*100:.1f}% | {avg("coll2")*100:.1f}% | '
                       f'{avg("coll10")*100:.1f}% | {avg("timeout")*100:.1f}% | {avg("success")*100:.1f}% | '
                       f'{avg("s2_success")*100:.1f}% | {avg("decel")*100:+.1f}% | '
                       f'{sum(m["too_fast"] for m in ms)} / {sum(m["stoppable"] for m in ms)} |')
    out += ['', '- 충돌률 분모는 돌발 장애물이 나타난 에피소드다. 시간 초과·성공은 학습 질량 전체(S1·S2) 분모다',
            '- 감속은 같은 시나리오를 0 kg과 25 kg에서 둘 다 성공한 에피소드끼리 평균 속도를 비교한 것이다. '
            '참고: 규칙 정책 −22%, v1 제안 약 −7% (성공 평균 기준)']
    text = '\n'.join(out)
    (args.root / 'judge.md').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
