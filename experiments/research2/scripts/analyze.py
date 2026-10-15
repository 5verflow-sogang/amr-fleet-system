"""연구 2 평가 결과(episodes.csv) → 시드별 요약 → H1~H3 판정 (experiment_design.md 6·7절).

    uv run python scripts/analyze.py raw/eval/*.csv --out raw/eval/analysis

표준 라이브러리만 쓴다. 분석 단위는 학습 시드다. 시드마다 지표를 먼저 내고(예: 그 시드의 충돌률),
비교군 간 차이를 시드 k끼리 짝지은 뒤 시드를 복원 추출하는 부트스트랩으로 95% 신뢰구간을 낸다.
짝짓기 키는 평가 목록 번호 `eval_list`다(experiment_design.md 5·6절). RL은 학습 시드 k가 목록 k를 풀고,
DWA(B1·B2)는 학습 시드 없이 `--eval-list k`로 목록 k를 풀어 시드 k와 짝지어진다.
`eval_list`가 없으면 `train_seed`, 그것도 없으면 run 이름(<variant>_s<seed>_<시각>)을 쓴다.
목록 번호가 전혀 없는 결과(DWA를 한 번만 평가)는 고정값으로 쓰되 경고한다(시드 간 흔들림이 CI에서 빠진다).
시나리오 짝짓기 키는 (`scenario`, `scenario_seed`)다. S1·S2가 같은 시드 번호를 쓰기 때문이다.

열 이름은 scripts/evaluate.py(#74)를 따른다. 아직 확정되지 않은 열은 없어도 동작한다:
- train_seed: 없으면 run 이름(<variant>_s<seed>_<시각>)에서 읽는다
- popout_appeared: 충돌률 분모(돌발 장애물이 나타난 S1 에피소드 전체, #72 연우 리뷰). 없으면 전체를 넣고 경고한다
- popout_stoppable: 분모에서 빼지 않고 충돌 원인을 나누는 데만 쓴다 ("설 수 있었는데 충돌" / "너무 빨라 못 섬").
  출현 순간의 실제 속도로 계산하므로 정책마다 달라져, 분모에서 빼면 너무 빨리 달려 생긴 충돌이 통계에서 사라진다
- mass_obs: true / none / scale:1.3 / scale:0.7 또는 exact / unknown / plus30 / minus30
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional

RESAMPLES = 10_000
MARGIN = 0.02          # 비열등 마진 (충돌률, experiment_design.md 7절)
# 시간 초과율 보조 기준 (7절): 충돌을 줄이는 대신 도착을 못 하는 정책이 통과하지 않게, 비교군보다 이만큼 넘게 늘면 안 된다
TIMEOUT_MARGIN = 0.05
TRAIN_MASSES = (0.0, 2.0, 10.0, 25.0)
UNSEEN_MASSES = (15.0, 30.0)
INFO_ERRORS = ('plus30', 'minus30', 'unknown')

INFO_ALIASES = {'true': 'exact', 'exact': 'exact', 'none': 'unknown', 'unknown': 'unknown',
                'scale:1.3': 'plus30', 'plus30': 'plus30', 'scale:0.7': 'minus30', 'minus30': 'minus30'}
RUN_SEED = re.compile(r'_s(\d+)(?:_|$)')


def truthy(v) -> bool:
    return str(v).strip().lower() in ('1', 'true', 'yes')


def normalize(row: dict, warn: set) -> dict:
    """CSV 한 행 → 분석에 쓰는 값."""
    variant = row.get('variant', '').lower()
    # 짝짓기 키: eval_list → train_seed → run 이름
    seed = row.get('eval_list', '')
    if seed in ('', None):
        seed = row.get('train_seed', '')
    if seed in ('', None):
        m = RUN_SEED.search(row.get('run', ''))
        seed = m.group(1) if m else ''
    info = INFO_ALIASES.get(str(row.get('mass_obs', 'true')).lower())
    if info is None:
        warn.add(f"알 수 없는 mass_obs: {row.get('mass_obs')}")
    if 'popout_appeared' in row and row['popout_appeared'] not in ('', None):
        appeared = truthy(row['popout_appeared'])
    else:
        warn.add('popout_appeared 열이 없어 모든 에피소드를 충돌률 분모에 넣었다')
        appeared = True
    return {
        'variant': variant,
        'seed': int(seed) if str(seed).strip() != '' else None,
        'mass': float(row['payload_kg']),
        'info': info or 'exact',
        # S1·S2가 같은 scenario_seed를 쓰므로 시나리오 종류와 함께 키로 쓴다
        'scenario': (row.get('scenario', ''), row.get('scenario_seed') or row.get('episode')),
        'collision': truthy(row.get('collision')),
        'success': truthy(row.get('success')),
        'time_s': float(row['time_s']) if row.get('time_s') not in ('', None) else math.nan,
        'appeared': appeared,
        'stoppable': truthy(row.get('popout_stoppable')),
    }


def load(paths: Iterable[Path]) -> tuple[list[dict], set]:
    warn: set = set()
    rows = []
    for path in paths:
        with Path(path).open(newline='', encoding='utf-8') as f:
            rows += [normalize(r, warn) for r in csv.DictReader(f)]
    return rows, warn


def group(rows: list[dict]) -> dict:
    """(variant, info, mass, seed) → 에피소드 목록."""
    g = defaultdict(list)
    for r in rows:
        g[(r['variant'], r['info'], r['mass'], r['seed'])].append(r)
    return g


def collision_rate(eps: list[dict]) -> Optional[float]:
    """돌발 장애물이 나타난 에피소드 중 충돌 비율."""
    den = [e for e in eps if e['appeared']]
    return sum(e['collision'] for e in den) / len(den) if den else None


def timeout_rate(eps: list[dict]) -> Optional[float]:
    """충돌도 도착도 아닌 에피소드(시간 초과) 비율. 분모는 에피소드 전체."""
    return sum(not e['collision'] and not e['success'] for e in eps) / len(eps) if eps else None


def success_rate(eps: list[dict]) -> Optional[float]:
    return sum(e['success'] for e in eps) / len(eps) if eps else None


def per_seed(g: dict, variant: str, info: str, mass: float, fn) -> dict:
    """{seed: 값}. 시드 없는 실행(DWA 단일 평가)은 키 None."""
    out = {}
    for (v, i, m, s), eps in g.items():
        if v == variant and i == info and m == mass:
            val = fn(eps)
            if val is not None:
                out[s] = val
    return out


FIXED_WARNINGS: set = set()


def paired(a: dict, b: dict) -> list[float]:
    """목록 k끼리 a − b. b에 목록 번호가 없으면(None) 그 값을 고정값으로 쓰고 경고한다."""
    if None in b and len(b) == 1:
        FIXED_WARNINGS.add('목록 번호(eval_list) 없는 결과를 고정값으로 짝지었다. 시드 간 흔들림이 CI에서 빠진다')
        return [va - b[None] for k, va in a.items() if k is not None]
    return [a[k] - b[k] for k in a if k is not None and k in b]


def degradation(g: dict, variant: str, info: str, mass: float) -> dict:
    """{seed: 충돌률(정보 오차 조건) − 충돌률(정확)}. 같은 시드끼리 뺀다."""
    bad = per_seed(g, variant, info, mass, collision_rate)
    exact = per_seed(g, variant, 'exact', mass, collision_rate)
    return {k: bad[k] - exact[k] for k in bad if k in exact}


def arrival_diffs(g: dict, va: str, vb: str, info: str, mass: float) -> list[float]:
    """시드 k마다, 두 비교군이 모두 성공한 같은 시나리오끼리 도착 시간 차이의 평균 (a − b)."""
    out = []
    a_seeds = {s for (v, i, m, s) in g if v == va and i == info and m == mass and s is not None}
    for k in sorted(a_seeds):
        ea = {e['scenario']: e for e in g.get((va, info, mass, k), []) if e['success']}
        eb_list = g.get((vb, info, mass, k)) or g.get((vb, info, mass, None), [])
        eb = {e['scenario']: e for e in eb_list if e['success']}
        both = [ea[s]['time_s'] - eb[s]['time_s'] for s in ea.keys() & eb.keys()]
        if both:
            out.append(sum(both) / len(both))
    return out


def bootstrap(values: list[float], seed: int = 0) -> dict:
    """평균, 95% 백분위 CI, 한쪽 p (부트스트랩 평균이 0 이상인 비율). 값이 2개 미만이면 CI 없음."""
    n = len(values)
    if n == 0:
        return {'n': 0, 'mean': None, 'ci_low': None, 'ci_high': None, 'p_ge0': None}
    mean = sum(values) / n
    if n < 2:
        return {'n': n, 'mean': mean, 'ci_low': None, 'ci_high': None, 'p_ge0': None}
    rng = random.Random(seed)
    means = sorted(sum(rng.choice(values) for _ in range(n)) / n for _ in range(RESAMPLES))
    lo, hi = means[int(0.025 * (RESAMPLES - 1))], means[int(0.975 * (RESAMPLES - 1))]
    return {'n': n, 'mean': mean, 'ci_low': lo, 'ci_high': hi, 'p_ge0': sum(m >= 0 for m in means) / RESAMPLES}


def _betacf(a: float, b: float, x: float) -> float:
    """불완전 베타 함수의 연분수 (Numerical Recipes betacf). t 분포 누적확률에 쓴다."""
    tiny = 1e-300
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    d = 1.0 / (d if abs(d) > tiny else tiny)
    h = d
    for m in range(1, 300):
        m2 = 2 * m
        for aa in (m * (b - m) * x / ((qam + m2) * (a + m2)), -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))):
            d = 1.0 + aa * d
            d = 1.0 / (d if abs(d) > tiny else tiny)
            c = 1.0 + aa / c
            c = c if abs(c) > tiny else tiny
            h *= d * c
        if abs(d * c - 1.0) < 1e-12:
            break
    return h


def t_cdf(t: float, df: int) -> float:
    """스튜던트 t 분포 누적확률 P(T <= t). 표준 라이브러리만 쓴다 (scipy 없이)."""
    if t == 0.0:
        return 0.5
    x = df / (df + t * t)
    a, b = df / 2.0, 0.5
    bt = math.exp(math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log1p(-x))
    tail = bt * _betacf(a, b, x) / a if x < (a + 1) / (a + b + 2) else 1.0 - bt * _betacf(b, a, 1 - x) / b
    tail = min(max(tail, 0.0), 1.0)          # P(|T| >= |t|)
    return 1.0 - tail / 2 if t > 0 else tail / 2


def t_quantile(q: float, df: int) -> float:
    """t 분포 분위수 (이분법)."""
    lo, hi = -100.0, 100.0
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if t_cdf(mid, df) < q else (lo, mid)
    return (lo + hi) / 2


def paired_t(values: list[float], bound: float) -> dict:
    """짝지은 차이 Δ_k의 t 검정. H0: 평균 ≥ bound, 대립: 평균 < bound (한쪽).

    시드 k끼리 같은 평가 목록을 풀어 짝지어졌으므로 Welch(독립 표본)가 아니라 짝지은 t다 (#100 연우 리뷰).
    95% CI 상한 < bound ⇔ 한쪽 p < 0.025. 값이 2개 미만이거나 분산이 0이면 None.
    """
    n = len(values)
    if n < 2:
        return {'t_ci_high': None, 'p_t': None}
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))
    if sd == 0.0:
        return {'t_ci_high': mean, 'p_t': 0.0 if mean < bound else 1.0}
    se = sd / math.sqrt(n)
    return {'t_ci_high': mean + t_quantile(0.975, n - 1) * se, 'p_t': t_cdf((mean - bound) / se, n - 1)}


def holm(pvalues: list[Optional[float]], alpha: float = 0.05) -> list[Optional[bool]]:
    """Holm 보정. 기각(True) 여부. p가 None이면 None."""
    idx = sorted((i for i, p in enumerate(pvalues) if p is not None), key=lambda i: pvalues[i])
    out: list[Optional[bool]] = [None] * len(pvalues)
    m, stop = len(idx), False
    for rank, i in enumerate(idx):
        reject = not stop and pvalues[i] <= alpha / (m - rank)
        stop = stop or not reject
        out[i] = reject
    return out


def check(name: str, condition: str, values: list[float], bound: float, seed: int) -> dict:
    """부트스트랩 CI 상한과 짝지은 t CI 상한이 둘 다 bound보다 작으면 통과 (둘이 다르면 보수적인 쪽, 6절)."""
    b = bootstrap(values, seed)
    t = paired_t(values, bound)
    highs = [h for h in (b['ci_high'], t['t_ci_high']) if h is not None]
    passed = None if b['ci_high'] is None else all(h < bound for h in highs)
    return {'hypothesis': name, 'condition': condition, 'bound': bound, **b, **t, 'pass': passed}


def hypothesis_verdicts(results: list[dict]) -> dict:
    """가설마다 판정. 조건이 여럿이면 모두 통과해야 통과다 (H1 "10·25 kg 각각" 등, #100 연우 리뷰).

    모든 조건을 다 통과해야 하는 판정(intersection-union)은 조건마다 유의수준 0.025(한쪽)로 봐도 가설 전체의
    1종 오류가 0.025를 넘지 않는다. 그래서 다중 비교 보정 없이 조건별 판정을 그대로 쓴다 (Berger 1982).
    Holm 열은 "어느 조건이 따로 유의한가"를 볼 때만 쓴다.
    """
    groups: dict[str, list] = {}
    for r in results:
        groups.setdefault(r['hypothesis'], []).append(r['pass'])
    out = {}
    for name, passes in groups.items():
        out[name] = False if False in passes else (None if None in passes else True)
    return out


def judge(rows: list[dict]) -> list[dict]:
    g = group(rows)
    variants = {r['variant'] for r in rows}
    results = []
    k = 0

    def add(name, cond, values, bound):
        nonlocal k
        results.append(check(name, cond, values, bound, seed=k))
        k += 1

    def guard(name, other, mass):
        # 시간 초과율 보조 기준: 제안 − 비교군 < +TIMEOUT_MARGIN (#184 G 보상에서 시간 초과가 8~10%로 늘어서)
        add(name, f'{mass:g} kg 시간 초과율 Δ(제안 − {other.upper()}) [보조]', paired(
            per_seed(g, 'proposed', 'exact', mass, timeout_rate),
            per_seed(g, other, 'exact', mass, timeout_rate)), TIMEOUT_MARGIN)

    if {'proposed', 'b1'} <= variants:
        for mass in (10.0, 25.0):
            add('H1', f'{mass:g} kg 충돌률 Δ(제안 − B1)', paired(
                per_seed(g, 'proposed', 'exact', mass, collision_rate),
                per_seed(g, 'b1', 'exact', mass, collision_rate)), 0.0)
            guard('H1', 'b1', mass)
    if {'proposed', 'b2'} <= variants:
        for mass in TRAIN_MASSES:
            add('H2-a', f'{mass:g} kg 충돌률 Δ(제안 − B2)', paired(
                per_seed(g, 'proposed', 'exact', mass, collision_rate),
                per_seed(g, 'b2', 'exact', mass, collision_rate)), MARGIN)
        for info in INFO_ERRORS:
            for mass in TRAIN_MASSES:
                add('H2-b', f'{info} {mass:g} kg 악화폭 차이(제안 − B2)',
                    paired(degradation(g, 'proposed', info, mass), degradation(g, 'b2', info, mass)), 0.0)
    if {'proposed', 'r2'} <= variants:
        for mass in UNSEEN_MASSES:
            add('H3', f'{mass:g} kg 충돌률 Δ(제안 − R2)', paired(
                per_seed(g, 'proposed', 'exact', mass, collision_rate),
                per_seed(g, 'r2', 'exact', mass, collision_rate)), MARGIN)
            add('H3', f'{mass:g} kg 도착 시간 Δ(제안 − R2) [s]',
                arrival_diffs(g, 'proposed', 'r2', 'exact', mass), 0.0)
            guard('H3', 'r2', mass)

    # 가설 하나 안의 여러 조건을 따로 볼 때의 Holm 보정 (보조 지표). 짝지은 t의 한쪽 p, 유의수준 0.025
    for name in sorted({r['hypothesis'] for r in results}):
        sub = [r for r in results if r['hypothesis'] == name]
        for r, rej in zip(sub, holm([r['p_t'] for r in sub], alpha=0.025)):
            r['holm_reject'] = rej
    return results


def seed_table(rows: list[dict]) -> list[dict]:
    g = group(rows)
    out = []
    for (v, info, mass, seed), eps in sorted(g.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2],
                                                                         -1 if kv[0][3] is None else kv[0][3])):
        times = [e['time_s'] for e in eps if e['success']]
        appeared = [e for e in eps if e['appeared']]
        out.append({'variant': v, 'info': info, 'mass': mass, 'seed': seed, 'episodes': len(eps),
                    'popout_appeared': len(appeared), 'collision_rate': collision_rate(eps),
                    # 충돌 원인: 출현 순간 설 수 있었는데 충돌 / 너무 빨라 못 섬
                    'collision_stoppable': sum(e['collision'] and e['stoppable'] for e in appeared),
                    'collision_too_fast': sum(e['collision'] and not e['stoppable'] for e in appeared),
                    'success_rate': success_rate(eps),
                    'arrival_time_s': sum(times) / len(times) if times else None})
    return out


def fmt(v, digits=3) -> str:
    return '—' if v is None else f'{v:.{digits}f}'


def to_markdown(results: list[dict], warn: set) -> str:
    lines = ['# 연구 2 가설 판정', '']
    for w in sorted(warn):
        lines.append(f'> 경고: {w}')
    if warn:
        lines.append('')
    lines += ['| 가설 | 판정 (조건 모두 통과해야 PASS) |', '| --- | --- |']
    for name, v in hypothesis_verdicts(results).items():
        lines.append(f"| {name} | {({True: 'PASS', False: 'FAIL', None: '—'})[v]} |")
    lines += ['', '| 가설 | 조건 | 시드 | 평균 | 95% CI (부트스트랩) | t CI 상한 | 기준 (CI 상한 <) | 판정 | Holm |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in results:
        verdict = {True: 'PASS', False: 'FAIL', None: '—'}[r['pass']]
        holm_mark = {True: '기각', False: '유지', None: ''}[r.get('holm_reject')]
        ci = '—' if r['ci_low'] is None else f"{fmt(r['ci_low'])} ~ {fmt(r['ci_high'])}"
        lines.append(f"| {r['hypothesis']} | {r['condition']} | {r['n']} | {fmt(r['mean'])} | {ci} | "
                     f"{fmt(r.get('t_ci_high'))} | {r['bound']:g} | {verdict} | {holm_mark} |")
    lines += ['', '충돌률 분모는 돌발 장애물이 나타난 에피소드 전체다(popout_appeared). 충돌 원인(설 수 있었는데 / 너무 빨라)은 per_seed.csv에 있다. '
              '도착 시간은 두 비교군이 모두 성공한 같은 시나리오끼리 비교한다.']
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='연구 2 episodes.csv → 시드별 요약, H1~H3 판정')
    ap.add_argument('csv', nargs='+', type=Path)
    ap.add_argument('--out', type=Path, help='결과 폴더 (hypotheses.md, hypotheses.json, per_seed.csv)')
    args = ap.parse_args(argv)
    rows, warn = load(args.csv)
    results = judge(rows)
    md = to_markdown(results, warn | FIXED_WARNINGS)
    print(md)
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / 'hypotheses.md').write_text(md, encoding='utf-8')
        (args.out / 'hypotheses.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
        table = seed_table(rows)
        with (args.out / 'per_seed.csv').open('w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(table[0]))
            w.writeheader()
            w.writerows(table)
    return 0


if __name__ == '__main__':
    sys.exit(main())
