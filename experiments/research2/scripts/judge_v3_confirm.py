"""v3 효율 확인 실험 판정 (run_v3_confirm.sh). 결과를 보기 전에 썼다.

    uv run python scripts/judge_v3_confirm.py raw/v3_confirm --cand pen100

벌점 곡선 실험(raw/v3_frontier, 시드 310~329)의 탐색 분석에서 "벌금 −100 단계끼리 비교하면 충돌률은 같고(−0.1%p) 0 kg에서
제안이 0.81 s 빠르다(CI −1.65 ~ −0.05)"가 나왔다. 결과를 본 뒤 고른 비교라, 같은 계산을 새 시드(330~369, 40개)에서 미리 정한
기준으로 다시 판정한다. 계산은 탐색 때와 같다:
  시드마다  coll = S1에서 돌발 장애물이 나타난 판 중 충돌 비율 (학습 질량 전체),  t0 = 0 kg 성공 판의 평균 도착 시간
  차이 = 같은 시드(같은 평가 목록)의 제안 − R2,  95% CI = 시드 부트스트랩 (4000회, 난수 0)

판정 (모두 만족해야 "같은 안전에서 가벼울 때 더 빠름" 확인):
  C1 효율: t0 차이의 95% CI 상한 < 0
  C2 안전 비열등: coll 차이의 95% CI 상한 ≤ +2%p
보조 (판정과 별개로 보고):
  C3 질량 의존: (25 kg 시간 − 0 kg 시간) 차이의 95% CI 하한 > 0 (벌점 곡선 실험 F2의 재현)
  참고: 두 비교군이 모두 성공한 같은 시나리오끼리 짝지은 0 kg 시간 차, 질량별 충돌·시간, 무너진 시드(성공 < 85% 또는
  시간 초과 > 10%) 수. 무너진 시드는 빼지 않는다
"""

import argparse
import csv
from pathlib import Path

import numpy as np

B = 4000


def coll(rows):
    pop = [r for r in rows if r['popout_appeared'] == 'True']
    return sum(r['outcome'] == 'collision' for r in pop) / len(pop)


def t_mass(rows, m):
    t = [float(r['time_s']) for r in rows if r['outcome'] == 'success' and r['payload_kg'] == m]
    return float(np.mean(t)) if t else np.nan


def collapsed(rows):
    n = len(rows)
    return (sum(r['outcome'] == 'success' for r in rows) / n < 0.85
            or sum(r['outcome'] == 'timeout' for r in rows) / n > 0.10)


def both_success_t0(p_rows, r_rows):
    key = lambda r: (r['scenario'], r['scenario_seed'])  # noqa: E731
    p = {key(r): float(r['time_s']) for r in p_rows if r['payload_kg'] == '0.0' and r['outcome'] == 'success'}
    q = {key(r): float(r['time_s']) for r in r_rows if r['payload_kg'] == '0.0' and r['outcome'] == 'success'}
    k = p.keys() & q.keys()
    return float(np.mean([p[x] - q[x] for x in k])) if k else np.nan


def ci(d, rng):
    d = np.asarray(d, dtype=float)
    d = d[~np.isnan(d)]
    b = [rng.choice(d, len(d)).mean() for _ in range(B)]
    lo, hi = np.percentile(b, [2.5, 97.5])
    return d.mean(), lo, hi, len(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path)
    ap.add_argument('--cand', default='pen100')
    args = ap.parse_args()
    runs = [line.split() for line in (args.root / 'runs.txt').read_text().splitlines() if line.strip()]
    seeds = sorted({int(s) for c, v, s, d in runs if c == args.cand})
    load = lambda v, s: list(csv.DictReader(open(args.root / f'eval_{args.cand}_{v}_s{s}' / 'episodes.csv')))  # noqa: E731
    data = {(v, s): load(v, s) for v in ('proposed', 'r2') for s in seeds}
    rng = np.random.default_rng(0)
    dt0 = [t_mass(data[('proposed', s)], '0.0') - t_mass(data[('r2', s)], '0.0') for s in seeds]
    dcol = [coll(data[('proposed', s)]) - coll(data[('r2', s)]) for s in seeds]
    dslope = [(t_mass(data[('proposed', s)], '25.0') - t_mass(data[('proposed', s)], '0.0'))
              - (t_mass(data[('r2', s)], '25.0') - t_mass(data[('r2', s)], '0.0')) for s in seeds]
    dpair = [both_success_t0(data[('proposed', s)], data[('r2', s)]) for s in seeds]
    m1, lo1, hi1, _ = ci(dt0, rng)
    m2, lo2, hi2, _ = ci(dcol, rng)
    m3, lo3, hi3, _ = ci(dslope, rng)
    m4, lo4, hi4, n4 = ci(dpair, rng)
    c1, c2, c3 = hi1 < 0, hi2 <= 0.02, lo3 > 0
    L = ['# v3 효율 확인 실험 판정', '',
         f'폴더 `{args.root}`, 벌금 단계 {args.cand}, 시드 {seeds[0]}~{seeds[-1]} ({len(seeds)}개). 차이는 제안 − R2, 괄호는 95% CI.', '',
         '## 비교군별 (시드 평균)', '', '| 비교군 | 충돌률 | 0 kg 충돌 | 25 kg 충돌 | 0 kg 시간 | 25 kg 시간 | 성공 | 무너진 시드 |',
         '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for v in ('proposed', 'r2'):
        rows = [data[(v, s)] for s in seeds]
        mc = lambda f: float(np.mean([f(r) for r in rows]))  # noqa: E731
        cm = lambda m: mc(lambda r: coll([x for x in r if x['payload_kg'] == m]))  # noqa: E731
        L.append(f"| {v} | {mc(coll)*100:.1f}% | {cm('0.0')*100:.1f}% | {cm('25.0')*100:.1f}% | "
                 f"{np.nanmean([t_mass(r, '0.0') for r in rows]):.2f} s | {np.nanmean([t_mass(r, '25.0') for r in rows]):.2f} s | "
                 f"{mc(lambda r: sum(x['outcome'] == 'success' for x in r) / len(r))*100:.1f}% | "
                 f"{sum(collapsed(r) for r in rows)}/{len(rows)} |")
    L += ['', '## 판정', '', '| 기준 | 값 | 결과 |', '| --- | --- | --- |',
          f'| C1 0 kg 도착 시간 차 (CI 상한 < 0) | {m1:+.2f} s ({lo1:+.2f} ~ {hi1:+.2f}) | {"통과" if c1 else "미달"} |',
          f'| C2 충돌률 차 (CI 상한 ≤ +2%p) | {m2*100:+.1f}%p ({lo2*100:+.1f} ~ {hi2*100:+.1f}) | {"통과" if c2 else "미달"} |',
          f'| C3 질량 의존 (25 kg − 0 kg 시간 차, CI 하한 > 0, 보조) | {m3:+.2f} s ({lo3:+.2f} ~ {hi3:+.2f}) | {"통과" if c3 else "미달"} |',
          '', f'**결론: {"같은 안전에서 가벼울 때 더 빠름 확인 (C1·C2 통과)" if c1 and c2 else "확인 안 됨"}**', '',
          f'참고: 두 비교군이 모두 성공한 같은 시나리오끼리 0 kg 시간 차 {m4:+.2f} s ({lo4:+.2f} ~ {hi4:+.2f}, 시드 {n4}개)', '']
    text = '\n'.join(L)
    (args.root / 'judge.md').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
