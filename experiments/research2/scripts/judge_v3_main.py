"""v3 본 실험 판정 (run_v3_main.sh, 사전 등록 v3_plan.md). 결과를 보기 전에 썼다.

    uv run python scripts/judge_v3_main.py raw/v3_main

입력: <폴더>/eval_<비교군>_s<시드>/episodes.csv. 제안(proposed)은 정보 조건 exact·unknown·told0·told25, R1·R2는 exact.
      <폴더>/mass_response.csv (probe_v3_mass_response.py, 있으면 질량 적응도를 계산한다).
학습 질량은 0·2·10·25 kg이다. 15·30 kg(처음 보는 질량)은 판정에 넣지 않고 따로 보고한다.
시드마다 계산하고 시드 평균의 95% CI는 시드 부트스트랩(4000회, 난수 0)이다.
  충돌률: S1에서 돌발 장애물이 나타난 판 중 충돌 비율
  짝지은 시간 차: 같은 시나리오(scenario, scenario_seed)·같은 질량에서 두 쪽 모두 성공한 판끼리 도착 시간 차의 평균
  자기 성공 판 시간: 그 비교군이 성공한 판의 평균 도착 시간

판정 (v3_plan.md 4절과 같다):
  주 지표   I1 0 kg 도착 시간 제안 exact − unknown (짝지음)                 95% CI 상한 < 0
            I2 학습 질량 전체 충돌률 제안 exact − unknown                    95% CI 상한 ≤ +2%p
  정보 의존 I3 25 kg 충돌률 제안 told0 − exact                               95% CI 하한 > 0
  질량 의존 M1 (25 kg − 0 kg 자기 성공 판 시간) 제안 exact − R2               95% CI 하한 > 0
  해석 기준 I0 0 kg 도착 시간 제안 unknown − R2 (짝지음): CI 하한 > 0이면 I1을 "덜 연습한 영향이 섞인 상한"으로 쓴다
  보조      C1 0 kg 자기 성공 판 시간 제안 exact − R2 CI 상한 < 0, C2 학습 질량 전체 충돌률 제안 exact − R2 CI 상한 ≤ +2%p
            R1 대비 10·25 kg 충돌률 제안 exact − R1 (보고), 처음 보는 질량 15·30 kg (보고), 질량 적응도 (보고)
  무너진 시드(학습 질량 성공 < 85% 또는 시간 초과 > 10%)는 빼지 않는다. 비교군별 수를 보고하고, 제안의 무너진 시드를 뺀
  I1·I2·I3 민감도 분석을 덧붙인다
"""

import argparse
import csv
from pathlib import Path

import numpy as np

B = 4000
TRAIN = ('0.0', '2.0', '10.0', '25.0')
UNSEEN = ('15.0', '30.0')
CONDS = ('exact', 'unknown', 'told0', 'told25')


def coll(rows, masses):
    pop = [r for r in rows if r['popout_appeared'] == 'True' and r['payload_kg'] in masses]
    return sum(r['outcome'] == 'collision' for r in pop) / len(pop) if pop else np.nan


def own_time(rows, m):
    t = [float(r['time_s']) for r in rows if r['payload_kg'] == m and r['outcome'] == 'success']
    return float(np.mean(t)) if t else np.nan


def paired_time(a, b, m):
    key = lambda r: (r['scenario'], r['scenario_seed'])  # noqa: E731
    A = {key(r): float(r['time_s']) for r in a if r['payload_kg'] == m and r['outcome'] == 'success'}
    Bm = {key(r): float(r['time_s']) for r in b if r['payload_kg'] == m and r['outcome'] == 'success'}
    k = A.keys() & Bm.keys()
    return float(np.mean([A[x] - Bm[x] for x in k])) if k else np.nan


def collapsed(rows):
    tr = [r for r in rows if r['payload_kg'] in TRAIN]
    n = len(tr)
    return (sum(r['outcome'] == 'success' for r in tr) / n < 0.85
            or sum(r['outcome'] == 'timeout' for r in tr) / n > 0.10)


def ci(d, rng):
    d = np.asarray(d, dtype=float)
    d = d[~np.isnan(d)]
    if len(d) == 0:
        return np.nan, np.nan, np.nan, 0
    b = [rng.choice(d, len(d)).mean() for _ in range(B)]
    lo, hi = np.percentile(b, [2.5, 97.5])
    return float(d.mean()), float(lo), float(hi), len(d)


def fmt(c, scale=1.0, unit=' s'):
    m, lo, hi, n = c
    return f'{m*scale:+.2f}{unit} ({lo*scale:+.2f} ~ {hi*scale:+.2f}, 시드 {n})'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path)
    args = ap.parse_args()
    R = args.root
    seeds = sorted(int(p.name.split('_s')[-1]) for p in R.glob('eval_proposed_s*') if (p / 'episodes.csv').exists())
    read = lambda v, s: list(csv.DictReader(open(R / f'eval_{v}_s{s}' / 'episodes.csv'))) if (R / f'eval_{v}_s{s}' / 'episodes.csv').exists() else []  # noqa: E731
    prop = {s: {c: [r for r in read('proposed', s) if r['mass_obs'] == c] for c in CONDS} for s in seeds}
    r2 = {s: read('r2', s) for s in seeds}
    r1 = {s: read('r1', s) for s in seeds}
    rng = np.random.default_rng(0)
    E, U, T0 = (lambda s: prop[s]['exact']), (lambda s: prop[s]['unknown']), (lambda s: prop[s]['told0'])
    has_r2 = [s for s in seeds if r2[s]]
    has_r1 = [s for s in seeds if r1[s]]

    def calc(ss):
        return {
            'I1': ci([paired_time(E(s), U(s), '0.0') for s in ss], rng),
            'I2': ci([coll(E(s), TRAIN) - coll(U(s), TRAIN) for s in ss], rng),
            'I3': ci([coll(T0(s), ('25.0',)) - coll(E(s), ('25.0',)) for s in ss], rng),
        }
    main_ = calc(seeds)
    i0t = ci([paired_time(U(s), r2[s], '0.0') for s in has_r2], rng)
    i0c = ci([coll(U(s), TRAIN) - coll(r2[s], TRAIN) for s in has_r2], rng)
    m1 = ci([(own_time(E(s), '25.0') - own_time(E(s), '0.0')) - (own_time(r2[s], '25.0') - own_time(r2[s], '0.0')) for s in has_r2], rng)
    m2 = ci([(own_time(E(s), '25.0') - own_time(E(s), '0.0')) - (own_time(U(s), '25.0') - own_time(U(s), '0.0')) for s in seeds], rng)
    c1 = ci([own_time(E(s), '0.0') - own_time(r2[s], '0.0') for s in has_r2], rng)
    c2 = ci([coll(E(s), TRAIN) - coll(r2[s], TRAIN) for s in has_r2], rng)
    p = {'I1': main_['I1'][2] < 0, 'I2': main_['I2'][2] <= 0.02, 'I3': main_['I3'][1] > 0, 'M1': m1[1] > 0,
         'C1': c1[2] < 0, 'C2': c2[2] <= 0.02}
    i0_slow = i0t[1] > 0

    L = ['# v3 본 실험 판정', '', f'폴더 `{R}`, 제안 정책 {len(seeds)}개 (시드 {seeds[0]}~{seeds[-1]}), R2 {len(has_r2)}개, R1 {len(has_r1)}개. 괄호는 95% CI.', '',
         '## 비교군·조건별 (시드 평균, 학습 질량)', '',
         '| 비교군·조건 | 학습 질량 충돌 | 0 kg 충돌 | 25 kg 충돌 | 0 kg 시간 | 25 kg 시간 | 무너진 시드 |', '| --- | --- | --- | --- | --- | --- | --- |']
    groups = [(f'제안 {c}', [prop[s][c] for s in seeds]) for c in CONDS] + [('R2', [r2[s] for s in has_r2]), ('R1', [r1[s] for s in has_r1])]
    for name, rs in groups:
        if not rs:
            continue
        L.append(f"| {name} | {np.nanmean([coll(r, TRAIN) for r in rs])*100:.1f}% | {np.nanmean([coll(r, ('0.0',)) for r in rs])*100:.1f}% | "
                 f"{np.nanmean([coll(r, ('25.0',)) for r in rs])*100:.1f}% | {np.nanmean([own_time(r, '0.0') for r in rs]):.2f} s | "
                 f"{np.nanmean([own_time(r, '25.0') for r in rs]):.2f} s | {sum(collapsed(r) for r in rs)}/{len(rs)} |")
    ok = lambda k: '통과' if p[k] else '미달'  # noqa: E731
    L += ['', '## 판정', '', '| 기준 | 값 | 결과 |', '| --- | --- | --- |',
          f"| I1 0 kg 도착 시간 exact − unknown (상한 < 0) | {fmt(main_['I1'])} | {ok('I1')} |",
          f"| I2 학습 질량 충돌률 exact − unknown (상한 ≤ +2%p) | {fmt(main_['I2'], 100, '%p')} | {ok('I2')} |",
          f"| I3 25 kg 충돌률 told0 − exact (하한 > 0) | {fmt(main_['I3'], 100, '%p')} | {ok('I3')} |",
          f"| M1 (25 kg − 0 kg 시간) 제안 − R2 (하한 > 0) | {fmt(m1)} | {ok('M1')} |",
          f"| C1 (보조) 0 kg 시간 제안 − R2 (상한 < 0) | {fmt(c1)} | {ok('C1')} |",
          f"| C2 (보조) 학습 질량 충돌률 제안 − R2 (상한 ≤ +2%p) | {fmt(c2, 100, '%p')} | {ok('C2')} |", '',
          f"- **주 결론 (I1·I2·I3)**: {'질량 정보가 정책 안에서 같은 안전의 효율을 높이고, 정책이 그 정보에 의존한다' if p['I1'] and p['I2'] and p['I3'] else '모두 통과하지는 못함'}",
          f"- **질량 의존 (M1)**: {'제안은 R2보다 무거울수록 더 느려진다' if p['M1'] else '확인 안 됨'}",
          f"- **학습 결과로서의 이득 (C1·C2, 보조)**: {'확인' if p['C1'] and p['C2'] else '확인 안 됨'}", '',
          '## 해석 기준 I0 (제안 unknown vs R2, 둘 다 정보 없음)', '',
          f'- 0 kg 도착 시간 unknown − R2: {fmt(i0t)}', f'- 학습 질량 충돌률 unknown − R2: {fmt(i0c, 100, "%p")}',
          f"- 해석: {'unknown이 R2보다 확실히 느리다 → I1은 덜 연습한 영향이 섞인 상한' if i0_slow else 'unknown을 정보 없는 운전의 공정한 기준으로 본다'}", '',
          '## 참고 (판정 아님)', '',
          f'- 같은 정책 안의 질량 의존 (25 kg − 0 kg 시간) exact − unknown: {fmt(m2)}',
          f"- 25 kg 충돌률 exact − unknown: {fmt(ci([coll(E(s), ('25.0',)) - coll(U(s), ('25.0',)) for s in seeds], rng), 100, '%p')}",
          f"- 0 kg 도착 시간 told25 − exact: {fmt(ci([paired_time(prop[s]['told25'], E(s), '0.0') for s in seeds], rng))}"]
    for m in ('10.0', '25.0'):
        if has_r1:
            L.append(f"- {float(m):g} kg 충돌률 제안 exact − R1: {fmt(ci([coll(E(s), (m,)) - coll(r1[s], (m,)) for s in has_r1], rng), 100, '%p')}")
    for m in UNSEEN:
        L.append(f"- 처음 보는 {float(m):g} kg: 충돌 제안 exact − R2 {fmt(ci([coll(E(s), (m,)) - coll(r2[s], (m,)) for s in has_r2], rng), 100, '%p')}, "
                 f"도착 시간 제안 exact − unknown {fmt(ci([paired_time(E(s), U(s), m) for s in seeds], rng))}")
    keep = [s for s in seeds if not collapsed(E(s))]
    if len(keep) < len(seeds):
        sens = calc(keep)
        L += ['', f'## 민감도: 제안의 무너진 시드 {len(seeds) - len(keep)}개를 뺀 I1·I2·I3', '',
              f"- I1 {fmt(sens['I1'])}, I2 {fmt(sens['I2'], 100, '%p')}, I3 {fmt(sens['I3'], 100, '%p')}"]
    resp = R / 'mass_response.csv'
    if resp.exists():
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from payload_nav.rule_policy import safe_cruise_speed
        rows = list(csv.DictReader(open(resp)))
        by = {(int(r['seed']), r['mass']): float(r['v_cmd']) for r in rows}
        ss = sorted({int(r['seed']) for r in rows})
        orc = 1 - safe_cruise_speed(25.0) / safe_cruise_speed(0.0)
        ad = ci([(1 - by[(s, '25.0')] / by[(s, '0.0')]) / orc for s in ss], rng)
        unk = np.mean([by[(s, 'unknown')] / by[(s, '0.0')] for s in ss])
        L += ['', '## 질량 적응도 (반응 곡선)', '',
              f'- 25 kg 감속 ÷ 오라클 감속({orc*100:.1f}%): {ad[0]*100:.0f}% ({ad[1]*100:.0f} ~ {ad[2]*100:.0f}%, 정책 {ad[3]}개)',
              f'- "모름"일 때 속도 명령 비율 (0 kg = 1): {unk:.3f}']
    text = '\n'.join(L) + '\n'
    (R / 'judge.md').write_text(text)
    print(text)


if __name__ == '__main__':
    main()
