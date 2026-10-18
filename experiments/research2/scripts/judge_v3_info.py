"""v3 정보 실험 판정 (run_v3_info.sh). 결과를 보기 전에 썼다.

    uv run python scripts/judge_v3_info.py raw/v3_info

같은 제안 정책에 질량 정보만 바꿔 준다: exact(정확), unknown(가림), told0(늘 "0 kg"), told25(늘 "25 kg").
정책·시나리오가 같으므로 시드(학습한 정책) 사이 편차가 빠진 짝지은 비교다. 제안 정책은 학습의 20%를 가림 상태로 연습했다.
입력: <폴더>/eval_proposed_s<시드>/episodes.csv (네 조건), <폴더>/eval_r2_s<시드>/episodes.csv (참고, exact만).

시드마다 계산하고, 시드 평균의 95% CI는 시드 부트스트랩(4000회, 난수 0)이다.
  시간 차: 같은 시나리오(scenario, scenario_seed)·같은 질량에서 두 조건 모두 성공한 판끼리 도착 시간 차의 평균
  충돌률: S1에서 돌발 장애물이 나타난 판 중 충돌 비율

판정 (미리 정함):
  I1 효율: 0 kg 도착 시간 exact − unknown 의 CI 상한 < 0           (알려 주면 가벼울 때 더 빨리 간다)
  I2 안전 유지: 학습 질량 전체 충돌률 exact − unknown 의 CI 상한 ≤ +2%p (알려 줘서 빨라진 만큼 덜 안전해지지 않는다.
     가벼울 때 빨라지면 가벼울 때 충돌이 늘 수 있어 25 kg만이 아니라 전체로 본다. 확인 실험 C2와 같은 기준)
  I3 인과: 25 kg 충돌률 told0 − exact 의 CI 하한 > 0               (무거운데 "가볍다"고 속이면 더 부딪힌다 = 정보에 의존)
  I1·I2·I3 모두 통과하면 "질량 정보가 정책 안에서 효율을 높이고, 정책이 그 정보에 의존한다"로 쓴다
타당성 점검 I0 (판정이 아니라 I1을 어떻게 읽을지 정한다, 미리 정함):
  제안의 unknown은 학습의 20%에서만 연습해, 정보 없이 늘 연습한 R2보다 필요 이상으로 느릴 수 있다. 그러면 I1에 정보 값어치
  외에 "덜 연습한 영향"이 섞인다. 0 kg 도착 시간 제안 unknown − R2 (같은 시드 번호·같은 시나리오, 둘 다 성공한 판)와
  학습 질량 전체 충돌률 차이를 보고한다. 시간 차의 95% CI 하한 > 0 (unknown이 R2보다 확실히 느림)이면 I1을 "연습 부족 영향이
  섞인 상한"으로 쓰고, 아니면 unknown을 정보 없는 운전의 공정한 기준으로 본다
참고 (판정 아님): 0 kg·25 kg 충돌률 exact − unknown, 0 kg 도착 시간 told25 − exact (가벼운데 "무겁다"고 하면 느려지는지),
  조건별 질량별 충돌·속도,
  R2(늘 모름으로 학습)와 제안 unknown의 비교(가림 상태 연습이 적은 영향을 가리는 데 참고)
"""

import argparse
import csv
from pathlib import Path

import numpy as np

B = 4000
MASSES = ['0.0', '2.0', '10.0', '25.0']
CONDS = ('exact', 'unknown', 'told0', 'told25')


def coll(rows, mass):
    pop = [r for r in rows if r['popout_appeared'] == 'True' and r['payload_kg'] == mass]
    return sum(r['outcome'] == 'collision' for r in pop) / len(pop) if pop else np.nan


def coll_all(rows):
    pop = [r for r in rows if r['popout_appeared'] == 'True']
    return sum(r['outcome'] == 'collision' for r in pop) / len(pop) if pop else np.nan


def paired_time(a_rows, b_rows, mass):
    key = lambda r: (r['scenario'], r['scenario_seed'])  # noqa: E731
    a = {key(r): float(r['time_s']) for r in a_rows if r['payload_kg'] == mass and r['outcome'] == 'success'}
    b = {key(r): float(r['time_s']) for r in b_rows if r['payload_kg'] == mass and r['outcome'] == 'success'}
    k = a.keys() & b.keys()
    return float(np.mean([a[x] - b[x] for x in k])) if k else np.nan


def speed(rows, mass):
    s = [float(r['mean_speed']) for r in rows if r['payload_kg'] == mass and r['outcome'] == 'success']
    return float(np.mean(s)) if s else np.nan


def ci(d, rng):
    d = np.asarray(d, dtype=float)
    d = d[~np.isnan(d)]
    b = [rng.choice(d, len(d)).mean() for _ in range(B)]
    lo, hi = np.percentile(b, [2.5, 97.5])
    return float(d.mean()), float(lo), float(hi), len(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path)
    args = ap.parse_args()
    seeds = sorted(int(p.name.split('_s')[-1]) for p in args.root.glob('eval_proposed_s*') if (p / 'episodes.csv').exists())
    prop, r2 = {}, {}
    for s in seeds:
        rows = list(csv.DictReader(open(args.root / f'eval_proposed_s{s}' / 'episodes.csv')))
        prop[s] = {c: [r for r in rows if r['mass_obs'] == c] for c in CONDS}
        p = args.root / f'eval_r2_s{s}' / 'episodes.csv'
        r2[s] = list(csv.DictReader(open(p))) if p.exists() else []
    rng = np.random.default_rng(0)
    i1 = ci([paired_time(prop[s]['exact'], prop[s]['unknown'], '0.0') for s in seeds], rng)
    i2 = ci([coll_all(prop[s]['exact']) - coll_all(prop[s]['unknown']) for s in seeds], rng)
    c0d = ci([coll(prop[s]['exact'], '0.0') - coll(prop[s]['unknown'], '0.0') for s in seeds], rng)
    c25d = ci([coll(prop[s]['exact'], '25.0') - coll(prop[s]['unknown'], '25.0') for s in seeds], rng)
    i3 = ci([coll(prop[s]['told0'], '25.0') - coll(prop[s]['exact'], '25.0') for s in seeds], rng)
    ref = ci([paired_time(prop[s]['told25'], prop[s]['exact'], '0.0') for s in seeds], rng)
    have_r2 = [s for s in seeds if r2[s]]
    i0t = ci([paired_time(prop[s]['unknown'], r2[s], '0.0') for s in have_r2], rng)
    i0c = ci([coll_all(prop[s]['unknown']) - coll_all(r2[s]) for s in have_r2], rng)
    i0_slow = i0t[1] > 0
    p1, p2, p3 = i1[2] < 0, i2[2] <= 0.02, i3[1] > 0
    L = ['# v3 정보 실험 판정', '', f'폴더 `{args.root}`, 제안 정책 {len(seeds)}개 (시드 {seeds[0]}~{seeds[-1]}). 괄호는 95% CI.', '',
         '## 조건별 (시드 평균)', '',
         '| 조건 | 전체 충돌 | 0 kg 충돌 | 25 kg 충돌 | 0 kg 속도 | 25 kg 속도 | 0 → 25 kg 속도 |', '| --- | --- | --- | --- | --- | --- | --- |']
    rows_by = [(f'제안 {c}', [prop[s][c] for s in seeds]) for c in CONDS] + [('R2 (참고)', [r2[s] for s in seeds if r2[s]])]
    for name, rs in rows_by:
        if not rs:
            continue
        c0, c25 = np.nanmean([coll(r, '0.0') for r in rs]), np.nanmean([coll(r, '25.0') for r in rs])
        v0, v25 = np.nanmean([speed(r, '0.0') for r in rs]), np.nanmean([speed(r, '25.0') for r in rs])
        ca = np.nanmean([coll_all(r) for r in rs])
        L.append(f'| {name} | {ca*100:.1f}% | {c0*100:.1f}% | {c25*100:.1f}% | {v0:.3f} | {v25:.3f} | {(v25 / v0 - 1)*100:+.1f}% |')
    L += ['', '## 판정', '', '| 기준 | 값 | 결과 |', '| --- | --- | --- |',
          f'| I1 0 kg 도착 시간 exact − unknown (CI 상한 < 0) | {i1[0]:+.2f} s ({i1[1]:+.2f} ~ {i1[2]:+.2f}) | {"통과" if p1 else "미달"} |',
          f'| I2 학습 질량 전체 충돌률 exact − unknown (CI 상한 ≤ +2%p) | {i2[0]*100:+.1f}%p ({i2[1]*100:+.1f} ~ {i2[2]*100:+.1f}) | {"통과" if p2 else "미달"} |',
          f'| I3 25 kg 충돌률 told0 − exact (CI 하한 > 0) | {i3[0]*100:+.1f}%p ({i3[1]*100:+.1f} ~ {i3[2]*100:+.1f}) | {"통과" if p3 else "미달"} |',
          '', f'**결론: {"질량 정보가 정책 안에서 효율을 높이고, 정책이 그 정보에 의존한다 (I1·I2·I3 통과)" if p1 and p2 and p3 else "모두 통과하지는 못함"}**', '',
          '## 타당성 점검 I0 (제안 unknown vs R2, 둘 다 정보 없음)', '',
          f'- 0 kg 도착 시간 제안 unknown − R2: {i0t[0]:+.2f} s ({i0t[1]:+.2f} ~ {i0t[2]:+.2f}), 시드 {i0t[3]}개',
          f'- 학습 질량 전체 충돌률 제안 unknown − R2: {i0c[0]*100:+.1f}%p ({i0c[1]*100:+.1f} ~ {i0c[2]*100:+.1f})',
          f'- 해석: ' + ('unknown이 R2보다 확실히 느리다 → I1은 "덜 연습한 영향이 섞인 상한"으로 쓴다' if i0_slow
                       else 'unknown을 정보 없는 운전의 공정한 기준으로 본다 (R2보다 확실히 느리지 않음)'), '',
          f'참고: 충돌률 exact − unknown 0 kg {c0d[0]*100:+.1f}%p ({c0d[1]*100:+.1f} ~ {c0d[2]*100:+.1f}), '
          f'25 kg {c25d[0]*100:+.1f}%p ({c25d[1]*100:+.1f} ~ {c25d[2]*100:+.1f})',
          f'참고: 0 kg 도착 시간 told25 − exact {ref[0]:+.2f} s ({ref[1]:+.2f} ~ {ref[2]:+.2f}) (가벼운데 "무겁다"고 하면 느려지는지)', '']
    text = '\n'.join(L)
    (args.root / 'judge.md').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
