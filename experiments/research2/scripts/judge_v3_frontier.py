"""v3 벌점 곡선 실험 판정 (run_v3_frontier.sh). 결과를 보기 전에 썼다.

    uv run python scripts/judge_v3_frontier.py raw/v3_frontier --cands pen100 pen300 pen1000

입력: <폴더>/eval_<후보>_<비교군>_s<시드>/episodes.csv (env_v0 시나리오 S1·S2 × 학습 질량 × 200, 질량 정보 정확),
     <폴더>/runs.txt (후보 비교군 시드 학습폴더). 출력은 표준 출력과 <폴더>/judge.md.

곡선: 벌점 단계마다 비교군의 점 (학습 질량 전체 충돌률, 도착 시간). 충돌률 분모는 S1에서 돌발 장애물이 나타난 판(analyze.py와
같음), 도착 시간은 그 비교군이 성공한 판의 평균이다. R2의 점들을 충돌률 순으로 이어(선형 보간) 제안 점과 같은 충돌률의
R2 도착 시간을 구한다. R2 충돌률 범위 밖의 제안 점은 비교하지 않는다(외삽 안 함). 95% CI는 시드를 부트스트랩한다
(같은 부트스트랩 표본으로 제안·R2 곡선을 같이 다시 만든다).

판정 (run_v3_frontier.sh 머리말과 같다):
  F1 핵심: 같은 충돌률(학습 질량 전체)에서 0 kg 도착 시간 제안 − R2 (범위 안 제안 점들의 평균)의 95% CI 상한 < 0
  F2 질량 의존: 시드마다 (25 kg 시간 − 0 kg 시간)의 제안 − R2 (벌점 단계 평균)의 95% CI 하한 > 0
  F3 보고: 무너진 시드(학습 질량 성공 < 85% 또는 시간 초과 > 10%) 수를 후보·비교군별로 (빼지 않는다)
참고: 25 kg 같은 비교, 질량별 충돌률·시간, 실제 정규화 벌점(학습 끝 보상 정규화 σ로 나눈 1 m/s 충돌 벌점)
"""

import argparse
import csv
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

MASSES = ['0.0', '2.0', '10.0', '25.0']
VARIANTS = ('proposed', 'r2')
B = 4000


def load(root: Path, cands, seeds):
    data = {}
    for c in cands:
        for v in VARIANTS:
            for s in seeds:
                with open(root / f'eval_{c}_{v}_s{s}' / 'episodes.csv') as f:
                    data[(c, v, s)] = list(csv.DictReader(f))
    return data


def coll(rows, mass=None):
    pop = [r for r in rows if r['popout_appeared'] == 'True' and (mass is None or r['payload_kg'] == mass)]
    return sum(r['outcome'] == 'collision' for r in pop) / len(pop) if pop else np.nan


def mean_time(rows, mass=None):
    t = [float(r['time_s']) for r in rows if r['outcome'] == 'success' and (mass is None or r['payload_kg'] == mass)]
    return float(np.mean(t)) if t else np.nan


def collapsed(rows):
    n = len(rows)
    return (sum(r['outcome'] == 'success' for r in rows) / n < 0.85
            or sum(r['outcome'] == 'timeout' for r in rows) / n > 0.10)


def curve_gaps(stats, cands, seeds, y_mass):
    """제안 점마다 (후보, 제안 충돌률, 제안 시간, 같은 충돌률의 R2 시간과의 차 또는 None)."""
    def avg(c, v, k):
        return float(np.nanmean([stats[(c, v, s)][k] for s in seeds]))
    ykey = f't_{y_mass}'
    r_pts = sorted((avg(c, 'r2', 'coll'), avg(c, 'r2', ykey)) for c in cands)
    rx, ry = np.array([p[0] for p in r_pts]), np.array([p[1] for p in r_pts])
    out = []
    for c in cands:
        px, py = avg(c, 'proposed', 'coll'), avg(c, 'proposed', ykey)
        g = py - float(np.interp(px, rx, ry)) if rx.min() <= px <= rx.max() else None
        out.append((c, px, py, g))
    return out, r_pts


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('--cands', nargs='+', required=True, help='벌점 단계 이름 (약한 → 센 순서)')
    args = p.parse_args()
    runs = [line.split() for line in (args.root / 'runs.txt').read_text().splitlines() if line.strip()]
    seeds = sorted({int(s) for c, v, s, d in runs if c in args.cands})
    data = load(args.root, args.cands, seeds)
    stats = {}
    for key, rows in data.items():
        st = {'coll': coll(rows), 't_all': mean_time(rows)}
        for m in MASSES:
            st[f'coll_{m}'], st[f't_{m}'] = coll(rows, m), mean_time(rows, m)
        stats[key] = st
    rng = np.random.default_rng(0)
    lines = ['# v3 벌점 곡선 실험 판정', '',
             f'폴더 `{args.root}`, 벌점 단계 {args.cands}, 시드 {seeds[0]}~{seeds[-1]} ({len(seeds)}개). 괄호는 95% CI (시드 부트스트랩).', '']

    # 실제 정규화 벌점
    sig = defaultdict(list)
    for c, v, s, d in runs:
        if c in args.cands:
            with open(Path(d) / 'final_vecnormalize.pkl', 'rb') as f:
                vn = pickle.load(f)
            sig[c].append(float(np.sqrt(vn.ret_rms.var + vn.epsilon)))
    lines += ['## 벌점 단계 (1 m/s 충돌, 정규화 뒤)', '', '| 단계 | 보상 정규화 σ (평균) | 정규화 벌점 |', '| --- | --- | --- |']
    for c in args.cands:
        raw = float(c.replace('pen', '')) + 40.0
        lines.append(f'| {c} | {np.mean(sig[c]):.1f} | −{raw / np.mean(sig[c]):.1f} |')

    # 점
    lines += ['', '## 곡선의 점 (시드 평균)', '', '| 단계 | 비교군 | 충돌률 | 0 kg 충돌 | 25 kg 충돌 | 도착 시간 | 0 kg 시간 | 25 kg 시간 | 무너진 시드 |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for c in args.cands:
        for v in VARIANTS:
            a = lambda k: float(np.nanmean([stats[(c, v, s)][k] for s in seeds]))  # noqa: E731
            n_col = sum(collapsed(data[(c, v, s)]) for s in seeds)
            lines.append(f"| {c} | {v} | {a('coll')*100:.1f}% | {a('coll_0.0')*100:.1f}% | {a('coll_25.0')*100:.1f}% | "
                         f"{a('t_all'):.2f} s | {a('t_0.0'):.2f} s | {a('t_25.0'):.2f} s | {n_col}/{len(seeds)} |")

    # F1 (+ 참고: 전체·25 kg)
    lines += ['', '## 같은 충돌률에서 제안 − R2 도착 시간 (음수 = 제안이 빠름)', '']
    verdict = {}
    for y, label in (('0.0', '0 kg (F1)'), ('all', '학습 질량 전체 (참고)'), ('25.0', '25 kg (참고)')):
        gaps, r_pts = curve_gaps(stats, args.cands, seeds, y)
        boot = []
        for _ in range(B):
            bs = list(rng.choice(seeds, len(seeds)))
            g = [x[3] for x in curve_gaps(stats, args.cands, bs, y)[0] if x[3] is not None]
            if g:
                boot.append(np.mean(g))
        inr = [x[3] for x in gaps if x[3] is not None]
        lines.append(f"**{label}**: R2 곡선 {[(round(cx*100, 1), round(cy, 2)) for cx, cy in r_pts]} (충돌률 %, 시간 s)")
        for c, px, py, g in gaps:
            lines.append(f'- 제안 {c}: 충돌 {px*100:.1f}%, 시간 {py:.2f} s → ' + (f'{g:+.2f} s' if g is not None else 'R2 범위 밖'))
        if inr and boot:
            lo, hi = np.percentile(boot, [2.5, 97.5])
            lines.append(f'- 평균 {np.mean(inr):+.2f} s ({lo:+.2f} ~ {hi:+.2f}), 부트스트랩 {len(boot)}/{B}회가 범위 안 점을 가짐')
            verdict[y] = hi < 0
        else:
            lines.append('- 범위 안 점이 없어 판정 불가')
            verdict[y] = None
        lines.append('')

    # F2
    d = np.array([np.nanmean([(stats[(c, 'proposed', s)]['t_25.0'] - stats[(c, 'proposed', s)]['t_0.0'])
                              - (stats[(c, 'r2', s)]['t_25.0'] - stats[(c, 'r2', s)]['t_0.0']) for c in args.cands])
                  for s in seeds])
    bd = [rng.choice(d, len(d)).mean() for _ in range(B)]
    lo2, hi2 = np.percentile(bd, [2.5, 97.5])
    lines += ['## 질량 의존 (F2)', '',
              f'(25 kg 시간 − 0 kg 시간) 제안 − R2, 벌점 단계 평균: {d.mean():+.2f} s ({lo2:+.2f} ~ {hi2:+.2f})', '']

    f1 = verdict.get('0.0')
    lines += ['## 판정', '',
              f"- F1 같은 충돌률에서 0 kg 제안이 빠름: {'통과' if f1 else ('판정 불가' if f1 is None else '미달')}",
              f"- F2 무거울수록 제안이 R2보다 더 느려짐: {'통과' if lo2 > 0 else '미달'}",
              '- F3 무너진 시드: 위 표의 마지막 열', '']
    text = '\n'.join(lines)
    (args.root / 'judge.md').write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
