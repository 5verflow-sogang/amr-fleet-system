"""scripts/analyze.py (H1~H3 판정). 가짜 episodes.csv로 판정 방향을 확인한다. 표준 라이브러리만 쓴다."""

import csv
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import analyze  # noqa: E402

FIELDS = ['run', 'variant', 'mass_obs', 'episode', 'payload_kg', 'collision', 'success', 'time_s',
          'popout_appeared', 'popout_stoppable']


def write_csv(path, rates, seeds=range(10), episodes=100, masses=(10.0, 25.0), mass_obs='true', times=None,
              appeared_rate=1.0):
    """rates: {variant: 충돌 확률}. 같은 에피소드 번호 = 같은 시나리오."""
    rng = random.Random(7)
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for variant, p in rates.items():
            for s in seeds:
                for mass in masses:
                    for ep in range(episodes):
                        coll = rng.random() < p
                        w.writerow({'run': f'{variant}_s{s}_20261010', 'variant': variant, 'mass_obs': mass_obs,
                                    'episode': ep, 'payload_kg': mass, 'collision': coll, 'success': not coll,
                                    'time_s': (times or {}).get(variant, 15.0) + rng.gauss(0, 0.2),
                                    'popout_appeared': rng.random() < appeared_rate,
                                    'popout_stoppable': not coll or rng.random() < 0.5})


def by_cond(results, prefix):
    return [r for r in results if r['condition'].startswith(prefix)]


def test_h1_pass_when_proposed_is_safer(tmp_path):
    write_csv(tmp_path / 'e.csv', {'proposed': 0.05, 'b1': 0.30})
    rows, _ = analyze.load([tmp_path / 'e.csv'])
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1']
    assert len(h1) == 2 and all(r['pass'] for r in h1)
    assert all(r['n'] == 10 and r['mean'] < -0.2 for r in h1)
    assert all(r['holm_reject'] for r in h1)


def test_h1_fail_when_no_difference(tmp_path):
    write_csv(tmp_path / 'e.csv', {'proposed': 0.2, 'b1': 0.2})
    rows, _ = analyze.load([tmp_path / 'e.csv'])
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1']
    assert not any(r['pass'] for r in h1)


def test_dwa_without_seed_is_fixed_value(tmp_path):
    # B1이 시드 없이 한 번만 평가돼도 RL 시드마다 같은 값과 짝짓는다
    write_csv(tmp_path / 'p.csv', {'proposed': 0.05})
    with (tmp_path / 'b.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for mass in (10.0, 25.0):
            for ep in range(200):
                w.writerow({'run': 'b1_dwa', 'variant': 'b1', 'mass_obs': 'true', 'episode': ep,
                            'payload_kg': mass, 'collision': ep < 60, 'success': ep >= 60, 'time_s': 15,
                            'popout_appeared': True, 'popout_stoppable': True})
    rows, _ = analyze.load([tmp_path / 'p.csv', tmp_path / 'b.csv'])
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1']
    assert all(r['n'] == 10 and r['mean'] == pytest.approx(0.05 - 0.30, abs=0.03) for r in h1)


def test_h2b_degradation_and_h3_arrival(tmp_path):
    # 정보 오차에서 제안은 거의 안 나빠지고 B2는 크게 나빠진다
    write_csv(tmp_path / 'exact.csv', {'proposed': 0.05, 'b2': 0.05}, masses=(10.0,))
    write_csv(tmp_path / 'unk.csv', {'proposed': 0.07, 'b2': 0.30}, masses=(10.0,), mass_obs='none')
    write_csv(tmp_path / 'h3.csv', {'proposed': 0.05, 'r2': 0.05}, masses=(15.0,), times={'proposed': 12.0, 'r2': 15.0})
    rows, _ = analyze.load([tmp_path / 'exact.csv', tmp_path / 'unk.csv', tmp_path / 'h3.csv'])
    res = analyze.judge(rows)
    h2b = [r for r in res if r['hypothesis'] == 'H2-b' and r['condition'].startswith('unknown 10')]
    assert len(h2b) == 1 and h2b[0]['pass'] and h2b[0]['mean'] < -0.15
    arrival = [r for r in res if r['hypothesis'] == 'H3' and '도착' in r['condition'] and '15 kg' in r['condition']]
    assert arrival[0]['pass'] and arrival[0]['mean'] == pytest.approx(-3.0, abs=0.2)
    # 비열등(충돌률 같음): CI 상한이 마진 2%p 안이어야 통과
    nonin = [r for r in res if r['hypothesis'] == 'H3' and '충돌률' in r['condition'] and '15 kg' in r['condition']]
    assert nonin[0]['bound'] == pytest.approx(0.02)


def test_denominator_is_appeared_and_stoppable_only_splits_cause(tmp_path):
    def row(appeared, stoppable, collision):
        return analyze.normalize({'run': 'proposed_s3_x', 'variant': 'proposed', 'mass_obs': 'true', 'episode': '0',
                                  'payload_kg': '25', 'collision': str(collision), 'success': str(not collision),
                                  'time_s': '20', 'popout_appeared': str(appeared),
                                  'popout_stoppable': str(stoppable)}, set())
    rows = [row(True, False, True), row(True, True, False), row(False, True, False)]
    assert rows[0]['seed'] == 3
    # 못 멈출 속도로 달려 난 충돌도 분모에 남는다 (1/2). 장애물이 안 나온 에피소드만 빠진다
    assert analyze.collision_rate(rows) == pytest.approx(0.5)
    t = analyze.seed_table(rows)[0]
    assert t['popout_appeared'] == 2 and t['collision_too_fast'] == 1 and t['collision_stoppable'] == 0
    warn = set()
    analyze.normalize({'run': 'r2_s1_x', 'variant': 'r2', 'mass_obs': 'scale:1.3', 'episode': '0',
                       'payload_kg': '0', 'collision': 'False', 'success': 'True', 'time_s': '10'}, warn)
    assert any('popout_appeared' in w for w in warn)


def test_holm():
    assert analyze.holm([0.01, 0.04, 0.03]) == [True, False, False]
    assert analyze.holm([0.001, 0.01, None]) == [True, True, None]
