"""scripts/analyze.py (H1~H3 판정). 가짜 episodes.csv로 판정 방향을 확인한다. 표준 라이브러리만 쓴다."""

import csv
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import analyze  # noqa: E402

FIELDS = ['run', 'variant', 'train_seed', 'eval_list', 'scenario', 'scenario_seed', 'mass_obs', 'episode',
          'payload_kg', 'collision', 'success', 'time_s', 'popout_appeared', 'popout_stoppable']


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
                        w.writerow({'run': f'{variant}_s{s}_20261010', 'variant': variant, 'train_seed': s,
                                    'eval_list': s, 'scenario': 'S1', 'scenario_seed': 100_000 + 1000 * s + ep,
                                    'mass_obs': mass_obs, 'episode': ep, 'payload_kg': mass, 'collision': coll, 'success': not coll,
                                    'time_s': (times or {}).get(variant, 15.0) + rng.gauss(0, 0.2),
                                    'popout_appeared': rng.random() < appeared_rate,
                                    'popout_stoppable': not coll or rng.random() < 0.5})


def by_cond(results, prefix):
    return [r for r in results if r['condition'].startswith(prefix)]


def test_h1_pass_when_proposed_is_safer(tmp_path):
    write_csv(tmp_path / 'e.csv', {'proposed': 0.05, 'b1': 0.30})
    rows, _ = analyze.load([tmp_path / 'e.csv'])
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1' and '충돌률' in r['condition']]
    assert len(h1) == 2 and all(r['pass'] for r in h1)
    assert all(r['n'] == 10 and r['mean'] < -0.2 for r in h1)
    assert all(r['holm_reject'] for r in h1)


def test_h1_fail_when_no_difference(tmp_path):
    write_csv(tmp_path / 'e.csv', {'proposed': 0.2, 'b1': 0.2})
    rows, _ = analyze.load([tmp_path / 'e.csv'])
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1' and '충돌률' in r['condition']]
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
    analyze.FIXED_WARNINGS.clear()
    h1 = [r for r in analyze.judge(rows) if r['hypothesis'] == 'H1' and '충돌률' in r['condition']]
    assert all(r['n'] == 10 and r['mean'] == pytest.approx(0.05 - 0.30, abs=0.03) for r in h1)
    # 목록 번호 없는 고정값 짝짓기는 시드 간 흔들림이 빠지므로 경고한다
    assert any('고정값' in w for w in analyze.FIXED_WARNINGS)


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


def row(variant, k, scenario, seed, collision, time_s, train_seed=None, mass=10.0):
    return {'run': f'{variant}_x', 'variant': variant, 'train_seed': k if train_seed is None else train_seed,
            'eval_list': k, 'scenario': scenario, 'scenario_seed': seed, 'mass_obs': 'exact', 'episode': 0,
            'payload_kg': mass, 'collision': collision, 'success': not collision, 'time_s': time_s,
            'popout_appeared': scenario == 'S1', 'popout_stoppable': True}


def write_rows(path, rows):
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)


def test_arrival_pairs_by_scenario_and_seed(tmp_path):
    # S1·S2가 같은 scenario_seed를 쓴다. 제안은 S1만 성공(10 s), R2는 S1 11 s·S2 21 s로 둘 다 성공
    # → 같은 시나리오(S1)끼리만 짝지어 −1 s. 시드 번호만 키로 쓰면 R2의 S2(21 s)가 S1을 덮어써 −11 s가 된다
    rows = []
    for k in range(5):
        rows += [row('proposed', k, 'S1', 7, False, 10.0), row('proposed', k, 'S2', 7, True, 30.0),
                 row('r2', k, 'S1', 7, False, 11.0), row('r2', k, 'S2', 7, False, 21.0)]
    write_rows(tmp_path / 'e.csv', rows)
    loaded, _ = analyze.load([tmp_path / 'e.csv'])
    g = analyze.group(loaded)
    assert analyze.arrival_diffs(g, 'proposed', 'r2', 'exact', 10.0) == pytest.approx([-1.0] * 5)


def test_dwa_lists_pair_with_seeds_one_by_one(tmp_path):
    # DWA(B1)는 train_seed 없이 목록 k = 0~9를 푼다. 목록 0에서만 전부 충돌, 나머지는 충돌 없음.
    # 제안은 모든 시드에서 충돌 없음 → 진짜 차이는 [−1, 0, …, 0]이라 H1을 판정할 수 없다(CI 상한 0).
    # 목록을 하나로 합치면 B1이 고정값 0.1이 돼 차이가 전부 −0.1로 좁아지고 H1 PASS가 잘못 나온다
    rows = []
    for k in range(10):
        for ep in range(20):
            for mass in (10.0, 25.0):
                rows.append(row('proposed', k, 'S1', ep, False, 15.0, mass=mass))
                rows.append(row('b1', k, 'S1', ep, k == 0, 15.0, train_seed='', mass=mass))
    write_rows(tmp_path / 'e.csv', rows)
    loaded, _ = analyze.load([tmp_path / 'e.csv'])
    h1 = [r for r in analyze.judge(loaded) if r['hypothesis'] == 'H1' and '충돌률' in r['condition']]
    assert len(h1) == 2
    assert all(r['n'] == 10 and r['mean'] == pytest.approx(-0.1) for r in h1)
    assert not any(r['pass'] for r in h1)


def test_t_distribution_matches_known_values():
    assert analyze.t_cdf(0.0, 9) == pytest.approx(0.5)
    assert analyze.t_quantile(0.975, 9) == pytest.approx(2.262, abs=1e-3)     # t 표
    assert analyze.t_quantile(0.975, 1000) == pytest.approx(1.962, abs=2e-3)  # 정규에 가까움
    assert analyze.t_cdf(-2.262, 9) == pytest.approx(0.025, abs=1e-4)


def test_paired_t_one_sided_against_bound():
    r = analyze.paired_t([-0.10, -0.12, -0.08, -0.11, -0.09], bound=0.0)
    assert r['p_t'] < 0.001 and r['t_ci_high'] < 0
    # 비열등 마진: 평균 +1%p, 마진 +2%p, 흔들림이 크면 통과 못 함
    r = analyze.paired_t([0.01, 0.05, -0.03, 0.04, -0.02], bound=0.02)
    assert r['t_ci_high'] > 0.02 and r['p_t'] > 0.025
    assert analyze.paired_t([0.1], 0.0) == {'t_ci_high': None, 'p_t': None}


def test_hypothesis_needs_every_condition():
    # H1 "10·25 kg 각각": 하나라도 통과 못 하면 H1 실패 (#100 연우 리뷰)
    rows = [{'hypothesis': 'H1', 'pass': True}, {'hypothesis': 'H1', 'pass': False},
            {'hypothesis': 'H2-a', 'pass': True}, {'hypothesis': 'H2-a', 'pass': True},
            {'hypothesis': 'H3', 'pass': True}, {'hypothesis': 'H3', 'pass': None}]
    assert analyze.hypothesis_verdicts(rows) == {'H1': False, 'H2-a': True, 'H3': None}


def test_check_uses_more_conservative_interval():
    # 값 둘: 부트스트랩 구간은 두 값 사이라 좁지만 t 구간은 넓다 → t 기준으로 FAIL
    r = analyze.check('H1', 'x', [-0.10, -0.01], 0.0, seed=0)
    assert r['ci_high'] < 0 and r['t_ci_high'] > 0 and r['pass'] is False


def test_timeout_guard_blocks_policy_that_stops_instead_of_colliding(tmp_path):
    # 제안이 충돌은 적지만 시간 초과가 훨씬 많으면 H1 보조 기준에서 걸려 H1 전체가 FAIL
    import csv as _csv
    rng = random.Random(3)
    with (tmp_path / 'e.csv').open('w', newline='') as f:
        w = _csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for variant, (pc, pt) in {'proposed': (0.02, 0.25), 'b1': (0.30, 0.0)}.items():
            for s in range(10):
                for mass in (10.0, 25.0):
                    for ep in range(100):
                        u = rng.random()
                        coll, timeout = u < pc, pc <= u < pc + pt
                        w.writerow({'run': f'{variant}_s{s}', 'variant': variant, 'train_seed': s, 'eval_list': s,
                                    'scenario': 'S1', 'scenario_seed': 1000 * s + ep, 'mass_obs': 'true',
                                    'episode': ep, 'payload_kg': mass, 'collision': coll,
                                    'success': not coll and not timeout, 'time_s': 15.0,
                                    'popout_appeared': True, 'popout_stoppable': True})
    rows, _ = analyze.load([tmp_path / 'e.csv'])
    res = analyze.judge(rows)
    h1 = [r for r in res if r['hypothesis'] == 'H1']
    assert len([r for r in h1 if '시간 초과율' in r['condition']]) == 2
    assert all(r['pass'] for r in h1 if '충돌률' in r['condition'])
    assert not any(r['pass'] for r in h1 if '시간 초과율' in r['condition'])
    assert analyze.hypothesis_verdicts(res)['H1'] is False
