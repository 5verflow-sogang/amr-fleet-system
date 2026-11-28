"""bpr.py와 analyze.py 시험 (가짜 측정)."""

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze import load, summarize, write_md  # noqa: E402
from bpr import bpr, congestion_weight, doubling_point, fit  # noqa: E402


def test_capacity_is_where_time_doubles():
    assert bpr(3, 10.0, 3, 2.0) == pytest.approx(20.0)


def test_fit_recovers_capacity_and_beta():
    meds = {n: bpr(n, 15.0, 2.5, 1.8) for n in (0, 1, 2, 4)}
    f = fit(meds)
    assert f['capacity'] == pytest.approx(2.5, abs=0.06)
    assert f['beta'] == pytest.approx(1.8, abs=0.06)
    assert f['rmse_rel'] < 0.01
    assert not f['at_edge']


def test_fit_needs_zero_and_two_more():
    with pytest.raises(ValueError):
        fit({1: 10.0, 2: 12.0})
    with pytest.raises(ValueError):
        fit({0: 10.0, 2: 12.0})


def test_doubling_point_interpolates():
    assert doubling_point({0: 10.0, 2: 14.0, 4: 26.0}) == pytest.approx(2 + 2 * 6 / 12)
    assert doubling_point({0: 10.0, 2: 12.0, 4: 15.0}) is None


def test_weight_is_one_when_linear():
    assert congestion_weight(1.0, 2.0, 2.0) == pytest.approx(1.0)
    assert congestion_weight(2.0, 2.0, 2.0) == pytest.approx(1.0)
    assert congestion_weight(2.0, 4.0, 2.0) == pytest.approx(2.0)


def _run(root: Path, name, zone, n, times, results=None):
    d = root / name
    (d / 'nav_kpi').mkdir(parents=True)
    (d / 'meta.json').write_text(json.dumps({'scenario': 'zone_pass', 'run_id': name, 'seed': 1,
                                             'params': {'zone': zone, 'guests_n': str(n), 'pass_timeout_s': '120'}}))
    with (d / 'nav_kpi' / 'nav_pairs.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['pair', 'result', 'actual_s'])
        w.writeheader()
        for i, t in enumerate(times):
            r = (results or {}).get(i, 'succeeded')
            w.writerow({'pair': i + 1, 'result': r, 'actual_s': t if r == 'succeeded' else ''})


def test_analyze_end_to_end(tmp_path):
    for n, t in ((0, 15.0), (1, 17.0), (2, 22.0), (4, 45.0)):
        _run(tmp_path, f'zone_pass-a{n}_00', 'aisle_3', n, [t] * 10)
    _run(tmp_path, 'other', 'x', 0, [1.0])
    (tmp_path / 'other' / 'meta.json').write_text(json.dumps({'scenario': 'nav_kpi', 'params': {}}))
    # 시간 초과 통과는 제한 시간(120 s) 검열값으로 들어간다
    _run(tmp_path, 'zone_pass-a4b_00', 'aisle_3', 4, [45.0, 0.0], results={1: 'aborted'})
    passes = load([tmp_path])
    assert len(passes) == 4 * 10 + 2
    cens = [p for p in passes if p['censored']]
    assert len(cens) == 1 and cens[0]['time_s'] == 120.0
    s = summarize(passes)['aisle_3']
    assert s['medians'][0] == 15.0 and s['counts'][4] == 12
    assert 2 < s['doubling_n'] < 4
    assert 2 < s['fit']['capacity'] < 4
    write_md(summarize(passes), tmp_path / 'r.md')
    assert 'aisle_3' in (tmp_path / 'r.md').read_text()
