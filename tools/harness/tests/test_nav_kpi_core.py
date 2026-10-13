"""Nav KPI 계산 (experiments/nav_kpi/navkpi_core.py, ROS 없이)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'experiments' / 'nav_kpi'))

from navkpi_core import (cumulative, curvature, p95, path_length, predicted_time, prediction_error,  # noqa: E402
                         project, summarize)


def line(n=101, step=0.05):
    return [(i * step, 0.0) for i in range(n)]


def arc(radius=2.0, n=200):
    return [(radius * math.sin(t), radius * (1 - math.cos(t))) for t in (i * (math.pi / 2) / (n - 1) for i in range(n))]


def test_predicted_time_trapezoid_and_triangle():
    # 10 m, 0.5 m/s, 1 m/s²: 가감속 0.25 m → 10/0.5 + 0.5 = 20.5 s
    assert predicted_time(10.0, 0.5, 1.0) == pytest.approx(20.5)
    # 순항 속도에 못 미치는 0.1 m: 삼각형 2√(0.1/1)
    assert predicted_time(0.1, 0.5, 1.0) == pytest.approx(2 * math.sqrt(0.1))
    assert prediction_error(20.0, 23.0) == pytest.approx(0.15)


def test_curvature_separates_straight_and_curve():
    assert max(curvature(line())) == pytest.approx(0.0, abs=1e-9)
    k = curvature(arc(radius=2.0))
    assert k[len(k) // 2] == pytest.approx(0.5, rel=0.05)   # 반지름 2 m → 0.5 1/m


def test_project_distance_and_arc_length():
    path = line()
    cum = cumulative(path)
    pr = project((2.02, 0.03), path, cum)
    assert pr.distance == pytest.approx(0.03)
    assert pr.s == pytest.approx(2.02)
    assert path_length(path) == pytest.approx(5.0)


def test_project_hint_keeps_to_nearby_part_of_returning_path():
    # 갔다가 0.1 m 옆으로 돌아오는 경로: 앞부분을 지나는 중이면 돌아오는 선에 붙지 않아야 한다
    go = [(i * 0.1, 0.0) for i in range(51)]
    back = [(5.0 - i * 0.1, 0.1) for i in range(51)]
    path = go + back
    cum = cumulative(path)
    pr = project((1.0, 0.07), path, cum, hint=8, search=20)
    assert pr.index < 50 and pr.distance == pytest.approx(0.07)


def test_p95_nearest_rank():
    assert p95(list(range(1, 101))) == 95
    assert p95([3.0]) == 3.0
    assert math.isnan(p95([]))


def test_summarize_kpi_verdicts():
    pairs = [{'plan_ok': True, 'nav_success': True, 'pred_err': 0.1}] * 49 + \
            [{'plan_ok': False, 'nav_success': False, 'pred_err': None}]
    cte = [(0.03, 'straight')] * 10 + [(0.12, 'curve')] * 10
    s = summarize(pairs, [100.0] * 95 + [600.0] * 5, cte)
    assert s['plan_success_rate'] == pytest.approx(0.98)
    assert s['replan_p95_ms'] == 100.0 and s['replan_max_ms'] == 600.0
    assert s['verdict'] == {'plan_success_rate': 'PASS', 'replan_p95_ms': 'PASS', 'pred_err_mean': 'PASS',
                            'cte_straight_mean_m': 'PASS', 'cte_curve_mean_m': 'FAIL'}
