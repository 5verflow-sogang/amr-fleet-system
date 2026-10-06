"""하네스 통계 (ROS 없이)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.stats import bootstrap_mean_ci, percentile, summarize_runs  # noqa: E402


def test_percentile_interpolates():
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.5) == pytest.approx(2.5)
    assert percentile([5.0], 0.95) == 5.0


def test_bootstrap_ci_contains_mean_and_is_reproducible():
    values = [0.031, 0.042, 0.038, 0.029, 0.045, 0.036, 0.040, 0.033, 0.039, 0.035]
    low, high = bootstrap_mean_ci(values, seed=3)
    mean = sum(values) / len(values)
    assert low < mean < high
    # 표본 범위 안이고, 같은 seed면 같은 구간
    assert min(values) <= low and high <= max(values)
    assert bootstrap_mean_ci(values, seed=3) == (low, high)


def test_bootstrap_ci_narrows_with_more_runs():
    few = [0.03, 0.05, 0.04, 0.06]
    many = few * 10
    lo1, hi1 = bootstrap_mean_ci(few)
    lo2, hi2 = bootstrap_mean_ci(many)
    assert (hi2 - lo2) < (hi1 - lo1)


def test_summarize_runs_edge_cases():
    assert summarize_runs([]) is None
    one = summarize_runs([0.02])
    assert one.n == 1 and one.ci_low is None and one.std == 0.0
    iv = summarize_runs([1.0, 2.0, 3.0, float('nan')])
    assert iv.n == 3 and iv.mean == pytest.approx(2.0) and iv.std == pytest.approx(1.0)
