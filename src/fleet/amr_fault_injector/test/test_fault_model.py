import random

import pytest

from amr_fault_injector.fault_model import DelayLine, FaultConfig, link_rng


def line(delay_ms=0.0, loss_pct=0.0, seed=0):
    return DelayLine(FaultConfig(delay_ms=delay_ms, loss_pct=loss_pct), random.Random(seed))


@pytest.mark.parametrize('kwargs', [
    {'delay_ms': -1.0}, {'delay_ms': 500.1}, {'delay_ms': float('nan')},
    {'loss_pct': -0.1}, {'loss_pct': 10.1}, {'loss_pct': float('nan')},
])
def test_out_of_spec_range_rejected(kwargs):
    with pytest.raises(ValueError):
        FaultConfig(**kwargs)


def test_spec_range_edges_accepted():
    FaultConfig(delay_ms=0.0, loss_pct=0.0)
    FaultConfig(delay_ms=500.0, loss_pct=10.0)


def test_zero_delay_passes_immediately():
    d = line()
    d.push('a', now=10.0)
    assert d.pop_due(10.0) == ['a']


def test_fixed_delay_holds_until_due():
    d = line(delay_ms=100.0)
    d.push('a', now=10.0)
    assert d.pop_due(10.099) == []
    assert d.next_due() == pytest.approx(10.1)
    assert d.pop_due(10.1) == ['a']
    assert len(d) == 0


def test_order_preserved():
    d = line(delay_ms=50.0)
    for i, t in enumerate([0.0, 0.01, 0.02]):
        d.push(i, now=t)
    assert d.pop_due(0.055) == [0]
    assert d.pop_due(1.0) == [1, 2]


def test_no_loss_by_default():
    d = line()
    for i in range(1000):
        assert d.push(i, now=0.0)
    assert d.dropped == 0


class _AlwaysZero(random.Random):
    def random(self):
        return 0.0


def test_dropped_when_loss_drawn():
    # 손실로 뽑히면 큐에 넣지 않고 센다
    d = DelayLine(FaultConfig(loss_pct=10.0), _AlwaysZero())
    assert not d.push('a', now=0.0)
    assert d.pop_due(1.0) == []
    assert (d.received, d.dropped) == (1, 1)


def test_loss_rate_close_to_config():
    d = line(loss_pct=10.0, seed=42)
    for i in range(10000):
        d.push(i, now=0.0)
    assert d.dropped / d.received == pytest.approx(0.10, abs=0.01)


def test_link_rng_reproducible_and_independent():
    a = [link_rng(1, 'amr_01', 'heartbeat').random() for _ in range(3)]
    b = [link_rng(1, 'amr_01', 'heartbeat').random() for _ in range(3)]
    assert a == b
    assert link_rng(1, 'amr_01', 'heartbeat').random() != link_rng(1, 'amr_02', 'heartbeat').random()
