import random

import pytest

from amr_fault_injector.fault_model import Blackout, DelayLine, FaultConfig, link_rng


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


def test_out_of_spec_loss_needs_flag():
    with pytest.raises(ValueError, match='allow_out_of_spec'):
        FaultConfig(loss_pct=30.0)
    FaultConfig(loss_pct=50.0, allow_out_of_spec=True)
    with pytest.raises(ValueError):
        FaultConfig(loss_pct=50.1, allow_out_of_spec=True)
    # 지연 상한은 플래그와 상관없이 그대로
    with pytest.raises(ValueError):
        FaultConfig(delay_ms=600.0, allow_out_of_spec=True)


def test_blackout_needs_flag():
    with pytest.raises(ValueError, match='allow_out_of_spec'):
        FaultConfig(blackout=Blackout(at_s=10.0, duration_s=3.0))
    FaultConfig(allow_out_of_spec=True, blackout=Blackout(at_s=10.0, duration_s=3.0))


@pytest.mark.parametrize('kwargs', [
    {'at_s': -1.0, 'duration_s': 1.0}, {'at_s': 0.0, 'duration_s': 0.0},
    {'at_s': 0.0, 'duration_s': 3.0, 'period_s': 3.0},   # 주기가 끊김보다 길어야 사이에 연결 구간이 있다
])
def test_bad_blackout_rejected(kwargs):
    with pytest.raises(ValueError):
        Blackout(**kwargs)


def test_blackout_window_once_and_periodic():
    once = Blackout(at_s=10.0, duration_s=3.0)
    assert [once.active(t) for t in (9.9, 10.0, 12.9, 13.0, 100.0)] == [False, True, True, False, False]
    periodic = Blackout(at_s=10.0, duration_s=3.0, period_s=20.0)
    assert [periodic.active(t) for t in (12.0, 13.0, 30.0, 32.9, 33.0, 50.5)] == [True, False, True, True, False, True]


def test_blackout_keeps_loss_positions_outside_window():
    """같은 seed면 끊김 밖의 손실 위치가 끊김이 없는 실행과 같다 (끊김 유무를 짝지어 비교)."""
    def run(blackout):
        cfg = FaultConfig(loss_pct=20.0, allow_out_of_spec=True, blackout=blackout)
        d = DelayLine(cfg, random.Random(7), start=100.0)
        return [d.push(i, now=100.0 + i * 0.1) for i in range(200)], d

    with_bo, d = run(Blackout(at_s=5.0, duration_s=3.0))
    without_bo, _ = run(None)
    inside = set(range(50, 80))   # 시작 후 5.0~8.0 s
    assert not any(with_bo[i] for i in inside)
    assert d.blacked_out == len(inside)
    assert [ok for i, ok in enumerate(with_bo) if i not in inside] == \
        [ok for i, ok in enumerate(without_bo) if i not in inside]
    assert d.dropped == d.blacked_out + sum(1 for i, ok in enumerate(with_bo) if not ok and i not in inside)


def test_blackout_only_on_target_link():
    cfg = FaultConfig(allow_out_of_spec=True, blackout=Blackout(at_s=0.0, duration_s=1.0))
    target = DelayLine(cfg, random.Random(0), start=0.0, blackout=True)
    other = DelayLine(cfg, random.Random(0), start=0.0, blackout=False)
    assert not target.push('a', now=0.5)
    assert other.push('a', now=0.5)
    assert target.push('b', now=1.5)


def test_link_rng_reproducible_and_independent():
    a = [link_rng(1, 'amr_01', 'heartbeat').random() for _ in range(3)]
    b = [link_rng(1, 'amr_01', 'heartbeat').random() for _ in range(3)]
    assert a == b
    assert link_rng(1, 'amr_01', 'heartbeat').random() != link_rng(1, 'amr_02', 'heartbeat').random()
