"""시뮬레이션 시계 멈춤 판정 (drive.py spin_for, ROS 없이)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.clock_watch import ClockStall  # noqa: E402


def test_slow_clock_is_not_stalled():
    # RTF 0.3: 실제 1초에 시뮬레이션 0.3초. 오래 돌아도 멈춤이 아니다
    watch = ClockStall(sim_now=0.0, wall_now=0.0, limit_s=60.0)
    assert not any(watch.stalled(0.3 * t, float(t)) for t in range(1, 600))


def test_frozen_clock_is_stalled_after_limit():
    # 시뮬레이션 100초에서 Gazebo가 죽어 시계가 멈춘다
    watch = ClockStall(sim_now=0.0, wall_now=0.0, limit_s=60.0)
    assert not watch.stalled(100.0, 200.0)
    assert not watch.stalled(100.0, 260.0)          # 정확히 60초는 아직
    assert watch.stalled(100.0, 260.5)
    assert watch.frozen_s(270.0) == 70.0


def test_clock_resuming_resets_the_limit():
    watch = ClockStall(sim_now=0.0, wall_now=0.0, limit_s=60.0)
    assert not watch.stalled(0.0, 50.0)
    assert not watch.stalled(0.1, 55.0)            # 다시 늘었다 → 기준을 55초로
    assert not watch.stalled(0.1, 110.0)
    assert watch.stalled(0.1, 115.5)
