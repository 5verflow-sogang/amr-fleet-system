"""도킹 제어기를 차동 구동 운동학과 측정 노이즈로 닫힌 고리로 돌린다."""
import math
import random

import pytest

from amr_docking.controller import (
    FAILED_TO_CONTROL, FAILED_TO_DETECT_DOCK, ControllerConfig, DockingController)
from amr_docking.geometry import DockError

DT = 0.05


def simulate(along, lateral, heading, seed=0, noise_pos=0.001, noise_yaw=math.radians(0.15),
             visible=lambda t, e: True, config=None, max_t=90.0):
    """목표 좌표계에서 로봇을 움직인다. 반환: (결과, 실제 최종 오차, 제어기)."""
    rng = random.Random(seed)
    ctl = DockingController(config or ControllerConfig())
    x, y, th = -along, lateral, heading   # 목표 좌표계 로봇 자세 (x: 판 쪽)
    t = 0.0
    while t < max_t:
        true = DockError(-x, y, th)
        meas = None
        if visible(t, true):
            meas = DockError(true.along + rng.gauss(0, noise_pos), true.lateral + rng.gauss(0, noise_pos),
                             true.heading + rng.gauss(0, noise_yaw))
        v, w, out = ctl.step(meas, DT)
        if out is not None:
            return out, true, ctl
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
        t += DT
    raise AssertionError('끝나지 않음')


STARTS = [
    (0.0, 0.0, 0.0), (0.2, 0.05, math.radians(8)), (0.25, -0.1, math.radians(-10)),
    (-0.15, 0.08, math.radians(5)), (0.05, -0.15, math.radians(12)), (0.3, 0.0, math.radians(-15)),
    (-0.25, -0.2, math.radians(-8)), (0.1, 0.2, 0.0),
]


@pytest.mark.parametrize('start', STARTS)
def test_converges_within_kpi(start):
    out, true, _ = simulate(*start)
    assert out.success, out.message
    assert true.position < 0.01, true          # 실제 오차 (KPI 2 cm보다 여유)
    assert abs(true.heading) < math.radians(0.5), true


def test_many_random_starts():
    rng = random.Random(42)
    worst_pos, worst_yaw = 0.0, 0.0
    for i in range(40):
        start = (rng.uniform(-0.25, 0.3), rng.uniform(-0.2, 0.2), math.radians(rng.uniform(-15, 15)))
        out, true, _ = simulate(*start, seed=i)
        assert out.success, (start, out.message)
        worst_pos = max(worst_pos, true.position)
        worst_yaw = max(worst_yaw, abs(true.heading))
    assert worst_pos < 0.015
    assert worst_yaw < math.radians(0.8)


def test_rotation_overshoot_is_corrected():
    # 회전을 멈춘 뒤 관성으로 0.6°씩 더 도는 로봇: VERIFY가 보고 ROTATE를 다시 한다
    cfg = ControllerConfig()
    ctl = DockingController(cfg)
    x, y, th, last_w = 0.0, 0.0, math.radians(5), 0.0
    for _ in range(4000):
        v, w, out = ctl.step(DockError(-x, y, th), DT)
        if out is not None:
            break
        if last_w != 0.0 and w == 0.0:
            th += math.copysign(math.radians(0.6), last_w)   # 멈출 때 더 돎
        last_w = w
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
    assert out.success
    assert abs(math.degrees(th)) <= 0.5 + 1e-6


def test_not_found_after_sweep():
    out, _, ctl = simulate(0.1, 0.0, 0.0, visible=lambda t, e: False)
    assert not out.success
    assert out.error_code == FAILED_TO_DETECT_DOCK
    assert ctl.elapsed < 15.0


def test_found_during_sweep():
    # 처음엔 안 보이다가 왼쪽으로 돌면 보인다 (판이 시야 밖에 있던 경우)
    out, _, _ = simulate(0.2, 0.0, math.radians(-30), visible=lambda t, e: e.heading > math.radians(-20))
    assert out.success, out.message


def test_lost_mid_approach():
    out, _, _ = simulate(0.3, 0.0, 0.0, visible=lambda t, e: t < 1.0)
    assert out.error_code == FAILED_TO_DETECT_DOCK
    assert 'lost' in out.message


def test_timeout():
    out, _, _ = simulate(0.3, 0.1, 0.0, config=ControllerConfig(timeout_s=1.0))
    assert out.error_code == FAILED_TO_CONTROL
    assert 'timeout' in out.message


def test_verify_fails_with_strict_tolerance():
    # 측정 노이즈보다 훨씬 작은 허용치면 재시도 뒤 FAILED_TO_CONTROL
    cfg = ControllerConfig(position_tol=1e-5, max_retries=1)
    out, _, _ = simulate(0.1, 0.0, 0.0, config=cfg, noise_pos=0.003)
    assert out.error_code == FAILED_TO_CONTROL
    assert 'after 2 tries' in out.message
