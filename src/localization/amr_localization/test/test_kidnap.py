import math

import numpy as np
import pytest

from amr_localization.kidnap import (LOCAL_REINIT, NOMOTION, REINIT, ROTATE, STOP, KidnapRecovery, RecoveryConfig,
                                     distance_field, endpoints, grid_from_occupancy, match_ratio)
from amr_localization.kidnap.match import GridMap


def corridor_grid(res=0.05):
    # 10 x 4 m, 아래(y=0.5)·위(y=3.5) 벽
    w, h = int(10 / res), int(4 / res)
    data = np.zeros((h, w), dtype=np.int16)
    data[int(0.5 / res), :] = 100
    data[int(3.5 / res), :] = 100
    return grid_from_occupancy(data.ravel().tolist(), w, h, res, 0.0, 0.0)


def wall_scan(pose, n=360):
    # 위·아래 벽까지의 거리를 정확히 계산한 360도 스캔 (센서는 로봇 중심)
    x, y, yaw = pose
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    world = a + yaw
    r = np.full(n, np.inf)
    s = np.sin(world)
    r[s > 1e-3] = (3.5 + 0.025 - y) / s[s > 1e-3]
    r[s < -1e-3] = (0.5 + 0.025 - y) / s[s < -1e-3]
    return r, -math.pi, 2 * math.pi / n


def test_distance_field_grows_from_occupied():
    occ = np.zeros((7, 7), dtype=bool)
    occ[3, 3] = True
    d = distance_field(occ, 0.05, 0.5)
    assert d[3, 3] == 0.0
    assert d[3, 5] == pytest.approx(0.10)
    assert d[5, 5] == pytest.approx(0.10)   # 체비쇼프 근사
    assert d[0, 0] == pytest.approx(0.15)


def test_match_ratio_high_at_true_pose_low_when_shifted():
    grid = corridor_grid()
    true_pose = (5.0, 2.0, 0.0)
    r, amin, ainc = wall_scan(true_pose)
    # 최대 거리 4.9 m: 통로가 10 m라 더 먼 빔은 지도 밖으로 나간다
    good = match_ratio(grid, endpoints(true_pose, (0, 0, 0), r, amin, ainc, 0.12, 4.9))
    # 같은 스캔을 0.6 m 아래로 잘못 놓으면 벽에서 떨어진다
    bad = match_ratio(grid, endpoints((5.0, 1.4, 0.0), (0, 0, 0), r, amin, ainc, 0.12, 4.9))
    assert good > 0.95
    assert bad < 0.2


def test_match_ratio_out_of_map_counts_as_miss():
    grid = GridMap(0.05, 0.0, 0.0, np.zeros((10, 10), dtype=np.float32))
    assert match_ratio(grid, np.array([[0.1, 0.1], [5.0, 5.0]])) == pytest.approx(0.5)
    assert math.isnan(match_ratio(grid, np.zeros((0, 2))))


def run(machine, seq, t0=0.0, dt=0.5):
    actions = []
    for k, (ratio, std) in enumerate(seq):
        actions.append(machine.step(t0 + k * dt, ratio, std))
    return actions


def test_detects_lost_and_recovers_without_rotation():
    m = KidnapRecovery(RecoveryConfig(lost_checks=3, ok_checks=2))
    acts = run(m, [(0.9, 0.05), (0.3, 0.05), (0.3, 0.05), (0.3, 0.05)])
    # 첫 시도는 국소 재초기화
    assert acts[-1] == [STOP, LOCAL_REINIT, NOMOTION] and m.state == 'settling'
    acts = run(m, [(0.4, 1.0), (0.85, 0.1), (0.9, 0.1)], t0=2.0)
    assert acts[0] == [NOMOTION] and acts[-1] == [STOP] and m.state == 'tracking'
    assert [e.kind for e in m.events] == ['lost', 'local_reinit', 'recovered']


def test_one_low_reading_is_not_lost():
    m = KidnapRecovery(RecoveryConfig(lost_checks=3))
    run(m, [(0.3, 0.05), (0.9, 0.05), (0.3, 0.05), (0.3, 0.05), (0.9, 0.05)])
    assert m.state == 'tracking' and not m.events


def test_rotates_then_fails_after_max_attempts():
    cfg = RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, max_attempts=2)
    m = KidnapRecovery(cfg)
    acts = run(m, [(0.1, 1.0)] * 12)
    assert ROTATE in [a for step in acts for a in step]
    assert m.state == 'failed'
    kinds = [e.kind for e in m.events]
    # 1차 국소, 2차 전역
    assert kinds.count('local_reinit') == 1 and kinds.count('reinit') == 1


def test_failed_returns_to_tracking_when_relocalized():
    # 실패한 뒤 사람이 위치를 다시 잡아 주면(일치율·σ 회복) 정상 추적으로 돌아와 BT가 다음 작업을 받는다
    m = KidnapRecovery(RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, max_attempts=1, ok_checks=2))
    run(m, [(0.1, 1.0)] * 8)
    assert m.state == 'failed'
    run(m, [(0.95, 0.05)] * 2, t0=10.0)
    assert m.state == 'tracking' and m.events[-1].kind == 'restored'
