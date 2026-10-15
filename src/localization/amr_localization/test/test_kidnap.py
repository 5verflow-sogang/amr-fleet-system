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
    acts = run(m, [(0.4, 1.0), (0.95, 0.1), (0.97, 0.1)], t0=2.0)
    assert acts[0] == [NOMOTION] and acts[-1] == [STOP] and m.state == 'tracking'
    assert [e.kind for e in m.events] == ['lost', 'local_reinit', 'recovered']


def test_one_low_reading_is_not_lost():
    m = KidnapRecovery(RecoveryConfig(lost_checks=3))
    run(m, [(0.3, 0.05), (0.9, 0.05), (0.3, 0.05), (0.3, 0.05), (0.9, 0.05)])
    assert m.state == 'tracking' and not m.events


def test_rotates_then_fails_after_max_attempts():
    cfg = RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, max_attempts=2, confirm_s=0.0)
    m = KidnapRecovery(cfg)
    acts = run(m, [(0.1, 1.0)] * 12)
    assert ROTATE in [a for step in acts for a in step]
    assert m.state == 'failed'
    kinds = [e.kind for e in m.events]
    # 1차 국소, 2차 전역
    assert kinds.count('local_reinit') == 1 and kinds.count('reinit') == 1


def test_failed_returns_to_tracking_when_relocalized():
    # 실패한 뒤 사람이 위치를 다시 잡아 주면(일치율·σ 회복) 정상 추적으로 돌아와 BT가 다음 작업을 받는다
    m = KidnapRecovery(RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, max_attempts=1, ok_checks=2,
                                      confirm_s=0.0))
    run(m, [(0.1, 1.0)] * 8)
    assert m.state == 'failed'
    run(m, [(0.95, 0.05)] * 2, t0=10.0)
    assert m.state == 'tracking' and m.events[-1].kind == 'restored'


def test_nomotion_requests_are_limited_per_attempt():
    # 제자리 갱신을 계속 부르면 파티클이 한 점에 몰린다 → 시도마다 nomotion_max번까지만
    m = KidnapRecovery(RecoveryConfig(lost_checks=1, settle_s=10.0, nomotion_max=3))
    acts = run(m, [(0.1, 1.0)] * 12)
    assert sum(step.count(NOMOTION) for step in acts) == 3


def test_confirms_after_rotation_before_next_attempt():
    # 회전 중에는 일치율이 낮게 나와 판정을 놓친다 → 멈추고 confirm_s 동안 한 번 더 보고, 수렴하면 다음 시도로 가지 않는다
    cfg = RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, confirm_s=2.0, ok_checks=2, max_attempts=2)
    m = KidnapRecovery(cfg)
    run(m, [(0.1, 1.0)] * 6)                       # 감지 → 정지 대기 → 회전 → 회전 끝
    assert m.state == 'settling' and m.events[-1].kind == 'confirm'
    run(m, [(0.95, 0.2), (0.95, 0.2)], t0=3.0)
    assert m.state == 'tracking' and m.events[-1].kind == 'recovered'
    assert [e.kind for e in m.events].count('reinit') == 0   # 맞던 자세를 전역 재초기화로 버리지 않음


def test_confirm_timeout_moves_to_next_attempt():
    cfg = RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=1.0, confirm_s=1.0, max_attempts=2)
    m = KidnapRecovery(cfg)
    run(m, [(0.1, 1.0)] * 10)
    assert [e.kind for e in m.events][:5] == ['lost', 'local_reinit', 'rotate', 'confirm', 'reinit']


def test_no_recovery_while_rotating_and_wrong_aisle_ratio_rejected():
    # 회전 중에는 판정하지 않고, 옆 통로 오수렴(일치율 0.87, 10/14)은 받지 않는다
    cfg = RecoveryConfig(lost_checks=1, settle_s=1.0, rotate_s=2.0, confirm_s=2.0, ok_checks=2)
    m = KidnapRecovery(cfg)
    run(m, [(0.1, 1.0)] * 3)                        # 감지 → 정지 → 회전 시작
    assert m.state == 'rotating'
    run(m, [(0.99, 0.05)] * 3, t0=1.5)              # 회전 중 높은 일치율이어도 복구로 안 침
    assert m.state in ('rotating', 'settling') and 'recovered' not in [e.kind for e in m.events]
    run(m, [(0.87, 0.09)] * 4, t0=3.5)              # 멈춘 확인 단계에서 0.87은 거부
    assert 'recovered' not in [e.kind for e in m.events]
