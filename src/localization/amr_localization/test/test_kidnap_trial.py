import math

import pytest

from amr_localization.eval.trajectory import Pose2D
from amr_localization.kidnap.trial import (AISLE_AB_Y, AISLE_BC_Y, AISLE_HALF_M, FAR, NEAR, PILLAR_CLEAR_M, PILLARS_BC,
                                           Criteria, Sample, evaluate, first_hold, pick_target, robot_frame_error)

START = Pose2D(0.0, 3.0, 1.5, 0.0)


def test_pick_target_is_repeatable_and_mode_dependent():
    assert pick_target(NEAR, 7, START) == pick_target(NEAR, 7, START)
    assert pick_target(NEAR, 7, START) != pick_target(NEAR, 8, START)


@pytest.mark.parametrize('seed', range(1, 51))
def test_near_target_stays_in_same_aisle_within_local_reinit_range(seed):
    t = pick_target(NEAR, seed, START)
    assert 1.0 <= abs(t.x - START.x) <= 2.5
    assert abs(t.y - AISLE_AB_Y) <= AISLE_HALF_M
    assert 0.2 <= abs(math.atan2(math.sin(t.yaw - START.yaw), math.cos(t.yaw - START.yaw))) <= 0.6 + 1e-9


@pytest.mark.parametrize('seed', range(1, 51))
def test_far_target_is_in_other_aisle_away_from_pillars(seed):
    t = pick_target(FAR, seed, START)
    assert abs(t.y - AISLE_BC_Y) <= AISLE_HALF_M
    assert all(math.hypot(t.x - px, AISLE_BC_Y - py) > PILLAR_CLEAR_M for px, py in PILLARS_BC)


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        pick_target('teleport', 1, START)


def test_robot_frame_error_splits_along_and_lateral():
    gt = Pose2D(0.0, 0.0, 0.0, math.pi / 2)            # 북쪽을 봄
    est = Pose2D(0.0, 0.05, 0.2, math.pi / 2 + 0.05)   # 앞으로 0.2, 오른쪽으로 0.05
    along, lateral, yaw = robot_frame_error(est, gt)
    assert along == pytest.approx(0.2)
    assert lateral == pytest.approx(-0.05)
    assert yaw == pytest.approx(0.05)


def samples(errors, t0=10.0, dt=0.1):
    return [Sample(t0 + i * dt, *e) for i, e in enumerate(errors)]


def test_first_hold_needs_continuous_window():
    c = Criteria(hold_s=1.0)
    bad, good = (1.0, 0.5, 0.3), (0.05, 0.02, 0.01)
    s = samples([bad] * 10 + [good] * 5 + [bad] + [good] * 15)   # 첫 좋은 구간은 0.4초뿐
    assert first_hold(s, 10.0, 70.0, c) == pytest.approx(11.6)


def test_first_hold_respects_window_end():
    c = Criteria(hold_s=1.0)
    s = samples([(1.0, 0.5, 0.3)] * 50 + [(0.0, 0.0, 0.0)] * 20)
    assert first_hold(s, 10.0, 12.0, c) is None


def test_evaluate_success_counts_from_detection():
    states = [(5.0, 'tracking'), (21.5, 'settling'), (30.0, 'tracking')]
    s = samples([(1.5, 1.2, 0.5)] * 80 + [(0.05, 0.03, 0.01)] * 50, t0=20.0)   # 28.0초부터 기준 안
    r = evaluate(20.0, states, s)
    assert r['detected'] and r['detect_s'] == 1.5
    assert r['recovered'] and r['recover_s'] == pytest.approx(6.5)
    assert r['success'] and r['false_alarms'] == 0
    assert r['final_state'] == 'tracking'


def test_evaluate_late_detection_is_not_success():
    states = [(5.0, 'tracking'), (27.0, 'settling')]       # 순간 이동 7초 뒤 감지 (기준 5초)
    s = samples([(0.0, 0.0, 0.0)] * 100, t0=20.0)
    r = evaluate(20.0, states, s)
    assert not r['detected'] and r['detect_s'] == 7.0
    assert r['recovered'] and not r['success']


def test_evaluate_wrong_convergence_is_not_recovered():
    # 다른 통로로 수렴: 옆 방향은 맞지만 진행 방향 7.5 m 틀림
    states = [(5.0, 'tracking'), (21.0, 'settling'), (40.0, 'tracking')]
    s = samples([(7.5, 0.02, 0.01)] * 700, t0=20.0)
    r = evaluate(20.0, states, s)
    assert r['detected'] and not r['recovered'] and not r['success']


def test_evaluate_counts_false_alarms_before_kidnap():
    states = [(1.0, 'tracking'), (3.0, 'settling'), (6.0, 'tracking'), (21.0, 'settling')]
    r = evaluate(20.0, states, [])
    assert r['false_alarms'] == 1
    assert not r['recovered'] and r['final_error'] is None
