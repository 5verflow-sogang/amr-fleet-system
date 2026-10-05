"""가짜 로봇의 수신 규칙과 이동 모델. rclpy 없이 돈다."""

import pytest

from amr_fleet_manager.robot_sim import DUPLICATE, NEW, STALE, AssignmentFilter, TaskRun


def test_filter_new_duplicate_stale():
    f = AssignmentFilter()
    assert f.classify(1) == NEW
    f.record(1, 'ok')
    assert f.classify(1) == DUPLICATE
    assert f.last_reply == 'ok'
    assert f.classify(3) == NEW
    f.record(3, 'ok')
    # 지연으로 늦게 온 2번은 무시
    assert f.classify(2) == STALE


def test_filter_resets_on_epoch_change():
    f = AssignmentFilter()
    assert not f.on_heartbeat(1)
    f.record(40, 'ok')
    assert not f.on_heartbeat(1)
    assert f.classify(1) == STALE
    # 서버 재시작: 새 서버는 1번부터 보낸다
    assert f.on_heartbeat(2)
    assert f.classify(1) == NEW


def test_delivery_phases_and_distance():
    run = TaskRun((0.0, 0.0), pickup=(3.0, 4.0), dropoff=(3.0, 0.0), speed=1.0, dwell_s=1.0)
    assert run.phase == 'move_to_pickup'

    assert not run.step(2.5)
    assert run.xy == pytest.approx((1.5, 2.0))
    assert run.velocity == pytest.approx((0.6, 0.8))

    assert run.step(3.0)  # 5m 도착 후 0.5초 대기
    assert run.phase == 'loading'
    assert run.step(0.5)
    assert run.phase == 'move_to_dropoff'
    assert run.step(4.0)
    assert run.phase == 'unloading'
    assert run.step(1.0)
    assert run.done
    assert run.distance == pytest.approx(9.0)
    assert run.velocity == (0.0, 0.0)


def test_charge_has_no_pickup():
    run = TaskRun((0.0, 0.0), pickup=None, dropoff=(2.0, 0.0), speed=2.0, dwell_s=1.0)
    assert run.phase == 'move_to_dropoff'
    run.step(1.0)
    assert run.done


def test_zero_dwell_skips_wait():
    run = TaskRun((0.0, 0.0), pickup=(1.0, 0.0), dropoff=(2.0, 0.0), speed=1.0, dwell_s=0.0)
    run.step(1.0)
    assert run.phase == 'move_to_dropoff'


def test_invalid_speed():
    with pytest.raises(ValueError):
        TaskRun((0.0, 0.0), None, (1.0, 0.0), speed=0.0, dwell_s=0.0)
