"""가짜 로봇의 수신 규칙과 이동 모델. rclpy 없이 돈다."""

import pytest

from amr_fleet_manager.robot_sim import DUPLICATE, NEW, STALE, AssignmentFilter, TaskRun, assignment_source


# 경매로 로봇 k가 수락한 명령 번호 (amr_fleet_decentralized #211: 1,000,000 + k × 100,000 + 순번)
def auction_id(robot: int, seq: int) -> int:
    return 1_000_000 + robot * 100_000 + seq


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


def test_assignment_source():
    assert assignment_source(1) == assignment_source(999_999) == 'server'
    assert assignment_source(auction_id(1, 0)) == 'auction:1'
    assert assignment_source(auction_id(3, 99_999)) == 'auction:3'


def test_filter_compares_within_source():
    """하이브리드: 경매 번호(1,000,000+) 뒤에 온 서버 명령도 새 명령이다 (#211)."""
    f = AssignmentFilter()
    f.record(5, 'server-ok')
    f.record(auction_id(2, 1), 'auction-ok')
    assert f.classify(6) == NEW                     # 서버 6번: 경매 번호보다 작지만 서버 5번보다 크다
    assert f.classify(4) == STALE                   # 서버끼리는 여전히 뒤늦은 명령을 버린다
    assert f.classify(auction_id(2, 0)) == STALE    # 경매도 출처 안에서 비교
    assert f.classify(auction_id(3, 0)) == NEW      # 다른 경매 로봇은 따로
    assert f.classify(5) == DUPLICATE and f.last_reply == 'server-ok'
    assert f.classify(auction_id(2, 1)) == DUPLICATE and f.last_reply == 'auction-ok'
    assert f.last_id == 5                           # RobotState.last_assignment_id는 서버 번호


def test_epoch_change_resets_only_server_source():
    f = AssignmentFilter()
    f.on_heartbeat(1)
    f.record(40, 'ok')
    f.record(auction_id(1, 7), 'ok')
    assert f.on_heartbeat(2)
    assert f.classify(1) == NEW                      # 새 서버는 1번부터
    assert f.classify(auction_id(1, 7)) == DUPLICATE  # 경매 번호는 서버 재시작과 무관


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
