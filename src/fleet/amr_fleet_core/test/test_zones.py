"""구역 허가: 정원·lease·우선순위 대기열·교착 해소."""

import pytest

from amr_fleet_core.traffic import (GRANT_GRANTED, GRANT_QUEUED, GRANT_REJECTED, GRANT_REVOKED,
                                    REASON_DEADLOCK_RESOLUTION, REASON_OCCUPIED, REASON_PRIORITY_YIELD, ZoneManager,
                                    ZoneSpec)

LEASE = 5.0
RELEASE_TIMEOUT = 30.0


def make(*specs):
    return ZoneManager(specs or [ZoneSpec('narrow_01')], lease_s=LEASE, release_timeout_s=RELEASE_TIMEOUT)


def test_invalid_config():
    with pytest.raises(ValueError):
        ZoneSpec('z', capacity=0)
    with pytest.raises(ValueError):
        ZoneManager([ZoneSpec('z'), ZoneSpec('z')], lease_s=LEASE, release_timeout_s=RELEASE_TIMEOUT)
    with pytest.raises(ValueError):
        ZoneManager([ZoneSpec('z')], lease_s=0.0, release_timeout_s=RELEASE_TIMEOUT)
    with pytest.raises(ValueError):
        ZoneManager([ZoneSpec('z')], lease_s=LEASE, release_timeout_s=LEASE)


def test_unknown_zone_rejected():
    [d] = make().request('amr_01', 'nowhere', 7, 0.0)
    assert (d.grant, d.request_id, d.zone_id) == (GRANT_REJECTED, 7, 'nowhere')


def test_grant_then_queue_then_handover():
    zm = make()
    [a] = zm.request('amr_01', 'narrow_01', 1, 0.0)
    assert (a.grant, a.lease_expiry, a.holder_robot_id) == (GRANT_GRANTED, LEASE, 'amr_01')

    [b] = zm.request('amr_02', 'narrow_01', 10, 1.0)
    assert (b.grant, b.reason, b.holder_robot_id) == (GRANT_QUEUED, REASON_OCCUPIED, 'amr_01')
    [c] = zm.request('amr_03', 'narrow_01', 20, 1.5)
    assert c.grant == GRANT_QUEUED
    assert zm.queue('narrow_01') == ['amr_02', 'amr_03']

    # 반납한 로봇에게는 답하지 않고, 먼저 기다린 amr_02가 허가받는다
    [d] = zm.release('amr_01', 'narrow_01', 2.0)
    assert (d.robot_id, d.grant, d.request_id, d.lease_expiry) == ('amr_02', GRANT_GRANTED, 10, 2.0 + LEASE)
    assert zm.holders('narrow_01') == ['amr_02']
    assert zm.queue('narrow_01') == ['amr_03']


def test_renewal_extends_lease():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    [d] = zm.request('amr_01', 'narrow_01', 1, 4.0)
    assert (d.grant, d.lease_expiry) == (GRANT_GRANTED, 4.0 + LEASE)
    assert zm.expire(LEASE + 1.0) == []
    assert zm.holders('narrow_01') == ['amr_01']


def test_expired_lease_keeps_zone_until_release():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    # amr_01의 연장이 끊긴다 (서버 끊김·유실). 대기 로봇은 계속 다시 요청한다
    for t in (1.0, 4.0, 8.0, 12.0):
        assert zm.request('amr_02', 'narrow_01', 5, t)[0].grant == GRANT_QUEUED
        # lease가 지나도 통로 안에 있을 수 있으므로 넘기지 않는다
        assert zm.expire(t) == []
    assert zm.holders('narrow_01') == ['amr_01']

    # 빠져나와 반납하면 그때 넘긴다
    [d] = zm.release('amr_01', 'narrow_01', 13.0)
    assert (d.robot_id, d.grant, d.request_id) == ('amr_02', GRANT_GRANTED, 5)


def test_silent_holder_released_after_timeout():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    zm.request('amr_02', 'narrow_01', 5, RELEASE_TIMEOUT - 1.0)
    assert zm.expire(RELEASE_TIMEOUT - 0.5) == []
    # amr_01이 30초 동안 아무 요청도 없으면 죽은 것으로 보고 푼다
    [d] = zm.expire(RELEASE_TIMEOUT)
    assert (d.robot_id, d.grant) == ('amr_02', GRANT_GRANTED)


def test_expired_holder_can_renew():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    [d] = zm.request('amr_01', 'narrow_01', 1, 10.0)
    assert (d.grant, d.lease_expiry) == (GRANT_GRANTED, 10.0 + LEASE)


def test_silent_waiter_dropped():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    zm.request('amr_02', 'narrow_01', 5, 1.0)
    zm.request('amr_01', 'narrow_01', 1, 5.0)
    # amr_02는 1초 이후 요청이 없어 6초에 대기열에서 빠진다
    assert zm.expire(6.0) == []
    assert zm.queue('narrow_01') == []


def test_release_while_waiting_leaves_queue():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    zm.request('amr_02', 'narrow_01', 5, 1.0)
    assert zm.release('amr_02', 'narrow_01', 2.0) == []
    assert zm.queue('narrow_01') == []
    assert zm.release('amr_01', 'narrow_01', 3.0) == []


def test_capacity_two():
    zm = make(ZoneSpec('wide', capacity=2))
    assert zm.request('amr_01', 'wide', 1, 0.0)[0].grant == GRANT_GRANTED
    assert zm.request('amr_02', 'wide', 1, 0.0)[0].grant == GRANT_GRANTED
    assert zm.request('amr_03', 'wide', 1, 0.0)[0].grant == GRANT_QUEUED


def test_zones_independent():
    zm = make(ZoneSpec('a'), ZoneSpec('b'))
    assert zm.request('amr_01', 'a', 1, 0.0)[0].grant == GRANT_GRANTED
    assert zm.request('amr_02', 'b', 1, 0.0)[0].grant == GRANT_GRANTED


# ---- 교차로 우선순위 ----

def test_queue_order_priority_then_time_then_id():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    zm.request('amr_04', 'narrow_01', 1, 1.0, priority=0)
    zm.request('amr_03', 'narrow_01', 1, 2.0, priority=5)
    zm.request('amr_02', 'narrow_01', 1, 2.0, priority=5)

    order = []
    holder = 'amr_01'
    for t in (3.0, 4.0, 5.0):
        [d] = zm.release(holder, 'narrow_01', t)
        order.append(d.robot_id)
        holder = d.robot_id
    # 급한 작업(5) 먼저, 같으면 먼저 온 로봇, 같은 시각이면 robot_id 순. 늦게 온 급한 로봇이 앞선다
    assert order == ['amr_02', 'amr_03', 'amr_04']


def test_queued_behind_higher_priority_says_priority_yield():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    [d] = zm.request('amr_02', 'narrow_01', 1, 1.0, priority=0)
    assert (d.grant, d.reason, d.holder_robot_id) == (GRANT_QUEUED, REASON_OCCUPIED, 'amr_01')
    zm.request('amr_03', 'narrow_01', 1, 2.0, priority=9)
    # amr_02가 다시 요청하면 더 급한 amr_03에게 양보 중이라는 답을 받는다
    [d] = zm.request('amr_02', 'narrow_01', 1, 3.0, priority=0)
    assert (d.grant, d.reason, d.holder_robot_id) == (GRANT_QUEUED, REASON_PRIORITY_YIELD, 'amr_03')


# ---- 교착 해소 (구역 점유·대기 순환) ----

def zones(*ids):
    return make(*[ZoneSpec(z) for z in ids])


def test_two_robot_deadlock_revokes_lower_priority():
    # 좁은 통로 양 끝에서 마주 본 두 대: 각자 한 구역을 점유하고 상대 구역을 기다린다
    zm = zones('narrow_01', 'narrow_02')
    zm.request('amr_01', 'narrow_01', 1, 0.0, priority=5)
    zm.request('amr_02', 'narrow_02', 7, 0.0, priority=1)
    assert zm.resolve_deadlocks(0.5) == []
    zm.request('amr_01', 'narrow_02', 2, 1.0, priority=5)
    zm.request('amr_02', 'narrow_01', 8, 1.0, priority=1)
    assert zm.wait_for() == {'amr_01': {'amr_02': 'narrow_02'}, 'amr_02': {'amr_01': 'narrow_01'}}

    [d] = zm.resolve_deadlocks(1.5)
    assert (d.robot_id, d.zone_id, d.request_id, d.grant, d.reason, d.holder_robot_id) == \
        ('amr_02', 'narrow_02', 7, GRANT_REVOKED, REASON_DEADLOCK_RESOLUTION, 'amr_01')
    # 해소 중인 교착은 다시 잡지 않는다
    assert zm.resolve_deadlocks(2.0) == []

    # 물러나는 중에 다시 요청하면 같은 회수 답을 받고, 연장되지 않는다
    [d] = zm.request('amr_02', 'narrow_02', 7, 2.5, priority=1)
    assert (d.grant, d.reason) == (GRANT_REVOKED, REASON_DEADLOCK_RESOLUTION)
    # 반납 전에는 구역을 넘기지 않는다 (아직 통로 안에 있을 수 있다)
    assert zm.holders('narrow_02') == ['amr_02']

    zm.release('amr_02', 'narrow_01', 3.0)  # 기다리던 구역도 포기
    [d] = zm.release('amr_02', 'narrow_02', 3.0)
    assert (d.robot_id, d.zone_id, d.grant, d.request_id) == ('amr_01', 'narrow_02', GRANT_GRANTED, 2)
    assert zm.wait_for() == {}


def test_three_robot_deadlock():
    zm = zones('z1', 'z2', 'z3')
    for robot, held in (('amr_01', 'z1'), ('amr_02', 'z2'), ('amr_03', 'z3')):
        zm.request(robot, held, 1, 0.0, priority=3)
    for robot, wanted in (('amr_01', 'z2'), ('amr_02', 'z3'), ('amr_03', 'z1')):
        zm.request(robot, wanted, 2, 1.0, priority=3)

    # 우선순위가 같으면 robot_id가 가장 뒤인 로봇이 물러난다. 그 로봇이 점유한 z3를 기다리던 amr_02에게 양보
    [d] = zm.resolve_deadlocks(1.5)
    assert (d.robot_id, d.zone_id, d.grant, d.holder_robot_id) == ('amr_03', 'z3', GRANT_REVOKED, 'amr_02')
    assert zm.resolve_deadlocks(2.0) == []
    [d] = zm.release('amr_03', 'z3', 3.0)
    assert (d.robot_id, d.zone_id, d.grant) == ('amr_02', 'z3', GRANT_GRANTED)


def test_queue_without_cycle_is_not_deadlock():
    zm = zones('z1', 'z2')
    zm.request('amr_01', 'z1', 1, 0.0)
    zm.request('amr_02', 'z1', 1, 0.0)
    zm.request('amr_03', 'z1', 1, 0.0)
    assert zm.resolve_deadlocks(1.0) == []


def test_revoked_holder_still_times_out():
    zm = zones('z1', 'z2')
    zm.request('amr_01', 'z1', 1, 0.0)
    zm.request('amr_02', 'z2', 1, 0.0)
    zm.request('amr_01', 'z2', 2, 0.0)
    zm.request('amr_02', 'z1', 2, 0.0)
    [d] = zm.resolve_deadlocks(0.0)
    assert d.robot_id == 'amr_02'
    # 회수당한 로봇이 죽어 반납이 없어도 release_timeout_s가 지나면 풀린다.
    # amr_01은 계속 z2를 다시 요청한다 (대기는 lease_s 안에 다시 요청해야 남는다)
    for t in range(4, int(RELEASE_TIMEOUT), 4):
        zm.request('amr_01', 'z2', 2, float(t))
    decisions = zm.expire(RELEASE_TIMEOUT)
    assert ('amr_01', 'z2', GRANT_GRANTED) in [(x.robot_id, x.zone_id, x.grant) for x in decisions]
