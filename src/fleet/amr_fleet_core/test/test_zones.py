"""구역 허가: 정원·lease·대기열."""

import pytest

from amr_fleet_core.traffic import (GRANT_GRANTED, GRANT_QUEUED, GRANT_REJECTED, REASON_OCCUPIED, ZoneManager,
                                    ZoneSpec)

LEASE = 5.0


def make(*specs):
    return ZoneManager(specs or [ZoneSpec('narrow_01')], lease_s=LEASE)


def test_invalid_config():
    with pytest.raises(ValueError):
        ZoneSpec('z', capacity=0)
    with pytest.raises(ValueError):
        ZoneManager([ZoneSpec('z'), ZoneSpec('z')], lease_s=LEASE)
    with pytest.raises(ValueError):
        ZoneManager([ZoneSpec('z')], lease_s=0.0)


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


def test_expired_lease_hands_over():
    zm = make()
    zm.request('amr_01', 'narrow_01', 1, 0.0)
    # 대기 로봇은 계속 다시 요청한다 (대기열 유지)
    zm.request('amr_02', 'narrow_01', 5, 1.0)
    zm.request('amr_02', 'narrow_01', 5, 4.0)
    [d] = zm.expire(LEASE)
    assert (d.robot_id, d.grant, d.request_id) == ('amr_02', GRANT_GRANTED, 5)
    assert zm.holders('narrow_01') == ['amr_02']


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
