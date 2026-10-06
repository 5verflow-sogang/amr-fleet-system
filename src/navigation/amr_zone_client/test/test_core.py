"""로봇 측 구역 진입 처리 상태 기계 (core.py). ROS 비의존, rclpy 없이 돈다."""

import pytest

from amr_zone_client.core import (
    GRANT_GRANTED, GRANT_QUEUED, GRANT_REJECTED, Phase, TYPE_ACCESS, TYPE_RELEASE,
    ZoneClientConfig, ZoneClientCore, ZoneConfig,
)
from amr_zone_client.geometry import speed_cap_for_stop

RECT = (5.0, 6.0, -1.0, 1.0)


def straight_path(x_from, x_to, step=0.1, y=0.0):
    n = int(round((x_to - x_from) / step))
    return [(x_from + i * step, y) for i in range(1, n + 1)]


def make_core(zones=None, **kwargs):
    zones = zones if zones is not None else [ZoneConfig('z1', RECT)]
    return ZoneClientCore(ZoneClientConfig(robot_id='amr_01', zones=zones, **kwargs))


def test_trigger_at_3m():
    core = make_core()
    outgoing, cap = core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', 1, TYPE_ACCESS)]
    assert core.phase('z1') is Phase.APPROACHING
    assert cap is not None

    core2 = make_core()
    outgoing2, _ = core2.update(0.0, straight_path(1.99, 10.0), (1.99, 0.0))
    assert outgoing2 == []
    assert core2.phase('z1') is Phase.IDLE


def test_request_id_increment_and_resend_cadence():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    assert outgoing[0].request_id == 1
    assert core.request_id('z1') == 1

    outgoing, _ = core.update(0.5, path, (2.0, 0.0))
    assert outgoing == []  # 1초가 안 지났으면 재전송하지 않는다

    outgoing, _ = core.update(1.0, path, (2.0, 0.0))
    assert [(o.request_id, o.type) for o in outgoing] == [(1, TYPE_ACCESS)]  # 같은 번호로 재전송(연장)

    outgoing, _ = core.update(1.9, path, (2.0, 0.0))
    assert outgoing == []

    outgoing, _ = core.update(2.0, path, (2.0, 0.0))
    assert [(o.request_id, o.type) for o in outgoing] == [(1, TYPE_ACCESS)]


def test_stop_speed_profile_without_grant():
    # Nav2 /plan은 매번 로봇 현재 위치에서부터 다시 나오므로, 매 호출마다 그 위치 앞쪽 경로를 새로 만든다
    core = make_core()

    _, cap = core.update(0.0, straight_path(2.5, 10.0), (2.5, 0.0))  # 경계까지 2.5 m
    assert cap == pytest.approx(speed_cap_for_stop(2.5, 0.5, 1.0))

    _, cap = core.update(0.1, straight_path(4.5, 10.0), (4.5, 0.0))  # 경계(5.0) 0.5 m 앞
    assert cap == 0.0


def test_queued_then_granted():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, cap = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    assert cap is not None and cap > 0.0

    assert core.on_grant('amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02') is None
    assert core.grant('z1') == GRANT_QUEUED
    _, cap_queued = core.update(1.0, path, (2.0, 0.0))
    assert cap_queued is not None  # 대기 중이라 여전히 0.5 m 앞에 멈춰야 한다

    assert core.on_grant('amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_01') is None
    assert core.grant('z1') == GRANT_GRANTED
    _, cap_granted = core.update(2.0, path, (2.0, 0.0))
    assert cap_granted is None  # 허가되면 제한이 풀린다


def test_rejected_blocks_entry_and_keeps_stop_cap():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    warning = core.on_grant('amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert warning is not None and 'z1' in warning
    assert core.phase('z1') is Phase.BLOCKED

    outgoing, cap = core.update(1.0, path, (2.0, 0.0))
    assert outgoing == []  # 거부됐으니 재시도(재전송)하지 않는다
    assert cap is not None  # 그래도 들어가지 못하게 계속 멈춰 세운다


def test_lease_expiry_inside_zone_continues_then_releases():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant('amr_01', 'z1', rid, GRANT_GRANTED, 10.0, 0, 'amr_01')

    # lease_expiry(10.0)가 이미 지난 시각(20.0)에 구역 안(x=5.5)에 있다
    outgoing_in, cap_in = core.update(20.0, path, (5.5, 0.0))
    assert outgoing_in == []
    assert cap_in is None  # 구역 안에서는 lease가 지나도 멈추지 않는다
    assert core.phase('z1') is Phase.INSIDE

    outgoing_out, _ = core.update(21.0, path, (7.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_out] == [('z1', rid, TYPE_RELEASE)]
    assert core.phase('z1') is Phase.IDLE


def test_ignores_other_robot_grant():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    assert core.on_grant('amr_99', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_99') is None
    assert core.grant('z1') is None  # 다른 로봇 앞 허가라 내 상태는 안 바뀐다

    _, cap = core.update(1.0, path, (2.0, 0.0))
    assert cap is not None  # 허가 못 받은 것으로 취급해 계속 멈춰 세운다


def test_consecutive_zones_independent():
    # Nav2 /plan은 매번 로봇 현재 위치에서부터 다시 나오므로, 매 호출마다 그 위치 앞쪽 경로를 새로 만든다
    core = make_core(zones=[ZoneConfig('z1', (5.0, 6.0, -1.0, 1.0)), ZoneConfig('z2', (8.0, 9.0, -1.0, 1.0))])

    outgoing, _ = core.update(0.0, straight_path(2.0, 15.0), (2.0, 0.0))
    assert [(o.zone_id, o.request_id) for o in outgoing] == [('z1', 1)]
    core.on_grant('amr_01', 'z1', 1, GRANT_GRANTED, 100.0, 0, 'amr_01')

    core.update(1.0, straight_path(5.5, 15.0), (5.5, 0.0))
    assert core.phase('z1') is Phase.INSIDE

    # z1을 막 벗어나면서(x=6.01) 동시에 z2 트리거 거리(8.0 - 6.01 = 1.99 m) 안으로 들어간다
    outgoing, _ = core.update(2.0, straight_path(6.01, 15.0), (6.01, 0.0))
    kinds = {(o.zone_id, o.type) for o in outgoing}
    assert ('z1', TYPE_RELEASE) in kinds
    assert ('z2', TYPE_ACCESS) in kinds
    z2_request = next(o for o in outgoing if o.zone_id == 'z2')
    assert z2_request.request_id == 2  # z1 반납과 상관없이 새 번호
    assert core.phase('z1') is Phase.IDLE
    assert core.phase('z2') is Phase.APPROACHING
