"""대치 양보 (zone_id standoff:back|forward, core.py '대치 양보')."""

import pytest

from amr_zone_client.core import (
    GRANT_GRANTED, GRANT_REVOKED, Phase, STANDOFF_ZONE_BACK, STANDOFF_ZONE_FORWARD,
    ZoneClientConfig, ZoneClientCore, ZoneConfig,
)

FAR = (50.0, 51.0, -1.0, 1.0)   # 경로에 안 걸리는 구역
DEADLOCK = 3


def make_core(**kwargs):
    return ZoneClientCore(ZoneClientConfig(robot_id='amr_01', zones=[ZoneConfig('z1', FAR)], **kwargs))


def order(core, zone_id, now=0.0, robot='amr_01'):
    return core.on_grant(now, robot, zone_id, 0, GRANT_REVOKED, None, DEADLOCK, 'amr_02')


def run(core, start_x, t0, t1, dt=0.1, path=()):
    """명령대로 x축 위를 움직이며 update를 부른다. (마지막 x, 낸 속도 목록)"""
    x, t, speeds = start_x, t0, []
    while t < t1 - 1e-9:
        _, _, v, _ = core.update(t, list(path), (x, 0.0))
        speeds.append(v)
        if v is not None:
            x += v * dt
        t += dt
    return x, speeds


def test_back_yield_moves_distance_holds_then_ends():
    core = make_core()
    assert 'amr_02' in order(core, STANDOFF_ZONE_BACK)
    assert core.state_label(0.0) == 'backing'
    x, speeds = run(core, 0.0, 0.0, 30.0)
    assert x == pytest.approx(-0.6, abs=0.03)
    assert min(v for v in speeds if v is not None) == pytest.approx(-0.15)
    # 다 움직인 뒤 yield_hold_s 동안 0을 직접 내다가, 끝나면 None(평소대로 Nav2)
    assert speeds[-1] is None
    assert core.state_label(30.0) == 'idle'
    held = [v for v in speeds if v == 0.0]
    assert len(held) >= 75   # 8 s / 0.1 s 가까이


def test_forward_yield_moves_forward():
    core = make_core()
    order(core, STANDOFF_ZONE_FORWARD)
    x, _ = run(core, 0.0, 0.0, 30.0)
    assert x == pytest.approx(0.6, abs=0.03)


def test_blocked_behind_times_out_then_holds_without_failing():
    core = make_core(yield_timeout_s=3.0)
    order(core, STANDOFF_ZONE_BACK)
    t, speeds = 0.0, []
    while t < 30.0:   # 뒤가 막혀 위치가 그대로다
        speeds.append(core.update(t, [], (0.0, 0.0))[2])
        t += 0.1
    assert speeds[-1] is None
    assert core.state_label(30.0) == 'idle'


def test_safety_stop_caps_yield_speed():
    core = make_core()
    order(core, STANDOFF_ZONE_BACK)
    # 안전 상한은 decel_mps2(1.0)로 내려온다 (제한 없음 2.0 → 0까지 2 s, 구역 후진과 같다)
    t = 0.0
    while t <= 3.0:
        _, _, v, _ = core.update(t, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
        t += 0.1
    assert v == 0.0
    assert core.state_label(t) == 'backing'   # 못 움직였으니 아직 양보 중


def test_other_robot_or_second_order_is_ignored():
    core = make_core()
    assert order(core, STANDOFF_ZONE_BACK, robot='amr_02') is None
    assert core.state_label(0.0) == 'idle'
    assert order(core, STANDOFF_ZONE_BACK) is not None
    assert order(core, STANDOFF_ZONE_FORWARD) is None   # 양보 중 새 지시


def test_ignored_while_zone_backing():
    rect = (5.0, 6.0, -1.0, 1.0)
    core = ZoneClientCore(ZoneClientConfig(robot_id='amr_01', zones=[ZoneConfig('z1', rect)]))
    path = [(4.0 + 0.1 * i, 0.0) for i in range(1, 40)]
    core.update(0.0, path, (3.0, 0.0))
    rid = core.request_id('z1')
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, '')
    core.update(0.1, path, (5.5, 0.0))
    assert core.phase('z1') is Phase.INSIDE
    core.on_grant(0.2, 'amr_01', 'z1', rid, GRANT_REVOKED, None, DEADLOCK, 'amr_02')
    core.update(0.3, path, (5.5, 0.0))
    assert core.phase('z1') is Phase.BACKING
    assert order(core, STANDOFF_ZONE_BACK, now=0.4) is None


def test_blocked_signal_from_scan_holds_in_place():
    core = make_core()
    order(core, STANDOFF_ZONE_BACK)
    assert core.yield_direction == -1.0
    t, speeds, x = 0.0, [], 0.0
    while t < 20.0:   # 첫 틱부터 뒤가 막혔다고 알린다 → 움직이지 않고 서 있기만
        v = core.update(t, [], (x, 0.0), yield_blocked=True)[2]
        speeds.append(v)
        if v:
            x += v * 0.1
        t += 0.1
    assert x == 0.0 and max(abs(v) for v in speeds if v is not None) == 0.0
    assert core.yield_direction is None            # 서 있는 동안은 더 확인하지 않는다
    assert speeds[-1] is None and core.state_label(20.0) == 'idle'
