"""로봇 측 구역 진입 처리 상태 기계 (core.py). ROS 비의존, rclpy 없이 돈다."""

import math

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


def test_trigger_with_path_starting_behind_robot():
    # Nav2 /plan이 항상 로봇 바로 앞부터 시작하는 건 아니다 - 로봇이 지나온 뒤쪽 점도 담고 있을 수
    # 있다. 남은 거리는 로봇과 가장 가까운 점부터 재야 한다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(0.0, 10.0)  # 0.1 ~ 10.0, 로봇(2.0) 뒤쪽 점(0.1~1.9)도 섞여 있다
    outgoing, cap = core.update(0.0, path, (2.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', 1, TYPE_ACCESS)]
    assert cap == pytest.approx(speed_cap_for_stop(3.0, 0.5, 1.0))


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

    assert core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02') is None
    assert core.grant('z1') == GRANT_QUEUED
    _, cap_queued = core.update(1.0, path, (2.0, 0.0))
    assert cap_queued is not None  # 대기 중이라 여전히 0.5 m 앞에 멈춰야 한다

    assert core.on_grant(1.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_01') is None
    assert core.grant('z1') == GRANT_GRANTED
    _, cap_granted = core.update(2.0, path, (2.0, 0.0))
    assert cap_granted is None  # 허가되면 제한이 풀린다


def test_crosses_boundary_without_valid_grant_keeps_zero_cap_and_requesting():
    # 허가 없이(또는 QUEUED인 채로) 경계를 넘어가 버려도 INSIDE로 바뀌면 안 된다 - 상한 0을 유지하고
    # 같은 request_id로 계속 요청해야 한다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02')  # 아직 허가 안 됨

    # 제어가 정지를 못 지켜 구역 안(5.5)까지 들어가 버렸다고 가정
    outgoing_in, cap_in = core.update(1.0, path, (5.5, 0.0))
    assert core.phase('z1') is Phase.APPROACHING  # INSIDE로 안 바뀐다
    assert cap_in == 0.0  # remaining=0 기준 상한도 0
    assert [(o.request_id, o.type) for o in outgoing_in] == [(rid, TYPE_ACCESS)]  # 같은 번호로 계속 요청


def test_lease_expires_outside_zone_reapplies_stop_cap():
    # 구역 밖에서 허가 lease가 지나면 허가 없음으로 보고 정지 상한을 다시 건다. request_id는 그대로다
    # (#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 2.0, 0, 'amr_01')  # lease 2.0초까지

    _, cap_before_expiry = core.update(1.0, path, (2.0, 0.0))
    assert cap_before_expiry is None  # 아직 유효한 허가

    outgoing_after, cap_after_expiry = core.update(3.0, path, (2.0, 0.0))  # lease(2.0) 지남, 아직 구역 밖
    assert cap_after_expiry is not None and cap_after_expiry > 0.0  # 정지 상한 다시 적용
    assert [(o.request_id, o.type) for o in outgoing_after] == [(rid, TYPE_ACCESS)]  # 같은 번호로 재요청


def test_rejected_blocks_entry_and_keeps_stop_cap():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    warning = core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert warning is not None and 'z1' in warning
    assert core.phase('z1') is Phase.BLOCKED

    outgoing, cap = core.update(1.0, path, (2.0, 0.0))
    assert outgoing == []  # retry_period_s(기본 5초)가 안 지났으면 재시도하지 않는다
    assert cap is not None  # 그래도 들어가지 못하게 계속 멈춰 세운다


def test_blocked_retries_with_new_request_id_after_retry_period():
    # REJECTED 뒤 경로에 구역이 남아 있으면 retry_period_s 뒤 새 request_id로 다시 요청한다
    # (#66 하연 리뷰)
    core = make_core(retry_period_s=5.0)
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert core.phase('z1') is Phase.BLOCKED

    outgoing, _ = core.update(4.9, path, (2.0, 0.0))
    assert outgoing == []  # 아직 5초가 안 지났다

    outgoing, _ = core.update(5.0, path, (2.0, 0.0))
    assert core.phase('z1') is Phase.APPROACHING  # 재시도하며 APPROACHING으로 돌아간다
    assert len(outgoing) == 1
    assert outgoing[0].type == TYPE_ACCESS
    assert outgoing[0].request_id == rid + 1  # 새 번호
    assert core.grant('z1') is None  # 이전 REJECTED가 안 남아 있다


def test_blocked_returns_to_idle_without_release_when_zone_leaves_path():
    # REJECTED 상태에서 경로가 바뀌어 그 구역을 더는 지나지 않고 로봇도 구역 밖이면 IDLE로 돌아간다.
    # 서버가 자리를 쥔 적이 없어서(REJECTED) 반납은 보내지 않는다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert core.phase('z1') is Phase.BLOCKED

    # 경로가 바뀌어 더는 z1을 지나지 않는다 (y=5로 우회)
    detour = straight_path(2.0, 10.0, y=5.0)
    outgoing, cap = core.update(1.0, detour, (2.0, 5.0))
    assert outgoing == []  # 반납 없음 - 쥔 적이 없다
    assert cap is None
    assert core.phase('z1') is Phase.IDLE


def test_path_change_drops_zone_releases_and_returns_idle():
    # 대기(QUEUED)·통과 허가(GRANTED) 중에 경로가 바뀌어 구역이 빠지고 로봇도 구역 밖이면, 서버가
    # 자리를 계속 쥐고 있지 않게 반납하고 IDLE로 돌아간다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02')

    # 경로가 바뀌어 더는 z1을 지나지 않는다 (y=5로 우회), 로봇은 아직 구역 밖
    detour = straight_path(2.0, 10.0, y=5.0)
    outgoing_release, cap = core.update(1.0, detour, (2.0, 5.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_release] == [('z1', rid, TYPE_RELEASE)]
    assert cap is None
    assert core.phase('z1') is Phase.IDLE
    assert core.grant('z1') is None

    # 그 뒤로는 완전히 새 진입처럼 다룬다(다음 트리거에서 새 request_id)
    outgoing_again, _ = core.update(2.0, path, (2.0, 0.0))
    assert outgoing_again[0].request_id == rid + 1


def test_lease_expiry_inside_zone_continues_then_releases():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 10.0, 0, 'amr_01')

    # lease(10.0)가 아직 유효한 t=5에 경계(5.0)에 닿아 INSIDE로 전환된다
    core.update(5.0, straight_path(5.0, 10.0), (5.0, 0.0))
    assert core.phase('z1') is Phase.INSIDE

    # lease_expiry(10.0)가 이미 지난 시각(20.0)에도 구역 안(x=5.5)이면 안 멈추고, 계속 연장한다
    outgoing_in, cap_in = core.update(20.0, path, (5.5, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_in] == [('z1', rid, TYPE_ACCESS)]
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
    assert core.on_grant(0.0, 'amr_99', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_99') is None
    assert core.grant('z1') is None  # 다른 로봇 앞 허가라 내 상태는 안 바뀐다

    _, cap = core.update(1.0, path, (2.0, 0.0))
    assert cap is not None  # 허가 못 받은 것으로 취급해 계속 멈춰 세운다


def test_consecutive_zones_independent():
    # Nav2 /plan은 매번 로봇 현재 위치에서부터 다시 나오므로, 매 호출마다 그 위치 앞쪽 경로를 새로 만든다
    core = make_core(zones=[ZoneConfig('z1', (5.0, 6.0, -1.0, 1.0)), ZoneConfig('z2', (8.0, 9.0, -1.0, 1.0))])

    outgoing, _ = core.update(0.0, straight_path(2.0, 15.0), (2.0, 0.0))
    assert [(o.zone_id, o.request_id) for o in outgoing] == [('z1', 1)]
    core.on_grant(0.0, 'amr_01', 'z1', 1, GRANT_GRANTED, 100.0, 0, 'amr_01')

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


def test_lookahead_defaults_to_trigger_distance_plus_margin():
    # lookahead_m을 안 주면 trigger_distance_m + 여유로 정한다. 직접 주면 그 값을 쓴다 (#66)
    core = make_core(trigger_distance_m=3.0)
    assert core.lookahead_m == pytest.approx(5.0)

    core_custom = make_core(trigger_distance_m=3.0, lookahead_m=10.0)
    assert core_custom.lookahead_m == pytest.approx(10.0)


# --- SafetyStatus.speed_limit 연동 (#69·#70): 구역 상한과 min(), 낮아질 때만 부드럽게 ---


def test_safety_speed_limit_inf_means_no_limit():
    core = make_core(zones=[])
    _, cap = core.update(0.0, [], (0.0, 0.0), safety_speed_limit_mps=math.inf)
    assert cap is None


def test_safety_speed_limit_warning_ramps_down_from_no_limit():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0))  # 시드 틱 (dt=0, 아직 램프를 시작할 기준 시각이 없다)
    _, cap = core.update(0.5, [], (0.0, 0.0), safety_speed_limit_mps=0.6)
    # dt=0.5s, decel=1.0 -> 최고속 2.0에서 최대 0.5만큼만 내려간다(1.5) - 아직 목표(0.6) 위
    assert cap == pytest.approx(1.5)


def test_safety_speed_limit_critical_ramps_to_zero_over_multiple_ticks():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0))
    _, cap_1s = core.update(1.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert cap_1s == pytest.approx(1.0)  # 2.0 - 1.0*1.0
    _, cap_3s = core.update(3.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert cap_3s == pytest.approx(0.0)  # 1.0 - 1.0*2.0 -> 0 아래로는 안 내려간다


def test_safety_speed_limit_critical_decelerates_from_actual_current_speed():
    # #93 연우 리뷰: 로봇이 실제로는 0.5m/s로만 가고 있는데(아직 제한이 없어서 prev가
    # 최고속 2.0으로 잡혀 있다) CRITICAL(0.0)이 오면, 가짜로 높은 2.0에서부터 내려가지
    # 않고 실제 속도 0.5에서 바로 decel_mps2(1.0)로 감속을 시작해 0.5초 안에 멈춘다.
    # (current_speed_mps 없이 2.0에서부터였다면 t=0.25에서 아직 1.75로, 0.5보다 높아
    # cmd_vel = min(명령, 상한)에 전혀 영향이 없었을 것.)
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0), current_speed_mps=0.5)  # 시드 틱 (dt=0)
    _, cap_mid = core.update(
        0.25, [], (0.0, 0.0), safety_speed_limit_mps=0.0, current_speed_mps=0.5)
    assert cap_mid == pytest.approx(0.25)  # 0.5 - 1.0*0.25
    _, cap_stop = core.update(
        0.5, [], (0.0, 0.0), safety_speed_limit_mps=0.0, current_speed_mps=0.5)
    assert cap_stop == pytest.approx(0.0)


def test_safety_speed_limit_combines_with_zone_cap_as_smaller():
    # 구역 상한(접근 중, sqrt(5)~=2.236)과 speed_limit(1.0) 중 더 작은 쪽을 쓴다
    core = make_core()
    core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0), safety_speed_limit_mps=1.0)
    _, cap = core.update(1.0, straight_path(2.0, 10.0), (2.0, 0.0), safety_speed_limit_mps=1.0)
    assert cap == pytest.approx(1.0)  # 1.0 < sqrt(5) ~= 2.236이라 speed_limit 쪽이 더 빡빡하다


def test_safety_speed_limit_releases_immediately_after_disconnect():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    _, capped = core.update(1.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert capped == pytest.approx(1.0)
    # 노드가 끊김(타임아웃)으로 보면 None을 넘긴다 - 완화라 바로 반영한다(제한 없음)
    _, released = core.update(1.1, [], (0.0, 0.0), safety_speed_limit_mps=None)
    assert released is None
