"""로봇 측 구역 진입 처리 상태 기계 (core.py). ROS 비의존, rclpy 없이 돈다."""

import math

import pytest

from amr_zone_client.core import (
    GRANT_GRANTED, GRANT_QUEUED, GRANT_REJECTED, GRANT_REVOKED, Phase, TYPE_ACCESS, TYPE_RELEASE,
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
    outgoing, cap, _, _ = core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', 1, TYPE_ACCESS)]
    assert core.phase('z1') is Phase.APPROACHING
    assert cap is not None

    core2 = make_core()
    outgoing2, _, _, _ = core2.update(0.0, straight_path(1.99, 10.0), (1.99, 0.0))
    assert outgoing2 == []
    assert core2.phase('z1') is Phase.IDLE


def test_trigger_with_path_starting_behind_robot():
    # Nav2 /plan이 항상 로봇 바로 앞부터 시작하는 건 아니다 - 로봇이 지나온 뒤쪽 점도 담고 있을 수
    # 있다. 남은 거리는 로봇과 가장 가까운 점부터 재야 한다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(0.0, 10.0)  # 0.1 ~ 10.0, 로봇(2.0) 뒤쪽 점(0.1~1.9)도 섞여 있다
    outgoing, cap, _, _ = core.update(0.0, path, (2.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', 1, TYPE_ACCESS)]
    assert cap == pytest.approx(speed_cap_for_stop(3.0, 0.5, 1.0))


def test_request_id_increment_and_resend_cadence():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    assert outgoing[0].request_id == 1
    assert core.request_id('z1') == 1

    outgoing, _, _, _ = core.update(0.5, path, (2.0, 0.0))
    assert outgoing == []  # 1초가 안 지났으면 재전송하지 않는다

    outgoing, _, _, _ = core.update(1.0, path, (2.0, 0.0))
    assert [(o.request_id, o.type) for o in outgoing] == [(1, TYPE_ACCESS)]  # 같은 번호로 재전송(연장)

    outgoing, _, _, _ = core.update(1.9, path, (2.0, 0.0))
    assert outgoing == []

    outgoing, _, _, _ = core.update(2.0, path, (2.0, 0.0))
    assert [(o.request_id, o.type) for o in outgoing] == [(1, TYPE_ACCESS)]


def test_stop_speed_profile_without_grant():
    # Nav2 /plan은 매번 로봇 현재 위치에서부터 다시 나오므로, 매 호출마다 그 위치 앞쪽 경로를 새로 만든다
    core = make_core()

    _, cap, _, _ = core.update(0.0, straight_path(2.5, 10.0), (2.5, 0.0))  # 경계까지 2.5 m
    assert cap == pytest.approx(speed_cap_for_stop(2.5, 0.5, 1.0))

    _, cap, _, _ = core.update(0.1, straight_path(4.5, 10.0), (4.5, 0.0))  # 경계(5.0) 0.5 m 앞
    assert cap == 0.0


def test_queued_then_granted():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, cap, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    assert cap is not None and cap > 0.0

    assert core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02') is None
    assert core.grant('z1') == GRANT_QUEUED
    _, cap_queued, _, _ = core.update(1.0, path, (2.0, 0.0))
    assert cap_queued is not None  # 대기 중이라 여전히 0.5 m 앞에 멈춰야 한다

    assert core.on_grant(1.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_01') is None
    assert core.grant('z1') == GRANT_GRANTED
    _, cap_granted, _, _ = core.update(2.0, path, (2.0, 0.0))
    assert cap_granted is None  # 허가되면 제한이 풀린다


def test_crosses_boundary_without_valid_grant_keeps_zero_cap_and_requesting():
    # 허가 없이(또는 QUEUED인 채로) 경계를 넘어가 버려도 INSIDE로 바뀌면 안 된다 - 상한 0을 유지하고
    # 같은 request_id로 계속 요청해야 한다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02')  # 아직 허가 안 됨

    # 제어가 정지를 못 지켜 구역 안(5.5)까지 들어가 버렸다고 가정
    outgoing_in, cap_in, _, _ = core.update(1.0, path, (5.5, 0.0))
    assert core.phase('z1') is Phase.APPROACHING  # INSIDE로 안 바뀐다
    assert cap_in == 0.0  # remaining=0 기준 상한도 0
    assert [(o.request_id, o.type) for o in outgoing_in] == [(rid, TYPE_ACCESS)]  # 같은 번호로 계속 요청


def test_lease_expires_outside_zone_reapplies_stop_cap():
    # 구역 밖에서 허가 lease가 지나면 허가 없음으로 보고 정지 상한을 다시 건다. request_id는 그대로다
    # (#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 2.0, 0, 'amr_01')  # lease 2.0초까지

    _, cap_before_expiry, _, _ = core.update(1.0, path, (2.0, 0.0))
    assert cap_before_expiry is None  # 아직 유효한 허가

    outgoing_after, cap_after_expiry, _, _ = core.update(3.0, path, (2.0, 0.0))  # lease(2.0) 지남, 아직 구역 밖
    assert cap_after_expiry is not None and cap_after_expiry > 0.0  # 정지 상한 다시 적용
    assert [(o.request_id, o.type) for o in outgoing_after] == [(rid, TYPE_ACCESS)]  # 같은 번호로 재요청


def test_rejected_blocks_entry_and_keeps_stop_cap():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    warning = core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert warning is not None and 'z1' in warning
    assert core.phase('z1') is Phase.BLOCKED

    outgoing, cap, _, _ = core.update(1.0, path, (2.0, 0.0))
    assert outgoing == []  # retry_period_s(기본 5초)가 안 지났으면 재시도하지 않는다
    assert cap is not None  # 그래도 들어가지 못하게 계속 멈춰 세운다


def test_blocked_retries_with_new_request_id_after_retry_period():
    # REJECTED 뒤 경로에 구역이 남아 있으면 retry_period_s 뒤 새 request_id로 다시 요청한다
    # (#66 하연 리뷰)
    core = make_core(retry_period_s=5.0)
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert core.phase('z1') is Phase.BLOCKED

    outgoing, _, _, _ = core.update(4.9, path, (2.0, 0.0))
    assert outgoing == []  # 아직 5초가 안 지났다

    outgoing, _, _, _ = core.update(5.0, path, (2.0, 0.0))
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

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert core.phase('z1') is Phase.BLOCKED

    # 경로가 바뀌어 더는 z1을 지나지 않는다 (y=5로 우회)
    detour = straight_path(2.0, 10.0, y=5.0)
    outgoing, cap, _, _ = core.update(1.0, detour, (2.0, 5.0))
    assert outgoing == []  # 반납 없음 - 쥔 적이 없다
    assert cap is None
    assert core.phase('z1') is Phase.IDLE


def test_path_change_drops_zone_releases_and_returns_idle():
    # 대기(QUEUED)·통과 허가(GRANTED) 중에 경로가 바뀌어 구역이 빠지고 로봇도 구역 밖이면, 서버가
    # 자리를 계속 쥐고 있지 않게 반납하고 IDLE로 돌아간다(#66 하연 리뷰)
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_QUEUED, None, 0, 'amr_02')

    # 경로가 바뀌어 더는 z1을 지나지 않는다 (y=5로 우회), 로봇은 아직 구역 밖
    detour = straight_path(2.0, 10.0, y=5.0)
    outgoing_release, cap, _, _ = core.update(1.0, detour, (2.0, 5.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_release] == [('z1', rid, TYPE_RELEASE)]
    assert cap is None
    assert core.phase('z1') is Phase.IDLE
    assert core.grant('z1') is None

    # 그 뒤로는 완전히 새 진입처럼 다룬다(다음 트리거에서 새 request_id)
    outgoing_again, _, _, _ = core.update(2.0, path, (2.0, 0.0))
    assert outgoing_again[0].request_id == rid + 1


def test_lease_expiry_inside_zone_continues_then_releases():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 10.0, 0, 'amr_01')

    # lease(10.0)가 아직 유효한 t=5에 경계(5.0)에 닿아 INSIDE로 전환된다
    core.update(5.0, straight_path(5.0, 10.0), (5.0, 0.0))
    assert core.phase('z1') is Phase.INSIDE

    # lease_expiry(10.0)가 이미 지난 시각(20.0)에도 구역 안(x=5.5)이면 안 멈추고, 계속 연장한다
    outgoing_in, cap_in, _, _ = core.update(20.0, path, (5.5, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_in] == [('z1', rid, TYPE_ACCESS)]
    assert cap_in is None  # 구역 안에서는 lease가 지나도 멈추지 않는다
    assert core.phase('z1') is Phase.INSIDE

    outgoing_out, _, _, _ = core.update(21.0, path, (7.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing_out] == [('z1', rid, TYPE_RELEASE)]
    assert core.phase('z1') is Phase.IDLE


def test_ignores_other_robot_grant():
    core = make_core()
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    assert core.on_grant(0.0, 'amr_99', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_99') is None
    assert core.grant('z1') is None  # 다른 로봇 앞 허가라 내 상태는 안 바뀐다

    _, cap, _, _ = core.update(1.0, path, (2.0, 0.0))
    assert cap is not None  # 허가 못 받은 것으로 취급해 계속 멈춰 세운다


def test_consecutive_zones_independent():
    # Nav2 /plan은 매번 로봇 현재 위치에서부터 다시 나오므로, 매 호출마다 그 위치 앞쪽 경로를 새로 만든다
    core = make_core(zones=[ZoneConfig('z1', (5.0, 6.0, -1.0, 1.0)), ZoneConfig('z2', (8.0, 9.0, -1.0, 1.0))])

    outgoing, _, _, _ = core.update(0.0, straight_path(2.0, 15.0), (2.0, 0.0))
    assert [(o.zone_id, o.request_id) for o in outgoing] == [('z1', 1)]
    core.on_grant(0.0, 'amr_01', 'z1', 1, GRANT_GRANTED, 100.0, 0, 'amr_01')

    core.update(1.0, straight_path(5.5, 15.0), (5.5, 0.0))
    assert core.phase('z1') is Phase.INSIDE

    # z1을 막 벗어나면서(x=6.01) 동시에 z2 트리거 거리(8.0 - 6.01 = 1.99 m) 안으로 들어간다
    outgoing, _, _, _ = core.update(2.0, straight_path(6.01, 15.0), (6.01, 0.0))
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
    _, cap, _, _ = core.update(0.0, [], (0.0, 0.0), safety_speed_limit_mps=math.inf)
    assert cap is None


def test_safety_speed_limit_warning_ramps_down_from_no_limit():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0))  # 시드 틱 (dt=0, 아직 램프를 시작할 기준 시각이 없다)
    _, cap, _, _ = core.update(0.5, [], (0.0, 0.0), safety_speed_limit_mps=0.6)
    # dt=0.5s, decel=1.0 -> 최고속 2.0에서 최대 0.5만큼만 내려간다(1.5) - 아직 목표(0.6) 위
    assert cap == pytest.approx(1.5)


def test_safety_speed_limit_critical_ramps_to_zero_over_multiple_ticks():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0))
    _, cap_1s, _, _ = core.update(1.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert cap_1s == pytest.approx(1.0)  # 2.0 - 1.0*1.0
    _, cap_3s, _, _ = core.update(3.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert cap_3s == pytest.approx(0.0)  # 1.0 - 1.0*2.0 -> 0 아래로는 안 내려간다


def test_safety_speed_limit_critical_decelerates_from_actual_current_speed():
    # #93 연우 리뷰: 로봇이 실제로는 0.5m/s로만 가고 있는데(아직 제한이 없어서 prev가
    # 최고속 2.0으로 잡혀 있다) CRITICAL(0.0)이 오면, 가짜로 높은 2.0에서부터 내려가지
    # 않고 실제 속도 0.5에서 바로 decel_mps2(1.0)로 감속을 시작해 0.5초 안에 멈춘다.
    # (current_speed_mps 없이 2.0에서부터였다면 t=0.25에서 아직 1.75로, 0.5보다 높아
    # cmd_vel = min(명령, 상한)에 전혀 영향이 없었을 것.)
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0), current_speed_mps=0.5)  # 시드 틱 (dt=0)
    _, cap_mid, _, _ = core.update(
        0.25, [], (0.0, 0.0), safety_speed_limit_mps=0.0, current_speed_mps=0.5)
    assert cap_mid == pytest.approx(0.25)  # 0.5 - 1.0*0.25
    _, cap_stop, _, _ = core.update(
        0.5, [], (0.0, 0.0), safety_speed_limit_mps=0.0, current_speed_mps=0.5)
    assert cap_stop == pytest.approx(0.0)


def test_safety_speed_limit_combines_with_zone_cap_as_smaller():
    # 구역 상한(접근 중, sqrt(5)~=2.236)과 speed_limit(1.0) 중 더 작은 쪽을 쓴다
    core = make_core()
    core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0), safety_speed_limit_mps=1.0)
    _, cap, _, _ = core.update(1.0, straight_path(2.0, 10.0), (2.0, 0.0), safety_speed_limit_mps=1.0)
    assert cap == pytest.approx(1.0)  # 1.0 < sqrt(5) ~= 2.236이라 speed_limit 쪽이 더 빡빡하다


def test_safety_speed_limit_releases_immediately_after_disconnect():
    core = make_core(zones=[], decel_mps2=1.0, max_linear_velocity_mps=2.0)
    core.update(0.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    _, capped, _, _ = core.update(1.0, [], (0.0, 0.0), safety_speed_limit_mps=0.0)
    assert capped == pytest.approx(1.0)
    # 노드가 끊김(타임아웃)으로 보면 None을 넘긴다 - 완화라 바로 반영한다(제한 없음)
    _, released, _, _ = core.update(1.1, [], (0.0, 0.0), safety_speed_limit_mps=None)
    assert released is None


# --- GRANT_REVOKED(교착 해소, 10/11 결정): APPROACHING → BLOCKED(반납 후 재시도) ---


def test_approaching_revoked_releases_and_retries_like_blocked():
    core = make_core(retry_period_s=5.0)
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    warning = core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REVOKED, None, 3, '')
    assert warning is not None and 'z1' in warning
    assert core.phase('z1') is Phase.BLOCKED

    # 다음 틱에 반납(RELEASE)을 보낸다 - REJECTED와 달리 이미 허가를 쥐고 있었어서다
    outgoing, _, _, _ = core.update(0.1, path, (2.0, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', rid, TYPE_RELEASE)]

    # retry_period_s 전에는 재시도하지 않고, 계속 멈춰 세운다
    outgoing, cap, _, _ = core.update(4.9, path, (2.0, 0.0))
    assert outgoing == []
    assert cap is not None

    # retry_period_s(5.0, REVOKED를 받은 시각부터) 뒤 새 request_id로 재시도
    outgoing, _, _, _ = core.update(5.0, path, (2.0, 0.0))
    assert core.phase('z1') is Phase.APPROACHING
    assert len(outgoing) == 1 and outgoing[0].type == TYPE_ACCESS
    assert outgoing[0].request_id == rid + 1
    assert core.grant('z1') is None


# --- GRANT_REVOKED: INSIDE → BACKING(직접 후진) → margin에서 반납 → IDLE ---


def _enter_zone_and_revoke(core, zone_id='z1', revoke_at=6.0):
    """구역 안(x=5.0)까지 들어가 INSIDE가 된 뒤 REVOKED를 받는다. request_id를 돌려준다."""
    path = straight_path(2.0, 10.0)
    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', zone_id, rid, GRANT_GRANTED, 100.0, 0, 'amr_01')
    core.update(5.0, straight_path(5.0, 10.0), (5.0, 0.0))
    assert core.phase(zone_id) is Phase.INSIDE
    warning = core.on_grant(revoke_at, 'amr_01', zone_id, rid, GRANT_REVOKED, None, 3, '')
    assert warning is not None and zone_id in warning
    assert core.phase(zone_id) is Phase.BACKING
    return rid


def test_inside_revoked_backs_out_decelerates_then_releases_and_returns_idle():
    # dt를 1.0 s씩 줘서(decel_mps2=1.0, backing_speed_mps=0.15) 매 틱 목표 속도에 바로 닿게 하고
    # - ramp 자체의 가감속 속도는 test_geometry.py의 ramp_speed 테스트가 따로 본다 - 여기서는
    # 구역을 벗어난 뒤 margin 앞에서 감속해 정확히 멈추고 반납하는 상태 기계만 본다.
    core = make_core(backing_speed_mps=0.15, backing_margin_m=0.3, decel_mps2=1.0)
    path = straight_path(2.0, 10.0)
    rid = _enter_zone_and_revoke(core)

    # 첫 BACKING 틱(x=5.0, 구역 안)은 시드 틱이다(dt=0, 아직 가속 기준 시각이 없다) - ramp_cap_down과
    # 같은 패턴. backing_since·backing_last_t가 여기서 7.0으로 잡힌다.
    outgoing, cap, backing, errors = core.update(7.0, path, (5.0, 0.0))
    assert outgoing == [] and cap is None and errors == []
    assert backing == pytest.approx(0.0)
    assert core.phase('z1') is Phase.BACKING

    # dt=1.0 s(여전히 구역 안) - 목표 속도(0.15)까지 가속
    outgoing, _, backing, _ = core.update(8.0, path, (5.0, 0.0))
    assert outgoing == []
    assert backing == pytest.approx(-0.15)

    # 막 구역을 벗어났다(x=4.9) - margin(0.3 m)까지 아직 0.3 m 남아 그대로 순항
    outgoing, _, backing, _ = core.update(9.0, path, (4.9, 0.0))
    assert outgoing == []
    assert backing == pytest.approx(-0.15)

    # margin까지 0.01 m 남음 - speed_cap_for_stop(0.01, 0, 1.0)로 감속
    outgoing, _, backing, _ = core.update(10.0, path, (4.61, 0.0))
    assert outgoing == []
    assert backing == pytest.approx(-math.sqrt(2.0 * 1.0 * 0.01), abs=1e-6)

    # margin(0.3 m)에 닿았다 - 정지·반납·IDLE
    outgoing, cap, backing, errors = core.update(11.0, path, (4.6, 0.0))
    assert [(o.zone_id, o.request_id, o.type) for o in outgoing] == [('z1', rid, TYPE_RELEASE)]
    assert backing == pytest.approx(0.0)
    assert core.phase('z1') is Phase.IDLE

    # 그 뒤로는 평소처럼 새 request_id로 다시 접근한다
    outgoing, _, _, _ = core.update(12.0, path, (2.0, 0.0))
    assert outgoing[0].request_id == rid + 1


def test_backing_timeout_stops_with_error_and_becomes_failed():
    # #137 지수 리뷰: 안전 한도 초과 뒤에는 BACKING에 계속 머무는 대신 FAILED로 바뀐다 - 상태
    # 토픽에서 "아직 정상 후진 중"과 "한도를 넘어 멈춰 섰다"를 구분할 수 있게.
    core = make_core(backing_timeout_s=10.0, backing_max_distance_m=100.0)
    path = straight_path(2.0, 10.0)
    _enter_zone_and_revoke(core)

    core.update(7.0, path, (5.0, 0.0))  # 첫 BACKING 틱 - backing_since가 여기서 7.0으로 잡힌다
    outgoing, cap, backing, errors = core.update(17.1, path, (4.9, 0.0))  # 경과 10.1 s > 10.0 s
    assert outgoing == []  # fail-closed: 반납하지 않고 멈춰 세운다
    assert backing == pytest.approx(0.0)
    assert len(errors) == 1 and 'z1' in errors[0]
    assert core.phase('z1') is Phase.FAILED  # 멈춰 선 채 그대로 - 수동 확인 필요
    assert core.state_label(17.1) == 'failed'

    # 계속 0만 내고, ERROR는 한 번만 남긴다
    outgoing2, _, backing2, errors2 = core.update(18.0, path, (4.9, 0.0))
    assert outgoing2 == [] and backing2 == pytest.approx(0.0) and errors2 == []
    assert core.phase('z1') is Phase.FAILED


def test_backing_max_distance_stops_with_error_and_becomes_failed():
    core = make_core(backing_max_distance_m=1.0, backing_timeout_s=1000.0)
    path = straight_path(2.0, 10.0)
    _enter_zone_and_revoke(core)

    core.update(7.0, path, (5.0, 0.0))  # backing_start_xy = (5.0, 0.0)
    outgoing, _, backing, errors = core.update(8.0, path, (3.9, 0.0))  # 1.1 m 이동 > 1.0 m
    assert outgoing == []
    assert backing == pytest.approx(0.0)
    assert len(errors) == 1 and 'z1' in errors[0]
    assert core.phase('z1') is Phase.FAILED
    assert core.state_label(8.0) == 'failed'


def test_failed_outranks_inside_in_state_label():
    # #137 지수 리뷰: 우선순위 failed > backing > blocked > waiting > inside > approaching > idle -
    # 구역 z2가 failed면 z1이 inside여도 전체 상태는 failed다. z1·z2를 같은 위치에 둬 로봇 위치를
    # 한 번도 안 바꾸고 "z1 inside 유지 + z2만 별도로 전개"를 볼 수 있게 했다 - state_label
    # 우선순위만 보는 순수 상태 기계 테스트라 실제로 두 구역이 겹칠 필요는 없다.
    core = make_core(zones=[ZoneConfig('z1', RECT), ZoneConfig('z2', RECT)],
                      backing_timeout_s=10.0, backing_max_distance_m=1000.0)
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    assert {o.zone_id for o in outgoing} == {'z1', 'z2'}
    rid1 = next(o.request_id for o in outgoing if o.zone_id == 'z1')
    rid2 = next(o.request_id for o in outgoing if o.zone_id == 'z2')
    core.on_grant(0.0, 'amr_01', 'z1', rid1, GRANT_GRANTED, 100.0, 0, 'amr_01')
    core.on_grant(0.0, 'amr_01', 'z2', rid2, GRANT_GRANTED, 100.0, 0, 'amr_01')

    core.update(5.0, path, (5.0, 0.0))
    assert core.phase('z1') is Phase.INSIDE
    assert core.phase('z2') is Phase.INSIDE

    core.on_grant(5.0, 'amr_01', 'z2', rid2, GRANT_REVOKED, None, 3, '')
    assert core.phase('z2') is Phase.BACKING

    # z2만 BACKING → FAILED로 가는 동안 로봇 위치는 그대로다(z1은 계속 안에 있다)
    core.update(6.0, path, (5.0, 0.0))  # 첫 BACKING 틱(z2) - backing_since=6.0
    core.update(17.0, path, (5.0, 0.0))  # 경과 11.0 s > backing_timeout_s(10.0)
    assert core.phase('z2') is Phase.FAILED
    assert core.phase('z1') is Phase.INSIDE  # z1은 그대로 inside

    assert core.state_label(17.0) == 'failed'


def test_waiting_outranks_inside_in_state_label():
    # #137 지수 리뷰: 우선순위에서 waiting이 inside보다 먼저다 - 구역 A가 inside여도 구역 B가
    # waiting이면 전체 상태는 waiting이다. (A·B를 같은 위치에 둬 로봇을 한 번만 옮기면 둘 다 동시에
    # "안"에 있게 했다 - A는 허가를 받아 inside로, B는 허가 없이 둬 remaining=0 ≤ 임계값이라
    # waiting이 된다.)
    core = make_core(zones=[ZoneConfig('A', RECT), ZoneConfig('B', RECT)])
    path = straight_path(2.0, 10.0)

    outgoing, _, _, _ = core.update(0.0, path, (2.0, 0.0))
    rid_a = next(o.request_id for o in outgoing if o.zone_id == 'A')
    core.on_grant(0.0, 'amr_01', 'A', rid_a, GRANT_GRANTED, 100.0, 0, 'amr_01')
    # B는 허가를 안 줘서 APPROACHING에 그대로 남는다

    core.update(5.0, path, (5.0, 0.0))
    assert core.phase('A') is Phase.INSIDE
    assert core.phase('B') is Phase.APPROACHING

    assert core.state_label(5.0) == 'waiting'


# --- state_label (zone_client/state 토픽, 10/11 결정) ---


def test_state_label_idle_initially():
    core = make_core()
    assert core.state_label(0.0) == 'idle'


def test_state_label_approaching_right_after_trigger_far_from_stop_line():
    # #137 지수 리뷰: 3m 앞 요청 순간은 아직 멀어서(stop_margin_m + waiting_margin_m = 1.0 m보다
    # 먼 3.0 m) waiting이 아니라 approaching이다 - 속도 상한만 걸릴 뿐 아직 멈춰 기다리는 중이 아니다.
    core = make_core()
    core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    assert core.state_label(0.0) == 'approaching'


def test_state_label_waiting_near_stop_line_without_valid_grant():
    # 경계(x=5.0)까지 0.9 m(= stop_margin_m 0.5 + waiting_margin_m 0.5 = 1.0 m 이내) 남았고
    # 아직 응답이 없다 - 이제서야 waiting이다.
    core = make_core()
    core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    core.update(1.0, straight_path(4.1, 10.0), (4.1, 0.0))
    assert core.state_label(1.0) == 'waiting'


def test_state_label_approaching_once_granted_but_not_yet_inside():
    core = make_core()
    outgoing, _, _, _ = core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_01')
    assert core.state_label(0.0) == 'approaching'


def test_state_label_blocked():
    core = make_core()
    outgoing, _, _, _ = core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_REJECTED, None, 1, '')
    assert core.state_label(0.0) == 'blocked'


def test_state_label_inside():
    core = make_core()
    outgoing, _, _, _ = core.update(0.0, straight_path(2.0, 10.0), (2.0, 0.0))
    rid = outgoing[0].request_id
    core.on_grant(0.0, 'amr_01', 'z1', rid, GRANT_GRANTED, 100.0, 0, 'amr_01')
    core.update(5.0, straight_path(5.0, 10.0), (5.0, 0.0))
    assert core.state_label(5.0) == 'inside'


def test_state_label_backing():
    core = make_core()
    _enter_zone_and_revoke(core)
    assert core.state_label(6.0) == 'backing'
