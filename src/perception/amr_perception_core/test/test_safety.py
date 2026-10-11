import math

import pytest

from amr_perception_core.safety import (
    LEVEL_CRITICAL,
    LEVEL_ESTOP,
    LEVEL_NORMAL,
    LEVEL_WARNING,
    ObstacleInfo,
    REASON_COLLISION_RISK,
    REASON_ESTOP_REQUEST,
    REASON_NONE,
    REASON_OBSTACLE_PROXIMITY,
    REASON_SENSOR_FAULT,
    SafetyConfig,
    SafetyMonitor,
)

FRESH = dict(scan_age=0.05, tracks_age=0.05, odom_age=0.05)


def _update(monitor, now=0.0, scan=5.0, obstacles=(), path=None, **ages):
    return monitor.update(now, scan, list(obstacles), **{**FRESH, **ages}, path_clearance=path)


def _obs(track_id=1, clearance=5.0, ttc=math.inf, dynamic=True):
    return ObstacleInfo(track_id, clearance, ttc, dynamic)


def test_normal_when_clear():
    state = _update(SafetyMonitor())
    assert (state.level, state.reason, state.speed_limit) == (LEVEL_NORMAL, REASON_NONE, math.inf)
    assert state.related_track_id == -1
    assert state.min_obstacle_distance == pytest.approx(5.0)


def test_obstacle_in_travel_path_stops():
    state = _update(SafetyMonitor(), scan=0.2, path=0.2)
    assert (state.level, state.reason, state.speed_limit) == (
        LEVEL_CRITICAL, REASON_OBSTACLE_PROXIMITY, 0.0)


def test_close_point_off_path_only_slows():
    # 벽 앞에 서서 후진하는 경우·좁은 통로: 0.3 m 안이지만 진행 경로가 아니면 저속
    state = _update(SafetyMonitor(), scan=0.2, path=None)
    assert (state.level, state.reason, state.speed_limit) == (
        LEVEL_WARNING, REASON_OBSTACLE_PROXIMITY, pytest.approx(0.6))
    assert _update(SafetyMonitor(), scan=0.31).level == LEVEL_NORMAL


@pytest.mark.parametrize('ttc, level', [(0.9, LEVEL_CRITICAL), (1.0, LEVEL_CRITICAL),
                                        (1.5, LEVEL_WARNING), (2.0, LEVEL_WARNING),
                                        (2.1, LEVEL_NORMAL), (math.inf, LEVEL_NORMAL)])
def test_ttc_thresholds(ttc, level):
    state = _update(SafetyMonitor(), obstacles=[_obs(track_id=7, ttc=ttc)])
    assert state.level == level
    if level != LEVEL_NORMAL:
        assert (state.reason, state.related_track_id) == (REASON_COLLISION_RISK, 7)


def test_warning_speed_limit_is_0_6():
    assert _update(SafetyMonitor(), obstacles=[_obs(ttc=2.0)]).speed_limit == pytest.approx(0.6)


def test_ttc_ignored_for_static_tracks():
    # 정적 장애물(벽·랙)로 다가가는 것은 진행 경로 판정이 맡는다
    state = _update(SafetyMonitor(), obstacles=[_obs(ttc=0.5, dynamic=False)])
    assert state.level == LEVEL_NORMAL


def test_proximity_warning_only_for_dynamic_tracks():
    near_static = _update(SafetyMonitor(), obstacles=[_obs(clearance=0.5, dynamic=False)])
    near_dynamic = _update(SafetyMonitor(),
                           obstacles=[_obs(track_id=3, clearance=1.5, dynamic=True)])
    far_dynamic = _update(SafetyMonitor(), obstacles=[_obs(clearance=1.6, dynamic=True)])
    assert near_static.level == LEVEL_NORMAL
    assert (near_dynamic.level, near_dynamic.reason, near_dynamic.related_track_id) == (
        LEVEL_WARNING, REASON_OBSTACLE_PROXIMITY, 3)
    assert far_dynamic.level == LEVEL_NORMAL


def test_worst_obstacle_wins():
    state = _update(SafetyMonitor(), obstacles=[_obs(1, ttc=2.0), _obs(2, ttc=0.5)])
    assert (state.level, state.related_track_id) == (LEVEL_CRITICAL, 2)


def test_scan_timeout_stops_and_never_received_stops():
    stale = _update(SafetyMonitor(), scan_age=0.6)
    never = _update(SafetyMonitor(), scan_age=None)
    for state in (stale, never):
        assert (state.level, state.reason, state.speed_limit) == (
            LEVEL_CRITICAL, REASON_SENSOR_FAULT, 0.0)


def test_stale_scan_value_is_ignored():
    # 끊긴 스캔의 마지막 값으로 근접 판정하지 않는다 (원인은 센서 고장)
    assert _update(SafetyMonitor(), scan=0.1, scan_age=1.0).reason == REASON_SENSOR_FAULT


@pytest.mark.parametrize('which', ['tracks_age', 'odom_age'])
def test_tracks_or_odom_timeout_slows(which):
    state = _update(SafetyMonitor(), **{which: 1.5})
    assert (state.level, state.reason, state.speed_limit) == (
        LEVEL_WARNING, REASON_SENSOR_FAULT, pytest.approx(0.6))


def test_sensor_fault_noted_in_detail_when_other_reason_wins():
    state = _update(SafetyMonitor(), scan=0.2, path=0.2, odom_age=2.0)
    assert state.reason == REASON_OBSTACLE_PROXIMITY
    assert 'odom 끊김' in state.detail


def test_estop_latches_until_released():
    monitor = SafetyMonitor()
    monitor.set_estop(True)
    for t in (0.0, 1.0, 10.0):
        state = _update(monitor, now=t)
        assert (state.level, state.reason, state.speed_limit) == (
            LEVEL_ESTOP, REASON_ESTOP_REQUEST, 0.0)
    monitor.set_estop(False)
    # 해제 뒤에도 바로 NORMAL이 아니라 release_hold_s를 지켜 내려온다
    assert _update(monitor, now=10.1).level == LEVEL_ESTOP
    assert _update(monitor, now=10.6).level == LEVEL_NORMAL


def test_estop_outranks_critical():
    monitor = SafetyMonitor()
    monitor.set_estop(True)
    assert _update(monitor, scan=0.1, scan_age=None).level == LEVEL_ESTOP


def test_rises_immediately_and_falls_after_hold():
    monitor = SafetyMonitor()
    assert _update(monitor, now=0.0).level == LEVEL_NORMAL
    assert _update(monitor, now=0.1, obstacles=[_obs(ttc=0.5)]).level == LEVEL_CRITICAL  # 즉시
    assert _update(monitor, now=0.2).level == LEVEL_CRITICAL               # 풀린 지 0 s
    assert _update(monitor, now=0.6).level == LEVEL_CRITICAL               # 0.4 s
    assert _update(monitor, now=0.7).level == LEVEL_NORMAL                 # 0.5 s


def test_fall_timer_restarts_if_condition_returns():
    monitor = SafetyMonitor()
    _update(monitor, now=0.0, obstacles=[_obs(ttc=0.5)])
    _update(monitor, now=0.1)                                             # 풀리기 시작
    _update(monitor, now=0.3, obstacles=[_obs(ttc=0.5)])                  # 다시 위험
    assert _update(monitor, now=0.4).level == LEVEL_CRITICAL
    assert _update(monitor, now=0.8).level == LEVEL_CRITICAL              # 0.4 s만 풀림
    assert _update(monitor, now=0.9).level == LEVEL_NORMAL


def test_critical_to_warning_also_waits():
    monitor = SafetyMonitor()
    _update(monitor, now=0.0, obstacles=[_obs(ttc=0.5)])
    assert _update(monitor, now=0.1, obstacles=[_obs(ttc=2.0)]).level == LEVEL_CRITICAL
    state = _update(monitor, now=0.6, obstacles=[_obs(ttc=2.0)])
    assert (state.level, state.speed_limit) == (LEVEL_WARNING, pytest.approx(0.6))


def test_held_state_reports_current_distance():
    monitor = SafetyMonitor()
    _update(monitor, now=0.0, obstacles=[_obs(ttc=0.5)])
    assert _update(monitor, now=0.1, scan=2.0).min_obstacle_distance == pytest.approx(2.0)


def test_path_stop_releases_immediately_when_command_moves_away():
    # 벽 앞 정지 → 후진 명령: 진행 경로 위험이 사라지면 기다리지 않고 바로 내린다 (교착·지연 없음)
    monitor = SafetyMonitor()
    assert _update(monitor, now=0.0, scan=0.2, path=0.2).level == LEVEL_CRITICAL
    state = _update(monitor, now=0.1, scan=0.2, path=None)
    assert (state.level, state.speed_limit) == (LEVEL_WARNING, pytest.approx(0.6))
    # 다시 다가가는 명령이면 즉시 정지
    assert _update(monitor, now=0.2, scan=0.2, path=0.2).level == LEVEL_CRITICAL


def test_ttc_stop_still_waits_for_hold():
    monitor = SafetyMonitor()
    _update(monitor, now=0.0, obstacles=[_obs(ttc=0.5)])
    assert _update(monitor, now=0.1).level == LEVEL_CRITICAL


def test_min_distance_uses_tracks_and_scan():
    state = _update(SafetyMonitor(), scan=3.0, obstacles=[_obs(clearance=1.2)])
    assert state.min_obstacle_distance == pytest.approx(1.2)
    assert _update(SafetyMonitor(), scan=None).min_obstacle_distance == math.inf


@pytest.mark.parametrize('kwargs', [dict(critical_distance=0.0), dict(ttc_warning=math.inf),
                                    dict(ttc_critical=4.0), dict(release_hold_s=-1.0)])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)
