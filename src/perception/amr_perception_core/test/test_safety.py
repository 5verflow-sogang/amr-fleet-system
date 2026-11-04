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


def test_close_dynamic_track_slows_further_whatever_its_direction():
    # I17 시드 1019·1029: 바로 옆에서 멀어지던 사람(TTC 무한)이 돌아서 출발한 로봇에 부딪혔다
    close = _update(SafetyMonitor(), obstacles=[_obs(track_id=4, clearance=0.5)])
    assert (close.level, close.reason, close.related_track_id) == (
        LEVEL_WARNING, REASON_OBSTACLE_PROXIMITY, 4)
    assert close.speed_limit == pytest.approx(0.15)
    assert '저속' in close.detail
    assert _update(SafetyMonitor(), obstacles=[_obs(clearance=0.51)]).speed_limit == pytest.approx(0.6)
    # 정적 트랙은 그대로 (진행 경로 판정이 맡는다)
    assert _update(SafetyMonitor(), obstacles=[_obs(clearance=0.2, dynamic=False)]).level == LEVEL_NORMAL
    # 거리 0이면 끈다
    off = SafetyMonitor(SafetyConfig(dynamic_slow_distance=0.0))
    assert _update(off, obstacles=[_obs(clearance=0.0)]).speed_limit == pytest.approx(0.6)


def test_close_dynamic_slow_skips_track_receding_behind_only_when_enabled():
    # 정면으로 마주 오던 사람을 비켜 지나친 뒤: 차체 뒤에서 멀어지는 중이면 근접 저속을 풀 수 있다
    receding = ObstacleInfo(7, 0.3, math.inf, True, receding_behind=True)
    on = SafetyMonitor(SafetyConfig(dynamic_slow_skip_receding_behind=True))
    state = _update(on, obstacles=[receding])
    assert state.level == LEVEL_WARNING and state.speed_limit == pytest.approx(0.6)
    assert '저속' not in state.detail
    # 기본값(끔)이면 그대로 저속
    assert _update(SafetyMonitor(), obstacles=[receding]).speed_limit == pytest.approx(0.15)
    # 켜도 뒤에서 멀어지는 트랙이 아니면(옆·앞, 돌아서는 사람) 저속
    beside = ObstacleInfo(7, 0.3, math.inf, True, receding_behind=False)
    assert _update(on, obstacles=[beside]).speed_limit == pytest.approx(0.15)
    # 뒤에서 멀어져도 TTC가 짧으면 정지 판정은 그대로
    assert _update(on, obstacles=[ObstacleInfo(7, 0.3, 0.5, True, receding_behind=True)]).level == LEVEL_CRITICAL


def test_close_fast_unconfirmed_track_slows_only_when_enabled():
    # 돌아선 작업자가 새 트랙으로 잡혀 아직 정적(동적 확정 전)인데 1.2 m/s로 다가온다
    fast = ObstacleInfo(9, 0.3, math.inf, False, speed=1.2, radius=0.15)
    def on():
        return SafetyMonitor(SafetyConfig(dynamic_slow_unconfirmed_speed=0.5))
    state = _update(on(), obstacles=[fast])
    assert (state.level, state.speed_limit, state.related_track_id) == (LEVEL_WARNING, pytest.approx(0.15), 9)
    assert '확정 전' in state.detail
    # 기본값(끔), 느린 정적 트랙(기둥·벽 조각), 0.5 m 밖은 그대로
    assert _update(SafetyMonitor(), obstacles=[fast]).level == LEVEL_NORMAL
    assert _update(on(), obstacles=[ObstacleInfo(9, 0.3, math.inf, False, speed=0.2, radius=0.15)]).level == LEVEL_NORMAL
    assert _update(on(), obstacles=[ObstacleInfo(9, 0.6, math.inf, False, speed=1.2, radius=0.15)]).level == LEVEL_NORMAL
    # 벽·랙 조각처럼 크거나(반경 0.6 m) 너무 빠르면(2.4 m/s) 빼다 (로봇이 움직이면 옆 조각이 빠르게 보인다)
    assert _update(on(), obstacles=[ObstacleInfo(9, 0.3, math.inf, False, speed=1.4, radius=0.6)]).level == LEVEL_NORMAL
    assert _update(on(), obstacles=[ObstacleInfo(9, 0.3, math.inf, False, speed=2.4, radius=0.2)]).level == LEVEL_NORMAL
    # TTC 정지는 동적 트랙만 (정적 트랙의 TTC는 쓰지 않는다)
    assert _update(on(), obstacles=[ObstacleInfo(9, 0.3, 0.2, False, speed=1.2, radius=0.15)]).level == LEVEL_WARNING


def test_close_dynamic_slow_is_reported_over_other_warnings():
    # 진행 경로 밖 LiDAR 근접(0.6)·TTC 경고와 같이 걸려도 상한은 더 낮은 쪽, 설명도 그 원인
    state = _update(SafetyMonitor(), scan=0.2, obstacles=[_obs(track_id=9, clearance=0.2, ttc=1.5)])
    assert (state.level, state.speed_limit, state.related_track_id) == (
        LEVEL_WARNING, pytest.approx(0.15), 9)
    assert '저속' in state.detail


def test_close_dynamic_slow_applies_at_once_and_lifts_after_hold():
    monitor = SafetyMonitor()
    assert _update(monitor, now=0.0, obstacles=[_obs(clearance=1.0)]).speed_limit == pytest.approx(0.6)
    # 같은 WARNING 안에서 더 낮아지는 것은 바로
    assert _update(monitor, now=0.1, obstacles=[_obs(clearance=0.4)]).speed_limit == pytest.approx(0.15)
    # 높아지는 것은 release_hold_s 동안 풀린 채여야
    assert _update(monitor, now=0.2, obstacles=[_obs(clearance=1.0)]).speed_limit == pytest.approx(0.15)
    assert _update(monitor, now=0.6, obstacles=[_obs(clearance=1.0)]).speed_limit == pytest.approx(0.15)
    state = _update(monitor, now=0.7, obstacles=[_obs(clearance=1.0)])
    assert (state.level, state.speed_limit) == (LEVEL_WARNING, pytest.approx(0.6))
    # 정지로는 바로, 정지에서 근접 저속으로는 기다린다
    assert _update(monitor, now=0.8, obstacles=[_obs(clearance=0.4, ttc=0.5)]).level == LEVEL_CRITICAL
    assert _update(monitor, now=0.9, obstacles=[_obs(clearance=0.4)]).level == LEVEL_CRITICAL
    state = _update(monitor, now=1.4, obstacles=[_obs(clearance=0.4)])
    assert (state.level, state.speed_limit) == (LEVEL_WARNING, pytest.approx(0.15))


def test_close_dynamic_slow_hold_does_not_shorten_warning_hold():
    # 근접 저속 유지는 따로 센다: WARNING → NORMAL은 모든 경고가 풀린 때부터 release_hold_s
    monitor = SafetyMonitor()
    _update(monitor, now=0.0, obstacles=[_obs(clearance=0.4)])
    _update(monitor, now=0.1, obstacles=[_obs(clearance=1.0)])            # 저속만 풀림
    assert _update(monitor, now=0.3).level == LEVEL_WARNING              # 경고도 풀림
    assert _update(monitor, now=0.7).level == LEVEL_WARNING              # 0.4 s
    assert _update(monitor, now=0.8).level == LEVEL_NORMAL               # 0.5 s


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
                                    dict(ttc_critical=4.0), dict(release_hold_s=-1.0),
                                    dict(dynamic_slow_distance=-0.1), dict(dynamic_slow_distance=2.0),
                                    dict(dynamic_slow_speed_limit=0.0),
                                    dict(dynamic_slow_speed_limit=0.7)])
def test_config_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)
