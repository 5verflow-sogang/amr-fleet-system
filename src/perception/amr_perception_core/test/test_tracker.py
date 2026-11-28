import numpy as np
import pytest

from amr_perception_core.kalman import KalmanConfig
from amr_perception_core.tracker import MultiTargetTracker, TrackerConfig

SMOOTH = TrackerConfig(kalman=KalmanConfig(accel_noise_std=0.3))


def confirm_static(tracker, position=(0.0, 0.0), frames=3, dt=0.1):
    for k in range(frames):
        tracker.step(k * dt, [position])


def test_two_targets_get_separate_tracks_and_velocities():
    rng = np.random.default_rng(0)
    tracker = MultiTargetTracker(SMOOTH)
    for k in range(30):
        t = k * 0.1
        a = np.array([1.0 * t, 0.0])
        b = np.array([10.0, 0.5 * t])
        tracks = tracker.step(t, [a + rng.normal(0, 0.03, 2), b + rng.normal(0, 0.03, 2)])
    assert sorted(track.track_id for track in tracks) == [0, 1]
    first, second = sorted(tracks, key=lambda track: track.kf.position[0])
    np.testing.assert_allclose(first.kf.velocity, [1.0, 0.0], atol=0.15)
    np.testing.assert_allclose(second.kf.velocity, [0.0, 0.5], atol=0.15)


def test_track_confirmed_after_confirm_hits():
    tracker = MultiTargetTracker(TrackerConfig(confirm_hits=3))
    assert tracker.step(0.0, [(0.0, 0.0)]) == []
    assert tracker.step(0.1, [(0.0, 0.0)]) == []
    assert len(tracker.step(0.2, [(0.0, 0.0)])) == 1


def test_lost_track_deleted_and_id_not_reused():
    tracker = MultiTargetTracker(TrackerConfig(max_coast_time=0.5))
    confirm_static(tracker)                 # 마지막 관측 0.2 s
    tracker.step(0.6, [])                   # 0.4 s 경과 → 유지 (예측만)
    assert len(tracker.tracks) == 1
    tracker.step(0.8, [])                   # 0.6 s 경과 → 삭제
    assert tracker.tracks == []
    tracker.step(0.9, [(5.0, 5.0)])
    assert [track.track_id for track in tracker.tracks] == [1]


def test_far_measurement_starts_new_track_instead_of_jumping():
    tracker = MultiTargetTracker()
    confirm_static(tracker)
    tracker.step(0.3, [(5.0, 0.0)])
    by_id = {track.track_id: track for track in tracker.tracks}
    assert set(by_id) == {0, 1}
    assert np.linalg.norm(by_id[0].kf.position) < 0.5


def test_each_measurement_and_track_used_at_most_once():
    tracker = MultiTargetTracker()
    confirm_static(tracker)
    tracker.step(0.3, [(0.02, 0.0), (0.05, 0.0)])  # 둘 다 게이트 안이지만 트랙은 하나
    assert len(tracker.tracks) == 2


def test_empty_frame_is_allowed():
    assert MultiTargetTracker().step(0.0, []) == []


@pytest.mark.parametrize('bad', [(np.nan, 0.0), (1.0, 2.0, 3.0)])
def test_rejects_invalid_measurement(bad):
    with pytest.raises(ValueError):
        MultiTargetTracker().step(0.0, [bad])


def test_rejects_time_going_backwards():
    tracker = MultiTargetTracker()
    tracker.step(1.0, [])
    with pytest.raises(ValueError):
        tracker.step(0.5, [])


@pytest.mark.parametrize('kwargs', [
    {'gate_mahalanobis_sq': 0.0},
    {'confirm_hits': 0},
    {'confirm_hits': 1.5},
    {'max_coast_time': -1.0},
    {'dynamic_speed_threshold': 0.0},
    {'dynamic_confirm_frames': 0},
    {'maneuver_max_speed': -1.0},
    {'maneuver_margin': float('nan')},
    {'maneuver_max_age': 0.0},
    {'maneuver_max_radius': 0.0},
    {'dynamic_min_displacement': -0.1},
    {'dynamic_evidence_window': 1.5},
    {'dynamic_evidence_window': 2},                          # 증거 3프레임을 모을 수 없다
    {'dynamic_release_frames': -1},
    {'dynamic_release_frames': 30, 'dynamic_pause_frames': 10},
    {'dynamic_release_speed': float('nan')},
])
def test_rejects_invalid_config(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)


def test_confirmed_track_takes_measurement_before_tentative_track():
    # 게이트 밖 측정이 만든 미확정 트랙이 다음 프레임에 기존 트랙의 측정을 가로채지 않는다
    tracker = MultiTargetTracker()
    confirm_static(tracker)
    tracker.step(0.3, [(0.0, 0.0), (0.3, 0.0)])     # (0.3, 0)은 게이트 밖 → 미확정 트랙 1
    tracker.step(0.4, [(0.15, 0.0)])                 # 두 트랙 사이의 측정 하나
    by_id = {track.track_id: track for track in tracker.tracks}
    assert by_id[0].last_update == 0.4


def test_static_obstacle_is_not_dynamic():
    rng = np.random.default_rng(5)
    tracker = MultiTargetTracker()
    for k in range(300):
        tracks = tracker.step(k * 0.1, [rng.normal(0.0, 0.05, 2)])
    assert [track.is_dynamic for track in tracks] == [False]


def test_moving_obstacle_becomes_dynamic_and_stays_after_stopping():
    rng = np.random.default_rng(6)
    tracker = MultiTargetTracker()
    position = np.zeros(2)
    for k in range(20):                               # 2 s 동안 1.0 m/s
        position = np.array([0.1 * k, 0.0])
        tracks = tracker.step(k * 0.1, [position + rng.normal(0.0, 0.05, 2)])
    assert tracks[0].is_dynamic
    for k in range(20, 60):                           # 멈춘 뒤에도 동적 유지
        tracks = tracker.step(k * 0.1, [position + rng.normal(0.0, 0.05, 2)])
    assert tracks[0].is_dynamic


def _slide(tracker, evidence, frames=20):
    """1.0 m/s로 미끄러지는 측정. evidence: 프레임마다 증거 여부 (함수)."""
    tracks = []
    for k in range(frames):
        tracks = tracker.step(k * 0.1, [(0.1 * k, 0.0)], free_evidence=[evidence(k)])
    return tracks


def test_fast_track_without_free_space_evidence_stays_static():
    # 벽을 따라 미끄러지는 조각: 속력은 빠르지만 빈 칸에 들어간 적이 없다
    assert [t.is_dynamic for t in _slide(MultiTargetTracker(), lambda k: False)] == [False]


def test_fast_track_with_evidence_becomes_dynamic():
    assert [t.is_dynamic for t in _slide(MultiTargetTracker(), lambda k: True)] == [True]


def test_single_evidence_frame_is_not_enough():
    tracker = MultiTargetTracker(TrackerConfig(dynamic_evidence_frames=2))
    assert [t.is_dynamic for t in _slide(tracker, lambda k: k == 3)] == [False]
    tracker = MultiTargetTracker(TrackerConfig(dynamic_evidence_frames=2))
    assert [t.is_dynamic for t in _slide(tracker, lambda k: k in (3, 9))] == [True]


def test_evidence_without_speed_is_not_dynamic():
    tracker = MultiTargetTracker()
    for k in range(20):
        tracks = tracker.step(k * 0.1, [(0.0, 0.0)], free_evidence=[True])
    assert [t.is_dynamic for t in tracks] == [False]


def test_free_evidence_length_must_match():
    with pytest.raises(ValueError):
        MultiTargetTracker().step(0.0, [(0.0, 0.0)], free_evidence=[True, False])


def test_reset_clears_tracks_and_keeps_counting_ids():
    tracker = MultiTargetTracker()
    confirm_static(tracker, frames=3)
    tracker.reset()
    assert tracker.tracks == [] and tracker.last_time is None
    tracker.step(0.0, [(1.0, 1.0)])                   # 시각이 처음으로 돌아가도 처리된다
    assert [track.track_id for track in tracker.tracks] == [1]


# --- 반경 ---
def test_radius_is_smoothed_from_measurements():
    tracker = MultiTargetTracker(TrackerConfig(radius_smoothing=0.5))
    tracker.step(0.0, [(0.0, 0.0)], [0.2])
    tracker.step(0.1, [(0.0, 0.0)], [0.4])
    tracks = tracker.step(0.2, [(0.0, 0.0)], [0.4])
    assert tracks[0].radius == pytest.approx(0.35)   # 0.2 → 0.3 → 0.35


def test_radius_is_none_without_radii():
    tracker = MultiTargetTracker(TrackerConfig())
    confirm_static(tracker)
    assert tracker.confirmed_tracks[0].radius is None


@pytest.mark.parametrize('radii', [[0.2, 0.3], [0.0], [float('nan')], [-0.1]])
def test_rejects_bad_radii(radii):
    with pytest.raises(ValueError):
        MultiTargetTracker(TrackerConfig()).step(0.0, [(0.0, 0.0)], radii)


@pytest.mark.parametrize('value', [0.0, 1.5])
def test_rejects_bad_radius_smoothing(value):
    with pytest.raises(ValueError):
        TrackerConfig(radius_smoothing=value)


MANEUVER = TrackerConfig(maneuver_max_speed=1.5, max_coast_time=2.0)   # tracking.yaml 값


def _reverse(tracker, speed, seed=7, frames_after=10, dt=0.1):
    """x = 8 m에서 -x로 3 s 걷다가 제자리에서 돌아서는 사람 (측정 잡음 0.05 m). 반전 뒤 프레임마다 (진짜 x, 확정 트랙)."""
    rng = np.random.default_rng(seed)
    turn = 8.0 - speed * 30 * dt
    after = []
    for k in range(31 + frames_after):
        x = 8.0 - speed * k * dt if k <= 30 else turn + speed * (k - 30) * dt
        tracks = tracker.step(k * dt, [(x + rng.normal(0.0, 0.05), rng.normal(0.0, 0.05))])
        if k > 30:
            after.append((x, tracks))
    return after


def _walk(tracker, frames=20, dt=0.1):
    """원점에서 +x로 1.0 m/s 걷는 동적 트랙 (잡음 없음). 마지막 관측은 ((frames-1)·0.1, 0), 시각 (frames-1)·dt."""
    for k in range(frames):
        tracker.step(k * dt, [(0.1 * k, 0.0)])
    assert tracker.tracks[0].is_dynamic


@pytest.mark.parametrize('speed', [1.0, 1.25, 1.5])
def test_reversal_keeps_one_track_and_follows_new_direction(speed):
    # 등속 예측은 1.0 m/s 이상으로 걷다 돌아서면 게이트를 벗어나 헛트랙이 남았다 (perception.md 1-7 P2 분석)
    tracker = MultiTargetTracker(MANEUVER)
    after = _reverse(tracker, speed)
    # 확정 트랙은 하나뿐이다. 잡음으로 갈 수 있는 거리를 조금 넘은 측정 하나는 미확정 트랙으로 남을 수 있다(밖으로 안 나감)
    assert {track.track_id for _, tracks in after for track in tracks} == {0}
    x, tracks = after[-1]                             # 돌아선 뒤 1 s
    np.testing.assert_allclose(tracks[0].kf.velocity, [speed, 0.0], atol=0.3)
    assert abs(tracks[0].kf.position[0] - x) < 0.15


def test_reversal_without_maneuver_association_leaves_ghost_track():
    # 끄면(코어 기본) 예전 동작: 새 트랙이 생기고 옛 트랙은 계속 -x로 미끄러진다
    tracker = MultiTargetTracker()
    x, tracks = _reverse(tracker, 1.5)[4]             # 돌아선 뒤 0.5 s
    by_id = {track.track_id: track for track in tracks}
    assert set(by_id) == {0, 1}
    assert by_id[0].kf.velocity[0] < 0.0
    assert abs(by_id[0].kf.position[0] - x) > 0.5


def test_maneuver_association_skips_static_track():
    # 정적 물체의 군집 중심이 튀면 새 트랙으로 둔다. 붙이면 0.35 m / 0.2 s = 1.75 m/s 속도가 생긴다
    for dynamic, expected_tracks in ((False, 2), (True, 1)):
        tracker = MultiTargetTracker(MANEUVER)
        confirm_static(tracker, frames=30)
        tracker.tracks[0].is_dynamic = dynamic        # 대조: 동적이었다면 붙는다
        tracker.step(3.1, [(0.35, 0.0)])              # 한 프레임 건너뜀. 게이트 밖(d² 18.5), 갈 수 있는 거리(0.45 m) 안
        assert len(tracker.tracks) == expected_tracks
    tracker = MultiTargetTracker(MANEUVER)
    confirm_static(tracker, frames=30)
    tracker.step(3.1, [(0.35, 0.0)])
    assert tracker.tracks[0].kf.speed < 0.1


def test_maneuver_association_skips_far_measurement():
    tracker = MultiTargetTracker(MANEUVER)
    _walk(tracker)
    tracker.step(2.0, [(1.9, 0.6)])                   # 마지막 관측에서 0.6 m > 1.5 × 0.1 + 0.15
    assert len(tracker.tracks) == 2
    assert tracker.tracks[0].last_update == pytest.approx(1.9)


def test_maneuver_association_skips_track_unseen_too_long():
    # 0.6 s 못 본 뒤 0.4 m 뒤로 돌아온 측정. 예측(2.5, 0)과는 1.0 m라 1차 게이트 밖
    for max_age, expected_tracks in ((0.5, 2), (1.0, 1)):
        tracker = MultiTargetTracker(TrackerConfig(maneuver_max_speed=1.5, maneuver_max_age=max_age,
                                                   max_coast_time=2.0))
        _walk(tracker)
        tracker.step(2.5, [(1.5, 0.0)])
        assert len(tracker.tracks) == expected_tracks


def test_maneuver_association_does_not_take_matched_measurement():
    # 동적 트랙 A가 사라진 프레임에, A가 갈 수 있는 거리 안의 정지 물체 B 측정은 B 몫이다
    tracker = MultiTargetTracker(MANEUVER)
    for k in range(21):
        tracker.step(k * 0.1, [(0.1 * k, 0.0), (2.0, 0.25)])
    walker, still = sorted(tracker.tracks, key=lambda track: track.kf.position[1])
    assert walker.is_dynamic and not still.is_dynamic
    tracker.step(2.1, [(2.0, 0.25), (10.0, 10.0)])   # 멀리 새 물체도 있어야 남은 측정 비교까지 간다
    assert len(tracker.tracks) == 3
    assert walker.last_update == pytest.approx(2.0)
    assert still.last_update == pytest.approx(2.1)


def test_maneuver_association_comes_before_tentative_tracks():
    # 잡음으로 생겨 1 s 예측만 한 미확정 트랙은 게이트가 넓다. 돌아선 사람의 측정을 먼저 가져가면 새 ID가 생긴다
    tracker = MultiTargetTracker(MANEUVER)
    for k in range(20):
        tracker.step(k * 0.1, [(0.1 * k, 0.0)] + ([(1.6, 0.0)] if k == 10 else []))
    walker, stale = tracker.tracks
    assert walker.is_dynamic and not stale.confirmed
    tracker.step(2.0, [(1.7, 0.0)])                   # 1.9에서 돌아섬: 걷던 트랙 게이트 밖, 미확정 트랙 게이트 안
    assert walker.last_update == pytest.approx(2.0)
    assert stale.last_update == pytest.approx(1.0)


def test_maneuver_association_restarts_velocity_from_last_two_measurements():
    tracker = MultiTargetTracker(MANEUVER)
    _walk(tracker)
    track = tracker.tracks[0]
    tracker.step(2.0, [(1.65, 0.0)])                  # 1.9 → 1.65: 게이트 밖, 0.30 m 안 → -2.5 m/s로 다시 잡는다
    assert tracker.tracks == [track]
    np.testing.assert_allclose(track.kf.position, [1.65, 0.0])
    np.testing.assert_allclose(track.kf.velocity, [-2.5, 0.0])
    assert track.confirmed and track.is_dynamic and track.hits == 21


def _walk_sized(tracker, radii, frames=20):
    """_walk와 같은 동적 트랙인데 프레임마다 측정 반경을 준다 (radii: 프레임 번호 → 반경)."""
    for k in range(frames):
        tracker.step(k * 0.1, [(0.1 * k, 0.0)], [radii(k)])
    assert tracker.tracks[0].is_dynamic


@pytest.mark.parametrize('radii, joined', [
    (lambda k: 0.15, True),                          # 사람 크기: 돌아서면 이어 붙인다
    (lambda k: 0.8, False),                          # 지게차 크기: 새 트랙으로 둔다
    (lambda k: 0.8 if k < 5 else 0.25, False),       # 한 번이라도 크게 보였으면 일부만 보여도 뺀다
])
def test_maneuver_association_skips_large_objects(radii, joined):
    # 큰 물체는 가려지거나 보이는 면이 바뀔 때 군집 중심이 튀어 기동으로 받으면 속도가 엉터리가 된다
    tracker = MultiTargetTracker(MANEUVER)
    _walk_sized(tracker, radii)
    tracker.step(2.0, [(1.65, 0.0)], [0.15])          # 1.9 → 1.65: 게이트 밖, 갈 수 있는 거리 0.30 m 안
    assert (len(tracker.tracks) == 1) == joined
    assert (tracker.tracks[0].last_update == pytest.approx(2.0)) == joined


# 동적 입구 - 실제 이동 (dynamic_min_displacement, ref_points), 증거 창, 출구 (dynamic_release_*)
ENTRY = dict(dynamic_min_displacement=0.3)
EXIT = dict(dynamic_release_frames=30, dynamic_pause_frames=100, dynamic_release_speed=0.1,
            dynamic_release_max_displacement=0.5)


def _run(tracker, frames, start=0, dt=0.1):
    """frames: 프레임마다 (군집 중심, 기준점 또는 None). 프레임마다 동적 트랙이 있는지를 돌려준다."""
    flags = []
    for k, (z, ref) in enumerate(frames, start):
        tracks = tracker.step(k * dt, [z], free_evidence=[True], ref_points=None if ref is None else [ref])
        flags.append(any(track.is_dynamic for track in tracks))
    return flags


def _first(flags, value=True, start=0):
    return next((k for k in range(start, len(flags)) if flags[k] == value), None)


def _table(slide_speed=0.5, slide_frames=8, still_frames=80, ref=(0.0, -0.5)):
    """정지 테이블 (식당 11/17): 다가가는 로봇에게 보이는 면이 바뀌어 군집 중심만 미끄러지고, 가장 가까운 점은 그대로다."""
    step = slide_speed * 0.1
    frames = [((0.0, 0.0), ref)] * 10
    frames += [((step * (i + 1), 0.0), ref) for i in range(slide_frames)]
    return frames + [((step * slide_frames, 0.0), ref)] * still_frames


def _walk_then_stand(speed=1.0, walk_frames=30, stand_frames=120):
    frames = [((speed * 0.1 * k, 0.0), (speed * 0.1 * k, -0.1)) for k in range(walk_frames)]
    end = speed * 0.1 * (walk_frames - 1)
    return frames + [((end, 0.0), (end, -0.1))] * stand_frames


def test_sliding_centroid_alone_makes_static_table_dynamic_for_good():
    # 지금 규칙의 문제: 중심이 0.8 s 미끄러진 테이블이 동적이 되고 풀리지 않는다
    flags = _run(MultiTargetTracker(), _table())
    assert _first(flags) is not None and flags[-1]


def test_displacement_measured_at_steadier_point_keeps_table_static():
    assert not any(_run(MultiTargetTracker(TrackerConfig(**ENTRY)), _table()))


def test_displacement_falls_back_to_centroid_without_ref_points():
    frames = [(z, None) for z, _ in _table()]                # 중심 0.4 m 이동 ≥ 0.3 m
    assert any(_run(MultiTargetTracker(TrackerConfig(**ENTRY)), frames))


def test_displacement_gate_does_not_delay_walking_person():
    frames = _walk_then_stand(stand_frames=0)
    assert _first(_run(MultiTargetTracker(TrackerConfig(**ENTRY)), frames)) == \
        _first(_run(MultiTargetTracker(), frames))


def test_displacement_gate_delays_slow_person_by_at_most_one_frame():
    # 0.3 m를 가는 데 0.5 m/s는 0.6 s: 속력 조건(5프레임, 0.5 s)보다 한 프레임 늦다
    frames = _walk_then_stand(speed=0.5, walk_frames=40, stand_frames=0)
    base = _first(_run(MultiTargetTracker(), frames))
    gated = _first(_run(MultiTargetTracker(TrackerConfig(**ENTRY)), frames))
    assert base is not None and base <= gated <= base + 1


def test_track_stopped_near_origin_is_released_after_release_frames():
    # 입구를 뚫은 제자리 미끄러짐 (중심만, 0.4 m)은 3 s 멈추면 정적으로 내린다
    frames = [(z, None) for z, _ in _table()]
    flags = _run(MultiTargetTracker(TrackerConfig(**EXIT)), frames)
    became = _first(flags)
    released = _first(flags, False, became)
    assert became is not None and released is not None
    assert 30 <= released - became <= 40


def test_person_who_walked_far_is_paused_then_released():
    flags = _run(MultiTargetTracker(TrackerConfig(**EXIT)), _walk_then_stand())   # 2.9 m 걷고 섬 (30프레임째)
    assert flags[30 + 50]                     # 선 지 5 s: '멈춤' (동적 표시 유지)
    released = _first(flags, False, _first(flags))
    assert released is not None and 30 + 100 <= released <= 30 + 115


def test_released_track_must_pass_entry_again():
    tracker = MultiTargetTracker(TrackerConfig(**ENTRY, **EXIT))
    flags = _run(tracker, _walk_then_stand())
    assert not flags[-1]
    end = 0.1 * 29
    # 선 자리에서 중심만 0.5 m/s로 0.8 s(0.4 m) 미끄러짐: 기준 자리를 선 자리로 옮겼으므로 다시 동적이 되지 않는다
    slide = [((end + 0.05 * min(k, 8), 0.0), (end, -0.1)) for k in range(1, 21)]
    assert not any(_run(tracker, slide, start=len(flags)))
    walk = [((end + 0.4 + 0.1 * k, 0.0), (end + 0.1 * k, -0.1)) for k in range(1, 15)]
    assert any(_run(tracker, walk, start=len(flags) + 20))     # 다시 걸으면 다시 동적


def test_slow_drift_does_not_open_entry_for_later_slide():
    # #361 리뷰: 긴 테이블 옆을 지나가면 중심·최근접 점이 같이 조금씩 밀린다 (0.04 m/s로 10 s, 0.4 m). 처음 본 자리에
    # 고정하면 그 뒤 0.6 s 미끄러짐(0.4 m/s, 0.24 m)에 입구가 열려 동적이 됐다(밀림 없이는 안 열림). 입구 기준 자리는
    # 속력 조건이 풀린 동안 따라온다
    def table(drift):
        frames = [((0.0, 0.0), (0.0, -0.5))] * 10
        frames += [((drift / 100 * k, 0.0), (drift / 100 * k, -0.5)) for k in range(1, 101)]
        frames += [((drift + 0.04 * k, 0.0), (drift + 0.04 * k, -0.5)) for k in range(1, 7)]
        return frames + [((drift + 0.24, 0.0), (drift + 0.24, -0.5))] * 30

    config = TrackerConfig(**ENTRY, **EXIT)
    assert not any(_run(MultiTargetTracker(config), table(0.0)))
    assert not any(_run(MultiTargetTracker(config), table(0.4)))


def test_person_who_walked_in_keeps_pause_with_entry_rule():
    # 입구 기준 자리가 멈출 때마다 따라와도, 해제 기준 자리는 동적이 된 순간에 고정이라 걸어와서 선 사람은
    # 3 s가 아니라 10 s '멈춤' 뒤에 풀린다
    flags = _run(MultiTargetTracker(TrackerConfig(**ENTRY, **EXIT)),
                 _walk_then_stand(walk_frames=15, stand_frames=130))                  # 1.4 m 걷고 섬 (15프레임째)
    assert _first(flags) is not None and flags[15 + 50]
    released = _first(flags, False, _first(flags))
    assert released is not None and 15 + 100 <= released <= 15 + 115


@pytest.mark.parametrize('speed', [0.4, 0.6])
def test_entry_rule_delays_noisy_slow_person_by_at_most_half_a_second(speed):
    # 속력 추정이 임계 근처에서 흔들려도(측정 잡음 0.03 m) 입구 기준 자리가 되돌아가 동적 판정이 크게 밀리지 않는다
    rng = np.random.default_rng(3)
    frames = []
    for k in range(60):
        z = (speed * 0.1 * k + rng.normal(0.0, 0.03), rng.normal(0.0, 0.03))
        frames.append((z, (z[0], z[1] - 0.1)))
    base = _first(_run(MultiTargetTracker(), frames))
    gated = _first(_run(MultiTargetTracker(TrackerConfig(**ENTRY)), frames))
    assert base is not None and gated is not None and gated - base <= 5


def _old_evidence_then_slide(tracker):
    """처음 3프레임에만 증거가 있고, 한참 뒤 증거 없이 1.0 m/s로 미끄러진다."""
    tracks = []
    for k in range(40):
        z = (0.0, 0.0) if k < 20 else (0.1 * (k - 20), 0.0)
        tracks = tracker.step(k * 0.1, [z], free_evidence=[k < 3])
    return tracks


def test_evidence_window_forgets_old_evidence():
    assert [t.is_dynamic for t in _old_evidence_then_slide(MultiTargetTracker())] == [True]
    windowed = MultiTargetTracker(TrackerConfig(dynamic_evidence_window=10))
    assert [t.is_dynamic for t in _old_evidence_then_slide(windowed)] == [False]


def test_ref_points_length_must_match():
    with pytest.raises(ValueError):
        MultiTargetTracker().step(0.0, [(0.0, 0.0)], ref_points=[])
