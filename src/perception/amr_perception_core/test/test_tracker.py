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
