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


def test_reset_clears_tracks_and_keeps_counting_ids():
    tracker = MultiTargetTracker()
    confirm_static(tracker, frames=3)
    tracker.reset()
    assert tracker.tracks == [] and tracker.last_time is None
    tracker.step(0.0, [(1.0, 1.0)])                   # 시각이 처음으로 돌아가도 처리된다
    assert [track.track_id for track in tracker.tracks] == [1]
