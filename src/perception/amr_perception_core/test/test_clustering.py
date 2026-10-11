import math

import numpy as np
import pytest

from amr_perception_core.clustering import (
    breakpoint_distance,
    cluster_scan,
    Cluster,
    ClusteringConfig,
    detect_obstacles,
    is_obstacle_candidate,
    merge_leg_pairs,
    scan_to_points,
    transform_points_2d,
)
from amr_perception_core.tracker import MultiTargetTracker, TrackerConfig

# 명세 LiDAR: 360°, 0.5° → 720 빔 (amr_description/config/sensors.yaml)
INC = math.radians(0.5)
ANGLE_MIN = -math.pi
N = 720
ANGLES = ANGLE_MIN + INC * np.arange(N)


def empty_scan():
    return np.full(N, np.inf)  # 반사 없음 (REP 117)


def add_circle(ranges, center, radius):
    """원기둥(사람·기둥) 하나를 레이캐스팅해 더 가까운 값만 남긴다."""
    cx, cy = center
    d = np.stack([np.cos(ANGLES), np.sin(ANGLES)], axis=1)
    b = d @ np.array([cx, cy])
    disc = b * b - (cx * cx + cy * cy - radius * radius)
    hit = disc >= 0.0
    t = np.where(hit, b - np.sqrt(np.where(hit, disc, 0.0)), np.inf)
    t[t <= 0.0] = np.inf
    return np.minimum(ranges, t)


def add_segment(ranges, p, q):
    """선분 벽 하나를 레이캐스팅한다."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    e = q - p
    out = ranges.copy()
    for i, a in enumerate(ANGLES):
        d = np.array([math.cos(a), math.sin(a)])
        den = d[0] * (-e[1]) - d[1] * (-e[0])
        if abs(den) < 1e-12:
            continue
        t = (p[0] * (-e[1]) - p[1] * (-e[0])) / den
        s = (d[0] * p[1] - d[1] * p[0]) / den
        if t > 0.0 and 0.0 <= s <= 1.0:
            out[i] = min(out[i], t)
    return out


def clusters_of(ranges, config=ClusteringConfig()):
    return cluster_scan(ranges, ANGLE_MIN, INC, config)


def candidates_of(ranges, config=ClusteringConfig()):
    return [c for c in clusters_of(ranges, config) if is_obstacle_candidate(c, config)]


# --- 점 변환 ---

def test_scan_to_points_marks_invalid_beams():
    points, valid = scan_to_points([1.0, np.nan, np.inf, -np.inf, 0.0, 2.0], 0.0, math.pi / 2)
    assert valid.tolist() == [True, False, False, False, False, True]
    np.testing.assert_allclose(points[0], [1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(points[5], [2.0 * math.cos(5 * math.pi / 2), 2.0], atol=1e-12)
    assert np.all(np.isnan(points[1:5]))


@pytest.mark.parametrize('args', [
    (np.zeros((2, 2)), 0.0, 0.1),
    ([1.0], 0.0, 0.0),
    ([1.0], np.nan, 0.1),
])
def test_scan_to_points_rejects_invalid_input(args):
    with pytest.raises(ValueError):
        scan_to_points(*args)


# --- breakpoint ---

def test_breakpoint_grows_with_range():
    config = ClusteringConfig()
    near = breakpoint_distance(1.0, INC, config)
    far = breakpoint_distance(10.0, INC, config)
    assert near < far
    # 10 m에서 0.5° 이웃 빔 간격(약 8.7 cm)보다 충분히 크다
    assert far > 10.0 * INC * 3


# --- 군집화 ---

def test_empty_and_all_invalid_scan_have_no_cluster():
    assert clusters_of(empty_scan()) == []
    assert clusters_of(np.full(N, np.nan)) == []


def test_person_sized_object_becomes_one_candidate():
    ranges = add_circle(empty_scan(), (3.0, 0.0), 0.25)
    clusters = clusters_of(ranges)
    assert len(clusters) == 1
    person = clusters[0]
    assert is_obstacle_candidate(person)
    # 보이는 앞면 점들의 평균이라 실제 중심(3, 0)보다 센서 쪽으로 치우친다 (반경 이내)
    assert np.linalg.norm(person.centroid - [3.0, 0.0]) < 0.25
    assert person.centroid[0] < 3.0
    assert person.extent <= 0.5 + 1e-6


def test_two_people_side_by_side_are_separate():
    ranges = add_circle(empty_scan(), (4.0, 0.6), 0.25)
    ranges = add_circle(ranges, (4.0, -0.6), 0.25)  # 표면 사이 0.7 m
    assert len(candidates_of(ranges)) == 2


def test_person_in_front_of_wall_is_separated_from_wall():
    ranges = add_segment(empty_scan(), (6.0, -5.0), (6.0, 5.0))
    ranges = add_circle(ranges, (3.0, 0.0), 0.25)
    clusters = clusters_of(ranges)
    candidates = [c for c in clusters if is_obstacle_candidate(c)]
    assert len(candidates) == 1
    np.testing.assert_allclose(candidates[0].centroid, [2.85, 0.0], atol=0.15)
    # 사람에 가려 벽이 둘로 나뉘고, 두 조각 모두 R1에서 구조물로 걸러진다
    assert sum(not is_obstacle_candidate(c) for c in clusters) == 2


def test_far_object_is_not_split():
    # 12 m 앞 0.5 m 물체: 이웃 빔 간격 약 10 cm. 고정 임계값 0.09 m(3σ)이면 쪼개진다
    ranges = add_circle(empty_scan(), (12.0, 0.0), 0.25)
    assert len(clusters_of(ranges)) == 1


def test_missing_beams_inside_object_are_bridged():
    ranges = add_circle(empty_scan(), (2.0, 0.0), 0.25)
    hit = np.flatnonzero(np.isfinite(ranges))
    ranges[hit[len(hit) // 2]] = np.nan          # 물체 가운데 빔 1개가 아웃라이어 필터로 빠짐
    ranges[hit[len(hit) // 2 + 1]] = np.nan      # 2개까지 허용
    assert len(clusters_of(ranges)) == 1
    assert len(clusters_of(ranges, ClusteringConfig(max_gap_beams=0))) == 2


def test_object_behind_robot_across_scan_boundary_is_one_cluster():
    # angle −π / +π 경계(로봇 정후방)에 걸친 물체
    ranges = add_circle(empty_scan(), (-3.0, 0.0), 0.3)
    hit = np.flatnonzero(np.isfinite(ranges))
    assert hit[0] == 0 and hit[-1] == N - 1      # 실제로 경계에 걸쳐 있는지
    clusters = clusters_of(ranges)
    assert len(clusters) == 1
    assert clusters[0].num_points == hit.size


def test_partial_scan_does_not_wrap():
    # 360°가 아닌 스캔은 양 끝을 잇지 않는다
    ranges = np.full(100, 2.0)
    clusters = cluster_scan(ranges, 0.0, INC)
    assert len(clusters) == 1
    ranges[50:52] = 10.0  # 가운데가 끊긴 두 덩어리 (두 빔: 튄 점 하나 건너뛰기로 잇지 않는다)
    assert len(cluster_scan(ranges, 0.0, INC)) == 2


def test_single_outlier_point_does_not_split():
    ranges = np.full(100, 2.0)
    ranges[50] = 2.5      # 잡음으로 한 점만 튐
    assert len(cluster_scan(ranges, 0.0, INC)) == 1
    # 끄면 둘로 끊긴다 (튄 점 하나는 min_points로 버려짐)
    off = ClusteringConfig(bridge_single_outlier=False)
    assert len(cluster_scan(ranges, 0.0, INC, off)) == 2


def _noisy_wall_clusters(config, trials=200, distance=0.6):
    rng = np.random.default_rng(1)
    counts = []
    for _ in range(trials):
        ranges = add_segment(empty_scan(), (distance, -3.0), (distance, 3.0))
        hit = np.isfinite(ranges)
        ranges[hit] += rng.normal(0.0, 0.03, hit.sum())   # sensors.yaml σ
        counts.append(len(clusters_of(ranges, config)))
    return np.array(counts)


def test_close_wall_with_lidar_noise_stays_one_cluster():
    # Gazebo에서 벽 0.6 m 앞에 서 있으면 벽이 프레임마다 다른 곳에서 조각났다 (조각이 동적 트랙이 됨)
    fragmented = _noisy_wall_clusters(ClusteringConfig(bridge_single_outlier=False))
    assert fragmented.mean() > 1.5                    # 고치기 전: 실제로 조각난다
    bridged = _noisy_wall_clusters(ClusteringConfig())
    assert np.mean(bridged == 1) >= 0.99


def test_person_close_to_wall_stays_separate_with_noise():
    # 벽 0.2 m 앞 사람 (1.5 m 거리 한 칸 한계 약 0.17 m). 튄 점 건너뛰기가 사람을 벽에 붙이면 안 된다
    rng = np.random.default_rng(2)
    for _ in range(100):
        ranges = add_segment(empty_scan(), (2.0, -1.5), (2.0, 1.5))
        ranges = add_circle(ranges, (1.65, 0.0), 0.15)
        hit = np.isfinite(ranges)
        ranges[hit] += rng.normal(0.0, 0.03, hit.sum())
        candidates = [c for c in clusters_of(ranges) if is_obstacle_candidate(c)]
        people = [c for c in candidates if abs(c.centroid[0] - 1.5) < 0.15]
        assert len(people) == 1
        assert people[0].extent < 0.4


def test_small_clusters_are_dropped_as_noise():
    ranges = empty_scan()
    ranges[100] = 5.0                            # 외톨이 점 1개
    ranges[300:302] = 5.0                        # 점 2개
    assert clusters_of(ranges) == []
    assert len(clusters_of(ranges, ClusteringConfig(min_points=1))) == 2


def test_long_wall_is_not_obstacle_candidate():
    ranges = add_segment(empty_scan(), (2.0, -3.0), (2.0, 3.0))
    clusters = clusters_of(ranges)
    assert len(clusters) == 1
    # 0.5° 빔이 벽 끝점에 정확히 맞지 않아 비스듬한 끝에서 빔 간격(약 6 cm)만큼 짧게 잡힌다
    assert clusters[0].extent == pytest.approx(6.0, abs=0.1)
    assert not is_obstacle_candidate(clusters[0])


def test_person_stays_one_cluster_with_lidar_noise():
    rng = np.random.default_rng(0)
    for _ in range(50):
        ranges = add_circle(empty_scan(), (3.0, 1.0), 0.25)
        hit = np.isfinite(ranges)
        ranges[hit] += rng.normal(0.0, 0.03, hit.sum())   # sensors.yaml σ
        assert len(clusters_of(ranges)) == 1


def test_radius_covers_visible_points():
    # 중심이 센서 쪽으로 치우쳐 있어 반경은 실제 반경(0.3 m)보다 조금 클 수 있다 (충돌 반경으로는 보수적)
    ranges = add_circle(empty_scan(), (2.0, 2.0), 0.3)
    cluster = clusters_of(ranges)[0]
    assert cluster.extent / 2.0 <= cluster.radius <= cluster.extent
    assert cluster.radius == pytest.approx(0.3, abs=0.06)


@pytest.mark.parametrize('kwargs', [
    {'breakpoint_lambda_deg': 0.0},
    {'breakpoint_lambda_deg': 90.0},
    {'range_noise_std': 0.0},
    {'max_obstacle_extent': -1.0},
    {'max_gap_beams': -1},
    {'max_gap_beams': 1.5},
    {'min_points': 0},
])
def test_rejects_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ClusteringConfig(**kwargs)


# --- 다리 묶기 ---
LEG_RADIUS = 0.06   # 다리 하나 (지름 약 12 cm)


def obstacles_of(ranges, config=ClusteringConfig()):
    return detect_obstacles(ranges, ANGLE_MIN, INC, config)


def test_two_legs_become_one_obstacle_between_them():
    ranges = add_circle(empty_scan(), (3.0, 0.15), LEG_RADIUS)
    ranges = add_circle(ranges, (3.0, -0.15), LEG_RADIUS)
    assert len(clusters_of(ranges)) == 2          # 군집화만 하면 다리 두 개
    obstacles = obstacles_of(ranges)
    assert len(obstacles) == 1                    # 묶으면 사람 하나
    assert obstacles[0].centroid[1] == pytest.approx(0.0, abs=0.03)
    assert 0.15 < obstacles[0].radius < 0.3       # 두 다리를 감싸는 크기


def test_small_objects_far_apart_are_not_merged():
    # 0.9 m 떨어진 두 물체는 사람 둘이다 (leg_merge_distance 0.6 m)
    ranges = add_circle(add_circle(empty_scan(), (3.0, 0.45), LEG_RADIUS), (3.0, -0.45), LEG_RADIUS)
    assert len(obstacles_of(ranges)) == 2


def test_leg_merging_can_be_disabled():
    ranges = add_circle(add_circle(empty_scan(), (3.0, 0.15), LEG_RADIUS), (3.0, -0.15), LEG_RADIUS)
    assert len(obstacles_of(ranges, ClusteringConfig(leg_merge_distance=0.0))) == 2


def test_each_cluster_is_merged_at_most_once():
    # 다리 세 개가 0.3 m 간격으로 서 있으면 둘만 묶이고 하나는 남는다
    ranges = empty_scan()
    for y in (0.3, 0.0, -0.3):
        ranges = add_circle(ranges, (3.0, y), LEG_RADIUS)
    clusters = clusters_of(ranges)
    assert len(clusters) == 3
    merged = merge_leg_pairs(clusters)
    assert len(merged) == 2
    # 점은 하나도 잃지 않고, 한 군집은 다리 두 개, 다른 하나는 다리 하나다
    assert sum(c.num_points for c in merged) == sum(c.num_points for c in clusters)
    singles = {c.num_points for c in clusters}
    assert sum(c.num_points in singles for c in merged) == 1


def test_large_clusters_are_not_merged_as_legs():
    # 사람 몸통 크기(지름 0.5 m) 물체 두 개는 다리가 아니라서 묶지 않는다
    ranges = add_circle(add_circle(empty_scan(), (4.0, 0.3), 0.25), (4.0, -0.35), 0.25)
    assert len(obstacles_of(ranges)) == 2


def test_detect_obstacles_drops_wall_and_keeps_person():
    ranges = add_segment(empty_scan(), (5.0, -3.0), (5.0, 3.0))
    ranges = add_circle(add_circle(ranges, (2.0, 0.15), LEG_RADIUS), (2.0, -0.15), LEG_RADIUS)
    obstacles = obstacles_of(ranges)
    people = [o for o in obstacles if o.centroid[0] < 3.0]
    assert len(people) == 1
    assert people[0].centroid[1] == pytest.approx(0.0, abs=0.03)
    # 다리 사이 틈으로 보이는 벽 조각은 짧아서 R1을 통과한다 (알려진 한계, R2 지도 대조가 거른다).
    # 긴 벽 자체는 후보가 아니다
    for fragment in obstacles:
        if fragment is not people[0]:
            assert fragment.centroid[0] == pytest.approx(5.0, abs=0.05)
            assert fragment.extent < 0.6


def test_rejects_negative_leg_merge_distance():
    with pytest.raises(ValueError):
        ClusteringConfig(leg_merge_distance=-0.1)


# --- 좌표 변환과 추적 ---
def test_transform_points_2d_rotates_then_translates():
    out = transform_points_2d(np.array([[1.0, 0.0], [0.0, 2.0]]), math.pi / 2, (10.0, -1.0))
    np.testing.assert_allclose(out, [[10.0, 0.0], [8.0, -1.0]], atol=1e-12)


def test_transform_points_2d_round_trip():
    pts = np.array([[1.2, -0.4], [3.0, 2.5]])
    yaw, t = 0.7, np.array([2.0, -3.0])
    back = transform_points_2d(transform_points_2d(pts, yaw, t) - t, -yaw, (0.0, 0.0))
    np.testing.assert_allclose(back, pts, atol=1e-12)


def _track_pillar_while_driving(to_fixed_frame):
    """로봇이 +x로 1 m/s 달리며 고정 좌표 (6, 1.5)의 기둥(반경 0.3 m)을 스캔 15번 본다."""
    tracker = MultiTargetTracker(TrackerConfig())
    tracks = []
    for k in range(15):
        t, robot_x = 0.1 * k, 0.1 * k
        ranges = add_circle(empty_scan(), (6.0 - robot_x, 1.5), 0.3)   # 로봇 기준 좌표로 본 기둥
        obstacles = obstacles_of(ranges)
        points = np.array([o.centroid for o in obstacles])
        if to_fixed_frame:
            points = transform_points_2d(points, 0.0, (robot_x, 0.0))
        tracks = tracker.step(t, points, [o.radius for o in obstacles])
    assert len(tracks) == 1
    return tracks[0]


def test_static_pillar_stays_static_while_robot_drives():
    track = _track_pillar_while_driving(to_fixed_frame=True)
    assert track.kf.speed < 0.15
    assert not track.is_dynamic
    assert 0.25 < track.radius < 0.4


def test_without_fixed_frame_static_pillar_looks_moving():
    # 로봇 기준 좌표를 그대로 넣으면 로봇의 움직임이 장애물 속도(−1 m/s)로 잡힌다
    track = _track_pillar_while_driving(to_fixed_frame=False)
    assert track.kf.velocity[0] == pytest.approx(-1.0, abs=0.15)


def test_u_shaped_cluster_with_close_ends_is_still_rejected():
    # 양 끝 점은 가깝지만 실제로는 2 m 넘게 펼쳐진 군집 (ㄷ자). 빠른 거르기가 놓쳐도 extent가 거른다
    u = np.array([[0.0, 0.0], [1.5, 0.0], [3.0, 0.0], [3.0, 0.5], [1.5, 0.5], [0.0, 0.5]])
    cluster = Cluster(points=u, beam_indices=np.arange(len(u)))
    assert np.linalg.norm(u[-1] - u[0]) < 2.0
    assert not is_obstacle_candidate(cluster)
