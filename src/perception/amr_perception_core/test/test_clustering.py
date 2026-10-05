import math

import numpy as np
import pytest

from amr_perception_core.clustering import (
    breakpoint_distance,
    cluster_scan,
    ClusteringConfig,
    is_obstacle_candidate,
    scan_to_points,
)

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
    ranges[50] = 10.0  # 가운데가 끊긴 두 덩어리
    assert len(cluster_scan(ranges, 0.0, INC)) == 2


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
