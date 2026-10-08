"""2D LiDAR 스캔 군집화와 크기 규칙 (ROS 비의존).

스캔의 유효 빔을 각도 순서대로 보며, 이웃한 두 점 사이 거리가 임계값을 넘으면 군집을 끊는다.
임계값은 거리에 비례해 늘리는 adaptive breakpoint(Borges & Aldon, 2004)다:

    D_max = r_prev · sin(Δφ) / sin(λ − Δφ) + 3σ_r

멀수록 이웃 빔 사이 간격이 벌어지므로(10 m에서 0.5°는 약 9 cm) 고정 임계값으로는 먼 물체가 쪼개지거나
가까운 물체끼리 붙는다. λ는 빔과 표면이 이루는 가장 얕은 각도로, 이보다 비스듬한 표면은 끊어진다.

LiDAR 높이(약 0.2 m)에서 사람은 다리 두 개로 보인다. 작은 군집 두 개가 가까우면 한 사람으로 묶는다
(merge_leg_pairs). 묶지 않으면 사람 하나가 가는 장애물 두 개로 추적된다.

무효 빔(REP 117: NaN, ±inf)은 점으로 쓰지 않는다. 물체 안에서 빠진 빔 몇 개(max_gap_beams 이하)는
건너뛰고 이어 붙이되, 빠진 각도만큼 임계값을 늘린다.
"""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class ClusteringConfig:
    """군집화·크기 규칙 파라미터. 실제 값은 노드 패키지가 YAML에서 읽어 채운다."""

    # 빔과 표면이 이루는 가장 얕은 각도 λ [deg]. 작을수록 비스듬한 벽도 이어 붙인다
    breakpoint_lambda_deg: float = 10.0
    # LiDAR 거리 잡음 σ [m]. amr_description/config/sensors.yaml의 LiDAR 값과 같게 둔다
    range_noise_std: float = 0.03
    # 군집 사이에 이만큼까지 빠진 빔(무효값)은 건너뛰고 이어 붙인다
    max_gap_beams: int = 2
    # 점이 이보다 적은 군집은 잡음으로 버린다. 1 m 앞 0.3 m 물체에 0.5° 빔이 약 34개 맞는다
    min_points: int = 3
    # R1 크기 규칙: 군집 안 두 점 사이 최대 거리(extent)가 이보다 크면 벽·랙 면 같은 구조물로 본다 [m].
    # 사람(약 0.5 m), 다른 AMR(0.6 × 0.4 m), 지게차 측면(약 2 m 이하 가정)을 통과시키는 값
    max_obstacle_extent: float = 2.0
    # 다리 묶기: 보이는 크기가 leg_max_extent [m] 이하인 군집 두 개의 중심이 leg_merge_distance [m]
    # 안이면 한 물체로 합친다. 0.6 m는 사람 어깨너비 수준 (Gazebo 작업자 다리 간격으로 확인한다).
    # leg_merge_distance를 0으로 두면 묶지 않는다
    leg_max_extent: float = 0.35
    leg_merge_distance: float = 0.6

    def __post_init__(self):
        if not 0.0 < self.breakpoint_lambda_deg < 90.0:
            raise ValueError(
                f'breakpoint_lambda_deg는 (0, 90) 범위여야 한다: {self.breakpoint_lambda_deg}')
        for name in ('range_noise_std', 'max_obstacle_extent', 'leg_max_extent'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')
        if int(self.max_gap_beams) != self.max_gap_beams or self.max_gap_beams < 0:
            raise ValueError(f'max_gap_beams는 0 이상의 정수여야 한다: {self.max_gap_beams}')
        if int(self.min_points) != self.min_points or self.min_points < 1:
            raise ValueError(f'min_points는 1 이상의 정수여야 한다: {self.min_points}')
        if not (math.isfinite(self.leg_merge_distance) and self.leg_merge_distance >= 0.0):
            raise ValueError(f'leg_merge_distance는 0 이상이어야 한다: {self.leg_merge_distance}')


@dataclass(frozen=True)
class Cluster:
    """한 군집. 좌표는 스캔 좌표계(보통 lidar_link) 기준이다."""

    points: np.ndarray        # (N, 2) [m]
    beam_indices: np.ndarray  # (N,) 스캔 안 빔 번호

    @property
    def num_points(self) -> int:
        return int(self.points.shape[0])

    @property
    def centroid(self) -> np.ndarray:
        """점들의 평균. LiDAR는 센서 쪽 표면만 보므로 물체 실제 중심보다 센서 쪽으로 치우친다."""
        return self.points.mean(axis=0)

    @property
    def radius(self) -> float:
        """중심에서 가장 먼 점까지 거리 [m]. 군집을 감싸는 원의 근사."""
        return float(np.max(np.linalg.norm(self.points - self.centroid, axis=1)))

    @property
    def extent(self) -> float:
        """군집 안 두 점 사이 최대 거리 [m]. 물체의 보이는 크기."""
        diff = self.points[:, None, :] - self.points[None, :, :]
        return float(np.sqrt(np.max(np.sum(diff * diff, axis=-1))))


def scan_to_points(
    ranges: Sequence[float], angle_min: float, angle_increment: float
) -> Tuple[np.ndarray, np.ndarray]:
    """스캔 거리 배열 → 빔별 (x, y)와 유효 마스크. 무효 빔(NaN, ±inf, 0 이하)의 좌표는 NaN이다."""
    r = np.asarray(ranges, dtype=float)
    if r.ndim != 1:
        raise ValueError(f'ranges는 1차원이어야 한다: shape={r.shape}')
    if not (math.isfinite(angle_min) and math.isfinite(angle_increment) and angle_increment > 0.0):
        raise ValueError(f'잘못된 각도: angle_min={angle_min}, angle_increment={angle_increment}')
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(r) & (r > 0.0)
    angles = angle_min + angle_increment * np.arange(r.size)
    points = np.full((r.size, 2), np.nan)
    points[valid, 0] = r[valid] * np.cos(angles[valid])
    points[valid, 1] = r[valid] * np.sin(angles[valid])
    return points, valid


def breakpoint_distance(
    previous_range: float, angle_step: float, config: ClusteringConfig
) -> float:
    """이웃한 두 점이 같은 군집으로 이어질 수 있는 최대 거리 [m]."""
    lam = math.radians(config.breakpoint_lambda_deg)
    if angle_step >= lam:
        return 3.0 * config.range_noise_std
    return (previous_range * math.sin(angle_step) / math.sin(lam - angle_step)
            + 3.0 * config.range_noise_std)


def _is_full_circle(num_beams: int, angle_increment: float) -> bool:
    return abs(num_beams * angle_increment - 2.0 * math.pi) < 1.5 * angle_increment


def cluster_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    config: ClusteringConfig = ClusteringConfig(),
) -> List[Cluster]:
    """스캔을 군집으로 나눈다. 점이 min_points보다 적은 군집은 버린다.

    360° 스캔이면 마지막 빔과 첫 빔도 이웃으로 보고, 둘에 걸친 물체를 한 군집으로 합친다.
    """
    r = np.asarray(ranges, dtype=float)
    points, valid = scan_to_points(r, angle_min, angle_increment)
    indices = np.flatnonzero(valid)
    if indices.size == 0:
        return []

    def connected(i: int, j: int, beam_gap: int) -> bool:
        if beam_gap - 1 > config.max_gap_beams:
            return False
        limit = breakpoint_distance(r[i], beam_gap * angle_increment, config)
        return float(np.linalg.norm(points[j] - points[i])) <= limit

    # 이웃한 유효 빔끼리 이어지는지를 numpy로 한 번에 판정한다 (connected와 같은 규칙, 빔 수백 개를
    # 한 쌍씩 Python으로 보던 것이 처리 시간의 대부분이었다)
    prev, cur = indices[:-1], indices[1:]
    gap = cur - prev
    step = gap * angle_increment
    lam = math.radians(config.breakpoint_lambda_deg)
    noise = 3.0 * config.range_noise_std
    shallow = step < lam
    denom = np.where(shallow, np.sin(lam - step), 1.0)
    limit = np.where(shallow, r[prev] * np.sin(step) / denom + noise, noise)
    dist = np.linalg.norm(points[cur] - points[prev], axis=1)
    linked = (gap - 1 <= config.max_gap_beams) & (dist <= limit)
    groups: List[List[int]] = [g.tolist() for g in np.split(indices, np.flatnonzero(~linked) + 1)]

    if len(groups) > 1 and _is_full_circle(r.size, angle_increment):
        last, first = groups[-1][-1], groups[0][0]
        if connected(last, first, int(first + r.size - last)):
            groups[0] = groups.pop() + groups[0]

    return [
        Cluster(points=points[group], beam_indices=np.asarray(group))
        for group in groups if len(group) >= config.min_points
    ]


def is_obstacle_candidate(cluster: Cluster, config: ClusteringConfig = ClusteringConfig()) -> bool:
    """R1 크기 규칙: 벽·랙 면처럼 큰 군집이 아니면 장애물 후보다.

    extent는 모든 점 쌍을 보므로(점 N개면 N²) 점이 수백 개인 벽 군집에서 느리다. 양 끝 점 거리는
    extent보다 클 수 없으므로, 그것만으로 기준을 넘으면 extent를 계산하지 않고 바로 뺀다.
    """
    ends = float(np.linalg.norm(cluster.points[-1] - cluster.points[0]))
    if ends > config.max_obstacle_extent:
        return False
    return cluster.extent <= config.max_obstacle_extent


def merge_leg_pairs(clusters: Sequence[Cluster],
                    config: ClusteringConfig = ClusteringConfig()) -> List[Cluster]:
    """작은 군집 두 개(다리 한 쌍)의 중심이 leg_merge_distance 안이면 하나로 합친다.

    가까운 쌍부터 묶고, 한 군집은 한 번만 묶는다 (사람 셋이 붙어 서도 둘씩만 합쳐진다).
    합친 군집의 중심은 두 다리 사이, 반경은 두 다리를 감싸는 크기가 된다.
    """
    clusters = list(clusters)
    if config.leg_merge_distance <= 0.0:
        return clusters
    small = [i for i, c in enumerate(clusters) if c.extent <= config.leg_max_extent]
    pairs = []
    for a_pos, i in enumerate(small):
        for j in small[a_pos + 1:]:
            d = float(np.linalg.norm(clusters[i].centroid - clusters[j].centroid))
            if d <= config.leg_merge_distance:
                pairs.append((d, i, j))
    pairs.sort()
    used, merged = set(), []
    for _, i, j in pairs:
        if i in used or j in used:
            continue
        used.update((i, j))
        merged.append(Cluster(
            points=np.vstack([clusters[i].points, clusters[j].points]),
            beam_indices=np.concatenate([clusters[i].beam_indices, clusters[j].beam_indices])))
    return [c for k, c in enumerate(clusters) if k not in used] + merged


def detect_obstacles(ranges: Sequence[float], angle_min: float, angle_increment: float,
                     config: ClusteringConfig = ClusteringConfig()) -> List[Cluster]:
    """스캔 → 군집화 → R1 크기 규칙 → 다리 묶기. 좌표는 스캔 좌표계 그대로다."""
    candidates = [c for c in cluster_scan(ranges, angle_min, angle_increment, config)
                  if is_obstacle_candidate(c, config)]
    return merge_leg_pairs(candidates, config)


def transform_points_2d(points: np.ndarray, yaw: float, translation: Sequence[float]) -> np.ndarray:
    """(N, 2) 점들을 yaw [rad]만큼 돌리고 translation (tx, ty)만큼 옮긴다 (2D 강체 변환).

    스캔 좌표계(lidar_link) → 고정 좌표계(odom)에 쓴다. LiDAR가 수평이라 2D 변환이면 된다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    return pts @ rotation.T + np.asarray(translation, dtype=float).reshape(2)
