"""2D LiDAR 스캔 필터 (ROS 비의존).

거리 필터 → 각도 필터 → 아웃라이어 제거 순서로 적용한다. 빔 개수와 각도 배열은 바꾸지 않고
걸러낸 빔의 값만 REP 117 규칙으로 바꾼다 (SLAM·AMCL이 스캔 구조를 그대로 쓸 수 있게).

    -inf: 측정 최소 거리보다 가까움   +inf: 반사 없음(최대 거리 밖)   NaN: 잘못된 측정·제거한 점
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ScanFilterConfig:
    # 거리 필터 [m]. 센서 사양(sensors.yaml: 0.12~25 m)보다 좁히면 그 범위만 쓴다
    range_min: float = 0.12
    range_max: float = 25.0
    # 각도 필터 [rad]. 이 범위 밖 빔은 NaN. 기본은 360° 전체
    angle_min: float = -math.pi
    angle_max: float = math.pi
    # 아웃라이어: 앞뒤 outlier_window개 빔 중 거리 차이가 (outlier_distance + outlier_ratio·r) 이하인
    # 이웃이 outlier_min_neighbors개 미만이면 혼자 떨어진 점으로 보고 제거한다.
    # 멀리서 벽을 비스듬히 비추면 이웃 빔끼리도 거리 차이가 크므로 허용 차이를 거리에 비례해 늘린다
    outlier_window: int = 2
    outlier_distance: float = 0.2
    outlier_ratio: float = 0.05
    outlier_min_neighbors: int = 1

    def __post_init__(self):
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError(
                f'0 <= range_min < range_max여야 한다: [{self.range_min}, {self.range_max}]')
        if not self.angle_min < self.angle_max:
            raise ValueError(f'angle_min < angle_max여야 한다: [{self.angle_min}, {self.angle_max}]')
        if self.outlier_window < 0 or self.outlier_min_neighbors < 0:
            raise ValueError('outlier_window, outlier_min_neighbors는 0 이상이어야 한다')
        # outlier_window 0은 아웃라이어 제거를 끈다는 뜻이라 이웃 수는 검사하지 않는다
        if self.outlier_window > 0 and self.outlier_min_neighbors > 2 * self.outlier_window:
            raise ValueError('outlier_min_neighbors는 2 × outlier_window 이하여야 한다')
        if not self.outlier_distance > 0.0:
            raise ValueError(f'outlier_distance는 양수여야 한다: {self.outlier_distance}')
        if self.outlier_ratio < 0.0:
            raise ValueError(f'outlier_ratio는 0 이상이어야 한다: {self.outlier_ratio}')


def filter_range(ranges: np.ndarray, sensor_min: float, sensor_max: float,
                 config: ScanFilterConfig) -> np.ndarray:
    """거리 필터. 센서 한계와 설정 중 좁은 쪽을 쓴다."""
    lo = max(sensor_min, config.range_min)
    hi = min(sensor_max, config.range_max)
    out = ranges.astype(np.float32, copy=True)
    finite = np.isfinite(out)
    with np.errstate(invalid='ignore'):
        out[finite & (out < lo)] = -np.inf
        out[finite & (out > hi)] = np.inf
    return out


def filter_angle(ranges: np.ndarray, angle_min: float, angle_increment: float,
                 config: ScanFilterConfig) -> np.ndarray:
    """각도 필터. 설정 범위 밖 빔은 NaN."""
    angles = angle_min + angle_increment * np.arange(len(ranges))
    out = ranges.copy()
    # YAML에 적은 ±3.14159265처럼 π를 반올림한 경계가 첫·끝 빔을 자르지 않게 약간 여유를 둔다
    eps = 1e-6
    out[(angles < config.angle_min - eps) | (angles > config.angle_max + eps)] = np.nan
    return out


def remove_outliers(ranges: np.ndarray, config: ScanFilterConfig) -> np.ndarray:
    """주변과 동떨어진 외톨이 점을 NaN으로 바꾼다. 360° 스캔이라 양 끝은 이어서 본다."""
    w, need = config.outlier_window, config.outlier_min_neighbors
    if w == 0 or need == 0:
        return ranges.copy()
    out = ranges.copy()
    finite = np.isfinite(ranges)
    support = np.zeros(len(ranges), dtype=int)
    with np.errstate(invalid='ignore'):
        allowed = config.outlier_distance + config.outlier_ratio * np.abs(ranges)
    for shift in [s for s in range(-w, w + 1) if s != 0]:
        neighbor = np.roll(ranges, shift)
        with np.errstate(invalid='ignore'):
            close = np.isfinite(neighbor) & (np.abs(neighbor - ranges) <= allowed)
        support += close
    out[finite & (support < need)] = np.nan
    return out


def filter_scan(ranges, angle_min: float, angle_increment: float, sensor_min: float,
                sensor_max: float, config: ScanFilterConfig = ScanFilterConfig()) -> np.ndarray:
    """세 필터를 차례로 적용한 거리 배열 (float32)."""
    r = np.asarray(ranges, dtype=np.float32)
    r = filter_range(r, sensor_min, sensor_max, config)
    r = filter_angle(r, angle_min, angle_increment, config)
    return remove_outliers(r, config)
