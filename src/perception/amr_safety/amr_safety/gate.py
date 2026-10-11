"""속도 명령 게이트와 스캔 점 (ROS 비의존).

- limit_velocity: speed_limit(SafetyStatus 규칙: inf 제한 없음 / 0.6 감속 / 0.0 정지)을 명령에 건다
- scan_body_points: LiDAR 원본 스캔 → 몸체 좌표 점 (진행 경로 판정 collision.path_hazard 입력)
- min_clearance: 점들 중 차체(사각형)에 가장 가까운 거리
- path_horizon: 진행 경로를 내다보는 시간
"""

import math
from typing import Sequence, Tuple

import numpy as np

from amr_perception_core.clustering import scan_to_points, transform_points_2d
from amr_perception_core.collision import footprint_clearance


def limit_velocity(linear_x: float, angular_z: float, speed_limit: float) -> Tuple[float, float]:
    """선속도 크기를 speed_limit 이하로 줄인다. 반환 (선속도, 각속도).

    - 정지(speed_limit ≤ 0)면 회전도 막는다. 차체가 사각형이라 제자리 회전도 모서리가 쓸고 지나간다
    - 감속이면 선속도·각속도를 같은 비율로 줄여 곡률(경로 모양)을 지킨다. 제자리 회전(선속도 0)은 그대로 둔다
    - 후진도 같은 크기 제한을 받는다
    - NaN·inf 명령은 정지로 바꾼다
    """
    if not (math.isfinite(linear_x) and math.isfinite(angular_z)) or speed_limit <= 0.0:
        return 0.0, 0.0
    speed = abs(linear_x)
    if not math.isfinite(speed_limit) or speed <= speed_limit:
        return linear_x, angular_z
    scale = speed_limit / speed
    return linear_x * scale, angular_z * scale


def scan_body_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                     range_min: float, range_max: float, yaw: float,
                     translation: Sequence[float]) -> np.ndarray:
    """스캔 점들을 몸체 좌표(yaw, translation: 스캔 프레임 → base_footprint)로 옮긴 (N, 2) 배열.

    range_min 미만·range_max 초과·NaN·inf 빔은 뺀다.
    """
    r = np.asarray(ranges, dtype=float)
    points, valid = scan_to_points(r, angle_min, angle_increment)
    with np.errstate(invalid='ignore'):
        valid &= (r >= range_min) & (r <= range_max)
    return transform_points_2d(points[valid], yaw, translation)


def min_clearance(points: np.ndarray, length: float, width: float) -> float:
    """차체에서 가장 가까운 점까지 [m]. 점이 없으면 inf."""
    if len(points) == 0:
        return math.inf
    return float(np.min(footprint_clearance(points, length, width)))


def path_horizon(linear_x: float, reaction_time: float, decel: float, minimum: float) -> float:
    """진행 경로를 내다보는 시간 [s] = 반응 지연 + 정지 시간(|v| / 감속). 회전·저속에도 minimum은 본다."""
    return max(minimum, reaction_time + abs(linear_x) / decel)
