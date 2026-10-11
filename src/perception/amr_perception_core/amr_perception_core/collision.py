"""차체-장애물 거리, 충돌까지 남은 시간(TTC), 진행 경로 위 장애물 (ROS 비의존).

- footprint_clearance: 로봇 차체(사각형)와 점들 사이 최소 거리
- footprint_time_to_collision: 차체(사각형)와 장애물 원이 닿기까지 남은 시간 (amr_tracking collision_risks)
- path_hazard: 명령 속도로 움직일 때 차체가 지나갈 자리의 장애물 (amr_safety 긴급정지)
- time_to_collision: 두 원의 TTC 닫힌 식 (원 모델이 필요한 곳용)

원(외접원 0.36 m)으로 본 TTC는 옆쪽이 실제보다 0.16 m 커서, 0.6 m 좁은 통로(옆 여유 0.1 m)에서 벽과
늘 겹쳐 TTC 0이 됐다 (Gazebo 실검증). 그래서 차체 판정은 실제 사각형으로 한다.
"""

import math
from typing import Optional, Sequence

import numpy as np


def circumscribed_radius(length: float, width: float) -> float:
    """사각형 차체를 감싸는 원의 반지름 [m]."""
    return 0.5 * math.hypot(length, width)


def footprint_clearance(points: np.ndarray, length: float, width: float) -> np.ndarray:
    """로봇 기준 좌표(base_footprint, x 앞·y 왼쪽, 원점 = 차체 중심)의 점들 (N, 2)과 차체 사이 거리 [m].

    차체 안의 점은 0이다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dx = np.maximum(np.abs(pts[:, 0]) - 0.5 * length, 0.0)
    dy = np.maximum(np.abs(pts[:, 1]) - 0.5 * width, 0.0)
    return np.hypot(dx, dy)


def time_to_collision(relative_position: Sequence[float], relative_velocity: Sequence[float],
                      radius: float) -> float:
    """|p + v t| = radius를 만족하는 가장 작은 t ≥ 0 [s].

    relative_position p = 장애물 − 로봇, relative_velocity v = 장애물 속도 − 로봇 속도 (같은 고정 좌표계).
    이미 겹쳐 있으면 0, 닿지 않거나 멀어지는 중이면 inf (CollisionRisk.ttc 규칙, #64).
    """
    p = np.asarray(relative_position, dtype=float).reshape(2)
    v = np.asarray(relative_velocity, dtype=float).reshape(2)
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v)) and math.isfinite(radius)):
        raise ValueError(f'유한한 값이어야 한다: p={p}, v={v}, radius={radius}')
    c = float(p @ p) - radius * radius
    if c <= 0.0:
        return 0.0
    b = float(p @ v)
    if b >= 0.0:
        return math.inf          # 멀어지거나 거리가 그대로
    a = float(v @ v)
    disc = b * b - a * c
    if disc < 0.0:
        return math.inf          # 비껴감
    return (-b - math.sqrt(disc)) / a


def footprint_time_to_collision(relative_position: Sequence[float],
                                relative_velocity: Sequence[float], obstacle_radius: float,
                                length: float, width: float, horizon: float = 5.0,
                                dt: float = 0.05) -> float:
    """몸체 좌표(base_footprint)의 장애물 원이 차체 사각형에 닿기까지 남은 시간 [s].

    relative_position: 장애물 중심 − 로봇 중심, relative_velocity: 장애물 속도 − 로봇 속도 (둘 다 몸체 축).
    로봇 회전은 무시한다 (등속 직선 상대 운동). dt 간격으로 따라가며 처음 닿는 칸을 선형 보간한다.
    - horizon 안에 닿지 않으면 inf
    - 이미 겹쳐 있으면 가까워지는 중일 때만 0, 그대로거나 멀어지면 inf (겹친 채 멀어지는 것을 막지 않는다)
    """
    p = np.asarray(relative_position, dtype=float).reshape(2)
    v = np.asarray(relative_velocity, dtype=float).reshape(2)
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v)) and math.isfinite(obstacle_radius)):
        raise ValueError(f'유한한 값이어야 한다: p={p}, v={v}, radius={obstacle_radius}')
    if not (dt > 0.0 and horizon > 0.0):
        raise ValueError(f'dt, horizon은 양수여야 한다: dt={dt}, horizon={horizon}')
    times = np.arange(0.0, horizon + 0.5 * dt, dt)
    clearance = footprint_clearance(p + times[:, None] * v, length, width) - obstacle_radius
    if clearance[0] <= 0.0:
        return 0.0 if clearance[1] < clearance[0] - 1e-9 else math.inf
    hit = np.flatnonzero(clearance <= 0.0)
    if hit.size == 0:
        return math.inf
    k = int(hit[0])
    before, after = clearance[k - 1], clearance[k]
    return float(times[k - 1] + dt * before / (before - after))


def path_hazard(points: np.ndarray, linear_x: float, angular_z: float, length: float,
                width: float, front_margin: float, side_margin: float, horizon: float,
                dt: float = 0.05) -> Optional[float]:
    """명령 (선속도, 각속도)로 horizon [s] 동안 움직일 때 차체 앞길에 걸리는 점이 있는지.

    points: 몸체 좌표(base_footprint) 점 (N, 2). 로봇 경로는 등속 원호(차동 구동)로 본다.
    점은 다음일 때 위험이다 (그 시점에 지금보다 가까워지는 점만. 멀어지는 명령은 막지 않는다):
    - 진행 방향 차체 앞(후진이면 뒤) front_margin 띠 안에 들어온다 (원 명세 긴급정지 거리 0.3 m)
    - 차체에서 side_margin 안으로 들어온다 (옆·모서리 스침, 제자리 회전)
    옆으로 나란히 지나가는 벽은 가까워지지 않으므로 위험이 아니다 (0.6 m 좁은 통로 통과).
    반환: 위험한 점들 중 지금 차체에서 가장 가까운 거리 [m], 없으면 None.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if pts.shape[0] == 0 or not (math.isfinite(linear_x) and math.isfinite(angular_z)):
        return None
    hx, hy = 0.5 * length, 0.5 * width
    times = np.arange(dt, horizon + 0.5 * dt, dt)
    theta = angular_z * times
    if abs(angular_z) < 1e-6:
        px, py = linear_x * times, np.zeros_like(times)
    else:
        px = linear_x / angular_z * np.sin(theta)
        py = linear_x / angular_z * (1.0 - np.cos(theta))
    # 각 시점 차체 좌표로 옮긴 점 (K, N)
    c, s = np.cos(theta)[:, None], np.sin(theta)[:, None]
    dx, dy = pts[None, :, 0] - px[:, None], pts[None, :, 1] - py[:, None]
    qx, qy = c * dx + s * dy, -s * dx + c * dy
    clearance = np.hypot(np.maximum(np.abs(qx) - hx, 0.0), np.maximum(np.abs(qy) - hy, 0.0))
    now = footprint_clearance(pts, length, width)
    closer = clearance < now[None, :] - 1e-3
    zone = clearance <= side_margin
    if linear_x > 1e-3:
        zone |= (qx > hx) & (qx <= hx + front_margin) & (np.abs(qy) <= hy)
    elif linear_x < -1e-3:
        zone |= (qx < -hx) & (qx >= -hx - front_margin) & (np.abs(qy) <= hy)
    hazard = np.any(closer & zone, axis=0)
    return float(np.min(now[hazard])) if np.any(hazard) else None
