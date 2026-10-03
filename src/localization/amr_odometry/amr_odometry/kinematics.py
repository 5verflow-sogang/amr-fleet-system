"""차동구동 순기구학과 휠 오도메트리 적분 (ROS 비의존).

좌표는 REP 103을 따른다: x 전방, y 왼쪽, yaw 반시계 양수.
입력은 한 주기 동안 좌우 바퀴가 굴러간 거리 [m]다 (바퀴 각도 변화 × 반지름).
"""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    """각도를 (-π, π]로 맞춘다."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def body_motion(d_left: float, d_right: float, wheel_separation: float) -> Tuple[float, float]:
    """좌우 바퀴 이동 거리 → 몸체 중심 이동 거리 ds [m], 회전량 dyaw [rad]."""
    ds = (d_right + d_left) / 2.0
    dyaw = (d_right - d_left) / wheel_separation
    return ds, dyaw


class DiffDriveOdometry:
    """휠 이동 거리를 적분해 odom 프레임 기준 위치와 공분산을 추정한다.

    위치 적분은 중점법(2차 Runge-Kutta)이다. 공분산은 Siegwart 모델로 전파한다:
    바퀴마다 이동 거리 오차 분산이 k²·|Δs|라고 보고 (amr_odometry 노이즈 모델과 같은 가정)
    Σ' = Fp Σ Fpᵀ + Fu Q Fuᵀ, Q = diag(k²|Δs_r|, k²|Δs_l|).
    계통 오차(바퀴 반지름 차이)는 평균이 0이 아니라 공분산으로 표현하지 않는다.
    """

    def __init__(self, wheel_separation: float, distance_noise_k: float = 0.0):
        if not wheel_separation > 0.0:
            raise ValueError(f'바퀴 간격은 양수여야 한다: {wheel_separation}')
        if distance_noise_k < 0.0:
            raise ValueError(f'distance_noise_k는 0 이상이어야 한다: {distance_noise_k}')
        self.wheel_separation = wheel_separation
        self.k2 = distance_noise_k ** 2
        self.reset()

    def reset(self, pose: Pose2D = None):
        self.pose = Pose2D() if pose is None else Pose2D(pose.x, pose.y, pose.yaw)
        self.covariance = np.zeros((3, 3))

    def update(self, d_left: float, d_right: float) -> Tuple[float, float]:
        """한 주기 바퀴 이동 거리로 위치·공분산을 갱신하고 (ds, dyaw)를 돌려준다."""
        b = self.wheel_separation
        ds, dyaw = body_motion(d_left, d_right, b)
        mid = self.pose.yaw + dyaw / 2.0
        c, s = math.cos(mid), math.sin(mid)

        fp = np.array([
            [1.0, 0.0, -ds * s],
            [0.0, 1.0, ds * c],
            [0.0, 0.0, 1.0],
        ])
        # 열 순서: (d_right, d_left). 중점 각도도 dyaw/2를 통해 바퀴 거리에 의존한다
        fu = np.array([
            [0.5 * c - ds / (2.0 * b) * s, 0.5 * c + ds / (2.0 * b) * s],
            [0.5 * s + ds / (2.0 * b) * c, 0.5 * s - ds / (2.0 * b) * c],
            [1.0 / b, -1.0 / b],
        ])
        q = np.diag([self.k2 * abs(d_right), self.k2 * abs(d_left)])
        self.covariance = fp @ self.covariance @ fp.T + fu @ q @ fu.T

        self.pose.x += ds * c
        self.pose.y += ds * s
        self.pose.yaw = normalize_angle(self.pose.yaw + dyaw)
        return ds, dyaw

    def twist_variance(self, d_left: float, d_right: float, dt: float) -> Tuple[float, float]:
        """한 주기 측정에서 나온 선속도·각속도의 분산 [(m/s)², (rad/s)²]."""
        if not dt > 0.0:
            raise ValueError(f'dt는 양수여야 한다: {dt}')
        var_sum = self.k2 * (abs(d_left) + abs(d_right))
        var_v = var_sum / 4.0 / dt ** 2
        var_w = var_sum / self.wheel_separation ** 2 / dt ** 2
        return var_v, var_w
