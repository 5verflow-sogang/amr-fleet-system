"""등속(Constant Velocity) 모델 칼만 필터 (ROS 비의존).

상태 x = [px, py, vx, vy] (m, m/s), 관측 z = [px, py] (LiDAR 군집 중심).
프로세스 잡음은 가속도를 백색잡음(σa)으로 보는 이산 모델이다: Q = σa² G Gᵀ, G = [dt²/2, dt] (축별).
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np

_H = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
])


@dataclass(frozen=True)
class KalmanConfig:
    """KF 잡음 파라미터. 실제 값은 노드 패키지가 YAML에서 읽어 채운다."""

    # 가속도 잡음 σa [m/s²]. 등속에서 벗어나는 정도(사람 방향 전환, 지게차 가감속)를 흡수한다
    accel_noise_std: float = 1.0
    # 위치 관측 잡음 σ [m]. LiDAR 거리 잡음(0.03 m) + 군집 중심이 표면 쪽으로 치우치는 오차
    measurement_noise_std: float = 0.05
    # 첫 관측 때 모르는 속도의 표준편차 [m/s]. 명세 장애물 최대 속도
    initial_velocity_std: float = 1.5

    def __post_init__(self):
        for name in ('accel_noise_std', 'measurement_noise_std', 'initial_velocity_std'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')


def transition_matrix(dt: float) -> np.ndarray:
    """등속 모델 상태 전이 F: 위치 += 속도 * dt."""
    f = np.eye(4)
    f[0, 2] = f[1, 3] = dt
    return f


def process_noise(dt: float, accel_noise_std: float) -> np.ndarray:
    """dt 동안 쌓이는 프로세스 잡음 Q = σa² G Gᵀ."""
    a = dt ** 4 / 4.0
    b = dt ** 3 / 2.0
    c = dt ** 2
    return accel_noise_std ** 2 * np.array([
        [a, 0.0, b, 0.0],
        [0.0, a, 0.0, b],
        [b, 0.0, c, 0.0],
        [0.0, b, 0.0, c],
    ])


def _finite_xy(value: Sequence[float], name: str) -> np.ndarray:
    xy = np.asarray(value, dtype=float)
    if xy.shape != (2,) or not np.all(np.isfinite(xy)):
        raise ValueError(f'{name}는 유한한 (x, y)여야 한다: {value}')
    return xy


class ConstantVelocityKF:
    """장애물 1개의 위치·속도 추정. predict(t)로 시각을 맞춘 뒤 update(z)로 관측을 반영한다."""

    def __init__(
        self,
        position: Sequence[float],
        timestamp: float,
        config: KalmanConfig = KalmanConfig(),
    ):
        px, py = _finite_xy(position, 'position')
        self.config = config
        self.t = float(timestamp)
        self.x = np.array([px, py, 0.0, 0.0])
        r = config.measurement_noise_std ** 2
        v = config.initial_velocity_std ** 2
        self.P = np.diag([r, r, v, v])

    def predict(self, timestamp: float) -> None:
        """상태와 공분산을 timestamp [s]로 전파한다. 같은 시각이면 아무것도 하지 않는다."""
        dt = float(timestamp) - self.t
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError(f'시각이 거꾸로 갔다: {timestamp} < {self.t}')
        if dt == 0.0:
            return
        f = transition_matrix(dt)
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + process_noise(dt, self.config.accel_noise_std)
        self.t = float(timestamp)

    def measurement_covariance(self) -> np.ndarray:
        return np.eye(2) * self.config.measurement_noise_std ** 2

    def innovation(
        self, z: Sequence[float], r: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """혁신 y = z - Hx와 그 공분산 S = H P Hᵀ + R."""
        z = _finite_xy(z, 'z')
        r = self.measurement_covariance() if r is None else np.asarray(r, dtype=float)
        if r.shape != (2, 2):
            raise ValueError(f'R은 2x2여야 한다: shape={r.shape}')
        return z - _H @ self.x, _H @ self.P @ _H.T + r

    def mahalanobis_sq(self, z: Sequence[float], r: Optional[np.ndarray] = None) -> float:
        """관측 z가 예측과 얼마나 떨어졌는지 (yᵀ S⁻¹ y). 연관 게이트에 쓴다."""
        y, s = self.innovation(z, r)
        return float(y @ np.linalg.solve(s, y))

    def mahalanobis_sq_many(self, zs: np.ndarray) -> np.ndarray:
        """여러 관측 (M, 2)의 마할라노비스 거리²를 한 번에 계산한다 (S⁻¹은 한 번만). R은 기본값.

        연관 단계에서 트랙마다 측정 전부와 비교하므로, 쌍마다 S를 다시 푸는 mahalanobis_sq보다 빠르다.
        """
        zs = np.asarray(zs, dtype=float).reshape(-1, 2)
        if not np.all(np.isfinite(zs)):
            raise ValueError('관측은 유한한 (x, y)여야 한다')
        s = _H @ self.P @ _H.T + self.measurement_covariance()
        y = zs - self.x[:2]
        return np.einsum('ij,jk,ik->i', y, np.linalg.inv(s), y)

    def update(self, z: Sequence[float], r: Optional[np.ndarray] = None) -> None:
        """위치 관측 z를 반영한다. r을 주지 않으면 measurement_noise_std로 만든 R을 쓴다."""
        r = self.measurement_covariance() if r is None else np.asarray(r, dtype=float)
        y, s = self.innovation(z, r)
        k = np.linalg.solve(s, _H @ self.P).T          # K = P Hᵀ S⁻¹ (S, P 대칭)
        self.x = self.x + k @ y
        i_kh = np.eye(4) - k @ _H
        # Joseph form: 반복 갱신에서도 P가 대칭·양의 정부호를 유지한다
        self.P = i_kh @ self.P @ i_kh.T + k @ r @ k.T
        self.P = (self.P + self.P.T) / 2.0

    @property
    def position(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:].copy()

    @property
    def position_covariance(self) -> np.ndarray:
        return self.P[:2, :2].copy()

    @property
    def velocity_covariance(self) -> np.ndarray:
        return self.P[2:, 2:].copy()

    @property
    def speed(self) -> float:
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def heading(self) -> float:
        """이동 방향 [rad], 좌표계 x축 기준 반시계."""
        return float(math.atan2(self.x[3], self.x[2]))
