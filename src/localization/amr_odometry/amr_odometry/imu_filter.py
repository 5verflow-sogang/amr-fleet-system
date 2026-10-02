"""IMU 전처리: 바이어스 보정 + 1차 저역통과 필터 (ROS 비의존).

1. 바이어스 보정: 시작 후 calibration_duration 동안 정지 상태의 평균으로 바이어스를 구한다.
   자이로 바이어스 = 평균 각속도, 가속도 바이어스 = 평균 가속도 − 중력(0, 0, g) (수평 설치 가정).
   그동안 흔들림(표준편차)이 크면 로봇이 움직인 것으로 보고 처음부터 다시 잰다.
   Gazebo IMU 바이어스는 실행마다 정해지면 바뀌지 않아 시작 때 한 번 재면 대부분 없어진다.
2. 저역통과: y ← y + α(x − y), α = dt / (τ + dt), τ = 1 / (2π·차단 주파수).
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

GRAVITY = 9.80665


@dataclass(frozen=True)
class ImuFilterConfig:
    # 저역통과 차단 주파수 [Hz]. 0이면 끈다. 100 Hz 샘플링이라 50 Hz 미만으로 둔다
    gyro_cutoff_hz: float = 20.0
    accel_cutoff_hz: float = 5.0
    # 바이어스 보정
    calibration_duration: float = 2.0     # s
    # 보정 중 이보다 흔들리면(축별 표준편차) 정지가 아니라고 본다
    stationary_gyro_stddev: float = 0.01   # rad/s
    stationary_accel_stddev: float = 0.1   # m/s²
    gravity: float = GRAVITY

    def __post_init__(self):
        for name in ('gyro_cutoff_hz', 'accel_cutoff_hz'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name}는 0 이상이어야 한다')
        if not self.calibration_duration > 0.0:
            raise ValueError('calibration_duration은 양수여야 한다')


class LowPass3:
    """3축 1차 저역통과 필터. 첫 샘플로 초기화한다."""

    def __init__(self, cutoff_hz: float):
        self.tau = 1.0 / (2.0 * math.pi * cutoff_hz) if cutoff_hz > 0.0 else 0.0
        self.y: Optional[np.ndarray] = None

    def update(self, x: np.ndarray, dt: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if self.y is None or self.tau == 0.0 or dt <= 0.0:
            self.y = x.copy()
        else:
            alpha = dt / (self.tau + dt)
            self.y = self.y + alpha * (x - self.y)
        return self.y.copy()


class BiasCalibrator:
    """정지 구간 평균으로 자이로·가속도 바이어스를 잰다."""

    def __init__(self, config: ImuFilterConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.gyro, self.accel, self.t0 = [], [], None
        self.gyro_bias: Optional[np.ndarray] = None
        self.accel_bias: Optional[np.ndarray] = None
        self.restarts = 0
        self.samples = 0

    @property
    def done(self) -> bool:
        return self.gyro_bias is not None

    def add(self, t: float, gyro, accel) -> bool:
        """샘플을 넣는다. 보정이 끝나면 True."""
        if self.done:
            return True
        if self.t0 is None:
            self.t0 = t
        self.gyro.append(np.asarray(gyro, dtype=float))
        self.accel.append(np.asarray(accel, dtype=float))
        if t - self.t0 < self.config.calibration_duration or len(self.gyro) < 10:
            return False
        g, a = np.array(self.gyro), np.array(self.accel)
        if (g.std(axis=0).max() > self.config.stationary_gyro_stddev
                or a.std(axis=0).max() > self.config.stationary_accel_stddev):
            # 움직였다 → 다시 잰다
            self.gyro, self.accel, self.t0 = [], [], None
            self.restarts += 1
            return False
        self.gyro_bias = g.mean(axis=0)
        self.accel_bias = a.mean(axis=0) - np.array([0.0, 0.0, self.config.gravity])
        self.samples = len(g)
        return True


class ImuFilter:
    """바이어스 보정 → 저역통과. 보정이 끝나기 전에는 결과를 내지 않는다(None)."""

    def __init__(self, config: ImuFilterConfig = ImuFilterConfig()):
        self.config = config
        self.calibrator = BiasCalibrator(config)
        self.gyro_lp = LowPass3(config.gyro_cutoff_hz)
        self.accel_lp = LowPass3(config.accel_cutoff_hz)
        self.last_t: Optional[float] = None

    def update(self, t: float, gyro, accel) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if not self.calibrator.done:
            self.calibrator.add(t, gyro, accel)
            return None
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        g = np.asarray(gyro, dtype=float) - self.calibrator.gyro_bias
        a = np.asarray(accel, dtype=float) - self.calibrator.accel_bias
        return self.gyro_lp.update(g, dt), self.accel_lp.update(a, dt)
