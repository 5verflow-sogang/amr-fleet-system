"""IMU 전처리: 바이어스 보정 + 1차 저역통과 필터 (ROS 비의존).

1. 바이어스 보정: 시작 후 calibration_duration 동안 정지 상태의 평균으로 바이어스를 구한다.
   자이로 바이어스 = 평균 각속도, 가속도 바이어스 = 평균 가속도 − 중력(0, 0, g) (수평 설치 가정).
   그동안 흔들림(표준편차)이 크거나, 평균 각속도가 크거나(일정 속도 회전), 평균 가속도가 중력과 다르면
   로봇이 움직인 것으로 보고 처음부터 다시 잰다. 일정 속도 회전은 흔들림이 작아 표준편차만으로는 못 거른다.
   Gazebo IMU 바이어스는 실행마다 정해지면 바뀌지 않아 시작 때 한 번 재면 대부분 없어진다.
2. 저역통과: y ← y + α(x − y), α = dt / (τ + dt), τ = 1 / (2π·차단 주파수).
3. 보정 폴백: 로봇이 계속 움직여 보정이 max_calibration_attempts번 연속 실패하면 바이어스 0(미보정)으로
   발행을 시작한다. 미보정으로 내는 동안에도 보정을 계속 시도해, 정지 구간이 생기면 그때 보정값으로 바꾼다.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

GRAVITY = 9.80665


@dataclass(frozen=True)
class ImuFilterConfig:
    # 저역통과 차단 주파수 [Hz]. 0이면 끈다. 100 Hz 샘플링이라 50 Hz 미만으로 둔다.
    # 자이로는 끈다: 노이즈(σ 2e-4 rad/s)가 작아 거를 게 거의 없고, 20 Hz(τ≈8 ms) 지연이 회전 가속 때
    # 노이즈보다 훨씬 큰 오차를 낸다 (#76·#81 IMU 분산비, 실측은 README)
    gyro_cutoff_hz: float = 0.0
    accel_cutoff_hz: float = 5.0
    # 바이어스 보정
    calibration_duration: float = 2.0     # s
    # 보정 중 이보다 흔들리면(축별 표준편차) 정지가 아니라고 본다
    stationary_gyro_stddev: float = 0.01   # rad/s
    stationary_accel_stddev: float = 0.1   # m/s²
    # 보정 구간 평균 각속도가 이보다 크면(축별 절댓값) 회전 중으로 본다. 실제 MEMS 바이어스(약 0.01 rad/s 이하)보다 크게
    stationary_gyro_mean_max: float = 0.05     # rad/s
    # 보정 구간 평균 가속도가 중력 (0, 0, g)에서 이만큼 넘게 벗어나면(축별) 가속·기울어짐으로 본다.
    # 수평 가속은 크기(노름)를 거의 안 바꿔서 축별로 본다. 바이어스(sensors.yaml 0.1 m/s²)보다 크게
    stationary_accel_mean_tolerance: float = 0.3   # m/s²
    gravity: float = GRAVITY
    # 보정이 이 횟수만큼 연속 실패하면 미보정으로 발행을 시작한다. 0이면 보정될 때까지 발행하지 않는다
    max_calibration_attempts: int = 5
    # 미보정 발행 때 공분산에 더하는 바이어스 불확실성 σ
    uncalibrated_gyro_bias_stddev: float = 0.01   # rad/s
    uncalibrated_accel_bias_stddev: float = 0.2   # m/s²

    def __post_init__(self):
        for name in ('gyro_cutoff_hz', 'accel_cutoff_hz'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name}는 0 이상이어야 한다')
        if not self.calibration_duration > 0.0:
            raise ValueError('calibration_duration은 양수여야 한다')
        if self.max_calibration_attempts < 0:
            raise ValueError('max_calibration_attempts는 0 이상이어야 한다')
        if self.uncalibrated_gyro_bias_stddev < 0.0 or self.uncalibrated_accel_bias_stddev < 0.0:
            raise ValueError('미보정 바이어스 σ는 0 이상이어야 한다')
        if self.stationary_gyro_mean_max < 0.0 or self.stationary_accel_mean_tolerance < 0.0:
            raise ValueError('정지 판정 기준은 0 이상이어야 한다')


def calibrated_variance(noise_stddev: float, samples: int, floor: float = 0.0) -> float:
    """보정이 끝난 뒤 한 축의 측정 분산.

    센서 노이즈 σ² + 보정 평균의 표준오차²(σ²/N) + 하한(floor).
    floor는 노이즈 모델에 없는 오차(가감속 순간의 시각 어긋남 등)를 실측에 맞춰 더하는 값이다.
    """
    if noise_stddev < 0.0 or floor < 0.0:
        raise ValueError('노이즈 σ와 floor는 0 이상이어야 한다')
    n = max(samples, 1)
    return noise_stddev ** 2 * (1.0 + 1.0 / n) + floor


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
        c = self.config
        if (g.std(axis=0).max() > c.stationary_gyro_stddev
                or a.std(axis=0).max() > c.stationary_accel_stddev
                or np.abs(g.mean(axis=0)).max() > c.stationary_gyro_mean_max
                or np.abs(a.mean(axis=0) - [0.0, 0.0, c.gravity]).max()
                > c.stationary_accel_mean_tolerance):
            # 움직였다 → 다시 잰다
            self.gyro, self.accel, self.t0 = [], [], None
            self.restarts += 1
            return False
        self.gyro_bias = g.mean(axis=0)
        self.accel_bias = a.mean(axis=0) - np.array([0.0, 0.0, self.config.gravity])
        self.samples = len(g)
        return True


class ImuFilter:
    """바이어스 보정 → 저역통과.

    보정 전에는 결과를 내지 않는다(None). 단 보정이 max_calibration_attempts번 실패하면
    바이어스 0으로 결과를 내고(uncalibrated = True), 보정은 뒤에서 계속 시도한다.
    """

    def __init__(self, config: ImuFilterConfig = ImuFilterConfig()):
        self.config = config
        self.calibrator = BiasCalibrator(config)
        self.gyro_lp = LowPass3(config.gyro_cutoff_hz)
        self.accel_lp = LowPass3(config.accel_cutoff_hz)
        self.last_t: Optional[float] = None

    @property
    def uncalibrated(self) -> bool:
        """보정 없이(바이어스 0으로) 내는 중인가."""
        n = self.config.max_calibration_attempts
        return not self.calibrator.done and n > 0 and self.calibrator.restarts >= n

    def update(self, t: float, gyro, accel) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if not self.calibrator.done:
            self.calibrator.add(t, gyro, accel)
        if self.calibrator.done:
            gyro_bias, accel_bias = self.calibrator.gyro_bias, self.calibrator.accel_bias
        elif self.uncalibrated:
            gyro_bias = accel_bias = np.zeros(3)
        else:
            return None
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        g = np.asarray(gyro, dtype=float) - gyro_bias
        a = np.asarray(accel, dtype=float) - accel_bias
        return self.gyro_lp.update(g, dt), self.accel_lp.update(a, dt)
