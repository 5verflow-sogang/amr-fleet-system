"""휠 엔코더 측정 모델 (ROS 비의존).

Gazebo에는 엔코더 센서가 없어 joint_states의 실제 바퀴 각도에 아래 오차를 넣는다.
값과 근거는 config/odometry.yaml과 PR #26 (민지 리뷰) 참고.

1. 양자화: 각도를 2π/ticks_per_rev 단위로 내림한다 (엔코더 카운트).
2. 계통 오차: 좌우 바퀴 반지름 배율 (1 + N(0, σ))을 실행마다 한 번 뽑아 고정한다.
3. 무작위 오차: 바퀴마다 한 주기 이동 거리 Δs에 N(0, k²·|Δs|)를 더한다.
   분산이 이동 거리에 비례하므로 누적 σ = k·√D이고 갱신 주기와 무관하다.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from amr_odometry.kinematics import normalize_angle

LEFT, RIGHT = 0, 1


@dataclass(frozen=True)
class EncoderNoiseConfig:
    ticks_per_rev: int = 4096
    wheel_radius_scale_stddev: float = 0.005
    distance_noise_k: float = 0.01
    # 0이면 실행마다 무작위. 하네스 반복 실험에서는 실행별로 지정해 재현한다
    seed: int = 0
    # False면 양자화만 하고 계통·무작위 오차는 넣지 않는다
    noise_enabled: bool = True

    def __post_init__(self):
        if self.ticks_per_rev <= 0:
            raise ValueError(f'ticks_per_rev는 양수여야 한다: {self.ticks_per_rev}')
        if self.wheel_radius_scale_stddev < 0.0 or self.distance_noise_k < 0.0:
            raise ValueError('노이즈 표준편차는 0 이상이어야 한다')
        if self.seed < 0:
            raise ValueError(f'seed는 0 이상이어야 한다: {self.seed}')


class EncoderModel:
    """좌우 바퀴의 연속 각도 → 엔코더로 잰 이동 거리."""

    def __init__(self, wheel_radius: float, config: EncoderNoiseConfig = EncoderNoiseConfig(),
                 rng: Optional[np.random.Generator] = None):
        if not wheel_radius > 0.0:
            raise ValueError(f'바퀴 반지름은 양수여야 한다: {wheel_radius}')
        self.wheel_radius = wheel_radius
        self.config = config
        self.tick = 2.0 * math.pi / config.ticks_per_rev
        if rng is None:
            rng = np.random.default_rng(config.seed if config.seed > 0 else None)
        self.rng = rng
        if config.noise_enabled:
            self.radius_scale = 1.0 + rng.normal(0.0, config.wheel_radius_scale_stddev, size=2)
        else:
            self.radius_scale = np.ones(2)
        self._last_ticks = [None, None]
        self._last_raw = [None, None]
        self._unwrapped = [0.0, 0.0]

    def quantize(self, angle: float) -> int:
        """연속 각도 [rad] → 엔코더 카운트."""
        return math.floor(angle / self.tick)

    def _unwrap(self, wheel: int, angle: float) -> float:
        # joint_states 각도가 ±π에서 넘어가도 누적 각도가 끊기지 않게 한다.
        # 한 주기 회전량은 π보다 훨씬 작다 (최대 20 rad/s × 0.02 s = 0.4 rad)
        last = self._last_raw[wheel]
        self._last_raw[wheel] = angle
        if last is None:
            self._unwrapped[wheel] = angle
        else:
            self._unwrapped[wheel] += normalize_angle(angle - last)
        return self._unwrapped[wheel]

    def measure(self, wheel: int, angle: float) -> float:
        """바퀴 각도 [rad]를 받아 직전 측정 이후 굴러간 거리 [m]를 돌려준다. 첫 호출은 0."""
        ticks = self.quantize(self._unwrap(wheel, angle))
        last = self._last_ticks[wheel]
        self._last_ticks[wheel] = ticks
        if last is None:
            return 0.0
        ds = (ticks - last) * self.tick * self.wheel_radius * self.radius_scale[wheel]
        if self.config.noise_enabled and ds != 0.0:
            ds += self.rng.normal(0.0, self.config.distance_noise_k * math.sqrt(abs(ds)))
        return ds
