"""Pinhole 카메라 모델 (ROS 비의존).

좌표는 ROS camera optical frame 관례를 따른다: Z 전방, X 오른쪽, Y 아래.
픽셀 (u, v)는 영상 좌상단이 원점이고 u는 오른쪽, v는 아래로 증가한다.
CameraInfo K와 같은 ROS/OpenCV 관례로, 픽셀 i의 중심이 정수 i다.
YOLO bbox(픽셀 경계 관례)의 중심은 depth_sampling.bbox_center_to_pixel로 바꿔서 넘긴다.
왜곡이 보정된(rectified) 영상을 가정한다.
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple, Union

import numpy as np


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f'{name}는 유한한 값이어야 한다: {value}')


@dataclass(frozen=True)
class PinholeCamera:
    """카메라 내부 파라미터. K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]."""

    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        _require_finite(fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(f'초점거리는 양수여야 한다: fx={self.fx}, fy={self.fy}')

    @classmethod
    def from_k(cls, k: Union[Sequence[float], np.ndarray]) -> 'PinholeCamera':
        """K에서 내부 파라미터를 읽는다.

        k: 행 우선 9개 원소(CameraInfo.k와 같은 순서) 또는 3x3 행렬.
        """
        k = np.asarray(k, dtype=float)
        if k.shape not in ((9,), (3, 3)):
            raise ValueError(f'K는 9개 원소 또는 3x3이어야 한다: shape={k.shape}')
        k = k.reshape(3, 3)
        return cls(fx=float(k[0, 0]), fy=float(k[1, 1]), cx=float(k[0, 2]), cy=float(k[1, 2]))

    def backproject(self, u: float, v: float, z: float) -> np.ndarray:
        """픽셀 (u, v)와 깊이 z [m] → optical frame 좌표 [X, Y, Z] [m].

        X = (u - cx) * z / fx,  Y = (v - cy) * z / fy,  Z = z
        """
        _require_finite(u=u, v=v, z=z)
        if z <= 0.0:
            raise ValueError(f'깊이는 양수여야 한다: z={z}')
        return np.array([
            (u - self.cx) * z / self.fx,
            (v - self.cy) * z / self.fy,
            float(z),
        ])

    def project(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """optical frame 좌표 (X, Y, Z) [m] → 픽셀 (u, v).

        u = fx * X / Z + cx,  v = fy * Y / Z + cy
        """
        _require_finite(x=x, y=y, z=z)
        if z <= 0.0:
            raise ValueError(f'카메라 앞(Z > 0)의 점만 투영할 수 있다: Z={z}')
        return self.fx * x / z + self.cx, self.fy * y / z + self.cy
