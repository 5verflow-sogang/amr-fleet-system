"""Pinhole 역투영의 측정 공분산과 좌표 변환 (ROS 비의존).

YOLO bbox 중심 (u, v)와 대표 깊이 z를 3D 위치로 바꿀 때, 픽셀 오차 σ_px와 깊이 오차 σ_z가
위치에 얼마나 퍼지는지를 야코비안으로 전파한다. 같은 픽셀 오차도 멀수록 큰 위치 오차가 된다
(옆 방향 ≈ σ_px · Z / f). 이 공분산이 DetectedObject3D.pose.covariance가 되고, 추적 KF의 R로 쓰인다.
"""

import math
from typing import Tuple

import numpy as np

from .camera_model import PinholeCamera


def backproject_with_covariance(camera: PinholeCamera, u: float, v: float, z: float,
                                sigma_px: float, sigma_z: float) -> Tuple[np.ndarray, np.ndarray]:
    """픽셀 (u, v), 깊이 z [m] → optical frame 위치 [X, Y, Z]와 3x3 공분산.

    J = ∂(X, Y, Z)/∂(u, v, z),  Σ = J · diag(σ_px², σ_px², σ_z²) · Jᵀ
    u, v는 CameraInfo K 관례(픽셀 i의 중심 = i)다. YOLO bbox 중심은 bbox_center_to_pixel로 바꿔 넘긴다.
    """
    for name, sigma in (('sigma_px', sigma_px), ('sigma_z', sigma_z)):
        if not (math.isfinite(sigma) and sigma >= 0.0):
            raise ValueError(f'{name}는 0 이상의 유한한 값이어야 한다: {sigma}')
    point = camera.backproject(u, v, z)
    jacobian = np.array([
        [z / camera.fx, 0.0, (u - camera.cx) / camera.fx],
        [0.0, z / camera.fy, (v - camera.cy) / camera.fy],
        [0.0, 0.0, 1.0],
    ])
    noise = np.diag([sigma_px ** 2, sigma_px ** 2, sigma_z ** 2])
    return point, jacobian @ noise @ jacobian.T


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언 (x, y, z, w) → 3x3 회전 행렬. TF 메시지 값이 정확히 단위 길이가 아닐 수 있어 정규화한다."""
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not (math.isfinite(norm) and norm > 0.0):
        raise ValueError(f'쿼터니언 크기가 0이거나 유한하지 않다: {(x, y, z, w)}')
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def transform_with_covariance(rotation: np.ndarray, translation: np.ndarray, point: np.ndarray,
                              covariance: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """강체 변환: p' = R p + t,  Σ' = R Σ Rᵀ. 이동은 공분산을 바꾸지 않고 회전은 축만 돌린다."""
    rotation = np.asarray(rotation, dtype=float)
    covariance = np.asarray(covariance, dtype=float)
    if rotation.shape != (3, 3) or covariance.shape != (3, 3):
        raise ValueError(f'회전·공분산은 3x3이어야 한다: {rotation.shape}, {covariance.shape}')
    point = np.asarray(point, dtype=float).reshape(3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    return rotation @ point + translation, rotation @ covariance @ rotation.T
