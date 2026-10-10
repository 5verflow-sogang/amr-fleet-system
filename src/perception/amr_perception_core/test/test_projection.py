import math

import numpy as np
import pytest

from amr_perception_core.camera_model import PinholeCamera
from amr_perception_core.projection import (backproject_with_covariance, quaternion_to_matrix,
                                            transform_with_covariance)

# 640x480, 수평 FOV 87도 (명세 depth 카메라): fx = 320 / tan(43.5°) ≈ 337 px
FX = 320.0 / math.tan(math.radians(43.5))
CAMERA = PinholeCamera(fx=FX, fy=FX, cx=320.0, cy=240.0)
SIGMA_PX, SIGMA_Z = 5.0, 0.02   # bbox 중심 오차 가정, sensors.yaml depth 잡음


def test_position_matches_backproject():
    point, _ = backproject_with_covariance(CAMERA, 500.0, 100.0, 4.0, SIGMA_PX, SIGMA_Z)
    np.testing.assert_allclose(point, CAMERA.backproject(500.0, 100.0, 4.0))


def test_image_center_keeps_axes_independent():
    # 광축 위에서는 깊이 오차가 옆으로 새지 않는다
    z = 5.0
    _, cov = backproject_with_covariance(CAMERA, CAMERA.cx, CAMERA.cy, z, SIGMA_PX, SIGMA_Z)
    lateral = (SIGMA_PX * z / FX) ** 2
    np.testing.assert_allclose(cov, np.diag([lateral, lateral, SIGMA_Z ** 2]), atol=1e-15)


def test_lateral_error_grows_with_distance():
    _, near = backproject_with_covariance(CAMERA, 320.0, 240.0, 1.0, SIGMA_PX, SIGMA_Z)
    _, far = backproject_with_covariance(CAMERA, 320.0, 240.0, 10.0, SIGMA_PX, SIGMA_Z)
    assert math.sqrt(far[0, 0]) == pytest.approx(10.0 * math.sqrt(near[0, 0]))
    # 5 px, 5 m → 옆 방향 σ 약 7.4 cm
    _, mid = backproject_with_covariance(CAMERA, 320.0, 240.0, 5.0, SIGMA_PX, SIGMA_Z)
    assert math.sqrt(mid[0, 0]) == pytest.approx(0.074, abs=0.001)


def test_covariance_matches_monte_carlo():
    # 영상 가장자리(깊이-옆 방향 상관이 큰 곳)에서 표본 공분산과 선형 전파가 맞는지
    u, v, z = 600.0, 60.0, 5.0
    _, cov = backproject_with_covariance(CAMERA, u, v, z, SIGMA_PX, SIGMA_Z)
    rng = np.random.default_rng(0)
    n = 20_000
    us = u + rng.normal(0.0, SIGMA_PX, n)
    vs = v + rng.normal(0.0, SIGMA_PX, n)
    zs = z + rng.normal(0.0, SIGMA_Z, n)
    samples = np.array([CAMERA.backproject(a, b, c) for a, b, c in zip(us, vs, zs)])
    sample_cov = np.cov(samples.T)
    np.testing.assert_allclose(np.diag(sample_cov), np.diag(cov), rtol=0.05)
    # 비대각(상관)도 맞는다: 오른쪽 가장자리에서 X와 Z는 양의 상관
    assert cov[0, 2] > 0.0
    assert sample_cov[0, 2] == pytest.approx(cov[0, 2], rel=0.1)


def test_quaternion_identity_and_z90():
    np.testing.assert_allclose(quaternion_to_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3))
    s = math.sqrt(0.5)
    rz = quaternion_to_matrix(0.0, 0.0, s, s)   # z축 +90°: x축 → y축
    np.testing.assert_allclose(rz @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_quaternion_is_normalized():
    np.testing.assert_allclose(quaternion_to_matrix(0.0, 0.0, 2.0, 2.0),
                               quaternion_to_matrix(0.0, 0.0, 1.0, 1.0))
    with pytest.raises(ValueError):
        quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)


def test_optical_frame_to_base_rotation():
    # api.md: camera_link → camera_optical_frame rpy (-π/2, 0, -π/2).
    # optical frame의 전방(Z)은 camera_link의 전방(X)이고, 오른쪽(X)은 −Y다
    s = 0.5
    r = quaternion_to_matrix(-s, s, -s, s)   # optical → link
    np.testing.assert_allclose(r @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(r @ [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], atol=1e-12)


def test_transform_rotates_covariance_only():
    _, cov = backproject_with_covariance(CAMERA, 600.0, 60.0, 5.0, SIGMA_PX, SIGMA_Z)
    rot = quaternion_to_matrix(0.1, -0.3, 0.5, 0.8)
    point, out = transform_with_covariance(rot, [10.0, -2.0, 0.3], [1.0, 2.0, 3.0], cov)
    np.testing.assert_allclose(point, rot @ [1.0, 2.0, 3.0] + [10.0, -2.0, 0.3])
    np.testing.assert_allclose(np.linalg.eigvalsh(out), np.linalg.eigvalsh(cov), rtol=1e-9)
    # 이동만 있으면 공분산은 그대로
    _, moved = transform_with_covariance(np.eye(3), [5.0, 5.0, 5.0], [0.0, 0.0, 1.0], cov)
    np.testing.assert_allclose(moved, cov)


@pytest.mark.parametrize('sigma_px, sigma_z', [(-1.0, 0.02), (5.0, -0.1), (math.nan, 0.02),
                                               (5.0, math.inf)])
def test_rejects_bad_sigma(sigma_px, sigma_z):
    with pytest.raises(ValueError):
        backproject_with_covariance(CAMERA, 320.0, 240.0, 2.0, sigma_px, sigma_z)


def test_rejects_bad_shapes():
    with pytest.raises(ValueError):
        transform_with_covariance(np.eye(2), [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], np.eye(3))
