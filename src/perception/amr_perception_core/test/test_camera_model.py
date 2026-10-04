import math

import numpy as np
import pytest

from amr_perception_core.camera_model import PinholeCamera

# 640x480, 수평 FOV 87도 (명세 depth 카메라): fx = 320 / tan(43.5°)
FX = 320.0 / math.tan(math.radians(43.5))
CAMERA = PinholeCamera(fx=FX, fy=FX, cx=320.0, cy=240.0)
K_ROWS = [[500.0, 0.0, 321.5], [0.0, 510.0, 239.5], [0.0, 0.0, 1.0]]
EXPECTED = PinholeCamera(fx=500.0, fy=510.0, cx=321.5, cy=239.5)


def test_from_k_parses_camera_info_order():
    assert PinholeCamera.from_k(np.ravel(K_ROWS).tolist()) == EXPECTED


def test_from_k_parses_3x3_matrix():
    assert PinholeCamera.from_k(np.array(K_ROWS)) == EXPECTED


@pytest.mark.parametrize('k', [[1.0] * 8, [1.0] * 10, np.eye(2), np.ones((3, 4))])
def test_from_k_rejects_wrong_shape(k):
    with pytest.raises(ValueError):
        PinholeCamera.from_k(k)


@pytest.mark.parametrize('fx, fy', [(0.0, 500.0), (500.0, 0.0), (-1.0, 500.0), (500.0, -1.0)])
def test_rejects_non_positive_focal_length(fx, fy):
    with pytest.raises(ValueError):
        PinholeCamera(fx=fx, fy=fy, cx=320.0, cy=240.0)


def test_from_k_rejects_non_positive_focal_length():
    with pytest.raises(ValueError):
        PinholeCamera.from_k([0.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0])


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_rejects_non_finite_intrinsics(bad):
    with pytest.raises(ValueError):
        PinholeCamera(fx=500.0, fy=500.0, cx=bad, cy=240.0)
    with pytest.raises(ValueError):
        PinholeCamera.from_k([bad, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0])


def test_principal_point_backprojects_onto_optical_axis():
    np.testing.assert_allclose(CAMERA.backproject(320.0, 240.0, 2.5), [0.0, 0.0, 2.5], atol=1e-12)


def test_backproject_formula():
    camera = PinholeCamera(fx=500.0, fy=400.0, cx=320.0, cy=240.0)
    np.testing.assert_allclose(camera.backproject(420.0, 140.0, 2.0), [0.4, -0.5, 2.0])


def test_optical_frame_axes_right_and_down_are_positive():
    x, y, _ = CAMERA.backproject(400.0, 300.0, 1.0)
    assert x > 0.0
    assert y > 0.0


def test_image_edge_matches_half_fov():
    x, _, z = CAMERA.backproject(640.0, 240.0, 3.0)
    assert math.degrees(math.atan2(x, z)) == pytest.approx(43.5)


@pytest.mark.parametrize('point', [(0.0, 0.0, 1.0), (0.8, -0.3, 3.0), (-1.2, 0.4, 7.5)])
def test_project_then_backproject_recovers_point(point):
    u, v = CAMERA.project(*point)
    np.testing.assert_allclose(CAMERA.backproject(u, v, point[2]), point, atol=1e-9)


@pytest.mark.parametrize('u, v, z', [(0.0, 0.0, 0.5), (123.4, 456.7, 3.2), (639.0, 1.0, 9.9)])
def test_backproject_then_project_recovers_pixel(u, v, z):
    assert CAMERA.project(*CAMERA.backproject(u, v, z)) == pytest.approx((u, v))


@pytest.mark.parametrize('z', [0.0, -1.0])
def test_rejects_non_positive_depth(z):
    with pytest.raises(ValueError):
        CAMERA.backproject(320.0, 240.0, z)
    with pytest.raises(ValueError):
        CAMERA.project(0.0, 0.0, z)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_backproject_rejects_non_finite_input(bad):
    for args in [(bad, 240.0, 1.0), (320.0, bad, 1.0), (320.0, 240.0, bad)]:
        with pytest.raises(ValueError):
            CAMERA.backproject(*args)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_project_rejects_non_finite_input(bad):
    for args in [(bad, 0.0, 1.0), (0.0, bad, 1.0), (0.0, 0.0, bad)]:
        with pytest.raises(ValueError):
            CAMERA.project(*args)
