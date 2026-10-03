import math

import numpy as np
import pytest

from amr_odometry.kinematics import body_motion, DiffDriveOdometry, normalize_angle, Pose2D

B = 0.34


def drive(odom, d_left, d_right, steps):
    for _ in range(steps):
        odom.update(d_left, d_right)


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi),
    (2 * math.pi + 0.1, 0.1), (-0.1, -0.1),
])
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def test_body_motion():
    assert body_motion(1.0, 1.0, B) == pytest.approx((1.0, 0.0))
    assert body_motion(-0.17, 0.17, B) == pytest.approx((0.0, 1.0))


def test_straight_line():
    odom = DiffDriveOdometry(B)
    drive(odom, 0.01, 0.01, 1000)
    assert (odom.pose.x, odom.pose.y, odom.pose.yaw) == pytest.approx((10.0, 0.0, 0.0))


def test_rotate_in_place_full_turn():
    odom = DiffDriveOdometry(B)
    # 바퀴가 반대로 같은 거리 → 제자리 회전. 한 바퀴(2π)는 좌우 거리 차이 합이 2π·B
    step = 2 * math.pi * B / 1000
    drive(odom, -step / 2, step / 2, 1000)
    assert odom.pose.x == pytest.approx(0.0, abs=1e-9)
    assert odom.pose.y == pytest.approx(0.0, abs=1e-9)
    assert odom.pose.yaw == pytest.approx(0.0, abs=1e-9)


def test_arc_matches_closed_form():
    # 반지름 R 원호를 1/4 바퀴 → (R, R, π/2)
    radius, n = 2.0, 500
    arc = math.pi / 2 * radius / n
    d_left = arc * (radius - B / 2) / radius
    d_right = arc * (radius + B / 2) / radius
    odom = DiffDriveOdometry(B)
    drive(odom, d_left, d_right, n)
    assert odom.pose.x == pytest.approx(radius, abs=1e-5)
    assert odom.pose.y == pytest.approx(radius, abs=1e-5)
    assert odom.pose.yaw == pytest.approx(math.pi / 2)


def test_start_pose_and_heading():
    odom = DiffDriveOdometry(B)
    odom.reset(Pose2D(1.0, 2.0, math.pi / 2))
    drive(odom, 0.01, 0.01, 100)
    assert (odom.pose.x, odom.pose.y) == pytest.approx((1.0, 3.0))


def test_covariance_zero_without_noise():
    odom = DiffDriveOdometry(B, distance_noise_k=0.0)
    drive(odom, 0.01, 0.02, 100)
    assert np.allclose(odom.covariance, 0.0)


def test_covariance_grows_and_stays_symmetric_psd():
    odom = DiffDriveOdometry(B, distance_noise_k=0.032)
    traces = []
    for _ in range(5):
        drive(odom, 0.01, 0.012, 100)
        traces.append(np.trace(odom.covariance))
        assert np.allclose(odom.covariance, odom.covariance.T)
        assert np.all(np.linalg.eigvalsh(odom.covariance) >= -1e-15)
    assert all(b > a for a, b in zip(traces, traces[1:]))


def test_straight_line_along_track_variance_matches_k2_d():
    # 직진에서 전방(x) 분산 = k²·D/2 (좌우 바퀴 분산 k²·D가 1/2씩 평균된다)
    k, distance = 0.032, 10.0
    odom = DiffDriveOdometry(B, distance_noise_k=k)
    drive(odom, 0.01, 0.01, 1000)
    assert odom.covariance[0, 0] == pytest.approx(k ** 2 * distance / 2)


def test_covariance_independent_of_step_count():
    # 같은 경로를 다른 주기로 적분해도 공분산이 같아야 한다 (분산 ∝ 거리)
    k = 0.032
    coarse, fine = DiffDriveOdometry(B, k), DiffDriveOdometry(B, k)
    drive(coarse, 0.02, 0.024, 250)
    drive(fine, 0.005, 0.006, 1000)
    assert np.allclose(coarse.covariance, fine.covariance, rtol=0.01)


def test_twist_variance():
    odom = DiffDriveOdometry(B, distance_noise_k=0.032)
    var_v, var_w = odom.twist_variance(0.02, 0.02, 0.02)
    assert var_v == pytest.approx(0.032 ** 2 * 0.04 / 4 / 0.02 ** 2)
    assert var_w == pytest.approx(0.032 ** 2 * 0.04 / B ** 2 / 0.02 ** 2)
    assert odom.twist_variance(0.0, 0.0, 0.02) == (0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'wheel_separation': 0.0}, {'wheel_separation': B, 'distance_noise_k': -1},
])
def test_invalid_arguments(kwargs):
    with pytest.raises(ValueError):
        DiffDriveOdometry(**kwargs)


def test_twist_variance_rejects_non_positive_dt():
    with pytest.raises(ValueError):
        DiffDriveOdometry(B).twist_variance(0.0, 0.0, 0.0)
