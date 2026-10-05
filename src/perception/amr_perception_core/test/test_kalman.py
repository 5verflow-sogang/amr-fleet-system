import math

import numpy as np
import pytest

from amr_perception_core.kalman import (
    ConstantVelocityKF,
    KalmanConfig,
    process_noise,
    transition_matrix,
)

SMOOTH = KalmanConfig(accel_noise_std=0.2)  # 진실이 정확한 등속일 때 쓰는 작은 σa


def simulate_cv(start, velocity, times, noise_std, rng):
    """등속 진실 궤적과 잡음 섞인 위치 관측."""
    truth = np.asarray(start, dtype=float) + np.outer(times, velocity)
    return truth, truth + rng.normal(0.0, noise_std, size=truth.shape)


def run(kf, times, measurements):
    for t, z in zip(times, measurements):
        kf.predict(t)
        kf.update(z)


def test_initial_state():
    kf = ConstantVelocityKF((1.0, 2.0), timestamp=0.0)
    np.testing.assert_allclose(kf.position, [1.0, 2.0])
    np.testing.assert_allclose(kf.velocity, [0.0, 0.0])
    np.testing.assert_allclose(np.diag(kf.P), [0.05 ** 2] * 2 + [1.5 ** 2] * 2)


def test_transition_matrix_moves_position_by_velocity():
    np.testing.assert_allclose(transition_matrix(0.5) @ [1, 2, 3, 4], [2.5, 4.0, 3, 4])


def test_process_noise_blocks():
    q = process_noise(0.1, 2.0)
    assert q[0, 0] == pytest.approx(4.0 * 0.1 ** 4 / 4)
    assert q[0, 2] == pytest.approx(4.0 * 0.1 ** 3 / 2)
    assert q[2, 2] == pytest.approx(4.0 * 0.1 ** 2)
    assert q[0, 1] == 0.0
    np.testing.assert_allclose(q, q.T)


def test_predict_moves_with_velocity_and_grows_covariance():
    kf = ConstantVelocityKF((0.0, 0.0), timestamp=0.0)
    kf.x[2:] = [1.0, -0.5]
    trace_before = np.trace(kf.P)
    kf.predict(2.0)
    np.testing.assert_allclose(kf.position, [2.0, -1.0])
    assert np.trace(kf.P) > trace_before


def test_update_shrinks_position_covariance():
    kf = ConstantVelocityKF((0.0, 0.0), timestamp=0.0)
    kf.predict(0.1)
    before = np.trace(kf.position_covariance)
    kf.update((0.02, -0.01))
    assert np.trace(kf.position_covariance) < before


def test_larger_measurement_noise_moves_estimate_less():
    trusted = ConstantVelocityKF((0.0, 0.0), 0.0)
    doubted = ConstantVelocityKF((0.0, 0.0), 0.0)
    trusted.update((1.0, 0.0))
    doubted.update((1.0, 0.0), r=np.eye(2) * 1.0)
    assert doubted.position[0] < trusted.position[0]


def test_same_timestamp_predict_is_noop():
    kf = ConstantVelocityKF((1.0, 1.0), timestamp=3.0)
    p = kf.P.copy()
    kf.predict(3.0)
    np.testing.assert_array_equal(kf.P, p)


def test_rejects_time_going_backwards():
    kf = ConstantVelocityKF((0.0, 0.0), timestamp=1.0)
    with pytest.raises(ValueError):
        kf.predict(0.9)


def test_velocity_converges_for_constant_velocity_target():
    rng = np.random.default_rng(1)
    times = np.arange(60) * 0.1
    truth, meas = simulate_cv((0.0, 0.0), (1.0, -0.5), times, 0.05, rng)
    kf = ConstantVelocityKF(meas[0], times[0], SMOOTH)
    run(kf, times[1:], meas[1:])
    np.testing.assert_allclose(kf.velocity, [1.0, -0.5], atol=0.1)
    np.testing.assert_allclose(kf.position, truth[-1], atol=0.1)
    assert kf.speed == pytest.approx(math.hypot(1.0, 0.5), abs=0.1)
    assert kf.heading == pytest.approx(math.atan2(-0.5, 1.0), abs=0.1)


def test_irregular_time_steps():
    # LiDAR 프레임 누락처럼 Δt가 들쭉날쭉해도 수렴한다
    rng = np.random.default_rng(2)
    times = np.cumsum(rng.uniform(0.05, 0.3, size=50))
    _, meas = simulate_cv((0.0, 0.0), (0.8, 0.3), times, 0.05, rng)
    kf = ConstantVelocityKF(meas[0], times[0], SMOOTH)
    run(kf, times[1:], meas[1:])
    np.testing.assert_allclose(kf.velocity, [0.8, 0.3], atol=0.1)


def test_covariance_stays_symmetric_positive_definite():
    rng = np.random.default_rng(3)
    kf = ConstantVelocityKF((0.0, 0.0), 0.0)
    for k in range(1, 300):
        kf.predict(k * 0.1)
        kf.update(rng.normal(0.0, 1.0, size=2))
    np.testing.assert_allclose(kf.P, kf.P.T)
    assert np.all(np.linalg.eigvalsh(kf.P) > 0.0)


def test_nees_is_consistent():
    """진실 궤적을 필터와 같은 잡음 모델로 만들면 평균 NEES ≈ 상태 차원(4).

    NEES = eᵀ P⁻¹ e. 작으면 P가 과대(보수적), 크면 P가 과소(자신 과잉)라는 뜻이다.
    """
    config = KalmanConfig()
    rng = np.random.default_rng(4)
    dt, steps, runs = 0.1, 30, 300
    f = transition_matrix(dt)
    g = np.array([[dt ** 2 / 2, 0.0], [0.0, dt ** 2 / 2], [dt, 0.0], [0.0, dt]])
    nees = []
    for _ in range(runs):
        x = np.array([0.0, 0.0, *rng.normal(0.0, config.initial_velocity_std, 2)])
        z0 = x[:2] + rng.normal(0.0, config.measurement_noise_std, 2)
        kf = ConstantVelocityKF(z0, 0.0, config)
        for k in range(1, steps):
            x = f @ x + g @ rng.normal(0.0, config.accel_noise_std, 2)
            kf.predict(k * dt)
            kf.update(x[:2] + rng.normal(0.0, config.measurement_noise_std, 2))
        e = x - kf.x
        nees.append(e @ np.linalg.solve(kf.P, e))
    # 300회 평균의 표준편차 ≈ sqrt(8/300) ≈ 0.16 → 4 ± 0.6은 약 ±3.7σ
    assert 3.4 <= np.mean(nees) <= 4.6


def test_mahalanobis_zero_at_prediction():
    kf = ConstantVelocityKF((1.0, 2.0), 0.0)
    assert kf.mahalanobis_sq((1.0, 2.0)) == pytest.approx(0.0)
    assert kf.mahalanobis_sq((2.0, 2.0)) > 9.21


@pytest.mark.parametrize('bad', [(np.nan, 0.0), (0.0, np.inf), (1.0, 2.0, 3.0)])
def test_rejects_invalid_position(bad):
    with pytest.raises(ValueError):
        ConstantVelocityKF(bad, 0.0)
    with pytest.raises(ValueError):
        ConstantVelocityKF((0.0, 0.0), 0.0).update(bad)


@pytest.mark.parametrize('kwargs', [
    {'accel_noise_std': 0.0},
    {'measurement_noise_std': -0.1},
    {'initial_velocity_std': np.nan},
])
def test_rejects_invalid_config(kwargs):
    with pytest.raises(ValueError):
        KalmanConfig(**kwargs)
