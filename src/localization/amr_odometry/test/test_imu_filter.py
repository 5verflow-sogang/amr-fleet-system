import math

import numpy as np
import pytest

from amr_odometry.imu_filter import (
    BiasCalibrator, calibrated_variance, GRAVITY, ImuFilter, ImuFilterConfig, LowPass3,
)

DT = 0.01   # 100 Hz
GYRO_BIAS = np.array([7.5e-6, -7.5e-6, 7.5e-6])
ACCEL_BIAS = np.array([0.1, -0.1, 0.1])


def samples(n, rng, gyro=(0.0, 0.0, 0.0), gyro_sigma=2e-4, accel_sigma=1.7e-2, t0=0.0):
    for i in range(n):
        g = np.array(gyro) + GYRO_BIAS + rng.normal(0, gyro_sigma, 3)
        a = np.array([0.0, 0.0, GRAVITY]) + ACCEL_BIAS + rng.normal(0, accel_sigma, 3)
        yield t0 + i * DT, g, a


def test_lowpass_first_sample_passes_through():
    lp = LowPass3(5.0)
    assert np.allclose(lp.update([1.0, 2.0, 3.0], DT), [1.0, 2.0, 3.0])


def test_lowpass_step_response_time_constant():
    # 1차 필터: τ 뒤 63.2%에 도달
    cutoff = 5.0
    tau = 1 / (2 * math.pi * cutoff)
    lp = LowPass3(cutoff)
    lp.update([0.0] * 3, DT)
    dt = 1e-4
    y = None
    for _ in range(int(round(tau / dt))):
        y = lp.update([1.0] * 3, dt)
    assert y[0] == pytest.approx(1 - math.exp(-1), abs=0.01)


def test_lowpass_reduces_noise():
    rng = np.random.default_rng(0)
    lp = LowPass3(5.0)
    x = rng.normal(0, 1.0, (2000, 3))
    y = np.array([lp.update(v, DT) for v in x])
    assert y[100:].std() < 0.5 * x.std()


def test_lowpass_disabled_when_cutoff_zero():
    lp = LowPass3(0.0)
    lp.update([0.0] * 3, DT)
    assert np.allclose(lp.update([1.0] * 3, DT), [1.0] * 3)


def test_calibration_recovers_bias():
    # 2초(200 샘플) 평균의 표준오차: 자이로 σ/√200 ≈ 1.4e-5, 가속도 ≈ 1.2e-3 → 허용 오차는 5배
    rng = np.random.default_rng(1)
    cal = BiasCalibrator(ImuFilterConfig(calibration_duration=2.0))
    done = False
    for t, g, a in samples(300, rng):
        done = cal.add(t, g, a)
        if done:
            break
    assert done
    assert np.allclose(cal.gyro_bias, GYRO_BIAS, atol=7e-5)
    assert np.allclose(cal.accel_bias, ACCEL_BIAS, atol=6e-3)


def test_calibration_restarts_when_moving():
    rng = np.random.default_rng(2)
    cal = BiasCalibrator(ImuFilterConfig(calibration_duration=1.0))
    # 처음 1초는 회전 중(각속도가 바뀜) → 정지로 보지 않는다
    for i, (t, g, a) in enumerate(samples(110, rng)):
        g = g + np.array([0.0, 0.0, 0.5 * math.sin(i * 0.2)])
        cal.add(t, g, a)
    assert not cal.done and cal.restarts >= 1
    # 다시 잰 구간에 움직인 샘플이 섞여 한 번 더 버려질 수 있으므로 넉넉히 정지 샘플을 넣는다
    for t, g, a in samples(350, rng, t0=1.2):
        cal.add(t, g, a)
    assert cal.done
    assert np.allclose(cal.gyro_bias, GYRO_BIAS, atol=7e-5)


def test_calibration_rejects_constant_rotation():
    # 일정 속도 회전은 흔들림이 작아도 정지가 아니다 → 회전 속도를 바이어스로 저장하면 안 된다
    rng = np.random.default_rng(8)
    cal = BiasCalibrator(ImuFilterConfig(calibration_duration=1.0))
    for t, g, a in samples(250, rng, gyro=(0.0, 0.0, 0.5)):
        cal.add(t, g, a)
    assert not cal.done and cal.restarts >= 2


def test_calibration_rejects_constant_acceleration():
    rng = np.random.default_rng(9)
    cal = BiasCalibrator(ImuFilterConfig(calibration_duration=1.0))
    for t, g, a in samples(250, rng):
        cal.add(t, g, a + np.array([1.0, 0.0, 0.0]))   # 1 m/s² 가속 중
    assert not cal.done


def test_filter_outputs_nothing_until_calibrated_then_unbiased():
    rng = np.random.default_rng(3)
    f = ImuFilter(ImuFilterConfig(calibration_duration=2.0))
    outputs = [f.update(t, g, a) for t, g, a in samples(400, rng)]
    first = next(i for i, o in enumerate(outputs) if o is not None)
    assert 195 <= first <= 210
    gyro = np.array([o[0] for o in outputs[first + 50:]])
    accel = np.array([o[1] for o in outputs[first + 50:]])
    assert np.allclose(gyro.mean(axis=0), 0.0, atol=7e-5)
    assert np.allclose(accel.mean(axis=0), [0.0, 0.0, GRAVITY], atol=6e-3)


def test_filter_tracks_rotation_after_calibration():
    rng = np.random.default_rng(4)
    f = ImuFilter(ImuFilterConfig(gyro_cutoff_hz=20.0))
    for t, g, a in samples(250, rng):
        f.update(t, g, a)
    out = None
    for t, g, a in samples(50, rng, gyro=(0.0, 0.0, 1.0), t0=2.5):
        out = f.update(t, g, a)
    # 20 Hz 저역통과를 켜도 0.5 s 안에 1 rad/s를 따라간다
    assert out[0][2] == pytest.approx(1.0, abs=0.01)


def test_gyro_lowpass_off_by_default_has_no_lag():
    # 기본값은 자이로 저역통과를 끈다 (#76·#81): 각속도가 바뀐 첫 샘플부터 그대로 나온다
    assert ImuFilterConfig().gyro_cutoff_hz == 0.0
    rng = np.random.default_rng(8)
    f = ImuFilter()
    for t, g, a in samples(250, rng):
        f.update(t, g, a)
    t, g, a = next(samples(1, rng, gyro=(0.0, 0.0, 1.0), t0=2.5))
    out = f.update(t, g, a)
    assert out[0][2] == pytest.approx(1.0, abs=1e-3)   # 노이즈 σ 2e-4의 5배 안


def test_calibrated_variance():
    # 노이즈 σ² × (1 + 1/N) + floor
    assert calibrated_variance(2e-4, 200) == pytest.approx(4e-8 * (1 + 1 / 200))
    assert calibrated_variance(2e-4, 200, 1e-7) == pytest.approx(4e-8 * (1 + 1 / 200) + 1e-7)
    # 샘플 수 0은 1로 본다
    assert calibrated_variance(1.0, 0) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        calibrated_variance(-1.0, 10)
    with pytest.raises(ValueError):
        calibrated_variance(1.0, 10, -1e-7)


def moving(n, rng, t0=0.0):
    """회전 속도가 계속 바뀌는(정지가 아닌) 샘플."""
    for i, (t, g, a) in enumerate(samples(n, rng, t0=t0)):
        yield t, g + np.array([0.0, 0.0, 0.5 * math.sin(i * 0.2)]), a


def test_fallback_publishes_uncalibrated_after_max_attempts():
    rng = np.random.default_rng(5)
    f = ImuFilter(ImuFilterConfig(calibration_duration=0.5, max_calibration_attempts=3))
    outputs = [f.update(t, g, a) for t, g, a in moving(250, rng)]
    first = next(i for i, o in enumerate(outputs) if o is not None)
    # 0.5 s × 3번 실패 → 약 1.5 s(150 샘플) 뒤부터 발행
    assert 140 <= first <= 170
    assert f.uncalibrated and not f.calibrator.done
    # 바이어스를 빼지 않으므로 가속도 z에 바이어스 0.1이 그대로 남는다
    accel_z = np.array([o[1][2] for o in outputs[first + 20:]])
    assert accel_z.mean() == pytest.approx(GRAVITY + ACCEL_BIAS[2], abs=0.01)


def test_fallback_switches_to_calibrated_when_stationary():
    rng = np.random.default_rng(6)
    f = ImuFilter(ImuFilterConfig(calibration_duration=0.5, max_calibration_attempts=2))
    for t, g, a in moving(150, rng):
        f.update(t, g, a)
    assert f.uncalibrated
    out = [f.update(t, g, a) for t, g, a in samples(300, rng, t0=1.5)]
    assert f.calibrator.done and not f.uncalibrated
    accel_z = np.array([o[1][2] for o in out[-100:]])
    assert accel_z.mean() == pytest.approx(GRAVITY, abs=6e-3)


def test_fallback_disabled_when_zero_attempts():
    rng = np.random.default_rng(7)
    f = ImuFilter(ImuFilterConfig(calibration_duration=0.5, max_calibration_attempts=0))
    assert all(f.update(t, g, a) is None for t, g, a in moving(500, rng))
    assert not f.uncalibrated


@pytest.mark.parametrize('kwargs', [
    {'gyro_cutoff_hz': -1.0}, {'accel_cutoff_hz': -1.0}, {'calibration_duration': 0.0},
    {'max_calibration_attempts': -1}, {'uncalibrated_gyro_bias_stddev': -0.1},
    {'uncalibrated_accel_bias_stddev': -0.1}, {'stationary_gyro_mean_max': -0.1},
    {'stationary_accel_mean_tolerance': -0.1},
])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ImuFilterConfig(**kwargs)
