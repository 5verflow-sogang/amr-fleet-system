import math

import numpy as np
import pytest

from amr_odometry.imu_filter import (
    BiasCalibrator, GRAVITY, ImuFilter, ImuFilterConfig, LowPass3,
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
    f = ImuFilter()
    for t, g, a in samples(250, rng):
        f.update(t, g, a)
    out = None
    for t, g, a in samples(50, rng, gyro=(0.0, 0.0, 1.0), t0=2.5):
        out = f.update(t, g, a)
    # 20 Hz 저역통과는 0.5 s 안에 1 rad/s를 따라간다
    assert out[0][2] == pytest.approx(1.0, abs=0.01)


@pytest.mark.parametrize('kwargs', [
    {'gyro_cutoff_hz': -1.0}, {'accel_cutoff_hz': -1.0}, {'calibration_duration': 0.0},
])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ImuFilterConfig(**kwargs)
