import math

import numpy as np
import pytest

from amr_odometry.encoder_model import EncoderModel, EncoderNoiseConfig, LEFT, RIGHT

R = 0.1
TICK = 2 * math.pi / 4096
IDEAL = EncoderNoiseConfig(noise_enabled=False)


def roll(model, wheel, total_angle, steps, start=0.0):
    """바퀴를 start에서 total_angle만큼 steps번에 나눠 굴린 측정 거리 합."""
    model.measure(wheel, start)
    return sum(model.measure(wheel, start + total_angle * (i + 1) / steps) for i in range(steps))


def test_first_measurement_is_zero():
    assert EncoderModel(R, IDEAL).measure(LEFT, 1.234) == 0.0


def test_quantization_floors_to_ticks():
    model = EncoderModel(R, IDEAL)
    assert model.quantize(0.0) == 0
    assert model.quantize(TICK * 0.999) == 0
    assert model.quantize(TICK * 1.001) == 1
    assert model.quantize(-TICK * 0.5) == -1


def test_ideal_encoder_distance_within_one_tick():
    model = EncoderModel(R, IDEAL)
    distance = roll(model, LEFT, 10 * 2 * math.pi, 500)
    assert distance == pytest.approx(R * 10 * 2 * math.pi, abs=R * TICK)


def test_sub_tick_motion_reads_zero_until_tick_crossed():
    model = EncoderModel(R, IDEAL)
    model.measure(RIGHT, 0.0)
    assert model.measure(RIGHT, TICK * 0.4) == 0.0
    assert model.measure(RIGHT, TICK * 0.8) == 0.0
    assert model.measure(RIGHT, TICK * 1.2) == pytest.approx(R * TICK)


def test_unwraps_angle_across_pi():
    model = EncoderModel(R, IDEAL)
    model.measure(LEFT, math.pi - 0.05)
    # Gazebo가 각도를 (-π, π]로 감아도 실제로는 0.1 rad 전진한 것
    d = model.measure(LEFT, -math.pi + 0.05)
    assert d == pytest.approx(R * 0.1, abs=R * TICK)


def test_systematic_scale_is_fixed_per_run_and_reproducible_with_seed():
    cfg = EncoderNoiseConfig(seed=42, distance_noise_k=0.0)
    a, b = EncoderModel(R, cfg), EncoderModel(R, cfg)
    assert np.array_equal(a.radius_scale, b.radius_scale)
    assert a.radius_scale[LEFT] != a.radius_scale[RIGHT]
    # k=0이면 측정 거리 = 실제 거리 × 고정 배율
    distance = roll(a, LEFT, 2 * math.pi * 10, 100)
    assert distance == pytest.approx(R * 2 * math.pi * 10 * a.radius_scale[LEFT], abs=2 * R * TICK)


def test_systematic_scale_spread_matches_config():
    scales = np.concatenate([
        EncoderModel(R, EncoderNoiseConfig(seed=s)).radius_scale for s in range(1, 2001)])
    assert scales.mean() == pytest.approx(1.0, abs=0.0005)
    assert scales.std() == pytest.approx(0.005, rel=0.05)


@pytest.mark.parametrize('steps', [100, 1000])
def test_random_error_sigma_is_k_sqrt_distance_regardless_of_rate(steps):
    # 10 m 주행 누적 오차 σ ≈ k·√10 ≈ 0.1 m, 갱신 주기(steps)와 무관해야 한다
    k = 0.032
    cfg = EncoderNoiseConfig(distance_noise_k=k, wheel_radius_scale_stddev=0.0)
    angle = 10.0 / R
    rng = np.random.default_rng(7)
    errors = [roll(EncoderModel(R, cfg, rng=rng), LEFT, angle, steps) - 10.0 for _ in range(400)]
    assert np.std(errors) == pytest.approx(k * math.sqrt(10.0), rel=0.12)
    assert abs(np.mean(errors)) < 0.02


def test_no_noise_while_stationary():
    model = EncoderModel(R, EncoderNoiseConfig(seed=3))
    model.measure(LEFT, 0.5)
    assert all(model.measure(LEFT, 0.5) == 0.0 for _ in range(100))


@pytest.mark.parametrize('kwargs', [
    {'ticks_per_rev': 0}, {'wheel_radius_scale_stddev': -0.1},
    {'distance_noise_k': -1.0}, {'seed': -1},
])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        EncoderNoiseConfig(**kwargs)


def test_invalid_radius():
    with pytest.raises(ValueError):
        EncoderModel(0.0)
