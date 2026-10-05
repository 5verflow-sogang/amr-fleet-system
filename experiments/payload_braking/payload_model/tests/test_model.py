import math

import pytest

from payload_braking_model import (
    BrakingModel,
    BrakingModelParams,
    braking_distance_m,
    max_acceleration_mps2,
    time_to_reach_speed_s,
)


def test_max_acceleration_is_torque_limited_by_default():
    # 기본값(토크 3 N·m, 반경 0.1 m, 28.9 kg)에서는 토크 쪽이 더 작다
    # (토크 한계 ~2.08 m/s^2 < 마찰 한계 0.7*9.81 ~6.87 m/s^2).
    params = BrakingModelParams()
    a = max_acceleration_mps2(payload_kg=0.0, params=params)
    torque_limited = (2 * 3.0) / (0.1 * 28.9)
    assert a == pytest.approx(torque_limited, rel=1e-9)
    assert a < params.floor_friction_mu * params.gravity_mps2


def test_max_acceleration_is_friction_limited_when_torque_is_huge():
    # 토크를 아주 크게 주면 마찰 한계(mu*g)에 걸려야 한다.
    params = BrakingModelParams(wheel_torque_max_nm=1000.0)
    a = max_acceleration_mps2(payload_kg=0.0, params=params)
    assert a == pytest.approx(params.floor_friction_mu * params.gravity_mps2, rel=1e-9)


def test_max_acceleration_decreases_with_payload():
    params = BrakingModelParams()
    a0 = max_acceleration_mps2(0.0, params)
    a25 = max_acceleration_mps2(25.0, params)
    assert a25 < a0
    # 토크 한계 구간에서는 질량에 반비례한다: a0 / a25 == (28.9+25) / 28.9
    assert a0 / a25 == pytest.approx((params.robot_mass_kg + 25.0) / params.robot_mass_kg)


def test_braking_distance_matches_hand_calculation():
    params = BrakingModelParams(t_delay_s=0.0)  # 지연 없이 순수 공식만 검증
    v = 1.0
    payload = 10.0
    a_max = max_acceleration_mps2(payload, params)
    expected = v**2 / (2.0 * a_max)
    assert braking_distance_m(v, payload, params) == pytest.approx(expected, rel=1e-9)


def test_braking_distance_includes_delay_term():
    params = BrakingModelParams(t_delay_s=0.02)
    v = 1.0
    payload = 0.0
    a_max = max_acceleration_mps2(payload, params)
    expected = v * 0.02 + v**2 / (2.0 * a_max)
    assert braking_distance_m(v, payload, params) == pytest.approx(expected, rel=1e-9)


def test_a_cmd_caps_below_a_max_is_used():
    params = BrakingModelParams(t_delay_s=0.0)
    payload = 0.0
    a_max = max_acceleration_mps2(payload, params)
    a_cmd = a_max / 2.0  # a_max보다 확실히 작은 명령 한계
    v = 1.0
    expected = v**2 / (2.0 * a_cmd)
    assert braking_distance_m(v, payload, params, a_cmd_mps2=a_cmd) == pytest.approx(
        expected, rel=1e-9
    )


def test_a_cmd_above_a_max_is_ignored():
    params = BrakingModelParams(t_delay_s=0.0)
    payload = 0.0
    a_max = max_acceleration_mps2(payload, params)
    # a_max보다 큰 a_cmd를 줘도 a_max로 잘려야 한다(모터가 그 이상을 못 낸다).
    without_cmd = braking_distance_m(1.0, payload, params, a_cmd_mps2=None)
    with_huge_cmd = braking_distance_m(1.0, payload, params, a_cmd_mps2=a_max * 100)
    assert with_huge_cmd == pytest.approx(without_cmd, rel=1e-9)


def test_time_to_reach_speed_uses_same_formula_shape():
    params = BrakingModelParams(t_delay_s=0.02)
    v = 1.0
    payload = 10.0
    a_max = max_acceleration_mps2(payload, params)
    expected = params.t_delay_s + v / a_max
    assert time_to_reach_speed_s(v, payload, params) == pytest.approx(expected, rel=1e-9)


def test_braking_model_class_delegates_to_functions():
    params = BrakingModelParams(t_delay_s=0.01)
    model = BrakingModel(params)
    assert model.max_acceleration_mps2(5.0) == max_acceleration_mps2(5.0, params)
    assert model.braking_distance_m(1.0, 5.0) == braking_distance_m(1.0, 5.0, params)
    assert model.time_to_reach_speed_s(1.0, 5.0) == time_to_reach_speed_s(1.0, 5.0, params)


def test_braking_model_default_construction():
    # BrakingModel()을 인자 없이 만들 수 있어야 한다(사용 예시의 핵심).
    model = BrakingModel()
    distance = model.braking_distance_m(v_mps=1.0, payload_kg=10.0)
    assert math.isfinite(distance)
    assert distance > 0.0


def test_accepts_numpy_like_scalars():
    # numpy 의존은 없지만, numpy 스칼라/배열이 들어와도 그냥 통과해야 한다
    # (연우 RL 환경이 numpy.float32 등을 넘길 수 있으므로). numpy 없이도
    # 비슷한 성질(산술 연산 지원)을 가진 객체로 흉내 낸다.
    class FloatLike(float):
        pass

    v = FloatLike(1.0)
    payload = FloatLike(10.0)
    result = braking_distance_m(v, payload)
    assert math.isfinite(result)
