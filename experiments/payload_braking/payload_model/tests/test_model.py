import math

import pytest

from payload_braking_model import (
    BrakingModel,
    BrakingModelParams,
    braking_distance_m,
    max_acceleration_mps2,
    step,
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


# --- step() : 연우 RL 환경(#51) step(v, w, v_cmd, w_cmd, m, dt) -> (v', w') ---


def _integrate_stop_distance(v0, payload_kg, params, dt=0.01, a_cmd_mps2=None):
    """v0에서 시작해 step()을 반복 호출(명령 0)해 완전히 멈출 때까지 이동 거리."""
    v, w = v0, 0.0
    distance = 0.0
    for _ in range(100_000):
        v_new, w_new = step(v, w, 0.0, 0.0, payload_kg, dt, params, a_cmd_mps2)
        distance += (v + v_new) / 2.0 * dt  # 사다리꼴 적분 - 등가속도면 오차 거의 0
        v, w = v_new, w_new
        if v <= 0.0:
            break
    else:
        raise AssertionError("100000 스텝 안에 멈추지 않음")
    return distance


def test_step_emergency_matches_braking_distance_within_1cm():
    # t_delay=0으로 둬서 braking_distance_m과 같은 기준으로 비교한다
    # (step()은 지연을 안 쓰므로 - README 참고).
    params = BrakingModelParams(t_delay_s=0.0)
    for payload in [0.0, 2.0, 10.0, 25.0]:
        integrated = _integrate_stop_distance(1.0, payload, params)
        expected = braking_distance_m(1.0, payload, params)  # a_cmd=None -> 급정지
        assert abs(integrated - expected) < 0.01, (
            f"payload={payload}: integrated={integrated} expected={expected}"
        )


def test_step_normal_stop_matches_braking_distance_within_1cm():
    params = BrakingModelParams(t_delay_s=0.0)
    for payload in [0.0, 10.0, 25.0]:
        integrated = _integrate_stop_distance(1.0, payload, params, a_cmd_mps2=1.0)
        expected = braking_distance_m(1.0, payload, params, a_cmd_mps2=1.0)
        assert abs(integrated - expected) < 0.01


def test_step_with_a_cmd_is_payload_independent():
    # a_cmd(1.0)가 토크 한계보다 작으면 a_limit=a_cmd로 고정돼 질량이 안 보여야 한다.
    params = BrakingModelParams()
    dt = 0.1
    results = [
        step(1.0, 0.0, 0.0, 0.0, payload, dt, params, a_cmd_mps2=1.0)
        for payload in [0.0, 2.0, 10.0, 25.0]
    ]
    v_news = [v for v, _w in results]
    assert all(v == pytest.approx(v_news[0], abs=1e-9) for v in v_news)


def test_step_without_a_cmd_depends_on_payload():
    # 대조: a_cmd 없이(급정지) 돌리면 토크 한계(a_max)가 그대로 적용돼 질량마다 달라야 한다.
    params = BrakingModelParams()
    dt = 0.1
    v_light, _ = step(1.0, 0.0, 0.0, 0.0, 0.0, dt, params)
    v_heavy, _ = step(1.0, 0.0, 0.0, 0.0, 25.0, dt, params)
    assert v_light != pytest.approx(v_heavy, abs=1e-6)


def test_step_linear_velocity_limit_is_respected():
    params = BrakingModelParams()
    # 아주 큰 dt·명령 속도를 줘도 선속도 상한을 넘지 않아야 한다.
    v_new, _ = step(0.0, 0.0, 100.0, 0.0, 0.0, 100.0, params)
    assert abs(v_new) <= params.linear_velocity_max_mps + 1e-9

    v_new_neg, _ = step(0.0, 0.0, -100.0, 0.0, 0.0, 100.0, params)
    assert abs(v_new_neg) <= params.linear_velocity_max_mps + 1e-9


def test_step_angular_velocity_limit_is_respected():
    params = BrakingModelParams()
    _, w_new = step(0.0, 0.0, 0.0, 100.0, 0.0, 100.0, params)
    assert abs(w_new) <= params.angular_velocity_max_radps + 1e-9


def test_step_angular_velocity_ignores_payload():
    # D7(v0 합의): 각속도는 질량을 반영하지 않는다.
    params = BrakingModelParams()
    dt = 0.1
    _, w_light = step(0.0, 0.0, 0.0, 1.5, 0.0, dt, params)
    _, w_heavy = step(0.0, 0.0, 0.0, 1.5, 25.0, dt, params)
    assert w_light == pytest.approx(w_heavy, abs=1e-12)


def test_step_one_step_matches_hand_calculation():
    params = BrakingModelParams()
    payload = 10.0
    dt = 0.1
    a_max = max_acceleration_mps2(payload, params)
    v_new, w_new = step(0.0, 0.0, 10.0, 10.0, payload, dt, params)
    # v_cmd(10)가 한 스텝으로 못 닿는 큰 값이라 a_max*dt만큼만 움직여야 한다
    assert v_new == pytest.approx(a_max * dt, rel=1e-9)
    assert w_new == pytest.approx(params.angular_accel_max_radps2 * dt, rel=1e-9)


def test_braking_model_step_delegates_to_function():
    params = BrakingModelParams()
    model = BrakingModel(params)
    direct = step(1.0, 0.0, 0.0, 0.0, 10.0, 0.1, params)
    via_model = model.step(1.0, 0.0, 0.0, 0.0, 10.0, 0.1)
    assert via_model == direct
