"""적재 질량에 따른 가속·제동 거리/시간 물리 모델.

순수 Python(표준 라이브러리만, ROS 의존 없음) - 연우의 2D RL 학습 환경에서
그대로 import해서 쓸 수 있다. 숫자 입력은 float든 numpy 스칼라/배열이든
상관없다(이 모듈이 쓰는 연산은 +, -, *, /, min뿐이라 numpy도 그대로 통과한다).

근거:
- experiments/payload_braking (PR #42): 최소 모델로 V(DiffDrive)/T(JointController)
  두 구동방식·normal/emergency 두 정지방식을 적재 0/2/10/25kg × 5회 실측.
- src/simulation/amr_description (PR #44, 지수): #42 권장값(바퀴 토크 3 N·m, 몸체
  25 kg)을 실제 로봇 URDF에 반영.
- PR #78(지수): 현재 URDF(28.9kg)로 급정지 거리를 0/2/10/25kg × 1회 다시 측정.
  t_delay_s 기본값은 이 데이터로 맞췄다 - README "t_delay 보정" 절 참고.

식 (전부 README "식" 절에 있는 그대로):
    a_max = min(2 * 토크 / (반경 * (전체 질량 + 적재)), mu * g)
    a     = min(a_cmd, a_max)          # a_cmd 없으면 a_max (급정지)
    제동 거리 = v * t_delay + v**2 / (2 * a)
    0->v 도달 시간 = t_delay + v / a    # 가속에도 같은 식을 쓴다
"""
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class BrakingModelParams:
    """지수 URDF 기준값 (PR #44, src/simulation/amr_description/urdf/properties.xacro).

    robot_mass_kg는 적재를 뺀 로봇 자체 질량이다: 몸체 25 + 바퀴 1.5*2 + 캐스터
    0.3*2 + 센서(LiDAR 0.2 + 카메라 0.1) = 28.9 kg. #42 실험 모델(최소 몸체+바퀴만,
    캐스터·센서 질량 없음)은 25 + 1.5*2 = 28 kg이라 조금 더 가볍다 - README
    "검증" 절 표에서 둘 다 비교한다.
    """

    wheel_torque_max_nm: float = 3.0
    wheel_radius_m: float = 0.1
    num_wheels: int = 2
    robot_mass_kg: float = 28.9
    floor_friction_mu: float = 0.7
    gravity_mps2: float = 9.81
    # #78 실측(28.9kg URDF, 0/2/10/25kg 급정지)과 지연 없는 이론값의 평균
    # 차이로 맞췄다(README "t_delay 보정" 절 참고). v=1 m/s 기준이라 값 자체가
    # 곧 보정 거리[m]다. step()에는 안 쓴다 - README "step()은 지연이 없다" 절 참고.
    t_delay_s: float = 0.015

    # step() 전용 한계값. nav2_params.yaml(FollowPath)·#44 지수 URDF와 동일하다.
    linear_velocity_max_mps: float = 2.0
    angular_velocity_max_radps: float = 1.5
    # 각가속은 D7(v0 합의)에 따라 질량을 반영하지 않는 고정값이다.
    angular_accel_max_radps2: float = 3.2


def max_acceleration_mps2(
    payload_kg: float, params: BrakingModelParams = BrakingModelParams()
):
    """바퀴 토크 한계로 낼 수 있는 최대 가속/감속[m/s^2].

    토크 한계(2개 바퀴 * 토크 / 반경 / 전체질량)와 바닥 마찰 한계(mu*g) 중
    작은 쪽 - 적재가 아주 작아 토크가 남아도 바닥이 미끄러지면 그 이상은 못 낸다.
    """
    total_mass = params.robot_mass_kg + payload_kg
    torque_limited = (params.num_wheels * params.wheel_torque_max_nm) / (
        params.wheel_radius_m * total_mass
    )
    friction_limited = params.floor_friction_mu * params.gravity_mps2
    return min(torque_limited, friction_limited)


def braking_distance_m(
    v_mps: float,
    payload_kg: float,
    params: BrakingModelParams = BrakingModelParams(),
    a_cmd_mps2: Optional[float] = None,
):
    """정지 명령 시점부터 멈출 때까지 이동 거리[m].

    a_cmd_mps2를 주면 "normal"(소프트웨어 감속 제한) 정지, 안 주면(None) 토크
    한계까지 그대로 쓰는 "emergency" 급정지다.
    """
    a_max = max_acceleration_mps2(payload_kg, params)
    a = a_max if a_cmd_mps2 is None else min(a_cmd_mps2, a_max)
    return v_mps * params.t_delay_s + v_mps**2 / (2.0 * a)


def time_to_reach_speed_s(
    v_mps: float,
    payload_kg: float,
    params: BrakingModelParams = BrakingModelParams(),
    a_cmd_mps2: Optional[float] = None,
):
    """정지 상태에서 속도 v_mps에 도달하는 데 걸리는 시간[s]. 가속에도 같은 식."""
    a_max = max_acceleration_mps2(payload_kg, params)
    a = a_max if a_cmd_mps2 is None else min(a_cmd_mps2, a_max)
    return params.t_delay_s + v_mps / a


def _clamp(value, low, high):
    return max(low, min(high, value))


def step(
    v_mps: float,
    w_radps: float,
    v_cmd_mps: float,
    w_cmd_radps: float,
    payload_kg: float,
    dt_s: float,
    params: BrakingModelParams = BrakingModelParams(),
    a_cmd_mps2: Optional[float] = None,
):
    """RL 환경의 한 제어 스텝: (v, w)를 (v_cmd, w_cmd) 쪽으로 dt만큼만 바꾼다.

    `연우의 2D RL 환경(#51)`이 쓰는 모양 `step(v, w, v_cmd, w_cmd, m, dt) ->
    (v', w')`을 그대로 따른다. 지연(`t_delay_s`)은 안 쓴다 - README "step()은
    지연이 없다" 절 참고.

    선속도: 이번 스텝에 낼 수 있는 가속·감속 한계는
    `min(max_acceleration_mps2(payload_kg, params), a_cmd_mps2)`다(`a_cmd_mps2`가
    `None`이면 토크 한계 그대로) - 가속·감속 모두 같은 한계를 쓴다(실제 바퀴는
    앞뒤로 같은 토크를 낼 수 있다). `v`를 그 한계 안에서 `v_cmd` 쪽으로만
    옮기고, 마지막에 `±linear_velocity_max_mps`로 자른다.

    각속도: 질량을 반영하지 않는다(D7 v0 합의) - `angular_accel_max_radps2`
    (고정값)로 `w`를 `w_cmd` 쪽으로 옮기고 `±angular_velocity_max_radps`로 자른다.

    반환: `(v_new, w_new)` 튜플.
    """
    a_limit = max_acceleration_mps2(payload_kg, params)
    if a_cmd_mps2 is not None:
        a_limit = min(a_limit, a_cmd_mps2)
    max_dv = a_limit * dt_s
    dv = _clamp(v_cmd_mps - v_mps, -max_dv, max_dv)
    v_new = _clamp(
        v_mps + dv, -params.linear_velocity_max_mps, params.linear_velocity_max_mps
    )

    max_dw = params.angular_accel_max_radps2 * dt_s
    dw = _clamp(w_cmd_radps - w_radps, -max_dw, max_dw)
    w_new = _clamp(
        w_radps + dw,
        -params.angular_velocity_max_radps,
        params.angular_velocity_max_radps,
    )

    return v_new, w_new


class BrakingModel:
    """RL 환경에서 쓰기 편하게 params를 묶어 둔 얇은 래퍼.

        from payload_braking_model import BrakingModel
        model = BrakingModel()  # 지수 URDF(#44) 기본값
        distance = model.braking_distance_m(v_mps=1.0, payload_kg=10.0)
    """

    def __init__(self, params: BrakingModelParams = BrakingModelParams()):
        self.params = params

    def max_acceleration_mps2(self, payload_kg: float):
        return max_acceleration_mps2(payload_kg, self.params)

    def braking_distance_m(
        self, v_mps: float, payload_kg: float, a_cmd_mps2: Optional[float] = None
    ):
        return braking_distance_m(v_mps, payload_kg, self.params, a_cmd_mps2)

    def time_to_reach_speed_s(
        self, v_mps: float, payload_kg: float, a_cmd_mps2: Optional[float] = None
    ):
        return time_to_reach_speed_s(v_mps, payload_kg, self.params, a_cmd_mps2)

    def step(
        self,
        v_mps: float,
        w_radps: float,
        v_cmd_mps: float,
        w_cmd_radps: float,
        payload_kg: float,
        dt_s: float,
        a_cmd_mps2: Optional[float] = None,
    ):
        return step(
            v_mps, w_radps, v_cmd_mps, w_cmd_radps, payload_kg, dt_s,
            self.params, a_cmd_mps2,
        )
