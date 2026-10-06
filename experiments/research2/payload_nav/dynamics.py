"""질량 의존 차동 구동 동역학과 충돌 판정 (ROS 비의존).

가감속 한계는 바퀴 토크 한계에서 나온다 (명세서 4-1·4-2절). 바퀴 하나가 낼 수 있는 접선 가속은
a_max(m) = 2τ / (r·(M_robot + m))로, 두 바퀴가 같은 토크를 낼 때의 선가속과 같다. 식과 기본값은
이진의 제동 모델(PR #55, experiments/payload_braking/payload_model)과 같다.

선속도와 회전은 바퀴 두 개의 토크를 나눠 쓴다. 왼·오른 바퀴 속도의 변화를 각각 a_max(m)로 자르므로
급정지와 급회전을 동시에 최대로 할 수 없고, 회전 가속도 적재 질량의 영향을 받는다. 여기에 DiffDrive의
회전 가속 한계(properties.xacro max_angular_acceleration)를 추가로 적용한다.
#55가 머지되면 max_linear_accel을 payload_braking_model.max_acceleration_mps2로 바꾼다.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotParams:
    # 바퀴 토크 한계와 질량 (지수 URDF #44, 이진 #42·#55)
    wheel_torque_max_nm: float = 3.0
    wheel_radius_m: float = 0.1
    num_wheels: int = 2
    robot_mass_kg: float = 28.9      # 적재 제외: 몸체 25 + 바퀴 3 + 캐스터 0.6 + 센서 0.3
    floor_friction_mu: float = 0.7
    gravity_mps2: float = 9.81
    wheel_separation_m: float = 0.34  # properties.xacro wheel_separation
    # DiffDrive 회전 가속 한계 (properties.xacro). 바퀴 토크 한계와 함께 적용한다
    max_angular_accel: float = 3.2   # rad/s²
    # 명세 동역학 한계
    v_max: float = 2.0               # m/s
    w_max: float = 1.5               # rad/s
    # 충돌 판정용 몸체 (600 × 400 mm)
    length: float = 0.6
    width: float = 0.4


DEFAULT_PARAMS = RobotParams()


def max_linear_accel(payload_kg: float, params: RobotParams = DEFAULT_PARAMS) -> float:
    """토크 한계와 바닥 마찰 한계 중 작은 쪽 [m/s²]. 가속과 감속에 같이 쓴다."""
    if payload_kg < 0.0:
        raise ValueError(f'적재 질량은 0 이상이어야 한다: {payload_kg}')
    total_mass = params.robot_mass_kg + payload_kg
    torque_limited = (params.num_wheels * params.wheel_torque_max_nm
                      / (params.wheel_radius_m * total_mass))
    return min(torque_limited, params.floor_friction_mu * params.gravity_mps2)


def braking_distance(v: float, payload_kg: float, params: RobotParams = DEFAULT_PARAMS) -> float:
    """토크 한계로 급정지할 때의 제동 거리 [m]. 반응 지연은 넣지 않는다 (v0)."""
    return v * v / (2.0 * max_linear_accel(payload_kg, params))


@dataclass(frozen=True)
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def step(state: RobotState, v_cmd: float, w_cmd: float, payload_kg: float, dt: float,
         params: RobotParams = DEFAULT_PARAMS) -> RobotState:
    """속도 명령을 dt 동안 적용한다.

    명령 (v, ω)를 왼·오른 바퀴 속도로 바꿔, 두 바퀴 속도 변화를 같은 비율로 줄여 더 큰 쪽이 a_max(m)를
    넘지 않게 한다 (명령의 회전·직진 비율은 유지). 회전 변화는 DiffDrive 회전 가속 한계로도 자른다.
    목표에 스텝 중간에 도달하면 그 뒤로는 유지한다고 보고 이동 거리를 정확히 적분하므로, 직진 급정지
    거리는 dt와 관계없이 v²/(2a)와 같다.
    """
    accel = max_linear_accel(payload_kg, params)
    half = 0.5 * params.wheel_separation_m
    dl = (v_cmd - w_cmd * half) - (state.v - state.w * half)   # 왼쪽 바퀴 속도 변화
    dr = (v_cmd + w_cmd * half) - (state.v + state.w * half)   # 오른쪽 바퀴 속도 변화
    dv, dw = v_cmd - state.v, w_cmd - state.w
    # 바퀴 토크 한계: 더 많이 바뀌는 바퀴 기준으로 목표에 닿는 시간
    reach = max(abs(dl), abs(dr)) / accel
    # DiffDrive 회전 가속 한계
    reach = max(reach, abs(dw) / params.max_angular_accel)
    reach_time = min(reach, dt)
    frac = 1.0 if reach <= 0.0 else reach_time / reach  # 이번 스텝에 이루는 변화의 비율
    v_new = state.v + frac * dv
    w_new = state.w + frac * dw
    # 변화 구간(reach_time)은 선형, 이후는 일정하다고 보고 평균 속도를 구한다
    v_avg = (0.5 * (state.v + v_new) * reach_time + v_new * (dt - reach_time)) / dt
    w_avg = (0.5 * (state.w + w_new) * reach_time + w_new * (dt - reach_time)) / dt
    theta_mid = state.theta + 0.5 * w_avg * dt
    return RobotState(
        x=state.x + v_avg * math.cos(theta_mid) * dt,
        y=state.y + v_avg * math.sin(theta_mid) * dt,
        theta=wrap_angle(state.theta + w_avg * dt),
        v=v_new,
        w=w_new,
    )


def footprint_corners(state: RobotState, params: RobotParams = DEFAULT_PARAMS):
    """로봇 사각형 몸체의 꼭짓점 4개 (전역 좌표)."""
    c, s = math.cos(state.theta), math.sin(state.theta)
    hl, hw = 0.5 * params.length, 0.5 * params.width
    return [(state.x + c * lx - s * ly, state.y + s * lx + c * ly)
            for lx, ly in ((hl, hw), (hl, -hw), (-hl, hw), (-hl, -hw))]


def footprint_clearance(state: RobotState, ox: float, oy: float, radius: float,
                        params: RobotParams = DEFAULT_PARAMS) -> float:
    """로봇 사각형 몸체와 원형 장애물 사이 최소 거리 [m]. 0 이하면 충돌이다."""
    dx, dy = ox - state.x, oy - state.y
    c, s = math.cos(state.theta), math.sin(state.theta)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    qx = max(abs(local_x) - 0.5 * params.length, 0.0)
    qy = max(abs(local_y) - 0.5 * params.width, 0.0)
    return math.hypot(qx, qy) - radius
