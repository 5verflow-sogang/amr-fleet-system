"""DWA 비교군 B1·B2: amr_planning_core DWA(#41) C++ 원본을 2D 환경에서 그대로 부른다 (명세서 4-4절).

경로 계획 판단(DwaPlanner::computeVelocityCommand, rotateToGoalYaw)은 C++ 원본(src/navigation/amr_planning_core,
수정 없음)이 한다. 공유 라이브러리로 컴파일해 ctypes로 부른다 (dwa_cpp/dwa_c_api.cpp, 처음 쓸 때 clang++/g++로
빌드). Python은 ROS 노드가 하던 일만 맡는다:

- local costmap: Nav2 설정(bringup nav2_params.yaml local_costmap)대로 래스터를 만든다. 로봇 중심 3 × 3 m
  rolling window, 0.05 m 셀. 치명 셀은 통로 벽(static_layer)과 LiDAR가 찍는 장애물의 지금 위치(obstacle_layer,
  2.5 m 안). inflation은 Nav2 InflationLayer 식 (inscribed 0.21 m, inflation_radius 0.5 m, cost_scaling 3.0)
- 추적 결과: TrackedObstacleArray 대신 환경 장애물의 위치·속도·반경. RL 정책처럼 나타난 뒤에만 보이고,
  서 있는 동안은 속도 0 (env._observe와 같다). 추적 지연·잡음은 넣지 않는다
- 플러그인(amr_nav2_plugins dwa_controller.cpp): 전역 경로(통로 중심선 전체)를 코어에 넘긴다(#195). heading
  기준점(lookahead_distance, adaptive_lookahead)과 목표 앞 감속의 최종 목표(경로 끝)는 코어가 고른다. 목표
  xy_goal_tolerance 안이면 제자리 회전. no_valid·goal_reached면 정지
- xy_goal_tolerance는 Gazebo(0.25 m)와 다르게 환경 도착 기준(0.2 m)으로 둔다. 플러그인 주석대로 goal_checker와
  같아야 하고, 0.25 m면 도착 반경 밖에서 서 버린다
- 샘플 수(v_samples·w_samples), 제자리 회전(rotate_in_place_*), TTC 회피 모드(avoid_mode_enabled)는 nav2_params를
  따라가지 않고 논문 값으로 고정한다 (20 x 30, 회전 끔, 회피 모드 끔). 창고 스택은 10 x 15·회전 45°·회피 모드 켬이다
  (#244, DwaParams 주석)

질량은 가감속 한계 acc_v로 들어간다 (나머지 파라미터는 위 예외 말고 nav2_params.yaml DWA 블록 그대로):
- B1 ('b1'): 빈 차 기준 a_max(0 kg) 고정
- B2 ('b2'): 관측 질량 기준 a_max(m_obs). 질량을 모르면 B1과 같다 (실험 설계 3절)
"""

import ctypes
from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
import subprocess
from typing import Optional

import numpy as np

from .dynamics import max_linear_accel

DWA_VARIANTS = ('b1', 'b2')

_RESEARCH2 = Path(__file__).resolve().parents[1]
_REPO = _RESEARCH2.parents[1]
_CORE = _REPO / 'src' / 'navigation' / 'amr_planning_core'
_SOURCES = [_CORE / 'src' / 'dwa.cpp', _CORE / 'src' / 'grid.cpp', _RESEARCH2 / 'dwa_cpp' / 'dwa_c_api.cpp']
_LIB = _RESEARCH2 / 'dwa_cpp' / 'build' / 'libamr_dwa.so'


@dataclass(frozen=True)
class DwaParams:
    """nav2_params.yaml controller_server.DWA (DwaConfig 필드 이름 그대로, acc_v 제외) + 플러그인 + local costmap.

    샘플 수·제자리 회전·xy_goal_tolerance는 nav2_params와 다른 논문 값이다 (모듈 docstring).
    """
    max_v: float = 2.0
    min_v: float = 0.0
    max_w: float = 1.5
    acc_w: float = 3.2
    dt: float = 0.1
    sim_time: float = 2.0
    # 샘플 수는 nav2_params.yaml을 따라가지 않고 논문(연구 2 비교군) 값 20 x 30으로 고정한다. 창고 스택은 계산량
    # 때문에 10 x 15로 줄인다(navigation.md 4-2) - 그걸 따라 바꾸면 B1·B2 결과가 달라진다. C++ DwaConfig 기본값도
    # 안 쓴다(dwa_c_api.cpp가 이 값을 그대로 넘긴다)
    v_samples: int = 20
    w_samples: int = 30
    footprint_half_length: float = 0.3
    footprint_half_width: float = 0.2
    lethal_cost_threshold: int = 253
    heading_weight: float = 0.8
    clearance_weight: float = 0.1
    velocity_weight: float = 0.1
    goal_tolerance: float = 0.15
    predict_dynamic_obstacles: bool = True
    dynamic_obstacle_safety_margin: float = 0.2
    dynamic_obstacle_influence_radius: float = 1.0
    dynamic_weight: float = 0.3
    use_admissible_velocity: bool = True
    use_goal_slowdown: bool = True
    adaptive_lookahead: bool = True
    # 제자리 회전(#244)은 논문 값으로 끈다(0 이하). 창고 스택(nav2_params.yaml)은 45°로 켠다 - 켜면 B1 25 kg이
    # 목표를 0.3 m 지나친 뒤 돌아와 성공해, 고정 한계의 위험(S1 지나침 49%, S2 성공 0%)이 결과에서 사라진다.
    # C++ DwaConfig 기본값(45°)도 안 쓴다(dwa_c_api.cpp가 이 값을 그대로 넘긴다)
    rotate_in_place_min_angle: float = 0.0
    rotate_in_place_exit_angle: float = 0.0
    # 제자리 회전 샘플의 풋프린트 여유(창고 스택 0.05 m, 안전 노드와 맞춤). 논문 값은 0(키우지 않음) - 회전을 꺼도
    # v≈0 회전 샘플은 평소 탐색에 있어서, 켜 두면 B1·B2가 바뀐다
    rotate_in_place_clearance: float = 0.0
    # TTC 회피 모드(창고 스택, navigation.md 4-10절)는 논문 값으로 끈다. 연구 2 환경에는 collision_risks가 없고,
    # 켜면 B1·B2의 안전 여유·가중치·sim_time이 바뀐다
    avoid_mode_enabled: bool = False
    follow_path: bool = True          # 플러그인처럼 전역 경로를 넘긴다 (#195). False면 예전 지점 호출
    path_resolution: float = 0.05
    # 플러그인
    lookahead_distance: float = 0.5
    xy_goal_tolerance: Optional[float] = None   # None이면 환경 도착 기준 (Gazebo goal_checker는 0.25 m)
    yaw_goal_tolerance: float = 0.25
    # local costmap
    costmap_size: float = 3.0
    resolution: float = 0.05
    inscribed_radius: float = 0.21   # footprint 반폭 0.2 + footprint_padding 0.01
    inflation_radius: float = 0.5
    cost_scaling_factor: float = 3.0
    obstacle_max_range: float = 2.5


class _CConfig(ctypes.Structure):
    """dwa_c_api.cpp DwaCConfig와 같은 순서."""
    _fields_ = [(n, ctypes.c_double) for n in ('max_v', 'min_v', 'max_w', 'acc_v', 'acc_w', 'dt', 'sim_time')] + [
        ('v_samples', ctypes.c_int), ('w_samples', ctypes.c_int),
        ('footprint_half_length', ctypes.c_double), ('footprint_half_width', ctypes.c_double),
        ('lethal_cost_threshold', ctypes.c_int),
        ('heading_weight', ctypes.c_double), ('clearance_weight', ctypes.c_double),
        ('velocity_weight', ctypes.c_double), ('goal_tolerance', ctypes.c_double),
        ('predict_dynamic_obstacles', ctypes.c_int),
        ('dynamic_obstacle_safety_margin', ctypes.c_double), ('dynamic_obstacle_influence_radius', ctypes.c_double),
        ('dynamic_weight', ctypes.c_double),
        ('use_admissible_velocity', ctypes.c_int), ('use_goal_slowdown', ctypes.c_int),
        ('lookahead_distance', ctypes.c_double), ('adaptive_lookahead', ctypes.c_int),
        ('rotate_in_place_min_angle', ctypes.c_double), ('rotate_in_place_exit_angle', ctypes.c_double),
        ('rotate_in_place_clearance', ctypes.c_double), ('avoid_mode_enabled', ctypes.c_int)]


_lib = None


def load_library() -> ctypes.CDLL:
    """C++ 원본을 공유 라이브러리로 빌드(소스가 라이브러리보다 새로우면 다시)하고 불러온다."""
    global _lib
    if _lib is not None:
        return _lib
    stale = not _LIB.exists() or any(src.stat().st_mtime > _LIB.stat().st_mtime for src in _SOURCES)
    if stale:
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            raise RuntimeError('C++ 컴파일러(clang++ 또는 g++)가 없다')
        _LIB.parent.mkdir(parents=True, exist_ok=True)
        cmd = [compiler, '-std=c++17', '-O2', '-shared', '-fPIC', '-I', str(_CORE / 'include'),
               *map(str, _SOURCES), '-o', str(_LIB)]
        if subprocess.run(cmd).returncode != 0:
            # macOS: Command Line Tools에 링커가 못 읽는 새 SDK가 기본으로 잡히면 실패한다
            # (예: MacOSX27.0 tbd "unknown architecture"). CLT 기본 바로가기 MacOSX.sdk로 다시 한다
            sdk = Path('/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk')
            if not sdk.exists():
                raise RuntimeError('DWA C++ 빌드 실패: ' + ' '.join(cmd))
            subprocess.run(cmd, check=True, env={**os.environ, 'SDKROOT': str(sdk)})
    lib = ctypes.CDLL(str(_LIB))
    dptr = ctypes.POINTER(ctypes.c_double)
    lib.dwa_compute_velocity_command.argtypes = [
        ctypes.POINTER(_CConfig), dptr, dptr, ctypes.c_int, ctypes.c_int, ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint8), dptr, ctypes.c_int, dptr]
    lib.dwa_compute_velocity_command.restype = ctypes.c_int
    lib.dwa_compute_velocity_command_path.argtypes = [
        ctypes.POINTER(_CConfig), dptr, dptr, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint8), dptr, ctypes.c_int, dptr]
    lib.dwa_compute_velocity_command_path.restype = ctypes.c_int
    lib.dwa_rotate_to_goal_yaw.argtypes = [ctypes.POINTER(_CConfig), dptr, ctypes.c_double, ctypes.c_double, dptr]
    lib.dwa_rotate_to_goal_yaw.restype = ctypes.c_int
    _lib = lib
    return lib


_STATUS = {0: 'ok', 1: 'goal_reached', 2: 'no_valid'}


def _dptr(a: np.ndarray):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


@dataclass
class DwaPolicy:
    variant: str = 'b2'
    params: DwaParams = DwaParams()

    def __post_init__(self):
        if self.variant not in DWA_VARIANTS:
            raise ValueError(f'variant는 {DWA_VARIANTS} 중 하나: {self.variant}')
        self._lib = load_library()

    @property
    def name(self) -> str:
        return f'dwa_{self.variant}'

    @property
    def mass_aware(self) -> bool:
        return self.variant == 'b2'

    def planning_mass(self, env) -> float:
        """가감속 한계를 정하는 질량. B1은 빈 차, B2는 관측 질량 (모르면 빈 차)."""
        if self.mass_aware and env.mass_obs is not None:
            return env.mass_obs
        return 0.0

    def acc_v(self, env) -> float:
        return max_linear_accel(self.planning_mass(env), env.cfg.robot)

    def config(self, acc_v: float) -> _CConfig:
        p = self.params
        return _CConfig(p.max_v, p.min_v, p.max_w, acc_v, p.acc_w, p.dt, p.sim_time, p.v_samples, p.w_samples,
                        p.footprint_half_length, p.footprint_half_width, p.lethal_cost_threshold,
                        p.heading_weight, p.clearance_weight, p.velocity_weight, p.goal_tolerance,
                        int(p.predict_dynamic_obstacles), p.dynamic_obstacle_safety_margin,
                        p.dynamic_obstacle_influence_radius, p.dynamic_weight,
                        int(p.use_admissible_velocity), int(p.use_goal_slowdown), p.lookahead_distance,
                        int(p.adaptive_lookahead), p.rotate_in_place_min_angle, p.rotate_in_place_exit_angle,
                        p.rotate_in_place_clearance, int(p.avoid_mode_enabled))

    # ------------------------------------------------------------ ROS 노드가 하던 일
    @staticmethod
    def obstacles(env):
        """추적 결과에 해당하는 장애물 (x, y, vx, vy, radius). 서 있는 동안은 속도 0 (env._observe와 같다)."""
        return [(o.x, o.y, o.vx if env.t > o.hold_until else 0.0, o.vy if env.t > o.hold_until else 0.0,
                 o.radius) for o in env.obstacles]

    def scanned_obstacles(self, env):
        """obstacle_layer가 찍는 장애물: 지금 위치, 센서 범위 안 (x, y, radius)."""
        s, p = env.state, self.params
        return [(o.x, o.y, o.radius) for o in env.obstacles
                if math.hypot(o.x - s.x, o.y - s.y) - o.radius <= p.obstacle_max_range]

    def costmap(self, env):
        """Nav2 local costmap 래스터. 반환 (원점 x, 원점 y, 셀 수, 비용 uint8 [n·n], 행 = y)."""
        p, s = self.params, env.state
        res = p.resolution
        n = int(round(p.costmap_size / res))
        ox = math.floor((s.x - 0.5 * p.costmap_size) / res) * res   # Costmap2D::updateOrigin (셀 단위로 맞춤)
        oy = math.floor((s.y - 0.5 * p.costmap_size) / res) * res
        cy = oy + (np.arange(n) + 0.5) * res
        cx = ox + (np.arange(n) + 0.5) * res
        # 벽: |y| ≥ half_width인 행이 치명. 줄 전체가 치명이라 가장 가까운 치명 셀은 같은 열의 가장 가까운 치명 행
        rows = np.arange(n)
        wall_rows = rows[np.abs(cy) >= env.half_width]
        if len(wall_rows):
            d_rows = np.abs(rows[:, None] - wall_rows[None, :]).min(axis=1).astype(float)
        else:
            d_rows = np.full(n, np.inf)
        dist = np.repeat(d_rows[:, None], n, axis=1)               # [y, x] 셀 단위 거리
        scanned = self.scanned_obstacles(env)
        if scanned:
            X, Y = np.meshgrid(cx, cy)
            gy, gx = np.mgrid[0:n, 0:n]
            for x, y, r in scanned:
                iy, ix = np.nonzero(np.hypot(X - x, Y - y) <= r)
                for a, b in zip(iy, ix):
                    dist = np.minimum(dist, np.hypot(gy - a, gx - b))
        dm = dist * res
        cost = np.where(dm <= p.inflation_radius,
                        np.floor(252.0 * np.exp(-p.cost_scaling_factor * (dm - p.inscribed_radius))), 0.0)
        cost = np.where(dm <= p.inscribed_radius, 253.0, cost)
        cost = np.where(dist == 0, 254.0, cost)
        return ox, oy, n, np.ascontiguousarray(cost.astype(np.uint8).ravel())

    def lookahead_waypoint(self, env):
        """전역 경로(중심선 y = 0, 출발 → 목표)에서 lookahead_distance 앞의 점. 경로 끝을 넘지 않는다."""
        return (min(env.goal[0], env.state.x + self.params.lookahead_distance), 0.0)

    def global_path(self, env):
        """전역 경로: 통로 중심선을 출발점(x = 0)부터 목표까지 path_resolution 간격으로."""
        n = max(2, int(math.ceil(env.goal[0] / self.params.path_resolution)) + 1)
        return [(x, 0.0) for x in np.linspace(0.0, env.goal[0], n)]

    # ------------------------------------------------------------ C++ 호출
    def compute_velocity_command_path(self, env, acc_v: float):
        """DwaPlanner::computeVelocityCommand(state, path, ...) (C++). 반환 (status, v, w)."""
        s = env.state
        ox, oy, n, cost = self.costmap(env)
        state = np.array([s.x - ox, s.y - oy, s.theta, s.v, s.w], dtype=np.float64)
        path = np.array([(x - ox, y - oy) for x, y in self.global_path(env)], dtype=np.float64).reshape(-1)
        obs = np.array([(x - ox, y - oy, vx, vy, r) for x, y, vx, vy, r in self.obstacles(env)],
                       dtype=np.float64).reshape(-1)
        out = np.zeros(2)
        cfg = self.config(acc_v)
        code = self._lib.dwa_compute_velocity_command_path(
            ctypes.byref(cfg), _dptr(state), _dptr(path), path.size // 2, n, n, self.params.resolution,
            cost.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), _dptr(obs) if obs.size else None,
            obs.size // 5, _dptr(out))
        return _STATUS[code], float(out[0]), float(out[1])

    def compute_velocity_command(self, env, goal, acc_v: float):
        """DwaPlanner::computeVelocityCommand (C++). 반환 (status, v, w)."""
        s = env.state
        ox, oy, n, cost = self.costmap(env)
        state = np.array([s.x - ox, s.y - oy, s.theta, s.v, s.w], dtype=np.float64)
        g = np.array([goal[0] - ox, goal[1] - oy], dtype=np.float64)
        obs = np.array([(x - ox, y - oy, vx, vy, r) for x, y, vx, vy, r in self.obstacles(env)],
                       dtype=np.float64).reshape(-1)
        out = np.zeros(2)
        cfg = self.config(acc_v)
        code = self._lib.dwa_compute_velocity_command(
            ctypes.byref(cfg), _dptr(state), _dptr(g), n, n, self.params.resolution,
            cost.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), _dptr(obs) if obs.size else None,
            obs.size // 5, _dptr(out))
        return _STATUS[code], float(out[0]), float(out[1])

    def rotate_to_goal_yaw(self, env, goal_yaw: float) -> float:
        """DwaPlanner::rotateToGoalYaw (C++) → 각속도."""
        s = env.state
        state = np.array([s.x, s.y, s.theta, s.v, s.w], dtype=np.float64)
        out = np.zeros(2)
        cfg = self.config(self.acc_v(env))
        self._lib.dwa_rotate_to_goal_yaw(ctypes.byref(cfg), _dptr(state), goal_yaw,
                                         self.params.yaw_goal_tolerance, _dptr(out))
        return float(out[1])

    # ------------------------------------------------------------ 플러그인 (dwa_controller.cpp)
    def command(self, env):
        """DwaController::computeVelocityCommands → (v, w)."""
        p, s = self.params, env.state
        tolerance = env.cfg.arrive_dist if p.xy_goal_tolerance is None else p.xy_goal_tolerance
        if math.hypot(env.goal[0] - s.x, env.goal[1] - s.y) <= tolerance:
            return 0.0, self.rotate_to_goal_yaw(env, 0.0)
        if p.follow_path:
            status, v, w = self.compute_velocity_command_path(env, self.acc_v(env))
        else:
            status, v, w = self.compute_velocity_command(env, self.lookahead_waypoint(env), self.acc_v(env))
        if status != 'ok':
            return 0.0, 0.0
        return v, w

    def __call__(self, env) -> np.ndarray:
        """환경 상태 → 정규화 행동 [v, ω] ∈ [−1, 1]² (env.step: v = (a₀ + 1)/2·v_max, ω = a₁·ω_max)."""
        r = env.cfg.robot
        v, w = self.command(env)
        return np.array([2.0 * v / r.v_max - 1.0, w / r.w_max])
