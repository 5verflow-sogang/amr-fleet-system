"""서 있다가 로봇 쪽으로 걷기 시작하는 사람 (#361 리뷰 2번): 추적 → 안전 판정 → 로봇 감속을 이어 붙인 모의 실험.

#361은 TTC·바깥 경고 원을 '지금 움직이는'(추정 속력 ≥ moving_min_speed) 동적 트랙에만 건다. 서 있던 사람이 걷기
시작하면 추정 속력이 오르는 동안(약 0.5 s) TTC가 빠지고, 오래 서 있어 정적으로 내려간 사람은 입구 조건을 다시 넘을
때까지 동적이 아니다. 그 사이는 진행 경로 판정(전방 0.3 m 띠, 0.8 m/s면 약 1.0 s 앞까지)과 근접 저속(0.4 m)만 지킨다.
여기서는 그 공백이 실제로 부딪힘으로 이어지는지 본다.

모의 조건 (값은 tracking.yaml, safety.yaml + safety_restaurant.yaml):
- 로봇: 서빙 로봇 0.565 x 0.537 m, 0.8 m/s 직진 명령(식당 DWA max_v). DWA 회피는 빼고 안전 노드만 본다(보수적)
  안전 노드 20 Hz, 반응 0.2 s 뒤 1.0 m/s²로 속도 상한을 따른다
- 사람: 몸 반경 0.2 m, 다리 군집 점 8개(반경 0.15 m), 추적 측정 10 Hz·잡음 0.03 m, 트랙 반경 0.25 m(사람 최소 반경)
  걷기 시작하면 0.4 s 동안 1.0 m/s까지 올리고 로봇 경로를 가로지른다. 빈 공간 증거는 움직이는 동안만 있다
- 사람이 서 있던 이력: 'static' = 처음부터 서 있음(동적이 된 적 없음), 'paused' = 걸어와서 1.5 s 서 있음(멈춤)
"""

import math

import numpy as np
import pytest

from amr_perception_core.collision import footprint_clearance, footprint_time_to_collision, path_hazard
from amr_perception_core.kalman import KalmanConfig
from amr_perception_core.safety import (LEVEL_CRITICAL, LEVEL_WARNING, ObstacleInfo, SafetyConfig,
                                        SafetyMonitor)
from amr_perception_core.tracker import MultiTargetTracker, TrackerConfig

LENGTH, WIDTH = 0.565, 0.537          # 서빙 로봇 (safety_restaurant.yaml)
PERSON_RADIUS = 0.2                   # 몸 (접촉 판정)
TRACK_RADIUS = 0.25                   # 트랙 반경 (tracking.yaml min_radius_values person)
REACTION, DECEL, ACCEL = 0.2, 1.0, 1.0
CRUISE = 0.8                          # 식당 DWA max_v (nav2_profile)
DT = 0.05                             # 안전 노드 20 Hz. 추적은 2주기(10 Hz)마다

TRACKING = dict(
    kalman=KalmanConfig(accel_noise_std=1.0, measurement_noise_std=0.05, initial_velocity_std=1.5),
    gate_mahalanobis_sq=13.82, confirm_hits=3, max_coast_time=2.0, dynamic_speed_threshold=0.25,
    dynamic_confirm_frames=5, dynamic_evidence_frames=3, radius_smoothing=0.3, maneuver_max_speed=1.5,
    maneuver_margin=0.15, maneuver_max_age=0.5, maneuver_max_radius=0.4)
TRACKING_361 = dict(dynamic_min_displacement=0.3, dynamic_evidence_window=10, dynamic_release_frames=30,
                    dynamic_pause_frames=100, dynamic_release_speed=0.1, dynamic_release_max_displacement=0.5)
SAFETY = dict(critical_distance=0.3, ttc_critical=1.0, ttc_warning=2.0, dynamic_warning_distance=1.0,
              warning_speed_limit=0.6, dynamic_slow_distance=0.4, dynamic_slow_speed_limit=0.3,
              dynamic_slow_skip_receding_behind=True, dynamic_slow_unconfirmed_speed=0.5,
              dynamic_slow_unconfirmed_max_radius=0.35, dynamic_slow_unconfirmed_max_speed=1.8,
              release_hold_s=0.5)


WALK_IN_END = 1.5                     # 'paused': 0~1.5 s 동안 1.0 m/s로 걸어와 선다


def _person_path(history, y0, x_p, t_walk, walk_speed=1.0, ramp=0.4):
    """사람 위치 함수 t -> ((x, y), 속력). history: 'static' | 'paused'."""
    def at(t):
        if t >= t_walk:                        # 로봇 경로(y = 0)를 가로질러 -y로
            s = t - t_walk
            d = walk_speed * (s * s / (2 * ramp) if s < ramp else s - ramp / 2)
            v = walk_speed * min(s / ramp, 1.0)
            return (x_p, y0 - d), v
        if history == 'paused' and t < WALK_IN_END:
            return (x_p + (WALK_IN_END - t) * 1.0, y0), 1.0
        return (x_p, y0), 0.0
    return at


def simulate(history, y0, ahead, moving_min_speed, tracker_361=True, seed=0, t_walk=3.0, t_end=8.0,
             x_person=None, trace=None):
    """한 번 돌리고 {contact, min_clearance, t_warning, t_critical, any_critical}을 돌려준다.

    t_warning·t_critical·min_clearance는 걷기 시작부터 잰다 [s]. any_critical은 처음부터.
    ahead: 걷기 시작할 때, 막지 않으면 둘이 만나는 자리까지 로봇이 가야 할 거리에서 더하거나 뺄 값 [m]
    (0이면 사람 중심이 로봇 중심 경로에 닿는 순간 로봇 중심도 그 x에 온다). x_person을 주면 그 자리에 둔다.
    trace에 리스트를 주면 주기마다 (걷기 시작부터 시각, 로봇 속도, 안전 단계, 판정 내용, 동적, 트랙 속력, 실제 간격)을 쌓는다.
    """
    rng = np.random.default_rng(seed)
    tracker = MultiTargetTracker(TrackerConfig(**TRACKING, **(TRACKING_361 if tracker_361 else {})))
    monitor = SafetyMonitor(SafetyConfig(**SAFETY, moving_min_speed=moving_min_speed))
    t_cross = (y0 / 1.0) + 0.2                          # 램프 0.4 s의 절반만큼 늦게 닿는다
    x_p = CRUISE * (t_walk + t_cross) + ahead if x_person is None else x_person
    person = _person_path(history, y0, x_p, t_walk)
    x, v = 0.0, CRUISE
    pending = []                                       # (적용 시각, 속도 상한): 반응 지연
    limit_now = math.inf
    tracks = []
    out = dict(contact=False, min_clearance=math.inf, t_warning=None, t_critical=None, any_critical=False)
    angles = np.linspace(0.0, 2 * math.pi, 8, endpoint=False)
    for k in range(int(round(t_end / DT))):
        t = k * DT
        (px, py), pspeed = person(t)
        body = np.array([[px - x, py]])
        true_clear = float(footprint_clearance(body, LENGTH, WIDTH)[0]) - PERSON_RADIUS
        if t >= t_walk:
            out['min_clearance'] = min(out['min_clearance'], true_clear)
            out['contact'] |= true_clear <= 0.0
        if k % 2 == 0:                                 # LiDAR·추적 10 Hz
            z = (px + rng.normal(0.0, 0.03), py + rng.normal(0.0, 0.03))
            to_robot = np.array([x - z[0], -z[1]])
            ref = np.array(z) + 0.2 * to_robot / max(np.hypot(*to_robot), 1e-6)
            tracks = tracker.step(t, [z], radii=[TRACK_RADIUS], free_evidence=[pspeed > 0.1],
                                  ref_points=[ref])
        infos = []
        for tr in tracks:
            (tx, ty), (tvx, tvy) = tr.kf.position, tr.kf.velocity
            rel = np.array([tx - x, ty])
            clear = float(footprint_clearance(rel[None, :], LENGTH, WIDTH)[0]) - tr.radius
            ttc = footprint_time_to_collision(rel, (tvx - v, tvy), tr.radius, LENGTH, WIDTH, horizon=5.0)
            behind = rel[0] < -(LENGTH / 2 + tr.radius)
            infos.append(ObstacleInfo(track_id=tr.track_id, clearance=max(clear, 0.0), ttc=ttc,
                                      is_dynamic=tr.is_dynamic, receding_behind=behind and tvx - v < 0.0,
                                      speed=math.hypot(tvx, tvy), radius=tr.radius))
        legs = np.stack([px - x + 0.15 * np.cos(angles), py + 0.15 * np.sin(angles)], axis=1)
        scan_min = float(np.min(footprint_clearance(legs, LENGTH, WIDTH)))
        horizon = max(0.5, REACTION + CRUISE / DECEL)
        path_clear = path_hazard(legs, CRUISE, 0.0, LENGTH, WIDTH, 0.3, 0.05, horizon)
        state = monitor.update(t, scan_min, infos, 0.0, 0.0, 0.0, path_clear)
        out['any_critical'] |= state.level >= LEVEL_CRITICAL
        if trace is not None:
            tr = tracks[0] if tracks else None
            trace.append((round(t - t_walk, 2), round(v, 3), state.level, state.detail,
                          tr.is_dynamic if tr else None, round(tr.kf.speed, 2) if tr else None,
                          round(true_clear, 3)))
        if t >= t_walk:
            s = round(t - t_walk, 3)
            if state.level >= LEVEL_WARNING and out['t_warning'] is None:
                out['t_warning'] = s
            if state.level >= LEVEL_CRITICAL and out['t_critical'] is None:
                out['t_critical'] = s
        pending.append((t + REACTION, state.speed_limit))
        while pending and pending[0][0] <= t + 1e-9:
            limit_now = pending.pop(0)[1]
        target = min(CRUISE, limit_now)
        v = max(target, v - DECEL * DT) if v > target else min(target, v + ACCEL * DT)
        x += v * DT
    return out


CASES = [(h, y0, ahead) for h in ('static', 'paused') for y0 in (0.8, 1.2, 1.6)
         for ahead in (-0.4, -0.2, 0.0, 0.2, 0.4)]


@pytest.mark.parametrize('history,y0,ahead', CASES)
def test_start_walking_is_not_worse_than_before_361(history, y0, ahead):
    # 경로로 바로 뛰어드는 사람은 #361 전에도 0.8 m/s 로봇이 다 피하지 못한다 (90번 중 62번 접촉: 사람이 로봇 옆구리로
    # 걸어 들어오는 경우가 많다). 그래서 '안 부딪힘'이 아니라 '#361 전보다 나빠지지 않음'을 본다
    for seed in (0, 1, 2):
        new = simulate(history, y0, ahead, moving_min_speed=0.15, seed=seed)
        old = simulate(history, y0, ahead, moving_min_speed=0.0, tracker_361=False, seed=seed)
        assert new['min_clearance'] >= old['min_clearance'] - 0.02, (seed, new, old)
        assert old['contact'] or not new['contact'], (seed, new, old)


@pytest.mark.parametrize('history', ['static', 'paused'])
def test_person_standing_beside_path_does_not_stop_robot(history):
    # 걷기 시작하지 않고 경로 옆 0.8 m에 서 있기만 하면 정지가 걸리지 않는다
    result = simulate(history, 0.8, 0.0, moving_min_speed=0.15, t_walk=100.0, t_end=7.0, x_person=4.0)
    assert not result['any_critical'], result
