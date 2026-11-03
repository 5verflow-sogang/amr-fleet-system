"""에피소드 1회 실행과 지표 계산 (학습 콜백·평가 스크립트 공용, 명세서 4-5절)."""

import math

import numpy as np

from .dynamics import footprint_clearance

NEAR_MISS_M = 0.3   # 명세 긴급정지 안전 거리
JERK_SKIP_STEPS = 2  # 출발 순간(초기 속도 → 첫 명령) 스파이크를 jerk에서 뺀다


def time_to_collision(env) -> float:
    """가장 가까운 장애물과 지금 상대 속도 그대로 갈 때 충돌까지 남은 시간 [s]. 멀어지면 inf.

    거리는 로봇 몸체~장애물 표면, 접근 속도는 중심 사이 거리가 줄어드는 속도로 근사한다.
    """
    s, p = env.state, env.cfg.robot
    vx, vy = s.v * math.cos(s.theta), s.v * math.sin(s.theta)
    best = math.inf
    for o in env.obstacles:
        moving = env.t > o.hold_until
        ovx, ovy = (o.vx, o.vy) if moving else (0.0, 0.0)
        dx, dy = o.x - s.x, o.y - s.y
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return 0.0
        closing = ((vx - ovx) * dx + (vy - ovy) * dy) / dist
        if closing > 1e-6:
            best = min(best, max(0.0, footprint_clearance(s, o.x, o.y, o.radius, p)) / closing)
    return best


def run_episode(model, vecnorm, env, seed: int, options=None) -> dict:
    """결정적 정책(평균 행동)으로 한 에피소드를 돌린다. vecnorm은 학습 때 저장한 VecNormalize."""
    return run_policy_episode(
        lambda obs: model.predict(vecnorm.normalize_obs(obs), deterministic=True)[0],
        env, seed, options)


def run_policy_episode(act, env, seed: int, options=None) -> dict:
    """act(obs) → 행동으로 한 에피소드를 돌린다. 규칙 정책은 obs 대신 env 상태를 읽는다."""
    obs, info = env.reset(seed=seed, options=options)
    speeds, min_clear, min_ttc = [info['v']], math.inf, math.inf
    x0 = env.state.x
    total_reward = 0.0
    while True:
        obs, reward, terminated, truncated, info = env.step(act(obs))
        total_reward += reward
        speeds.append(info['v'])
        min_clear = min(min_clear, info['min_clearance'])
        min_ttc = min(min_ttc, time_to_collision(env))
        if terminated or truncated:
            break

    dt = env.cfg.dt
    accel = np.diff(speeds[JERK_SKIP_STEPS:]) / dt
    jerk = np.diff(accel) / dt if len(accel) > 1 else np.zeros(1)
    return {
        'outcome': info['outcome'],
        'success': info['outcome'] == 'success',
        'return': total_reward,                       # 환경 보상 합 (할인 없음). v3 체크포인트 선택에 쓴다
        'collision': info['outcome'] == 'collision',
        'timeout': info['outcome'] == 'timeout',
        'collision_with': info['collision_with'],     # obstacle / wall / None
        'impact_speed': info['impact_speed'],         # 충돌 순간 속도 [m/s]
        'time_s': info['time'],
        'mean_speed': (env.state.x - x0) / info['time'] if info['time'] > 0 else 0.0,
        'min_clearance': min_clear,
        'near_miss': min_clear < NEAR_MISS_M,
        'min_ttc': min_ttc,
        'overshoot_m': info['overshoot'],
        'jerk_rms': float(np.sqrt(np.mean(jerk ** 2))) if jerk.size else 0.0,
        'jerk_max': float(np.max(np.abs(jerk))) if jerk.size else 0.0,
        'payload_kg': info['payload_kg'],
        'corridor_width': info['corridor_width'],
        'popout_appeared': info['popout_appeared'],
        'popout_kind': info['popout_kind'],                # crossing / standing / None
        'popout_stoppable': info['popout_stoppable'],      # 반응 지연 포함 급정지로 설 수 있었는지
        'popout_blocked': info['popout_blocked'],          # 옆으로 비켜 갈 공간이 없었는지
        'popout_result': info['popout_result'],            # stopped / passed / collision / None
        'popout_stop_distance': info['popout_stop_distance'],  # 실제 제동 거리 [m]
        'popout_stop_margin': info['popout_stop_margin'],      # 제동 거리 여유 [m]
        'mass_known': info['mass_known'],
    }
