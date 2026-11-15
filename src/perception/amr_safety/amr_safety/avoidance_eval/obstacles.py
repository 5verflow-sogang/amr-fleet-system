"""회피 KPI(test-plan I17) 시나리오의 동적 장애물 설정을 시드로 만든다. ROS에 의존하지 않는다.

하네스 nav_avoidance가 실행마다 부른다. 고정된 장애물 하나로 30회를 돌리면 시드를 바꿔도 같은 시험이 되므로
(#162 리뷰), 작업자가 통로를 건너는 위치·방향·처음 위치·속도를 시드로 바꾼다. 같은 시드면 같은 설정이다.

- 로봇은 world (0, 1.5)에서 world (6, 1.5)로 A–B 통로(y 0.4~4.4)를 따라 동쪽으로 간다
- 작업자 1명이 x = CROSS_X 안의 한 곳에서 통로를 남북으로 왕복한다 (amr_gazebo straight 패턴)
- 처음 위치를 건너는 선 위 아무 곳으로 두어 로봇과 만나는 시점을 바꾼다 (시작 지연과 같은 효과)
- 시드 3개 중 1개는 원 명세 "1.0 m/s 장애물"로 고정하고, 나머지는 명세 동적 장애물 속도 범위 안에서 뽑는다
- 작업자는 로봇을 피하지 않지만, 로봇이 멈춘 채 앞길을 막으면 그 앞에서 선다 (amr_gazebo yield_to_robots).
  사람은 멈춘 로봇에 걸어 들어가지 않는다. 아무것도 보지 않는 작업자로 돌린 시험 30회(11/1)에서는
  충돌 29건 중 22건이 멈춰 있는 로봇에 작업자가 걸어 들어간 것이었다

형식은 amr_gazebo/config/dynamic_obstacles.yaml과 같다 (dynamic_obstacles.launch.py config:=<파일>).

    ros2 run amr_safety avoidance_obstacles --seed 3 --out /tmp/obstacles.yaml
"""

from __future__ import annotations

import argparse
import math
import random
from typing import Optional

OBSTACLE_ID = 'worker_cross'     # id 앞부분(worker)으로 avoidance_eval이 충돌체 모양(원 r 0.25 m)을 정한다
CROSS_X = (2.5, 4.0)             # world x [m]. 로봇이 출발 뒤 가속을 마치고 목표(6 m) 전에 만나는 구간
CORRIDOR_Y = (0.8, 4.0)          # world y [m]. 통로 벽(0.4·4.4)에서 작업자 반경 0.25 + 0.15 띄움
SPEED_RANGE = (0.5, 1.5)         # m/s. 원 명세 동적 장애물 속도 범위 안
SPEC_SPEED = 1.0                 # m/s. 원 명세 "1.0 m/s 장애물 재계획 회피"
SPEC_SPEED_EVERY = 3             # 시드 % 3 == 0이면 SPEC_SPEED로 고정
YIELD_TO = ('amr_01',)           # 작업자가 멈춰 있으면 비키는 로봇 (시나리오 로봇 1대)

# 정면 접근 (mode headon, nav_avoidance_headon): 작업자가 로봇 경로 위를 마주 걸어와 로봇이 비켜 가야 한다.
# 가로지르기(cross)에서는 로봇이 서서 기다리면 돼 경로를 벗어나지 않아 복귀 시간·재계획 회피가 잡히지 않았다(I17 11/3)
HEADON_ID = 'worker_headon'
HEADON_PATH_Y = 1.5              # world y [m]. 로봇 경로 (출발 world (0, 1.5))
HEADON_OFFSET = 0.3              # m. 경로에서 옆으로 벗어난 정도 최대. 작업자 반경 0.25 + 로봇 반폭 0.2 = 0.45 안이라 늘 길을 막는다
HEADON_SPAWN_X = (10.0, 11.0)    # world x [m]. 목표(map x 9) 너머 1 m 밖에서 출발해 첫 전역 경로에 끼지 않는다
HEADON_END_X = -10.0             # world x [m]. 서쪽 끝. 로봇을 지나친 뒤 측정 동안 다시 돌아오지 않게 멀리 둔다
HEADON_SPEEDS = (0.3, 1.5)       # m/s. 원 명세 동적 장애물 속도 범위 (0.3 = 가장 느린 작업자)


def make_obstacles(seed: int, mode: str = 'cross', speed: Optional[float] = None) -> dict:
    """시드 → dynamic_obstacles.launch.py 설정 (사전). 같은 시드면 같은 값이다.

    mode cross: 통로를 가로지르는 작업자 (I17 본 측정). headon: 로봇 경로를 마주 걸어오는 작업자, speed [m/s] 고정
    (주지 않으면 원 명세 1.0 m/s).
    """
    if mode == 'headon':
        return _headon(seed, SPEC_SPEED if speed is None else speed)
    if mode != 'cross' or speed is not None:
        raise ValueError(f'mode는 cross 또는 headon이고, speed는 headon에서만 쓴다: {mode}, {speed}')
    rng = random.Random(seed)
    x = round(rng.uniform(*CROSS_X), 3)
    southward = rng.random() < 0.5
    start_y = round(rng.uniform(*CORRIDOR_Y), 3)
    speed = SPEC_SPEED if seed % SPEC_SPEED_EVERY == 0 else round(rng.uniform(*SPEED_RANGE), 3)
    # straight 패턴은 처음에 두 번째 waypoint로 향한다. 그쪽이 처음 진행 방향이다
    near, far = (CORRIDOR_Y[1], CORRIDOR_Y[0]) if southward else (CORRIDOR_Y[0], CORRIDOR_Y[1])
    return {'obstacles': [{
        'id': OBSTACLE_ID,
        'type': 'worker',
        'pattern': 'straight',
        'speed_range_mps': [speed, speed],
        'spawn': {'x': x, 'y': start_y, 'yaw': round(-math.pi / 2 if southward else math.pi / 2, 4)},
        'waypoints': [{'x': x, 'y': near}, {'x': x, 'y': far}],
        'yield_to_robots': list(YIELD_TO),
        'note': (f"avoidance_obstacles seed {seed}: x = {x}에서 {'남' if southward else '북'}쪽으로 먼저, "
                 f"{speed} m/s, 처음 y = {start_y}"),
    }]}


def _headon(seed: int, speed: float) -> dict:
    if not HEADON_SPEEDS[0] <= speed <= HEADON_SPEEDS[1]:
        raise ValueError(f'headon speed는 {HEADON_SPEEDS} m/s 안이어야 한다: {speed}')
    rng = random.Random(seed)
    x = round(rng.uniform(*HEADON_SPAWN_X), 3)
    y = round(HEADON_PATH_Y + rng.uniform(-HEADON_OFFSET, HEADON_OFFSET), 3)
    return {'obstacles': [{
        'id': HEADON_ID,
        'type': 'worker',
        'pattern': 'straight',
        'speed_range_mps': [speed, speed],
        'spawn': {'x': x, 'y': y, 'yaw': round(math.pi, 4)},     # 처음에 두 번째 waypoint(서쪽)로 간다
        'waypoints': [{'x': x, 'y': y}, {'x': HEADON_END_X, 'y': y}],
        'yield_to_robots': list(YIELD_TO),
        'note': f'avoidance_obstacles seed {seed} headon: x = {x}에서 서쪽으로 {speed} m/s, y = {y}',
    }]}


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description='I17 회피 시나리오의 동적 장애물 설정을 시드로 만든다')
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True, help='저장할 YAML 경로')
    ap.add_argument('--mode', choices=('cross', 'headon'), default='cross',
                    help='cross: 통로를 가로지른다 (기본, I17 본 측정). headon: 로봇 경로를 마주 걸어온다')
    ap.add_argument('--speed', type=float, help='headon 작업자 속도 [m/s] (기본 1.0)')
    args = ap.parse_args(argv)
    import yaml   # ROS 환경(PyYAML)에서만 쓰는 의존성이라 여기서 읽는다

    config = make_obstacles(args.seed, args.mode, args.speed)
    with open(args.out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
    print(config['obstacles'][0]['note'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
