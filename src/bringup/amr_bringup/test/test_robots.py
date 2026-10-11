"""로봇 목록·좌표 변환·seed. ROS 없이 돈다."""

import math
from pathlib import Path

import pytest

import yaml

from amr_bringup.robots import encoder_seed, load_robots, obstacle_clearance, world_to_map

ROBOTS = str(Path(__file__).resolve().parents[1] / 'config' / 'robots.yaml')
# 같은 저장소의 amr_gazebo 동적 장애물 설정 (src/bringup/amr_bringup/test → src/simulation/amr_gazebo)
OBSTACLES = Path(__file__).resolve().parents[3] / 'simulation' / 'amr_gazebo' / 'config' / 'dynamic_obstacles.yaml'
# 로봇 반 길이 0.3 m + 지게차 반 폭 + 여유
CLEARANCE_M = 1.0


def test_shipped_robots_in_waiting_zone():
    robots = load_robots(ROBOTS, 5)
    assert [r.name for r in robots] == ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']
    # 대기 구역 x 16~26, y −20~−14 (amr_gazebo README)
    assert all(16 < r.x < 26 and -20 < r.y < -14 for r in robots)
    # 로봇 몸체 0.6 m보다 넉넉히 떨어져 있다
    xs = sorted(r.x for r in robots)
    assert min(b - a for a, b in zip(xs, xs[1:])) >= 1.5


def test_load_robots_count():
    assert len(load_robots(ROBOTS, 3)) == 3
    with pytest.raises(ValueError):
        load_robots(ROBOTS, 6)
    with pytest.raises(ValueError):
        load_robots(ROBOTS, 0)


def test_world_to_map():
    # 지도 v0: world (0, 1.5)가 map (0, 0)
    assert world_to_map(17.0, -17.0, math.pi / 2, [0.0, 1.5, 0.0]) == pytest.approx((17.0, -18.5, math.pi / 2))
    x, y, yaw = world_to_map(1.0, 0.0, 0.0, [0.0, 0.0, math.pi / 2])
    assert (x, y, yaw) == pytest.approx((0.0, -1.0, -math.pi / 2))


def test_encoder_seed():
    # 0은 amr_odometry에서 "매번 무작위"라 그대로 넘긴다
    assert encoder_seed(0, 3) == 0
    seeds = [encoder_seed(s, i) for s in (1, 2) for i in range(5)]
    assert len(set(seeds)) == len(seeds)
    assert all(v > 0 for v in seeds)


def test_obstacle_clearance():
    random_area = {'pattern': 'random', 'waypoints': [{'x': 5.0, 'y': -13.0}, {'x': 18.0, 'y': -17.0}]}
    assert obstacle_clearance(17.0, -15.0, random_area) == 0.0          # 범위 안
    assert obstacle_clearance(17.0, -19.0, random_area) == pytest.approx(2.0)
    path = {'pattern': 'straight', 'waypoints': [{'x': 0.0, 'y': 0.0}, {'x': 10.0, 'y': 0.0}]}
    assert obstacle_clearance(5.0, 3.0, path) == pytest.approx(3.0)
    assert obstacle_clearance(-4.0, 3.0, path) == pytest.approx(5.0)    # 끝점까지


def test_spawns_clear_of_dynamic_obstacles():
    # dynamic_obstacles:=true일 때 서 있는 로봇을 장애물이 치지 않게 한다 (#95 지수 리뷰: forklift_2와 겹쳤음)
    if not OBSTACLES.exists():
        pytest.skip('amr_gazebo 설정이 없다 (패키지만 따로 받은 경우)')
    obstacles = yaml.safe_load(OBSTACLES.read_text(encoding='utf-8'))['obstacles']
    for robot in load_robots(ROBOTS, 5):
        for obstacle in obstacles:
            clearance = obstacle_clearance(robot.x, robot.y, obstacle)
            assert clearance >= CLEARANCE_M, (robot.name, obstacle['id'], round(clearance, 2))
