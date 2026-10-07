"""로봇 목록·좌표 변환·seed. ROS 없이 돈다."""

import math
from pathlib import Path

import pytest

from amr_bringup.robots import encoder_seed, load_robots, world_to_map

ROBOTS = str(Path(__file__).resolve().parents[1] / 'config' / 'robots.yaml')


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
