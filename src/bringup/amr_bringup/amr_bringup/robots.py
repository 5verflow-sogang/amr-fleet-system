"""system.launch.py가 쓰는 로봇 목록·좌표 변환. rclpy와 launch를 import하지 않아 ROS 없이 테스트한다."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml


@dataclass(frozen=True)
class RobotSpawn:
    name: str
    # world 좌표 스폰 자세
    x: float
    y: float
    yaw: float


def load_robots(path: str, count: int) -> list[RobotSpawn]:
    """config/robots.yaml에서 앞의 count대를 읽는다. 모자라면 ValueError."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    robots = [RobotSpawn(str(name), float(p[0]), float(p[1]), float(p[2]))
              for name, p in (data.get('robots') or {}).items()]
    if count < 1:
        raise ValueError(f'num_robots는 1 이상이어야 한다: {count}')
    if count > len(robots):
        raise ValueError(f'num_robots={count}인데 {path}에는 {len(robots)}대뿐이다')
    return robots[:count]


def world_to_map(x: float, y: float, yaw: float, map_origin_in_world: Sequence[float]) -> tuple[float, float, float]:
    """world 좌표 자세를 map 좌표로. map_origin_in_world는 world에서 본 map 원점 [x, y, yaw]."""
    ox, oy, oyaw = map_origin_in_world
    dx, dy = x - ox, y - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    rel_yaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
    return c * dx - s * dy, s * dx + c * dy, rel_yaw


def encoder_seed(run_seed: int, index: int) -> int:
    """로봇별 엔코더 노이즈 seed. 실행 seed가 0이면 0(amr_odometry에서 "매번 무작위")을 그대로 넘긴다.

    하네스는 seed를 1부터 준다. 로봇마다 노이즈가 독립이도록 실행 seed마다 다른 구간을 쓴다.
    """
    return 0 if run_seed == 0 else run_seed * 100 + index + 1
