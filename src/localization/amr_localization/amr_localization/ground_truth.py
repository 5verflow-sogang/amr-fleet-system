"""world 좌표 정답 위치를 map 좌표로 바꾸는 순수 함수. ROS 없이 돈다.

amr_bringup.robots.world_to_map과 같은 식이다 - 패키지마다 자기 변환을 갖는 관례(각 패키지가
rclpy 없이 테스트 가능한 자기 수학을 둔다)를 따라 여기서도 그대로 둔다.
"""

from __future__ import annotations

import math
from typing import Sequence


def world_pose_to_map(
    x: float, y: float, yaw: float, map_origin_in_world: Sequence[float],
) -> tuple[float, float, float]:
    """world 좌표 자세를 map 좌표로. map_origin_in_world는 world에서 본 map 원점 [x, y, yaw].

    쿼터니언 ↔ yaw 변환은 amr_localization.eval.trajectory의 yaw_from_quaternion·
    quaternion_from_yaw를 쓴다 (같은 패키지 안이라 중복 안 둔다).
    """
    ox, oy, oyaw = map_origin_in_world
    dx, dy = x - ox, y - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    rel_yaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
    return c * dx - s * dy, s * dx + c * dy, rel_yaw
