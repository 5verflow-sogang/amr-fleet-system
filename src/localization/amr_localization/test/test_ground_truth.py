"""world → map 변환 (amr_localization.ground_truth). ROS 없이 돈다."""

import math

import pytest

from amr_localization.ground_truth import world_pose_to_map


def test_zero_origin_is_identity():
    assert world_pose_to_map(3.0, -1.5, 0.4, [0.0, 0.0, 0.0]) == pytest.approx((3.0, -1.5, 0.4))


def test_translation_only_origin_shifts_position_and_keeps_yaw():
    x, y, yaw = world_pose_to_map(5.0, 2.0, 0.0, [2.0, 1.0, 0.0])
    assert (x, y, yaw) == pytest.approx((3.0, 1.0, 0.0))


def test_rotation_only_origin_rotates_position_and_subtracts_yaw():
    # map 원점이 world 기준 90도 돌아 있으면, world (1, 0)은 map (0, -1)이 된다
    x, y, yaw = world_pose_to_map(1.0, 0.0, math.pi / 2, [0.0, 0.0, math.pi / 2])
    assert (x, y, yaw) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_yaw_wraps_to_minus_pi_pi():
    x, y, yaw = world_pose_to_map(0.0, 0.0, -math.pi + 0.1, [0.0, 0.0, 0.3])
    assert -math.pi <= yaw <= math.pi
    # -pi + 0.1 - 0.3 = -pi - 0.2는 범위를 벗어나므로 2*pi를 더해 감는다
    expected = -math.pi + 0.1 - 0.3 + 2 * math.pi
    assert yaw == pytest.approx(expected)
