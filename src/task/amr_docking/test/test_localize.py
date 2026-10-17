"""마커판 → 로봇 자세 (localize.py)."""
import math
import random

import pytest

from amr_docking.geometry import dock_error
from amr_docking.localize import (Pose2D, SigmaModel, board_in_base, inverse, robot_from_board, transform)
from amr_docking.markers import BOARDS, DockStation


def markers_in_base(robot: Pose2D, station: DockStation):
    """로봇이 판을 바라볼 때 (왼쪽, 오른쪽) 마커 중심을 로봇 기준으로."""
    bx, by, _ = station.board_center
    t = station.board_yaw
    ux, uy = -math.sin(t), math.cos(t)    # n(판 바깥)을 +90° 돌린 것 = 로봇이 볼 때 왼→오
    half = station.board.spacing / 2
    to_base = inverse(robot)
    out = []
    for k in (-half, half):
        p = transform(Pose2D(bx + k * ux, by + k * uy, 0.0), to_base)
        out.append((p.x, p.y))
    return out


RACK = DockStation('rack_a5', 15, 7.5, 3.6, math.pi / 2, 0.795, BOARDS['small'])
DOCK = DockStation('dock_1', 0, -27.0, 16.0, math.pi, 1.495, BOARDS['large'])


@pytest.mark.parametrize('station', [RACK, DOCK])
def test_at_dock_pose_board_is_straight_ahead(station):
    robot = Pose2D(station.x, station.y, station.yaw)
    board = board_in_base(*markers_in_base(robot, station))
    assert board.x == pytest.approx(station.standoff)
    assert board.y == pytest.approx(0.0, abs=1e-9)
    assert abs(board.yaw) == pytest.approx(math.pi)          # 판은 로봇을 바라본다
    got = robot_from_board(board, Pose2D(*station.board_center[:2], station.board_yaw))
    assert (got.x, got.y, got.yaw) == pytest.approx((robot.x, robot.y, robot.yaw))


@pytest.mark.parametrize('seed', range(20))
def test_recovers_robot_pose_from_any_view(seed):
    rng = random.Random(seed)
    station = rng.choice([RACK, DOCK])
    # 판 앞 0.5~5 m, 옆 ±1.5 m, 판을 대충 바라보는 자세
    bx, by, _ = station.board_center
    n = station.board_yaw
    d, side = rng.uniform(0.5, 5.0), rng.uniform(-1.5, 1.5)
    x = bx + d * math.cos(n) - side * math.sin(n)
    y = by + d * math.sin(n) + side * math.cos(n)
    robot = Pose2D(x, y, math.atan2(by - y, bx - x) + rng.uniform(-0.4, 0.4))
    board = board_in_base(*markers_in_base(robot, station))
    got = robot_from_board(board, Pose2D(bx, by, station.board_yaw))
    assert (got.x, got.y) == pytest.approx((robot.x, robot.y), abs=1e-9)
    assert math.cos(got.yaw - robot.yaw) == pytest.approx(1.0)


def test_agrees_with_dock_error():
    """도킹 오차(geometry.dock_error)로 구한 로봇 자세와 같다: 같은 판 규칙을 쓴다."""
    station = RACK
    robot = Pose2D(station.x - 0.3, station.y - 0.5, station.yaw + 0.1)
    left, right = markers_in_base(robot, station)
    e = dock_error(left, right, station.standoff)
    got = robot_from_board(board_in_base(left, right), Pose2D(*station.board_center[:2], station.board_yaw))
    # 도킹 자세 기준 로봇: 진행 방향 −along, 옆 lateral, 방향 heading
    expect = transform(Pose2D(-e.along, e.lateral, e.heading), Pose2D(station.x, station.y, station.yaw))
    assert (got.x, got.y) == pytest.approx((expect.x, expect.y), abs=1e-9)
    assert got.yaw == pytest.approx(expect.yaw)


def test_inverse_and_transform_round_trip():
    f = Pose2D(1.0, -2.0, 0.7)
    p = Pose2D(0.3, 0.4, -1.2)
    back = transform(transform(p, f), inverse(f))
    assert (back.x, back.y, back.yaw) == pytest.approx((p.x, p.y, p.yaw))


def test_overlapping_markers_rejected():
    with pytest.raises(ValueError):
        board_in_base((1.0, 0.0), (1.0, 0.0))


def test_sigma_grows_with_distance_and_shrinks_with_board_size():
    m = SigmaModel()
    near, far = m.sigma(1.0), m.sigma(5.0)
    assert far[0] > near[0] and far[1] > near[1]
    assert m.sigma(0.0) == pytest.approx((m.xy0, m.yaw0))
    small, large = m.sigma(4.0, BOARDS['small'].spacing), m.sigma(4.0, BOARDS['large'].spacing)
    assert large[0] < small[0] and large[1] < small[1]
