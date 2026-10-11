import math

import numpy as np
import pytest

from amr_perception_core.collision import (
    circumscribed_radius,
    footprint_clearance,
    footprint_time_to_collision,
    path_hazard,
    time_to_collision,
)

# 명세 차체 0.6 × 0.4 m
LENGTH, WIDTH = 0.6, 0.4


def test_circumscribed_radius_of_spec_body():
    assert circumscribed_radius(LENGTH, WIDTH) == pytest.approx(math.hypot(0.3, 0.2))


def test_footprint_clearance_front_side_corner_inside():
    points = [[1.0, 0.0],     # 앞: 1.0 − 0.3
              [0.0, -0.5],    # 오른쪽: 0.5 − 0.2
              [0.6, 0.6],     # 모서리 (0.3, 0.2)에서 (0.3, 0.4)만큼
              [0.1, 0.1]]     # 차체 안
    np.testing.assert_allclose(footprint_clearance(points, LENGTH, WIDTH),
                               [0.7, 0.3, 0.5, 0.0])


def test_footprint_clearance_empty():
    assert footprint_clearance(np.empty((0, 2)), LENGTH, WIDTH).shape == (0,)


def test_ttc_head_on_closed_form():
    # 5 m 앞 장애물이 1 m/s로 다가옴, 반지름 1 → (5 − 1) / 1
    assert time_to_collision([5.0, 0.0], [-1.0, 0.0], 1.0) == pytest.approx(4.0)


def test_ttc_off_axis_hit():
    # 옆으로 0.6 떨어져 지나가는데 반지름 1 → 원이 닿는다: x² + 0.36 = 1 → x = 0.8
    assert time_to_collision([5.0, 0.6], [-2.0, 0.0], 1.0) == pytest.approx((5.0 - 0.8) / 2.0)


def test_ttc_miss_receding_parallel_are_inf():
    assert time_to_collision([5.0, 2.0], [-1.0, 0.0], 1.0) == math.inf     # 비껴감
    assert time_to_collision([5.0, 0.0], [1.0, 0.0], 1.0) == math.inf      # 멀어짐
    assert time_to_collision([5.0, 0.0], [0.0, 0.0], 1.0) == math.inf      # 둘 다 정지
    assert time_to_collision([0.0, 5.0], [1.0, 0.0], 1.0) == math.inf      # 거리 그대로 (b = 0)


def test_ttc_overlap_is_zero():
    assert time_to_collision([0.5, 0.0], [1.0, 0.0], 1.0) == 0.0


def test_ttc_robot_approaching_static_obstacle():
    # 정지 장애물 쪽으로 로봇이 0.5 m/s: 상대 속도 = 0 − 0.5
    assert time_to_collision([3.0, 0.0], [-0.5, 0.0], 0.5) == pytest.approx(5.0)


def test_ttc_rejects_nan():
    with pytest.raises(ValueError):
        time_to_collision([math.nan, 0.0], [0.0, 0.0], 1.0)


# --- 차체 사각형 TTC ---

def test_footprint_ttc_head_on():
    # 앞 2 m 장애물(반경 0.2)이 1 m/s로 다가옴 → 앞면 0.3 + 반경 0.2에 닿을 때: (2 − 0.5) / 1
    ttc = footprint_time_to_collision([2.0, 0.0], [-1.0, 0.0], 0.2, LENGTH, WIDTH)
    assert ttc == pytest.approx(1.5, abs=1e-6)


def test_footprint_ttc_side_pass_in_narrow_gap_is_inf():
    # 옆 0.1 m 틈으로 나란히 지나감 (0.6 m 통로의 벽). 외접원(0.36 m) TTC는 충돌로 본다
    assert footprint_time_to_collision([1.0, 0.35], [-0.5, 0.0], 0.05, LENGTH, WIDTH) == math.inf
    assert math.isfinite(time_to_collision([1.0, 0.35], [-0.5, 0.0], 0.36 + 0.05))


def test_footprint_ttc_overlap_only_zero_when_approaching():
    assert footprint_time_to_collision([0.4, 0.0], [-0.5, 0.0], 0.2, LENGTH, WIDTH) == 0.0
    assert footprint_time_to_collision([0.4, 0.0], [0.5, 0.0], 0.2, LENGTH, WIDTH) == math.inf
    assert footprint_time_to_collision([0.4, 0.0], [0.0, 0.0], 0.2, LENGTH, WIDTH) == math.inf


def test_footprint_ttc_beyond_horizon_is_inf():
    assert footprint_time_to_collision([10.0, 0.0], [-1.0, 0.0], 0.2, LENGTH, WIDTH,
                                       horizon=5.0) == math.inf


def test_footprint_ttc_crossing_from_side():
    # 왼쪽 1.2 m에서 오른쪽으로 1 m/s: 옆면 0.2 + 반경 0.25 → (1.2 − 0.45) / 1
    ttc = footprint_time_to_collision([0.0, 1.2], [0.0, -1.0], 0.25, LENGTH, WIDTH)
    assert ttc == pytest.approx(0.75, abs=1e-6)


# --- 진행 경로 위험 ---

def _hazard(points, v, w, horizon=0.8):
    return path_hazard(np.asarray(points, float), v, w, LENGTH, WIDTH, front_margin=0.3,
                       side_margin=0.05, horizon=horizon)


def test_path_hazard_wall_ahead_blocks_forward_but_not_reverse():
    wall = [[0.5, y] for y in np.linspace(-1.0, 1.0, 41)]      # 앞면에서 0.2 m
    assert _hazard(wall, 0.5, 0.0) == pytest.approx(0.2)
    assert _hazard(wall, -0.3, 0.0) is None                       # 벽에서 멀어짐 (교착 해소)
    assert _hazard(wall, 0.0, 0.0) is None                        # 서 있으면 위험 없음


def test_path_hazard_obstacle_ahead_beyond_front_band():
    # 앞면 0.5 m 앞 점: 0.5 m/s로 0.8 s면 0.4 m 가서 띠(0.3 m) 안에 들어온다
    assert _hazard([[0.8, 0.0]], 0.5, 0.0) == pytest.approx(0.5)
    assert _hazard([[1.5, 0.0]], 0.5, 0.0) is None               # 아직 멀다


def test_path_hazard_narrow_corridor_side_walls_pass():
    # 0.6 m 통로: 양옆 벽이 차체 옆면에서 0.1 m. 직진은 막지 않는다
    walls = [[x, s * 0.3] for x in np.linspace(-1.0, 3.0, 81) for s in (1, -1)]
    assert _hazard(walls, 0.6, 0.0) is None


def test_path_hazard_rear_obstacle_blocks_reverse_only():
    rear = [[-0.5, 0.0]]
    assert _hazard(rear, -0.3, 0.0) == pytest.approx(0.2)
    assert _hazard(rear, 0.5, 0.0) is None


def test_path_hazard_rotation_sweeps_corner():
    # 앞 왼쪽 모서리 (0.3, 0.2) 근처 45° 방향 점 (중심에서 0.354 m < 모서리 회전 반경 0.36 m).
    # 왼쪽(반시계)으로 돌면 모서리가 쓸고 지나가고, 오른쪽으로 돌면 멀어진다
    point = [[0.25, 0.25]]
    assert _hazard(point, 0.0, 1.0) is not None
    assert _hazard(point, 0.0, -1.0) is None
    assert _hazard([[2.0, 0.0]], 0.0, 1.0) is None


def test_path_hazard_turning_into_wall():
    # 왼쪽 0.25 m 벽 (옆면에서 0.05 m 초과)을 향해 왼쪽으로 꺾으며 전진
    wall = [[x, 0.45] for x in np.linspace(-1.0, 2.0, 61)]
    assert _hazard(wall, 0.5, 1.0) is not None
    assert _hazard(wall, 0.5, 0.0) is None


def test_path_hazard_ignores_nan_and_empty():
    assert _hazard(np.empty((0, 2)), 0.5, 0.0) is None
    assert _hazard([[math.nan, 0.0]], 0.5, 0.0) is None
