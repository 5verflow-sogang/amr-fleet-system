import math

import numpy as np
import pytest

from amr_safety.gate import limit_velocity, min_clearance, path_horizon, scan_body_points

LENGTH, WIDTH = 0.6, 0.4
LIDAR_X = 0.15     # lidar_link는 base_footprint에서 앞으로 0.15 m (api.md 2절)


def test_no_limit_passes_through():
    assert limit_velocity(1.8, -0.4, math.inf) == (1.8, -0.4)


def test_stop_blocks_rotation_too():
    assert limit_velocity(0.0, 1.0, 0.0) == (0.0, 0.0)


def test_warning_scales_keeping_curvature():
    vx, wz = limit_velocity(1.2, 0.6, 0.6)
    assert vx == pytest.approx(0.6)
    assert wz / vx == pytest.approx(0.6 / 1.2)


def test_warning_limits_reverse_and_keeps_slow_commands():
    assert limit_velocity(-1.0, 0.0, 0.6) == pytest.approx((-0.6, 0.0))
    assert limit_velocity(0.3, 0.5, 0.6) == (0.3, 0.5)
    assert limit_velocity(0.0, 1.0, 0.6) == (0.0, 1.0)      # 제자리 회전은 그대로


@pytest.mark.parametrize('cmd', [(math.nan, 0.0), (0.5, math.inf)])
def test_invalid_command_becomes_stop(cmd):
    assert limit_velocity(*cmd, math.inf) == (0.0, 0.0)


def _clearance(ranges, angle_min=0.0, increment=math.pi / 2, range_min=0.12, range_max=25.0):
    points = scan_body_points(ranges, angle_min, increment, range_min, range_max,
                              0.0, (LIDAR_X, 0.0))
    return min_clearance(points, LENGTH, WIDTH)


def test_scan_clearance_uses_body_frame():
    # 빔 0 (정면) 1.0 m → 몸체 x = 1.15, 앞면 0.3에서 0.85. 빔 1 (왼쪽) 0.5 m → y 0.5, 옆면 0.2에서 0.3
    assert _clearance([1.0, 0.5]) == pytest.approx(0.3)


def test_scan_clearance_skips_invalid_beams():
    assert _clearance([math.inf, math.nan, 0.05, 30.0]) == math.inf
    assert _clearance([0.05, 2.0]) == pytest.approx(2.0 - 0.2)


def test_scan_clearance_rear_point():
    # 빔 2 (뒤) 0.6 m → 몸체 x = 0.15 − 0.6 = −0.45, 뒷면 −0.3에서 0.15
    assert _clearance([np.inf, np.inf, 0.6]) == pytest.approx(0.15)


def test_scan_body_points_shape():
    points = scan_body_points([1.0, math.nan, 0.5], 0.0, math.pi / 2, 0.12, 25.0,
                              0.0, (LIDAR_X, 0.0))
    np.testing.assert_allclose(points, [[1.15, 0.0], [LIDAR_X - 0.5, 0.0]], atol=1e-9)


def test_path_horizon_reaction_plus_stop_time():
    assert path_horizon(0.6, 0.2, 1.0, 0.5) == pytest.approx(0.8)
    assert path_horizon(-2.0, 0.2, 1.0, 0.5) == pytest.approx(2.2)
    assert path_horizon(0.0, 0.2, 1.0, 0.5) == pytest.approx(0.5)     # 제자리 회전
