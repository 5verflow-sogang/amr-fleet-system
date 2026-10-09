import math

import numpy as np
import pytest

from amr_perception_core.free_space import FreeSpaceGrid

N = 720
ANGLES = -math.pi + np.radians(0.5) * np.arange(N)
DIRS = np.c_[np.cos(ANGLES), np.sin(ANGLES)]


def wall_ranges(x_wall):
    """센서 원점 기준 x = x_wall 벽 (앞쪽 빔만 맞고 나머지는 반사 없음)."""
    with np.errstate(divide='ignore'):
        r = np.where(np.cos(ANGLES) > 0.05, x_wall / np.cos(ANGLES), np.inf)
    return r


def test_point_entering_seen_free_space_is_evidence():
    g = FreeSpaceGrid()
    for t in (0.0, 0.1, 0.2):
        g.integrate((0.0, 0.0), DIRS, wall_ranges(5.0), t)
    # 1 m 앞(빔이 지나간 빈 칸)에 새 점 → 증거. 벽 위 점 → 아님 (계속 맞는 칸)
    assert g.evidence([[1.0, 0.0], [5.0, 0.0], [5.0, 1.0]], 0.3).tolist() == [True, False, False]


def test_never_observed_cell_is_not_evidence():
    g = FreeSpaceGrid()
    g.integrate((0.0, 0.0), DIRS, wall_ranges(3.0), 0.0)
    # 벽 뒤(가려진 곳)와 격자 밖
    assert g.evidence([[4.0, 0.0], [100.0, 0.0]], 0.1).tolist() == [False, False]


def test_old_free_observation_expires():
    g = FreeSpaceGrid(free_memory=1.0)
    g.integrate((0.0, 0.0), DIRS, wall_ranges(5.0), 0.0)
    assert g.evidence([[2.0, 0.0]], 0.5).tolist() == [True]
    assert g.evidence([[2.0, 0.0]], 2.0).tolist() == [False]


def test_recent_hit_cell_is_not_evidence():
    # 사람이 서 있던 칸 (1초 안에 맞음)은 비어 보였다가도 증거가 아니다 (hit_quiet)
    g = FreeSpaceGrid(hit_quiet=1.0)
    g.integrate((0.0, 0.0), DIRS, wall_ranges(2.0), 0.0)     # 2 m에 벽(사람)이 맞음
    g.integrate((0.0, 0.0), DIRS, wall_ranges(5.0), 0.3)     # 비켜서 2 m 칸이 빈 칸으로 보임
    assert g.evidence([[2.0, 0.0]], 0.5).tolist() == [False]
    assert g.evidence([[2.0, 0.0]], 1.2).tolist() == [True]


def test_sliding_wall_fragment_gets_no_evidence():
    # 로봇이 벽과 나란히 움직여도 벽 위 점들은 늘 맞는 칸에 있다
    g = FreeSpaceGrid()
    for k in range(10):
        sensor = (0.0, 0.1 * k)
        g.integrate(sensor, DIRS, wall_ranges(4.0), 0.1 * k)
    fragment = [[4.0, y] for y in np.linspace(0.5, 1.5, 11)]
    assert not g.evidence(fragment, 1.0).any()


def test_endpoint_margin_keeps_wall_front_unmarked():
    g = FreeSpaceGrid(endpoint_margin=0.2)
    g.integrate((0.0, 0.0), DIRS, wall_ranges(3.0), 0.0)
    # 벽 바로 앞 0.1 m는 비어 있다고 적지 않는다 (거리 잡음)
    assert g.evidence([[2.9, 0.0]], 0.1).tolist() == [False]
    assert g.evidence([[2.6, 0.0]], 0.1).tolist() == [True]


def test_no_return_beam_marks_free_up_to_max_range():
    g = FreeSpaceGrid(max_range=5.0)
    g.integrate((0.0, 0.0), DIRS, np.full(N, np.inf), 0.0)
    assert g.evidence([[-4.0, 0.0], [-6.0, 0.0]], 0.1).tolist() == [True, False]


def test_recenter_keeps_overlapping_cells():
    g = FreeSpaceGrid(size=10.0, max_range=3.0)
    g.integrate((0.0, 0.0), DIRS, np.full(N, np.inf), 0.0)
    assert g.evidence([[2.0, 0.0]], 0.1).tolist() == [True]
    g.integrate((4.0, 0.0), DIRS[:1], np.array([math.nan]), 0.2)   # 로봇이 4 m 이동 → 격자 이동
    assert g.evidence([[2.0, 0.0]], 0.3).tolist() == [True]


def test_ignores_nan_points():
    g = FreeSpaceGrid()
    g.integrate((0.0, 0.0), DIRS, wall_ranges(5.0), 0.0)
    assert g.evidence([[math.nan, 0.0]], 0.1).tolist() == [False]


@pytest.mark.parametrize('kwargs', [dict(resolution=0.0), dict(max_range=-1.0),
                                    dict(endpoint_margin=-0.1)])
def test_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        FreeSpaceGrid(**kwargs)


def test_grazing_beam_next_to_pillar_does_not_mark_free():
    # 1 m 앞 기둥(반경 0.1)을 빔이 스치고 지나가도 기둥 옆 0.15 m 칸은 비었다고 적지 않는다.
    # 적으면 로봇이 움직여 기둥 옆면이 보일 때 그 점이 "빈 칸 침입"이 된다
    cx, r0 = 1.0, 0.1
    b = DIRS @ np.array([cx, 0.0])
    disc = b * b - (cx * cx - r0 * r0)
    ranges = np.where(disc >= 0, b - np.sqrt(np.maximum(disc, 0)), np.inf)
    ranges[ranges <= 0] = np.inf
    g = FreeSpaceGrid(hit_clearance=0.2)
    g.integrate((0.0, 0.0), DIRS, ranges, 0.0)
    side = [[1.0, 0.15]]
    assert g.evidence(side, 1.5).tolist() == [False]
    g_off = FreeSpaceGrid(hit_clearance=0.0, hit_quiet=0.0)
    g_off.integrate((0.0, 0.0), DIRS, ranges, 0.0)
    assert g_off.evidence(side, 1.5).tolist() == [True]       # 끄면 스치는 빔이 비었다고 적음
