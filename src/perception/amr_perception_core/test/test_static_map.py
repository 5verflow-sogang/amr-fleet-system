import math

import numpy as np
import pytest

from amr_perception_core.static_map import StaticMapMask


def _grid():
    # 10 × 10 칸, 0.1 m. 가운데 (행 5, 열 5) 한 칸만 막힘, (행 0, 열 9)는 미지(−1)
    grid = np.zeros((10, 10), dtype=int)
    grid[5, 5] = 100
    grid[0, 9] = -1
    return grid


def test_occupied_cell_is_static_and_free_is_not():
    m = StaticMapMask(_grid(), 0.1, (0.0, 0.0, 0.0), inflation=0.0)
    # 칸 (열 5, 행 5) = x 0.5~0.6, y 0.5~0.6
    assert m.is_static([[0.55, 0.55], [0.25, 0.25], [0.95, 0.05]]).tolist() == [True, False, False]


def test_inflation_grows_disk():
    m = StaticMapMask(_grid(), 0.1, (0.0, 0.0, 0.0), inflation=0.2)
    near = [[0.75, 0.55], [0.55, 0.35]]          # 두 칸 옆 (반경 2칸 안)
    diagonal = [[0.75, 0.75]]                     # 대각 (2, 2) 칸: √8 > 2 → 원판 밖
    far = [[0.85, 0.55]]                          # 세 칸 옆
    assert m.is_static(near).all()
    assert not m.is_static(diagonal).any()
    assert not m.is_static(far).any()


def test_points_outside_map_and_nan_are_not_static():
    m = StaticMapMask(np.full((4, 4), 100), 0.1, (0.0, 0.0, 0.0), inflation=0.0)
    assert m.is_static([[-0.05, 0.1], [0.45, 0.1], [math.nan, 0.1]]).tolist() == [False] * 3


def test_origin_offset_and_yaw():
    grid = np.zeros((10, 10), dtype=int)
    grid[0, 3] = 100                               # 칸 x 0.3~0.4, y 0~0.1 (지도 축)
    m = StaticMapMask(grid, 0.1, (2.0, 1.0, math.pi / 2), inflation=0.0)
    # 지도 축 x가 map 좌표 +y 방향: 칸 중심 (0.35, 0.05) → map (2 − 0.05, 1 + 0.35)
    assert m.is_static([[1.95, 1.35]]).tolist() == [True]
    assert m.is_static([[2.35, 1.05]]).tolist() == [False]


def test_from_grid_message_row_major():
    data = [0] * 12
    data[1 * 4 + 2] = 100                          # 행 1, 열 2 (너비 4, 높이 3)
    m = StaticMapMask.from_grid_message(4, 3, 0.5, (0.0, 0.0, 0.0), data, inflation=0.0)
    assert m.is_static([[1.25, 0.75]]).tolist() == [True]


@pytest.mark.parametrize('kwargs', [dict(resolution=0.0), dict(inflation=-0.1)])
def test_rejects_bad_values(kwargs):
    args = dict(occupancy=_grid(), resolution=0.1, origin=(0.0, 0.0, 0.0))
    args.update(kwargs)
    with pytest.raises(ValueError):
        StaticMapMask(**args)


def test_best_offset_recovers_localization_shift():
    # 0.05 m 칸 40 × 40에 L자 벽. 스캔 점이 (+0.15, −0.20)만큼 어긋나 들어오면 되돌리는 (−0.15, +0.20)을 찾는다
    grid = np.zeros((40, 40), dtype=int)
    grid[10, 5:35] = 100
    grid[10:35, 30] = 100
    m = StaticMapMask(grid, 0.05, (0.0, 0.0, 0.0), inflation=0.05)
    wall = [[0.25 + 0.05 * i + 0.025, 0.525] for i in range(30)] + \
           [[1.525, 0.525 + 0.05 * i] for i in range(25)]
    seen = np.array(wall) + [0.15, -0.20]
    dx, dy, frac, frac0 = m.best_offset(seen, 0.3, 0.05)
    assert (dx, dy) == pytest.approx((-0.15, 0.20), abs=0.051)
    assert frac == pytest.approx(1.0) and frac0 < 0.2


def test_best_offset_prefers_no_shift_when_aligned():
    grid = np.zeros((20, 20), dtype=int)
    grid[5, :] = 100
    m = StaticMapMask(grid, 0.1, (0.0, 0.0, 0.0), inflation=0.2)
    line = [[0.1 * i + 0.05, 0.55] for i in range(20)]
    dx, dy, frac, frac0 = m.best_offset(line, 0.3, 0.1)     # 부풀림 안에서는 어느 쪽이든 100%
    assert (dx, dy) == (0.0, 0.0) and frac == frac0 == 1.0
