import os

import numpy as np
import pytest
import yaml

from amr_mapping.gt_map import (
    FREE, OCCUPIED, UNKNOWN, cell_centers_world, main, passage_widths, rasterize, write_map,
)
from amr_mapping.map_quality import Box, read_pgm

RES = 0.025
SDF = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'simulation', 'amr_gazebo', 'worlds',
                   'warehouse.sdf')


def two_racks(gap=0.6):
    """y 방향으로 gap만큼 떨어진 랙 두 개 (world). 통로는 y −gap/2~gap/2, x −2.5~2.5."""
    return [Box('south', 'warehouse_rack', 0.0, -gap / 2 - 0.6, 0.0, 2.5, 0.6),
            Box('north', 'warehouse_rack', 0.0, gap / 2 + 0.6, 0.0, 2.5, 0.6)]


@pytest.mark.parametrize('origin_in_world', [(0.0, 0.0, 0.0), (-0.0562, 1.5317, 0.00009), (0.013, -0.007, 0.0)])
def test_gap_keeps_cell_count_when_boundary_falls_mid_cell(origin_in_world):
    """경계가 칸 중간에 걸려도(정렬 원점이 칸의 배수가 아니어도) 0.6 m 통로는 칸 24개다."""
    ox, oy = -5.0 - origin_in_world[0], -5.0 - origin_in_world[1]
    img = rasterize(two_racks(), size=(400, 400), resolution=RES, origin=(ox, oy), map_origin_in_world=origin_in_world)
    w = passage_widths(img, RES, (ox, oy), origin_in_world, (-2.5, 2.5), (-0.3, 0.3))
    assert set(w) == {24}


def test_passage_width_drops_when_a_rack_moves_one_cell():
    """랙이 한 칸 통로 쪽으로 오면 바로 23칸으로 잡힌다 (막힘 판정 직전 상황을 놓치지 않는다)."""
    objs = two_racks()
    objs[1] = Box('north', 'warehouse_rack', 0.0, objs[1].cy - RES, 0.0, 2.5, 0.6)
    img = rasterize(objs, size=(400, 400), resolution=RES, origin=(-5.0, -5.0), map_origin_in_world=(0.0, 0.0, 0.0))
    assert set(passage_widths(img, RES, (-5.0, -5.0), (0.0, 0.0, 0.0), (-2.5, 2.5), (-0.3, 0.3))) == {23}


def test_inside_walls_free_outside_unknown_objects_filled():
    walls = [Box('wall_west', 'wall', -30.1, 0.0, 0.0, 0.1, 20.0)]
    rack = [Box('r', 'warehouse_rack', -25.0, 0.0, 0.0, 2.5, 0.6)]
    img = rasterize(walls + rack, size=(400, 80), resolution=0.05, origin=(-32.0, -2.0),
                    map_origin_in_world=(0.0, 0.0, 0.0))
    wx, wy = cell_centers_world((400, 80), 0.05, (-32.0, -2.0), (0.0, 0.0, 0.0))
    assert (img[wx < -30.2] == UNKNOWN).all()
    assert (img[(wx > -30.2) & (wx < -30.0)] == OCCUPIED).all()
    assert (img[(np.abs(wx + 25.0) < 2.5) & (np.abs(wy) < 0.6)] == OCCUPIED).all()   # 속도 채운다
    assert img[40, int((-20.0 + 32.0) / 0.05)] == FREE


def test_map_frame_follows_origin_in_world():
    """map 좌표 (0, 0)은 world map_origin_in_world 위치다 (system.launch.py world → map TF와 같은 방향)."""
    origin = (1.0, 2.0, 0.0)
    pillar = [Box('p', 'warehouse_pillar', 1.0, 2.0, 0.0, 0.3, 0.3)]
    img = rasterize(pillar, size=(100, 100), resolution=0.05, origin=(-2.5, -2.5), map_origin_in_world=origin)
    rows, cols = np.nonzero(img == OCCUPIED)
    assert cols.mean() == pytest.approx(49.5) and rows.mean() == pytest.approx(49.5)


def test_write_map_matches_v1_format(tmp_path):
    img = np.full((3, 4), FREE, np.uint8)
    img[1, 2] = OCCUPIED
    pgm = write_map(str(tmp_path / 'm.yaml'), img, 0.025, (-30.2, -21.8), header='# 주석\n')
    with open(tmp_path / 'm.yaml', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    assert meta == {'image': 'm.pgm', 'mode': 'trinary', 'resolution': 0.025, 'origin': [-30.2, -21.8, 0],
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    assert (read_pgm(pgm) == img).all()


@pytest.mark.skipif(not os.path.exists(SDF), reason='amr_gazebo 월드가 없다')
def test_warehouse_gt_map_same_grid_as_v1_and_narrow_passages_06(tmp_path, capsys):
    out = tmp_path / 'warehouse_gt.yaml'
    assert main([SDF, str(out)]) == 0
    img = read_pgm(str(tmp_path / 'warehouse_gt.pgm'))
    assert img.shape == (1616, 2417)
    log = capsys.readouterr().out
    assert log.count('빈 칸 24~24칸') == 2   # 좁은 통로 ①·② 모두 0.6 m
