"""식당 혼잡도 구역 파일(config/congestion_zones_restaurant.yaml) 점검: 12구역, 서로 겹치지 않음, 홀을 빈틈없이 덮음."""

import itertools
from pathlib import Path

import pytest
import yaml

from amr_fleet_core.congestion import zones_from_dict

ZONES_FILE = Path(__file__).resolve().parents[1] / 'config' / 'congestion_zones_restaurant.yaml'


@pytest.fixture(scope='module')
def zones():
    with open(ZONES_FILE) as f:
        return zones_from_dict(yaml.safe_load(f))


def test_twelve_zones_with_names_other_files_use(zones):
    ids = [z.zone_id for z in zones]
    assert len(ids) == 12
    # 손님 파일·1-2 우회 도구(scene_plan)·대시보드 테스트가 이 이름으로 찾는다
    assert {'buffet_line', 'west_aisle', 'cafe'} <= set(ids)
    assert [f'aisle_{i}' for i in range(1, 6)] == [i for i in ids if i.startswith('aisle_')]


def test_zones_do_not_overlap(zones):
    for a, b in itertools.combinations(zones, 2):
        ax0, ax1, ay0, ay1 = a.area
        bx0, bx1, by0, by1 = b.area
        overlap = ax0 <= bx1 and bx0 <= ax1 and ay0 <= by1 and by0 <= ay1
        assert not overlap, f'{a.zone_id}와 {b.zone_id}가 겹친다'


@pytest.mark.parametrize('x', [-11.5, -9.0, -7.275, -4.225, -1.175, 1.875, 4.4])
@pytest.mark.parametrize('y', [-6.0, -2.45, 1.0, 3.45])
def test_hall_walkways_are_covered(zones, x, y):
    # 서쪽 통로·세로 통로·가로 통로·매장 앞 통로·남쪽 통로 위의 점은 어느 한 구역에 들어간다 (구역 사이 0.01 m 틈 제외)
    inside = [z.zone_id for z in zones if z.contains((x, y))]
    assert len(inside) == 1, f'({x}, {y}): {inside}'


def test_vertical_aisles_split_at_table_column_centres(zones):
    # 세로 통로 구역의 경계는 테이블 열 가운데(x -8.8, -5.75, -2.7, 0.35, 3.4)에 있어 손님이 경계를 따라 걷지 않는다
    aisles = sorted((z for z in zones if z.zone_id.startswith('aisle_')), key=lambda z: z.area[0])
    for col, z in zip([-9.72, -5.75, -2.7, 0.35, 3.4], aisles):
        assert z.area[0] == pytest.approx(col, abs=0.02)
    # 각 통로의 가운데 줄(x -7.275, -4.225, -1.175, 1.875)은 그 구역 안에 있다
    for x, z in zip([-7.275, -4.225, -1.175, 1.875], aisles):
        assert z.area[0] < x < z.area[1]
