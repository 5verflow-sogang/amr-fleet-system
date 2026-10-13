import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from amr_docking.markers import (
    BOARDS, LIDAR_HEIGHT, LIDAR_MARGIN, black_runs, board_height_range, clears_lidar, load_stations,
    markers_sdf)

HERE = os.path.dirname(os.path.abspath(__file__))
STATIONS = os.path.join(HERE, '..', 'config', 'dock_stations.yaml')
FLEET_STATIONS = os.path.join(HERE, '..', '..', '..', 'fleet', 'amr_fleet_manager', 'config', 'stations.yaml')


def test_load_and_ids_unique():
    stations = load_stations(STATIONS)
    ids = [i for s in stations.values() for i in s.marker_ids]
    assert len(ids) == len(set(ids))
    assert max(ids) < 100


@pytest.mark.parametrize('board', list(BOARDS))
def test_board_clears_lidar(board):
    low, high = board_height_range(BOARDS[board])
    assert clears_lidar(BOARDS[board])
    assert high < LIDAR_HEIGHT - LIDAR_MARGIN
    assert low > 0.05


def test_docks_use_large_board():
    stations = load_stations(STATIONS)
    assert all((s.board is BOARDS['large']) == n.startswith('dock_') for n, s in stations.items())


@pytest.mark.parametrize('name,x,y', [
    ('rack_a1', -22.5, 4.395),        # 랙 A열 남쪽 면 y 4.4에서 5 mm 앞
    ('rack_c1', -22.5, -8.395),       # 랙 C열 북쪽 면 y -8.4
    ('charger_c1', -24.0, -19.695),   # 충전기 본체 앞면 y -19.7
    ('dock_1', -28.495, 16.0),        # 서쪽 도크 플랫폼 앞면 x -28.5에서 5 mm 앞
    ('dock_a', 28.495, 16.0),
])
def test_board_on_face(name, x, y):
    bx, by, _ = load_stations(STATIONS)[name].board_center
    assert bx == pytest.approx(x, abs=1e-6)
    assert by == pytest.approx(y, abs=1e-6)


@pytest.mark.skipif(not os.path.exists(FLEET_STATIONS), reason='amr_fleet_manager 소스 없음')
def test_poses_match_fleet_stations():
    with open(FLEET_STATIONS, encoding='utf-8') as f:
        fleet = yaml.safe_load(f)['stations']
    stations = load_stations(STATIONS)
    for name, s in stations.items():
        assert name in fleet, name
        assert [s.x, s.y, s.yaw] == pytest.approx(fleet[name], abs=1e-6), name
    # 도킹 안 하는 곳은 대기 위치뿐
    assert {n for n in fleet if n not in stations} == {n for n in fleet if n.startswith('wait_')}


def test_black_runs():
    assert black_runs([[1, 1, 0, 1], [0, 0, 0, 0], [0, 1, 1, 1]]) == [(0, 0, 2), (0, 3, 1), (2, 1, 3)]


def test_sdf_is_valid_xml():
    stations = list(load_stations(STATIONS).values())[:2]
    checker = [[1] * 6] + [[1, 0, 1, 0, 1, 1]] * 4 + [[1] * 6]
    root = ET.fromstring(markers_sdf(stations, cells_of=lambda i: checker))
    links = root.findall('./model/link')
    assert [link.get('name') for link in links] == ['board_dock_1', 'board_dock_2']
    assert root.find('./model/static').text == 'true'
    assert all(link.find('collision') is None for link in links)   # 시각만
