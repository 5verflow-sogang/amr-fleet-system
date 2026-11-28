"""지도에서 좁은 통로 구역 찾기 (amr_fleet_core.traffic.permit_zones)."""

from pathlib import Path

import numpy as np
import pytest

from amr_fleet_core.traffic import permit_zones as pz

SRC = Path(__file__).resolve().parents[3]
RESTAURANT_MAP = SRC / 'localization' / 'amr_mapping' / 'maps' / 'restaurant_gt.yaml'
FLEET_CONFIG = SRC / 'fleet' / 'amr_fleet_manager' / 'config'
RES = 0.05


def room_with_passage(width, length=2.0):
    """12 x 6 m 방. 가운데 벽(x 5 ~ 5 + length)에 y 3 중심, 폭 width인 통로 하나."""
    free = np.zeros((int(6 / RES), int(12 / RES)), bool)
    free[1:-1, 1:-1] = True
    x0, x1 = int(5 / RES), int((5 + length) / RES)
    free[:, x0:x1] = False
    y0, y1 = int(round((3 - width / 2) / RES)), int(round((3 + width / 2) / RES))
    free[y0:y1, x0:x1] = True
    return pz.Grid(free, RES, (0.0, 0.0))


def test_narrow_passage_between_two_stations_becomes_one_zone_with_end_margins():
    zones = pz.permit_zones(room_with_passage(1.0), {'a': (2.0, 3.0), 'b': (10.0, 3.0)})
    assert len(zones) == 1
    x0, x1, y0, y1 = zones[0].rect
    # 통로(x 5~7) 양 끝: 로봇 몸(반길이 0.28 m)이 입구를 다 빠져나간 뒤에 구역 밖이 된다. 여유는 원(지름 max_width)이
    # 못 들어가는 곳부터 재므로 입구 밖으로는 END_MARGIN_M보다 조금 짧다
    assert 5.0 - pz.END_MARGIN_M <= x0 <= 5.0 - 0.3
    assert 7.0 + 0.3 <= x1 <= 7.0 + pz.END_MARGIN_M
    # 옆으로는 통로 안 로봇 중심 범위만
    assert 2.5 <= y0 < 3.0 < y1 <= 3.5
    assert zones[0].width_m == pytest.approx(1.0, abs=1.5 * RES)    # 칸 수가 짝수면 가운데 칸이 없어 한 칸 좁게 잰다


def test_passage_wide_enough_for_two_robots_is_not_a_zone():
    assert pz.permit_zones(room_with_passage(1.6), {'a': (2.0, 3.0), 'b': (10.0, 3.0)}) == []


def test_narrow_place_nobody_drives_through_is_not_a_zone():
    """두 정차 자리가 모두 서쪽이면 통로를 아무도 지나지 않는다."""
    assert pz.permit_zones(room_with_passage(1.0), {'a': (2.0, 2.0), 'b': (3.0, 4.0)}) == []


def test_short_narrow_gap_is_not_a_zone():
    assert pz.permit_zones(room_with_passage(1.0, length=0.5), {'a': (2.0, 3.0), 'b': (10.0, 3.0)}) == []


def test_waiting_spot_inside_the_margin_is_cut_out_and_the_passage_stays_covered():
    stations = {'a': (2.0, 3.0), 'b': (10.0, 3.0), 'wait_1': (7.3, 3.0)}
    (zone,) = pz.permit_zones(room_with_passage(1.0), stations)
    assert zone.rect[1] < 7.3 - pz.ROBOT_HALF_WIDTH_M
    assert zone.rect[0] <= 5.0 and zone.rect[1] >= 7.0
    assert 'wait_1' not in zone.stations


def test_station_off_the_map_is_an_error():
    with pytest.raises(ValueError, match='lost'):
        pz.permit_zones(room_with_passage(1.0), {'a': (2.0, 3.0), 'lost': (6.0, 0.5)})


@pytest.fixture(scope='module')
def restaurant():
    grid = pz.read_map(RESTAURANT_MAP)
    stations = pz.read_stations(FLEET_CONFIG / 'stations_restaurant.yaml')
    return grid, stations, pz.permit_zones(grid, stations)


def test_committed_restaurant_zones_match_the_map(restaurant):
    """지도나 정차 자리를 바꾸면 zones_restaurant.yaml을 다시 만든다 (모듈 설명의 명령)."""
    _, _, zones = restaurant
    header = pz.HEADER.format(map=RESTAURANT_MAP.name, stations='stations_restaurant.yaml', max_width=pz.MAX_WIDTH_M,
                              min_length=pz.MIN_LENGTH_M, end_margin=pz.END_MARGIN_M)
    assert (FLEET_CONFIG / 'zones_restaurant.yaml').read_text(encoding='utf-8') == pz.to_yaml(zones, header)


def test_restaurant_zone_covers_the_hand_drawn_central_aisle(restaurant):
    """11/16 restaurant_headon에서 손으로 그려 정면 대치 3/3을 푼 중앙 통로(로봇 중심 x 7.2~7.7)를 자동 구역이 덮는다."""
    _, _, zones = restaurant
    assert any(z.rect[0] <= 7.2 and z.rect[1] >= 7.7 and z.rect[2] <= -1.45 and z.rect[3] >= 4.35 for z in zones)


def test_no_robot_parks_inside_a_restaurant_zone(restaurant):
    _, stations, zones = restaurant
    for z in zones:
        assert not [s for s in z.stations if s.startswith(pz.KEEP_OUT)], z


def test_restaurant_zones_do_not_overlap(restaurant):
    _, _, zones = restaurant
    for i, a in enumerate(zones):
        for b in zones[i + 1:]:
            assert not pz._overlap(a.rect, b.rect)
