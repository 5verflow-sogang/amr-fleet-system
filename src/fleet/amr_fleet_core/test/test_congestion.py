"""구역별 혼잡도와 costmap 마스크 (docs/api.md 4-5절)."""

import pytest

from amr_fleet_core.congestion import CongestionTracker, CongestionZone, count_people, make_mask, zones_from_dict

LINE = CongestionZone('buffet_line', (2.0, 6.0, 1.0, 2.5), 4)
HALL = CongestionZone('hall_east', (6.0, 9.0, 1.0, 5.0), 2)


def test_invalid_zone():
    with pytest.raises(ValueError):
        CongestionZone('z', (1.0, 0.0, 0.0, 1.0), 1)
    with pytest.raises(ValueError):
        CongestionZone('z', (0.0, 1.0, 0.0, 1.0), 0)
    with pytest.raises(ValueError):
        zones_from_dict({'zones': [{'id': 'z', 'area': [0, 1, 0, 1], 'capacity': 1}] * 2})


def test_zones_from_dict():
    zones = zones_from_dict({'zones': [{'id': 'buffet_line', 'area': [2, 6, 1, 2.5], 'capacity': 4}]})
    assert zones == [LINE]
    assert zones_from_dict({}) == [] and zones_from_dict(None) == []


def test_count_includes_boundary_and_overlap():
    # (6, 2)는 두 구역 경계 위라 양쪽에 센다
    assert count_people([LINE, HALL], [(3.0, 2.0), (6.0, 2.0), (8.0, 4.0), (0.0, 0.0)]) == [2, 2]


def test_tracker_starts_raw_then_smooths_and_caps():
    t = CongestionTracker([LINE, HALL], alpha=0.5)
    first = t.update([(3.0, 2.0)] * 2 + [(7.0, 2.0)] * 5)
    assert [(v.people_count, v.value) for v in first] == [(2, 0.5), (5, 1.0)]   # 정원 넘으면 1.0
    second = t.update([])
    assert [v.value for v in second] == [0.25, 0.5]
    # 다 빠지면 4주기 뒤 0.1 아래 (api.md 4-5절 "약 2초", 2 Hz)
    for _ in range(3):
        last = t.update([])
    assert last[1].value < 0.1


def test_tracker_rejects_bad_alpha():
    with pytest.raises(ValueError):
        CongestionTracker([LINE], alpha=0.0)


def test_mask_covers_all_zones_with_scaled_values():
    m = make_mask([LINE, HALL], [1.0, 0.5], resolution=1.0, max_value=90)
    assert (m.origin, m.width, m.height) == ((2.0, 1.0), 7, 4)
    rows = [m.data[j * m.width:(j + 1) * m.width] for j in range(m.height)]
    # 아래 행(칸 중심 y 1.5)부터. buffet_line은 y 1~2.5라 칸 중심 2.5(경계)까지 들어간다
    assert rows[0] == [90, 90, 90, 90, 45, 45, 45]
    assert rows[1] == [90, 90, 90, 90, 45, 45, 45]
    assert rows[2] == [0, 0, 0, 0, 45, 45, 45]
    assert rows[3] == [0, 0, 0, 0, 45, 45, 45]


def test_mask_overlap_takes_max_and_zero_value_is_free():
    a = CongestionZone('a', (0.0, 2.0, 0.0, 1.0), 1)
    b = CongestionZone('b', (1.0, 3.0, 0.0, 1.0), 1)
    m = make_mask([a, b], [0.2, 0.6], resolution=1.0, max_value=100)
    assert m.data == [20, 60, 60]
    assert make_mask([a, b], [0.0, 0.0], resolution=1.0).data == [0, 0, 0]
    assert make_mask([], []) is None


def test_mask_fine_resolution_size():
    m = make_mask([LINE], [1.0], resolution=0.1)
    assert (m.width, m.height) == (40, 15)
    assert sum(1 for c in m.data if c == 90) == 40 * 15
