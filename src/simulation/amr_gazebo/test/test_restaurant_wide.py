"""넓은 식당 restaurant_wide (tools/make_restaurant_wide.py가 restaurant.sdf에서 만든다). ROS 없이 돈다.

배치: 원래 식당을 90° 돌린 서쪽 홀 + 남쪽 벽 중심선에 댄 거울 복사본(_e) 동쪽 홀. 매장 줄이 양 끝, 가운데 벽은 같이 쓴다.
원래 식당 파일을 고치고 도구를 다시 안 돌리면 여기서 걸린다 (만든 파일은 손으로 고치지 않는다).
"""

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parents[1]
FLEET = SRC / 'fleet' / 'amr_fleet_manager' / 'config'
WIDE = PKG / 'worlds' / 'restaurant_wide.sdf'
SHIFT = 16.2          # 동쪽 홀 = 서쪽 홀을 x = 8.1에 대고 거울 (원래 남쪽 벽 중심선 y = −8.1)
HALL_CENTER = {False: (0.0, 0.0), True: (SHIFT, 0.0)}


def tool():
    spec = importlib.util.spec_from_file_location('make_restaurant_wide', PKG / 'tools' / 'make_restaurant_wide.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def world():
    return ET.parse(WIDE).getroot().find('world')


def pose(el):
    text = el.findtext('pose')
    return [float(v) for v in text.split()] if text else [0.0] * 6


def test_world_is_up_to_date(tmp_path):
    out = tmp_path / 'restaurant_wide.sdf'
    tool().make_world(PKG / 'worlds' / 'restaurant.sdf', out)
    assert out.read_text(encoding='utf-8') == WIDE.read_text(encoding='utf-8')


def test_every_model_has_east_copy_and_shared_center_wall():
    names = [m.get('name') for m in world().findall('model')]
    original = [m.get('name') for m in ET.parse(PKG / 'worlds' / 'restaurant.sdf').getroot().find('world')
                .findall('model')]
    # 카운터 앞 정지 손님은 걷는 손님으로 빠지고, 주방 직원 정지 모델이 들어온다
    counter = {f'standing_{i}' for i in range(1, 6)}
    staff = [o['id'] for o in load(PKG / 'config' / 'dynamic_obstacles_restaurant_staff.yaml')['obstacles']]
    west = [n for n in original if n not in counter] + staff
    # 원래 모델 + 이름 끝 _e 복사본 + 가운데 벽 (원래 이름이 _e로 끝나는 것도 있다: L01_bench_e는 동쪽 벤치)
    assert sorted(names) == sorted(west + [n + '_e' for n in west] + ['center_wall'])
    walls = [e.get('name') for m in world().findall('model') if m.get('name').startswith('outer_walls')
             for e in m.iter() if e.tag in ('collision', 'visual')]
    assert not any(w.startswith(('south', 'wainscot_south')) for w in walls)


def test_stalls_at_both_ends():
    # 한식 카운터: 원래 (−4.5, 5.0)은 서쪽 끝 (−5.0, −4.5), 복사본은 동쪽 끝 (21.2, −4.5). 서로 마주 본다
    models = {m.get('name'): pose(m) for m in world().findall('model')}
    west, east = models['stall_1_counter'], models['stall_1_counter_e']
    assert west[:3] == pytest.approx([-5.0, -4.5, 0.0])
    assert east[:3] == pytest.approx([SHIFT + 5.0, -4.5, 0.0])
    assert math.cos(east[5]) == pytest.approx(-math.cos(west[5]), abs=1e-6)
    assert math.sin(east[5]) == pytest.approx(math.sin(west[5]), abs=1e-6)


@pytest.mark.parametrize('east', [False, True])
def test_signs_face_their_hall(east):
    # 간판 글자 메시는 앞면이 로컬 −y. 양쪽 홀 모두 홀 가운데를 봐야 글자가 보인다
    model = next(m for m in world().findall('model') if m.get('name') == ('signs_e' if east else 'signs'))
    mx, my, _, _, _, myaw = pose(model)
    cx, cy = HALL_CENTER[east]
    for v in model.iter('visual'):
        x, y, _, _, _, yaw = pose(v)
        wx = mx + math.cos(myaw) * x - math.sin(myaw) * y
        wy = my + math.sin(myaw) * x + math.cos(myaw) * y
        front = (math.sin(myaw + yaw), -math.cos(myaw + yaw))     # R(yaw)·(0, −1)
        assert front[0] * (cx - wx) + front[1] * (cy - wy) > 0, v.get('name')


def test_stations_and_zones_follow_the_layout():
    base = load(FLEET / 'stations_restaurant.yaml')['stations']
    wide = load(FLEET / 'stations_restaurant_wide.yaml')['stations']
    assert set(wide) == set(base) | {n + '_e' for n in base}
    for name, (x, y, yaw) in base.items():
        # 서쪽: 90° 회전 (−y, x, yaw + π/2). 동쪽: 거울 뒤 회전 (y + 16.2, x, π/2 − yaw)
        assert wide[name][:2] == pytest.approx([-y, x])
        assert wide[name + '_e'][:2] == pytest.approx([y + SHIFT, x])
        for got, want in ((wide[name][2], yaw + math.pi / 2), (wide[name + '_e'][2], math.pi / 2 - yaw)):
            assert (math.cos(got), math.sin(got)) == pytest.approx((math.cos(want), math.sin(want)), abs=1e-6)
    zones = {z['id']: z['area'] for z in load(FLEET / 'congestion_zones_restaurant_wide.yaml')['zones']}
    for z in load(FLEET / 'congestion_zones_restaurant.yaml')['zones']:
        x0, x1, y0, y1 = z['area']
        assert zones[z['id']] == pytest.approx([-y1, -y0, x0, x1])
        assert zones[z['id'] + '_e'] == pytest.approx([y0 + SHIFT, y1 + SHIFT, x0, x1])


def test_peak_crowds_west_hall_only():
    # 혼잡이 시간대마다 바뀐다: 평상시는 양쪽 같은 수, 피크는 서쪽 홀에 몰린다. 카운터 앞 손님(counter_N)은 양쪽 다 걷는다
    cfg = PKG / 'config'
    normal = load(cfg / 'dynamic_obstacles_restaurant_wide.yaml')['obstacles']
    peak = load(cfg / 'dynamic_obstacles_restaurant_wide_peak.yaml')['obstacles']

    def halves(obstacles):
        west = sum(o['spawn']['x'] < SHIFT / 2 for o in obstacles)
        return west, len(obstacles) - west

    west, east = halves(normal)
    assert west == east == 12
    west, east = halves(peak)
    assert west >= 2 * east and east > 0
    for obstacles in (normal, peak):
        assert sum(o['id'].startswith('counter_') for o in obstacles) == 10
    for obstacles in (normal, peak):
        assert len({o['id'] for o in obstacles}) == len(obstacles)
        assert all(-8 < w['x'] < 24.2 and -12 < w['y'] < 12 for o in obstacles for w in o['waypoints'])
