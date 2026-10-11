"""화물 상자 배치(config/cargo_boxes.yaml)가 규칙을 지키는지 실제 월드 SDF로 확인한다."""

import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from amr_gazebo.cargo import (LIDAR_HEIGHT, SIZES, CargoBox, cargo_sdf, check_lidar_clearance, footprint,
                              load_boxes)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, 'config', 'cargo_boxes.yaml')
WORLD = os.path.join(PKG, 'worlds', 'warehouse.sdf')
OBSTACLES = os.path.join(PKG, 'config', 'dynamic_obstacles.yaml')

# 모델 원점 기준 바닥 사각형 (중심 x, 중심 y, 반길이, 반폭)
FOOTPRINTS = {
    'warehouse_rack': (0.0, 0.0, 2.5, 0.6),
    'warehouse_pillar': (0.0, 0.0, 0.3, 0.3),
    'loading_dock': (0.75, 0.0, 0.75, 1.75),     # 플랫폼 (높이 1.0 m)
}
NARROW_PASSAGES = [((-5.0, -4.4), (-2.6, -1.4)), ((-4.4, 0.6), (-1.4, -0.8))]


def world_objects():
    objs = []
    for inc in ET.parse(WORLD).getroot().iter('include'):
        model = inc.find('uri').text.replace('model://', '')
        if model not in FOOTPRINTS:
            continue
        x, y, _, _, _, yaw = (float(v) for v in inc.find('pose').text.split())
        ox, oy, hx, hy = FOOTPRINTS[model]
        c, s = math.cos(yaw), math.sin(yaw)
        if abs(s) > 0.5:
            hx, hy = hy, hx
        objs.append((inc.find('name').text, model, x + c * ox - s * oy, y + s * ox + c * oy, hx, hy))
    return objs


def inside(px, py, obj, margin=0.0):
    _, _, cx, cy, hx, hy = obj
    return abs(px - cx) < hx + margin and abs(py - cy) < hy + margin


@pytest.fixture(scope='module')
def boxes():
    return load_boxes(CONFIG)


def test_counts_and_sizes(boxes):
    assert len(boxes) >= 20
    assert {b.size for b in boxes} == set(SIZES)


def test_lidar_scan_plane_is_clear(boxes):
    assert check_lidar_clearance(boxes) == []
    # 큰 상자를 바닥에 두면 걸린다
    assert check_lidar_clearance([CargoBox('x', 'large', 0, 0)]) == ['x']
    assert LIDAR_HEIGHT == pytest.approx(0.18 + 0.2)


def test_camera_height_range(boxes):
    assert all(b.z + b.dims[2] <= 1.5 for b in boxes)


def test_floor_boxes_do_not_overlap_objects(boxes):
    objs = world_objects()
    assert sum(o[1] == 'warehouse_rack' for o in objs) == 22
    for b in boxes:
        if b.z > 0.0:
            continue
        for px, py in footprint(b):
            hit = [o[0] for o in objs if inside(px, py, o, margin=0.005)]
            assert hit == [], f'{b.id}가 {hit}와 겹친다'


def test_floor_boxes_touch_a_rack(boxes):
    # 통로 한가운데가 아니라 랙 면에 붙어 있다 (주행 경로를 막지 않게)
    racks = [o for o in world_objects() if o[1] == 'warehouse_rack']
    for b in boxes:
        if b.z == 0.0:
            assert any(inside(b.x, b.y, r, margin=b.dims[1] / 2 + 0.05) for r in racks), b.id


def test_raised_boxes_sit_on_dock_platform(boxes):
    docks = [o for o in world_objects() if o[1] == 'loading_dock']
    for b in boxes:
        if b.z > 0.0:
            assert b.z == pytest.approx(1.0), b.id
            assert any(all(inside(px, py, d) for px, py in footprint(b)) for d in docks), b.id


def test_boxes_avoid_narrow_passages(boxes):
    for b in boxes:
        for (x0, x1), (y0, y1) in NARROW_PASSAGES:
            assert not (x0 - 0.3 < b.x < x1 + 0.3 and y0 - 0.3 < b.y < y1 + 0.3), b.id


def _segment_distance(px, py, a, c):
    (ax, ay), (bx, by) = a, c
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def test_boxes_away_from_dynamic_obstacle_paths(boxes):
    with open(OBSTACLES, encoding='utf-8') as f:
        obstacles = yaml.safe_load(f)['obstacles']
    for o in obstacles:
        if o['pattern'] == 'random':
            continue      # 무작위는 영역 힌트만 있다
        pts = [(w['x'], w['y']) for w in o['waypoints']]
        for b in boxes:
            d = min(_segment_distance(b.x, b.y, pts[i], pts[i + 1]) for i in range(len(pts) - 1))
            assert d > 0.5, f'{b.id}가 {o["id"]} 경로에서 {d:.2f} m'


def test_boxes_clear_of_signs(boxes):
    # 표지판(#118, model://sign_*)은 랙 끝면·벽에 붙은 0.4 m 판이다. 상자와 공간이 겹치거나 판 바로 앞을 막지 않는다
    signs = []
    for inc in ET.parse(WORLD).getroot().iter('include'):
        if inc.find('uri').text.startswith('model://sign_'):
            x, y, z = (float(v) for v in inc.find('pose').text.split()[:3])
            signs.append((inc.find('name').text, x, y, z))
    for name, sx, sy, sz in signs:
        for b in boxes:
            near = math.hypot(b.x - sx, b.y - sy) < 0.6
            overlap_z = b.z < sz + 0.2 and b.z + b.dims[2] > sz - 0.2
            assert not (near and overlap_z), f'{b.id}가 표지판 {name}을 가린다'


def test_sdf_has_one_link_per_box_at_center(boxes):
    root = ET.fromstring(cargo_sdf(boxes))
    model = root.find('model')
    assert model.find('static').text == 'true'
    links = {lk.get('name'): lk for lk in model.findall('link')}
    assert set(links) == {b.id for b in boxes}
    b = boxes[0]
    pose = [float(v) for v in links[b.id].find('pose').text.split()]
    assert pose[:3] == pytest.approx(list(b.center))


def test_duplicate_id_rejected(tmp_path):
    p = tmp_path / 'c.yaml'
    p.write_text('boxes:\n  - {id: a, size: small, x: 0, y: 0}\n  - {id: a, size: small, x: 1, y: 0}\n')
    with pytest.raises(ValueError):
        load_boxes(str(p))
