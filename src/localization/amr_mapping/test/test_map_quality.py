import math

import numpy as np
import pytest

from amr_mapping.map_quality import (
    Box, apply_rigid, boundary_distance, evaluate, fit_rigid, load_objects, occupied_world_points, read_pgm,
    write_png,
)

RES = 0.05


def draw_box(img, origin, b, res=RES):
    """사각형 테두리를 점유 칸(0)으로 그린다 (축 정렬만)."""
    h = img.shape[0]
    for x in np.arange(b.cx - b.hx, b.cx + b.hx + 1e-9, res / 2):
        for y in (b.cy - b.hy, b.cy + b.hy):
            img[h - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)] = 0
    for y in np.arange(b.cy - b.hy, b.cy + b.hy + 1e-9, res / 2):
        for x in (b.cx - b.hx, b.cx + b.hx):
            img[h - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)] = 0


def test_boundary_distance_inside_outside_rotated():
    b = Box('r', 'warehouse_rack', 0.0, 0.0, 0.0, 2.5, 0.6)
    d = boundary_distance(np.array([0.0, 0.0, 3.5, 2.5]), np.array([0.0, 1.6, 0.0, 0.6]), b)
    assert d == pytest.approx([0.6, 1.0, 1.0, 0.0])
    # 90° 돌린 사각형: 긴 변이 y축
    rb = Box('r', 'warehouse_rack', 0.0, 0.0, math.pi / 2, 2.5, 0.6)
    assert boundary_distance(np.array([0.0]), np.array([3.0]), rb)[0] == pytest.approx(0.5)


def test_perfect_map_passes_within_half_cell():
    origin = (-5.0, -5.0)
    img = np.full((200, 200), 254, dtype=np.uint8)
    rack = Box('rack', 'warehouse_rack', 0.0, 1.0, 0.0, 2.5, 0.6)
    pillar = Box('pillar', 'warehouse_pillar', 0.0, -2.0, 0.0, 0.3, 0.3)
    draw_box(img, origin, rack)
    draw_box(img, origin, pillar)
    wx, wy = occupied_world_points(img, RES, origin, (0.0, 0.0, 0.0))
    r = evaluate(wx, wy, [rack, pillar])
    assert r['missing'] == []
    assert r['max'] <= RES          # 칸 중심과 실제 테두리 차이는 한 칸 안
    assert r['passed']
    assert r['ghost_cells'] == 0
    assert r['offset_max'] <= RES


def test_offset_ignores_band_thickness():
    # 벽이 실제 테두리 앞뒤로 같은 두께만큼 두껍게 찍히면 |거리| 중앙값은 커져도 위치 어긋남은 0 근처다
    origin = (-5.0, -5.0)
    img = np.full((200, 200), 254, dtype=np.uint8)
    for dy in (-0.1, -0.05, 0.0, 0.05, 0.1):
        draw_box(img, origin, Box('band', 'warehouse_rack', 0.0, 1.0 + dy, 0.0, 2.5, 0.6))
    wx, wy = occupied_world_points(img, RES, origin, (0.0, 0.0, 0.0))
    r = evaluate(wx, wy, [Box('rack', 'warehouse_rack', 0.0, 1.0, 0.0, 2.5, 0.6)])
    assert r['median'] >= 0.04
    assert r['offset_median'] <= RES / 2 + 1e-9


def test_region_unknown_ratio_for_narrow_passage():
    from amr_mapping.map_quality import region_unknown_ratio
    res, origin = 0.05, (-10.0, -10.0)
    img = np.full((400, 400), 254, dtype=np.uint8)
    xr, yr = (-4.4, 0.6), (-1.4, -0.8)      # 좁은 통로 ② (world, 시작 자세 원점)
    assert region_unknown_ratio(img, res, origin, (0.0, 0.0, 0.0), xr, yr) == 0.0
    # 통로 서쪽 절반을 미관측으로
    h = img.shape[0]
    c0, c1 = int((-4.4 - origin[0]) / res), int((-1.9 - origin[0]) / res)
    r0, r1 = h - 1 - int((-0.8 - origin[1]) / res), h - 1 - int((-1.4 - origin[1]) / res)
    img[r0:r1 + 1, c0:c1] = 205
    assert region_unknown_ratio(img, res, origin, (0.0, 0.0, 0.0), xr, yr) == pytest.approx(0.5, abs=0.1)


def test_unknown_inside_ratio_counts_only_open_floor():
    from amr_mapping.map_quality import unknown_inside_ratio
    res, origin = 0.5, (-30.0, -20.0)
    img = np.full((80, 120), 254, dtype=np.uint8)   # 60 × 40 m 전부 관측
    rack = Box('rack', 'warehouse_rack', 0.0, 0.0, 0.0, 2.5, 0.6)
    assert unknown_inside_ratio(img, res, origin, (0.0, 0.0, 0.0), [rack]) == 0.0
    img[40, 60] = 205                                # 랙 안쪽(원래 못 봄)은 세지 않는다
    assert unknown_inside_ratio(img, res, origin, (0.0, 0.0, 0.0), [rack]) == 0.0
    img[10, 10] = 205                                # 빈 바닥의 미관측 칸
    assert unknown_inside_ratio(img, res, origin, (0.0, 0.0, 0.0), [rack]) > 0.0


def test_start_offset_and_ghost_cells():
    origin = (-5.0, -5.0)
    img = np.full((200, 200), 254, dtype=np.uint8)
    rack = Box('rack', 'warehouse_rack', 0.0, 1.0, 0.0, 2.5, 0.6)
    draw_box(img, origin, rack)
    img[10, 10] = 0   # 물체에서 먼 유령 칸 하나
    # 같은 지도를 world 기준 (0, 1.5)에서 시작했다고 보면 실제 랙은 y 2.5에 있다
    shifted = Box('rack', 'warehouse_rack', 0.0, 2.5, 0.0, 2.5, 0.6)
    wx, wy = occupied_world_points(img, RES, origin, (0.0, 1.5, 0.0))
    r = evaluate(wx, wy, [shifted])
    assert r['ghost_cells'] == 1
    assert r['shift_max'] <= RES     # 시작 자세를 넣으면 랙 위치는 맞는다
    assert not r['passed']           # 유령 칸이 하나라도 있으면 불합격 (#81)


def test_shifted_map_fails():
    origin = (-5.0, -5.0)
    img = np.full((200, 200), 254, dtype=np.uint8)
    # 폭 방향(y)으로 20 cm 어긋난 랙: 긴 변의 점 대부분이 20 cm 떨어진다
    # (길이 방향으로 밀면 긴 변 점은 여전히 테두리 위라 중앙값이 작다)
    draw_box(img, origin, Box('rack', 'warehouse_rack', 0.0, 1.2, 0.0, 2.5, 0.6))
    wx, wy = occupied_world_points(img, RES, origin, (0.0, 0.0, 0.0))
    r = evaluate(wx, wy, [Box('rack', 'warehouse_rack', 0.0, 1.0, 0.0, 2.5, 0.6)])
    assert r['shift_max'] == pytest.approx(0.2, abs=RES)
    assert not r['passed']          # 20 cm 어긋남 > 15 cm


SCENE = [Box('a', 'warehouse_rack', -2.0, 2.0, 0.0, 2.5, 0.6), Box('b', 'warehouse_rack', 3.0, -2.0, 0.0, 2.5, 0.6),
         Box('c', 'warehouse_pillar', 3.0, 3.0, 0.0, 0.3, 0.3), Box('d', 'warehouse_pillar', -3.0, -3.0, 0.0, 0.3, 0.3),
         Box('e', 'warehouse_rack', 0.0, 0.0, 0.0, 0.6, 2.5)]


def scene_points(moved=None, dy=0.0):
    origin = (-8.0, -8.0)
    img = np.full((320, 320), 254, dtype=np.uint8)
    for b in SCENE:
        draw_box(img, origin, Box(b.name, b.kind, b.cx, b.cy + (dy if b.name == moved else 0.0), b.yaw, b.hx, b.hy))
    return occupied_world_points(img, RES, origin, (0.0, 0.0, 0.0))


def test_fit_rigid_recovers_map_offset():
    # 지도 전체가 (+4, −3 cm)·0.2° 틀어져 그려졌다고 보고, 정합이 그 반대 보정을 찾는지
    wx, wy = scene_points()
    bad = (0.04, -0.03, math.radians(0.2))
    mx, my = apply_rigid(wx, wy, bad, (0.0, 0.0))
    fit = fit_rigid(mx, my, SCENE, (0.0, 0.0))
    fx, fy = apply_rigid(mx, my, fit, (0.0, 0.0))
    # 보정한 점이 원래 점 자리로 돌아온다 (칸 양자화만큼 남는다)
    assert np.median(np.hypot(fx - wx, fy - wy)) < RES / 2
    assert math.degrees(fit[2]) == pytest.approx(-0.2, abs=0.05)
    assert evaluate(fx, fy, SCENE)['passed']


def test_folded_map_fails_after_alignment():
    # 물체 하나만 20 cm 옮겨진 지도(접힘): 정합은 나머지에 맞고, 그 물체 밀림으로 불합격
    wx, wy = scene_points(moved='e', dy=0.2)
    fit = fit_rigid(wx, wy, SCENE, (0.0, 0.0))
    assert abs(fit[0]) < 0.02 and abs(fit[1]) < 0.02
    r = evaluate(*apply_rigid(wx, wy, fit, (0.0, 0.0)), SCENE)
    assert r['shift_worst'] == 'e' and r['shift_max'] > 0.15
    assert not r['passed']


def test_signed_median_misses_translation_but_shift_catches_it():
    # 랙을 폭 방향으로 5 cm 옮기면 양쪽 긴 변이 +5·−5 cm로 상쇄돼 부호 있는 중앙값은 작다. 물체별 밀림은 5 cm를 잡는다
    wx, wy = scene_points(moved='a', dy=0.05)
    r = evaluate(wx, wy, SCENE)
    assert r['shifts']['a'] == pytest.approx(0.05, abs=0.02)


def test_load_objects_reads_includes_and_walls(tmp_path):
    sdf = tmp_path / 'w.sdf'
    sdf.write_text('''<sdf version="1.8"><world name="w">
      <include><uri>model://warehouse_rack</uri><name>rack_a1</name><pose>-22.5 5 0 0 0 0</pose></include>
      <include><uri>model://loading_dock</uri><name>dock_a</name><pose>30 16 0 0 0 3.14159265</pose></include>
      <include><uri>model://unknown</uri><name>x</name><pose>0 0 0 0 0 0</pose></include>
    </world></sdf>''')
    objs = {b.name: b for b in load_objects(str(sdf))}
    assert set(objs) == {'rack_a1', 'dock_a', 'wall_north', 'wall_south', 'wall_east', 'wall_west'}
    # 도크 플랫폼은 모델 원점에서 +x로 0.75 m. 180° 돌아 있으면 x 29.25
    assert objs['dock_a'].cx == pytest.approx(29.25)


def test_pgm_png_roundtrip(tmp_path):
    img = np.array([[0, 205, 254], [254, 0, 205]], dtype=np.uint8)
    p = tmp_path / 'm.pgm'
    p.write_bytes(b'P5\n# comment\n3 2\n255\n' + img.tobytes())
    assert np.array_equal(read_pgm(str(p)), img)
    png = tmp_path / 'm.png'
    write_png(str(png), img)
    assert png.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'
