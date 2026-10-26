#!/usr/bin/env python3
"""지도 v1(SLAM)과 정답 지도(gt_map)를 겹쳐 그려 어디가 얼마나 휘었는지 보인다 (#183).

    python3 draw_map_bend.py maps/warehouse_v1_0025.yaml maps/warehouse_gt.yaml \\
        ../../simulation/amr_gazebo/worlds/warehouse.sdf maps/warehouse_v1_bend.png
    # 일부만 2배로: --crop X0 X1 Y0 Y1 (map 좌표) --scale 2
    # 다른 SLAM 지도: --label 'map v2' --map-origin-in-world X Y YAW (그 지도의 정렬 원점)

- 파랑: 정답 지도 물체, 빨강: 지도 v1 점유 칸. 두 지도는 같은 칸(0.025 m, 원점 [−30.2, −21.8])이라 칸끼리 겹친다.
  칸이 다른 지도(예: v2)면 정답 지도를 그 지도 칸과 정렬 원점에 맞춰 SDF에서 다시 그린다 (gt_map.rasterize)
- 물체마다 v1이 정답에서 밀린 방향·크기를 화살표(20배)와 cm로 적는다. map_quality 구조 정확도와 같은
  "물체별 밀림"인데, 지도 전체 정렬을 다시 구하지 않고 launch 정렬 원점(map_origin_in_world) 그대로 잰다 -
  내비·Fleet 실험에서 로봇이 실제로 겪는 어긋남이다
- 파란 점선: 위치 KPI 측정 구역 (draw_kpi_zone.py ZONE, map 기준)

PIL·numpy·PyYAML이 필요하다. amr_mapping 패키지를 import하므로 패키지 폴더에서 돌리거나 PYTHONPATH에 넣는다.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from amr_mapping.gt_map import MAP_ORIGIN_IN_WORLD, rasterize  # noqa: E402
from amr_mapping.map_quality import (  # noqa: E402
    GHOST_DISTANCE, associate, load_objects, occupied_world_points, read_pgm,
)

ZONE = (-10.0, -15.0, 15.0, 5.0)   # map x0, y0, x1, y1 (draw_kpi_zone.py와 같다)
ARROW_GAIN = 20.0


def load_map(path):
    with open(path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    return read_pgm(os.path.join(os.path.dirname(os.path.abspath(path)), meta['image'])), meta


def shift_vectors(img, meta, objs, origin_in_world):
    """물체마다 지도가 그 물체를 world에서 얼마나 밀어 그렸는지 (dx, dy) [m]. 점 3개 미만이면 뺀다."""
    wx, wy = occupied_world_points(img, meta['resolution'], meta['origin'][:2], origin_in_world)
    k, d, nx, ny = associate(wx, wy, objs)
    out = {}
    for i, b in enumerate(objs):
        sel = (k == i) & (np.abs(d) < GHOST_DISTANCE)
        if sel.sum() < 3:
            continue
        a = np.column_stack([nx[sel], ny[sel]])
        # map_quality.object_shift와 같은 식, 부호만 "지도가 밀린 쪽"으로
        t = np.linalg.solve(a.T @ a + 0.01 * sel.sum() * np.eye(2), a.T @ d[sel])
        out[b.name] = (float(t[0]), float(t[1]))
    return out


def font(size, bold=False):
    names = (['DejaVuSans-Bold.ttf', 'arialbd.ttf'] if bold else ['DejaVuSans.ttf', 'arial.ttf'])
    for n in names + ['/usr/share/fonts/truetype/dejavu/' + names[0]]:
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


def color(cm):
    if cm <= 5.0:
        return (30, 150, 60)
    if cm <= 10.0:
        return (230, 140, 0)
    return (210, 30, 30)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('v1_yaml')
    ap.add_argument('gt_yaml')
    ap.add_argument('sdf')
    ap.add_argument('out_png')
    ap.add_argument('--map-origin-in-world', nargs=3, type=float, default=list(MAP_ORIGIN_IN_WORLD))
    ap.add_argument('--crop', nargs=4, type=float, metavar=('X0', 'X1', 'Y0', 'Y1'), help='map 좌표 구역만')
    ap.add_argument('--scale', type=int, default=1)
    ap.add_argument('--label', default='map v1', help='SLAM 지도 이름 (범례)')
    a = ap.parse_args(argv)

    v1, m1 = load_map(a.v1_yaml)
    gt, mg = load_map(a.gt_yaml)
    res, (ox, oy) = m1['resolution'], m1['origin'][:2]
    h, w = v1.shape
    origin = tuple(a.map_origin_in_world)
    objs = load_objects(a.sdf)
    if v1.shape != gt.shape or res != mg['resolution'] or (ox, oy) != tuple(mg['origin'][:2]):
        gt = rasterize(objs, (w, h), res, (ox, oy), origin)
    shifts = shift_vectors(v1, m1, objs, origin)

    rgb = np.full((h, w, 3), 255, np.uint8)
    rgb[gt == 205] = (235, 235, 235)
    rgb[gt == 0] = (150, 185, 245)
    rgb[v1 == 0] = (215, 30, 30)
    img = Image.fromarray(rgb)

    x0, x1, y0, y1 = a.crop if a.crop else (ox, ox + w * res, oy, oy + h * res)
    c0, c1 = int((x0 - ox) / res), int(math.ceil((x1 - ox) / res))
    r0, r1 = h - int(math.ceil((y1 - oy) / res)), h - int((y0 - oy) / res)
    img = img.crop((c0, r0, c1, r1))
    s = a.scale
    img = img.resize((img.width * s, img.height * s), Image.NEAREST)

    def px(mx, my):   # map → 그림 픽셀
        return ((mx - ox) / res - c0) * s, ((h - (my - oy) / res) - r0) * s

    oyaw = origin[2]
    cw, sw = math.cos(-oyaw), math.sin(-oyaw)

    def w2m(x, y):
        dx, dy = x - origin[0], y - origin[1]
        return cw * dx - sw * dy, sw * dx + cw * dy

    d = ImageDraw.Draw(img)
    fsize = max(14, int(img.width / 110))
    f, fb = font(fsize), font(fsize, bold=True)
    # 측정 구역 (점선)
    zx0, zy0 = px(ZONE[0], ZONE[3])
    zx1, zy1 = px(ZONE[2], ZONE[1])
    dash = 12 * s
    for x in np.arange(zx0, zx1, 2 * dash):
        d.line([(x, zy0), (min(x + dash, zx1), zy0)], fill=(20, 90, 220), width=3)
        d.line([(x, zy1), (min(x + dash, zx1), zy1)], fill=(20, 90, 220), width=3)
    for y in np.arange(zy0, zy1, 2 * dash):
        d.line([(zx0, y), (zx0, min(y + dash, zy1))], fill=(20, 90, 220), width=3)
        d.line([(zx1, y), (zx1, min(y + dash, zy1))], fill=(20, 90, 220), width=3)
    d.text((zx0 + 8, zy0 + 6), 'KPI zone (map x -10..15, y -15..5)', fill=(20, 90, 220), font=fb)
    # 물체별 밀림
    for b in objs:
        if b.name not in shifts:
            continue
        sx, sy = shifts[b.name]
        cm = math.hypot(sx, sy) * 100
        mx, my = w2m(b.cx, b.cy)
        u, v = px(mx, my)
        if not (0 <= u < img.width and 0 <= v < img.height):
            continue
        col = color(cm)
        # 화살표: world 밀림을 map 방향으로 (회전 0.005°라 사실상 같다)
        ex, ey = px(mx + ARROW_GAIN * sx, my + ARROW_GAIN * sy)
        d.line([(u, v), (ex, ey)], fill=col, width=max(3, s * 2))
        ang = math.atan2(ey - v, ex - u)
        hl = fsize * 0.8
        d.polygon([(ex, ey), (ex - hl * math.cos(ang - 0.4), ey - hl * math.sin(ang - 0.4)),
                   (ex - hl * math.cos(ang + 0.4), ey - hl * math.sin(ang + 0.4))], fill=col)
        label = f'{cm:.1f}'
        tb = d.textbbox((u, v - fsize), label, font=fb, anchor='mm')
        d.rectangle([tb[0] - 3, tb[1] - 2, tb[2] + 3, tb[3] + 2], fill=(255, 255, 255))
        d.text((u, v - fsize), label, fill=col, font=fb, anchor='mm')

    # 범례
    lh = int(fsize * 2.4)
    leg = Image.new('RGB', (img.width, lh), (255, 255, 255))
    g = ImageDraw.Draw(leg)
    x = 12
    items = [((150, 185, 245), 'ground truth (world SDF)'), ((215, 30, 30), f'{a.label} (SLAM) occupied'),
             (color(3), '<= 5 cm'), (color(8), '5-10 cm'), (color(15), '> 10 cm')]
    for c, label in items:
        g.rectangle([x, lh // 2 - fsize // 2, x + fsize, lh // 2 + fsize // 2], fill=c)
        g.text((x + fsize + 6, lh // 2), label, fill=(0, 0, 0), font=f, anchor='lm')
        x += int(g.textlength(label, font=f)) + fsize + 30
    o = ', '.join(np.format_float_positional(v) for v in origin)
    note = f'arrow = {a.label.split()[-1]} object shift x{ARROW_GAIN:.0f}, label cm (map_origin_in_world [{o}])'
    g.text((x, lh // 2), note, fill=(0, 0, 0), font=f, anchor='lm')
    full = Image.new('RGB', (img.width, img.height + lh), (255, 255, 255))
    full.paste(img, (0, 0))
    full.paste(leg, (0, img.height))
    full.save(a.out_png, optimize=True)
    print(a.out_png, full.size)
    for name, (sx, sy) in sorted(shifts.items(), key=lambda kv: -math.hypot(*kv[1]))[:8]:
        print(f'  {name}: {math.hypot(sx, sy) * 100:.1f} cm (dx {sx * 100:+.1f}, dy {sy * 100:+.1f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
