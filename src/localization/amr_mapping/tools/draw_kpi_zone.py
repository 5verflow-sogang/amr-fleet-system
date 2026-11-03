#!/usr/bin/env python3
"""지도 v1 위에 위치 KPI 측정 구역과 5 m 칸별 위치 오차 중앙값을 그린다 (docs/algorithms/localization.md 3-1절).

    python3 draw_kpi_zone.py maps/warehouse_v1.pgm maps/warehouse_v1_kpi_zone.png

구역과 칸 값은 #183 Nav KPI 50쌍 투어(로컬, seed 1001)에서 민지가 정한 것을 그대로 옮겼다 (10/16 팀 합의).
좌표는 map 기준이다 (world = map + (0, 1.5)). 그림을 그릴 때만 PIL이 필요하다 - ZONE 등 상수는
다른 스크립트(예: experiments/nav_kpi/recompute_zone_cte.py)가 이 파일을 임포트해서 그대로 쓴다.
"""
import sys

RES, OX, OY = 0.05, -30.2, -21.8
ZONE = (-10.0, -15.0, 15.0, 5.0)   # map x0, y0, x1, y1
# 칸 왼쪽 아래 (x, y): 중앙값 cm
CELLS = {
    (-15, 15): 18.5, (-10, 15): 19.4,
    (-25, 10): 19.2, (-20, 10): 19.0, (-15, 10): 16.8, (-10, 10): 17.0, (-5, 10): 13.4, (0, 10): 22.2,
    (-25, 5): 17.0, (-20, 5): 16.2, (-15, 5): 15.7, (-10, 5): 13.1, (-5, 5): 9.5, (0, 5): 6.9, (5, 5): 6.5,
    (10, 5): 2.0,
    (-25, 0): 15.9, (-20, 0): 14.2, (-15, 0): 10.5, (-10, 0): 6.1, (-5, 0): 5.3, (0, 0): 3.2, (5, 0): 1.1,
    (10, 0): 2.0,
    (-30, -5): 9.4, (-20, -5): 7.5, (-15, -5): 4.4, (-10, -5): 4.8, (-5, -5): 2.3, (0, -5): 2.2, (5, -5): 0.4,
    (-30, -10): 14.8, (-25, -10): 13.8, (-20, -10): 11.1, (-10, -10): 3.6, (-5, -10): 2.3, (0, -10): 4.4,
    (-30, -15): 16.0, (-25, -15): 13.5, (-20, -15): 10.6, (-15, -15): 7.8, (-10, -15): 6.7, (-5, -15): 2.8,
    (0, -15): 4.3,
    (-25, -20): 14.0, (-20, -20): 11.3, (-15, -20): 11.8, (-10, -20): 13.5,
    (-25, -25): 15.3, (-10, -25): 10.9,
}
# world 좌표 → map = world − (0, 1.5)
MARKS = {'start': (0.0, 0.0), 'dock_1': (-27.0, 14.5), 'dock_2': (-27.0, 9.5)}


def main():
    from PIL import Image, ImageDraw, ImageFont

    pgm, out = sys.argv[1], sys.argv[2]

    img = Image.open(pgm).convert('RGB')
    W, H = img.size
    SCALE = 2
    img = img.resize((W * SCALE, H * SCALE), Image.NEAREST)

    def px(x, y):
        return (x - OX) / RES * SCALE, H * SCALE - (y - OY) / RES * SCALE

    def color(v, a):
        if v <= 8.0:
            return (40, 170, 70, a)
        if v <= 14.0:
            return (240, 170, 30, a)
        return (220, 50, 40, a)

    over = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(over)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 22)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    for (cx, cy), v in CELLS.items():
        x0, y0 = px(cx, cy + 5)
        x1, y1 = px(cx + 5, cy)
        d.rectangle([x0, y0, x1, y1], fill=color(v, 70), outline=(90, 90, 90, 120))
        d.text(((x0 + x1) / 2, (y0 + y1) / 2), f'{v:.1f}', fill=(20, 20, 20, 255), font=small, anchor='mm')
    x0, y0 = px(ZONE[0], ZONE[3])
    x1, y1 = px(ZONE[2], ZONE[1])
    d.rectangle([x0, y0, x1, y1], outline=(20, 90, 220, 255), width=7)
    d.text((x0 + 10, y0 + 8), 'KPI zone  x -10..15, y -15..5 m (map)', fill=(20, 90, 220, 255), font=font)
    for name, (x, y) in MARKS.items():
        u, v = px(x, y)
        d.ellipse([u - 9, v - 9, u + 9, v + 9], fill=(20, 20, 20, 255))
        d.text((u + 14, v), name, fill=(20, 20, 20, 255), font=font, anchor='lm')
    img = Image.alpha_composite(img.convert('RGBA'), over).convert('RGB')

    # 범례
    leg = Image.new('RGB', (img.width, 60), (255, 255, 255))
    g = ImageDraw.Draw(leg)
    x = 20
    for label, v in (('<= 8 cm', 5), ('8-14 cm', 10), ('> 14 cm', 20)):
        g.rectangle([x, 18, x + 30, 42], fill=color(v, 255)[:3])
        g.text((x + 40, 30), label, fill=(0, 0, 0), font=font, anchor='lm')
        x += 190
    g.text((x + 20, 30), 'median localization error per 5 m cell (Nav KPI tour, #183)', fill=(0, 0, 0), font=font,
           anchor='lm')
    full = Image.new('RGB', (img.width, img.height + 60), (255, 255, 255))
    full.paste(img, (0, 0))
    full.paste(leg, (0, img.height))
    full = full.resize((full.width // 2 * 1, full.height // 2 * 1), Image.LANCZOS)
    full.save(out, optimize=True)
    print(out, full.size)


if __name__ == '__main__':
    main()
