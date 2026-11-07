#!/usr/bin/env python3
"""넓은 식당 월드 restaurant_wide: 지금 식당(restaurant.sdf, 24 x 16 m)을 두 벌 붙여 매장 줄이 양 끝에 오게 한다 (32 x 24 m).

    python3 src/simulation/amr_gazebo/tools/make_restaurant_wide.py           # 저장소 루트에서. 아래 파일을 다시 쓴다
    python3 src/localization/amr_mapping/tools/restaurant_map.py src/simulation/amr_gazebo/worlds/restaurant_wide.sdf \\
        src/localization/amr_mapping/maps/restaurant_wide_gt.yaml --extent -8 24.2 -12 12 \\
        --stations src/fleet/amr_fleet_manager/config/stations_restaurant_wide.yaml \\
        --png src/localization/amr_mapping/maps/restaurant_wide_gt.png

배치 (위가 북쪽):
    매장 │ 홀 │ 퇴식·충전 ┃ 퇴식·충전 │ 홀 │ 매장        (서 → 동: 한식 - 책상 - 책상 - 한식)
1. 원래 식당을 남쪽 벽 중심선(y = −8.1)에 대고 거울로 한 벌 더 만든다 (복사본 이름 끝에 _e)
2. 둘을 함께 90° 돌린다 (x, y) → (−y, x): 원래 북쪽(매장 줄)이 서쪽 끝, 복사본의 매장 줄이 동쪽 끝, 원래 남쪽(퇴식대·정수대·
   충전소)이 가운데에서 등을 맞댄다. 원래 동쪽 벽(부스·대기 줄)은 북쪽 벽, 원래 서쪽 벽(입구)은 남쪽 벽이 된다
- 가운데 벽: 원래 남쪽 벽을 두 홀이 같이 쓴다 (퇴식대 창문·충전기·간판이 붙어 있다). 벽에서 비어 있던 두 곳에 문을 낸다
  (원래 x −6.9 ~ −4.4 자판기와 쓰레기통 사이, x 8.6 ~ 10.6 충전소와 대기 줄 사이)
- 물체는 거울상으로 뒤집지 않고 놓는다: 좌우 대칭인 가구·사람은 같아 보인다. 간판(signs 모델의 글자 메시)은 앞면이 로컬
  −y쪽이라 그대로 두면 벽을 보므로 180° 더 돌린다 (글자는 거꾸로 되지 않는다). 주방 바닥 메시(y 5 ~ 8 띠)도 같은 이유로 돌린다
- 원래 식당 파일이 바뀌면 이 도구를 다시 돌린다 (만든 파일은 손으로 고치지 않는다, test/test_restaurant_wide.py가 잡는다)

만드는 파일 (저장소 루트 기준):
- src/simulation/amr_gazebo/worlds/restaurant_wide.sdf
- src/simulation/amr_gazebo/config/dynamic_obstacles_restaurant_wide{,_peak}.yaml, static_people_restaurant_wide.yaml
- src/fleet/amr_fleet_manager/config/stations_restaurant_wide.yaml, congestion_zones_restaurant_wide.yaml
- src/bringup/amr_bringup/config/robots_restaurant_wide.yaml

사람 (원래 식당과 다른 점):
- 매장 카운터 앞 손님(원래 정지 모델 standing_1 ~ 5)은 걷는 손님 counter_N으로 바꿨다. 카운터 앞 좁은 사각형(픽업 자리와
  0.1 m 이상)에서 무작위로 서성인다. 손님 파일에 들어가 혼잡도에 센다
- 주방 직원은 걷지 않는다: 원래 직원 파일(dynamic_obstacles_restaurant_staff.yaml)의 시작 자리에 정지 모델 staff_N을
  카운터 쪽을 보게 세운다 (월드에 들어 있어 따로 띄우지 않는다)
- 평상시(_wide.yaml): 양쪽 홀에 12명씩 고르게 (원래 7명 + 카운터 앞 5명, 복사 같음)
- 피크(_wide_peak.yaml): 서쪽 홀에 몰린다 (원래 피크 13명 + 카운터 앞 5명) + 동쪽 홀은 평상시 3명 + 카운터 앞 5명.
  같은 메뉴(예: 한식 stall_1 / stall_1_e)를 붐비는 쪽과 한산한 쪽 중 고르는 장면이 생긴다
"""

import copy
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

MIRROR_Y = -8.1          # 원래 남쪽 벽 중심선. 복사본은 y → 2·MIRROR_Y − y
SUFFIX = '_e'
# 원래 남쪽 벽 (가운데 벽으로 다시 만든다)
DROP = {('outer_walls', 'south_collision'), ('outer_walls', 'south_visual'), ('outer_walls', 'wainscot_south_visual')}
# 가운데 벽 문 (원래 좌표 x 구간)
DOORS = ((-6.9, -4.4), (8.6, 10.6))
WALL_X = (-12.2, 12.2)
# 복사본에서 제자리 180° 더 돌릴 visual (모델 이름 → visual 이름들, None이면 전부). 거울상 대신 돌려 놓으므로
# 로컬 y쪽으로 치우친 메시는 반대편에 놓인다: 간판 글자(앞면이 로컬 −y), 주방 바닥(y 5 ~ 8 띠, 좌우 대칭 사각형)
TURN = {'signs': None, 'floor': ('kitchen_visual',)}
# 피크 때 한산한 동쪽 홀에 남길 평상시 손님 (복사본)
PEAK_EAST = ('customer_1', 'customer_4', 'customer_6')
# 걷는 손님으로 바꿀 카운터 앞 정지 손님. 서성이는 사각형 반폭 x, y 범위 (원래 좌표, 픽업 로봇과 0.1 m 이상)
COUNTER_PEOPLE = ('standing_1', 'standing_2', 'standing_3', 'standing_4', 'standing_5')
COUNTER_HALF_X = 0.2
COUNTER_Y = (3.95, 4.3)
COUNTER_SPEED = [0.2, 0.4]

ROOT = Path(__file__).resolve().parents[4]
GZ = ROOT / 'src/simulation/amr_gazebo'
FLEET = ROOT / 'src/fleet/amr_fleet_manager/config'
BRINGUP = ROOT / 'src/bringup/amr_bringup/config'


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def fmt(v: float) -> str:
    v = round(v, 7)
    return '0' if v == 0 else f'{v:.7f}'.rstrip('0').rstrip('.')


# ---------------- 좌표 변환 (원래 식당 → 넓은 식당) ----------------

def mirror_xy(x: float, y: float, yaw: float):
    """원래 남쪽 벽 중심선에 대고 거울: (x, 2·MIRROR_Y − y), 방향 yaw → −yaw."""
    return x, 2 * MIRROR_Y - y, -yaw


def rotate_xy(x: float, y: float, yaw: float):
    """전체를 90° 돌린다 (원래 북쪽 → 서쪽)."""
    return -y, x, wrap(yaw + math.pi / 2)


def place(x: float, y: float, yaw: float, east: bool):
    """원래 식당 자세 → 넓은 식당 자세. east면 복사본(거울 뒤 회전)."""
    if east:
        x, y, yaw = mirror_xy(x, y, yaw)
    return rotate_xy(x, y, yaw)


def place_area(area, east: bool):
    """혼잡 구역 [x_min, x_max, y_min, y_max] → 넓은 식당 (축에 맞춘 사각형은 돌려도 사각형)."""
    x0, x1, y0, y1 = area
    corners = [place(x, y, 0.0, east)[:2] for x in (x0, x1) for y in (y0, y1)]
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    return [round(min(xs), 4), round(max(xs), 4), round(min(ys), 4), round(max(ys), 4)]


# ---------------- 월드 ----------------

def _pose_text(el):
    p = el.find('pose')
    if p is None:
        p = ET.SubElement(el, 'pose')
        p.text = '0 0 0 0 0 0'
    vals = ([float(v) for v in p.text.split()] + [0.0] * 6)[:6]
    return p, vals


def transform_top(el, east: bool) -> None:
    """최상위 모델·include·조명 자세. 복사본은 거울(roll → −roll, yaw → −yaw) 뒤 회전."""
    p, (x, y, z, r, pi, yaw) = _pose_text(el)
    if east:
        r = -r
    x, y, yaw = place(x, y, yaw, east)
    p.text = ' '.join(fmt(v) for v in (x, y, z, r, pi, yaw))


def mirror_inner(el, top: bool = True) -> None:
    """복사본 모델 안쪽 자세: 로컬 y 뒤집기 (x, −y, z, −roll, pitch, −yaw). 모델 자신의 자세(top)는 건너뛴다."""
    for child in el:
        if child.tag == 'pose' and not top and child.text and child.text.strip():
            x, y, z, r, pi, yaw = ([float(v) for v in child.text.split()] + [0.0] * 6)[:6]
            child.text = ' '.join(fmt(v) for v in (x, -y, z, -r, pi, -yaw))
        if child.tag != 'pose':
            mirror_inner(child, top=False)


def turn_visuals(model, names=None) -> None:
    """모델 안 visual(names, None이면 전부)을 제자리에서 180° 돌린다 (간판: 앞면이 다시 홀을 보게)."""
    for v in model.iter('visual'):
        if names is not None and v.get('name') not in names:
            continue
        p = v.find('pose')
        if p is None:
            p = ET.SubElement(v, 'pose')
            p.text = '0 0 0 0 0 0'
        x, y, z, r, pi, yaw = ([float(t) for t in p.text.split()] + [0.0] * 6)[:6]
        p.text = ' '.join(fmt(t) for t in (x, y, z, r, pi, wrap(yaw + math.pi)))


def _name(el) -> str:
    return el.get('name') or (el.findtext('name') or '')


def _drop(model) -> None:
    for parent in model.iter():
        for child in list(parent):
            if (_name(model), child.get('name')) in DROP:
                parent.remove(child)


def center_wall() -> ET.Element:
    """두 홀이 같이 쓰는 가운데 벽 (원래 남쪽 벽 자리, 문 두 곳). 원래 좌표로 만들고 transform_top으로 돌린다."""
    edges = [WALL_X[0]] + [v for d in DOORS for v in d] + [WALL_X[1]]
    parts = []
    for i, (a, b) in enumerate(zip(edges[0::2], edges[1::2])):
        cx, w = (a + b) / 2, b - a
        box = f'<geometry><box><size>{fmt(w)} 0.2 3.2</size></box></geometry>'
        pose = f'<pose>{fmt(cx)} {MIRROR_Y} 1.6 0 0 0</pose>'
        parts.append(f'<collision name="wall_{i}_collision">{pose}{box}</collision>'
                     f'<visual name="wall_{i}_visual">{pose}{box}<material><ambient>0.93 0.9 0.84 1</ambient>'
                     f'<diffuse>0.93 0.9 0.84 1</diffuse></material></visual>')
    return ET.fromstring(f'<model name="center_wall"><static>true</static><link name="link">{"".join(parts)}'
                         '</link></model>')


HEADER = """
  넓은 식당 월드 (32 x 24 m, 서빙 로봇 5대용). 자동 생성 파일 - 손으로 고치지 않는다:
    amr_gazebo/tools/make_restaurant_wide.py가 restaurant.sdf(24 x 16 m)를 두 벌 붙여 매장 줄이 양 끝에 오게 했다.

  좌표계: x -8~24.2 (서→동), y -12~12 (남→북), 천장 3.2 m. map = world (map_origin_in_world [0, 0, 0]).
  원래 식당을 90° 돌린 것이 서쪽 홀(x -8~8), 그 거울 복사본(이름 끝 _e)이 동쪽 홀(x 8.2~24.2)이다.

    y=+12 ┌──────────── 부스·대기 줄 ─────────┬────────── 대기 줄_e·부스_e ────────┐
          │배│  6인석 L01~L04        충전·정수 ┃ 충전_e·정수_e      6인석_e         │배│
          │식│                       퇴식대    ┃ 퇴식대_e                           │식│
          │대│  4인석 t01~t15                  문                  4인석_e        │대│
          │·│                                 ┃                                    │_e│
          │매│                       키오스크  ┃ 키오스크_e                         │매│
          │장│  셀프 코너                      문                      셀프 코너_e │장│
    y=-12 └──────────────── 입구 ─────────────┴─────────────── 입구_e ─────────────┘
         x=-8                               x=8.1                                x=24.2

  - 가운데 벽(center_wall)은 원래 남쪽 벽 자리를 두 홀이 같이 쓴다. 문 두 곳 (y -6.9~-4.4, y 8.6~10.6)으로 이어진다
  - 물체는 거울상으로 뒤집지 않고 놓았다. 간판은 앞면이 홀을 보게 180° 더 돌렸다 (글자는 그대로 읽힌다)
  - 손님: 평상시 양쪽 12명씩(dynamic_obstacles_restaurant_wide.yaml), 피크는 서쪽 홀에 몰린다(_wide_peak.yaml).
    카운터 앞 손님은 걷는 손님(counter_N)이고, 주방 직원(staff_N)은 이 월드에 서 있는 정지 모델이다
  - 치수·통로 폭·가구 출처는 restaurant.sdf 머리말과 같다 (방향만 90° 돌았다)
"""


def staff_models(world) -> list:
    """주방 직원 정지 모델 (원래 좌표). 서 있는 손님 모델을 본떠 직원 파일의 시작 자리에 카운터 쪽(원래 남쪽)을 보게 세운다."""
    template = next(m for m in world.findall('model') if m.get('name') == COUNTER_PEOPLE[0])
    staff = load(GZ / 'config/dynamic_obstacles_restaurant_staff.yaml')['obstacles']
    models = []
    for o in staff:
        m = copy.deepcopy(template)
        m.set('name', o['id'])
        m.find('pose').text = f"{fmt(o['spawn']['x'])} {fmt(o['spawn']['y'])} 0 0 0 {fmt(-math.pi / 2)}"
        models.append(m)
    return models


def make_world(src: Path, dst: Path) -> None:
    tree = ET.parse(src)
    world = tree.getroot().find('world')
    # 카운터 앞 손님은 걷는 손님으로 (손님 파일), 주방 직원은 정지 모델로 (월드)
    staff = staff_models(world)
    for m in [m for m in world.findall('model') if m.get('name') in COUNTER_PEOPLE]:
        world.remove(m)
    world.extend(staff)
    originals = [c for c in world if c.tag in ('model', 'include', 'light')]
    for el in originals:
        if el.tag == 'model':
            _drop(el)
    copies = []
    for el in originals:
        c = copy.deepcopy(el)
        if c.tag == 'include':
            n = c.find('name')
            if n is not None:
                n.text += SUFFIX
        else:
            c.set('name', c.get('name') + SUFFIX)
        transform_top(c, east=True)
        if c.tag == 'model':
            mirror_inner(c)
            if el.get('name') in TURN:
                turn_visuals(c, TURN[el.get('name')])
        copies.append(c)
    for el in originals:
        transform_top(el, east=False)
    wall = center_wall()
    transform_top(wall, east=False)
    world.extend(copies + [wall])
    ET.indent(tree, space='  ')
    body = ET.tostring(tree.getroot(), encoding='unicode')
    dst.write_text(f'<?xml version="1.0"?>\n<!--{HEADER}-->\n{body}\n', encoding='utf-8', newline='\n')


# ---------------- 설정 yaml ----------------

def _flow(d: dict) -> str:
    return '{' + ', '.join(f'{k}: {_v(v)}' for k, v in d.items()) + '}'


def _v(v) -> str:
    if isinstance(v, float):
        return fmt(v)
    if isinstance(v, (list, tuple)):
        return '[' + ', '.join(_v(x) for x in v) + ']'
    if isinstance(v, str) and (not v or any(ch in v for ch in ':#{}[],"\'') or v != v.strip()):
        return '"' + v.replace('"', '\\"') + '"'
    return str(v)


def place_obstacle(o: dict, east: bool) -> dict:
    o = copy.deepcopy(o)
    if east:
        o['id'] += SUFFIX
        if 'note' in o:
            o['note'] = '(동쪽 복사) ' + o['note']
    s = o['spawn']
    x, y, yaw = place(s['x'], s['y'], s.get('yaw', 0.0), east)
    o['spawn'] = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 7)}
    o['waypoints'] = [dict(zip(('x', 'y'), (round(v, 4) for v in place(w['x'], w['y'], 0.0, east)[:2])))
                      for w in o['waypoints']]
    return o


def dump_obstacles(path: Path, header: str, obstacles: list) -> None:
    lines = [header.rstrip(), 'obstacles:']
    for o in obstacles:
        lines.append(f'  - id: {o["id"]}')
        for k in ('type', 'pattern'):
            lines.append(f'    {k}: {o[k]}')
        lines.append(f'    speed_range_mps: {_v(o["speed_range_mps"])}')
        lines.append(f'    spawn: {_flow(o["spawn"])}')
        lines.append('    waypoints:')
        lines += [f'      - {_flow(w)}' for w in o['waypoints']]
        if 'note' in o:
            lines.append(f'    note: {_v(o["note"])}')
    write(path, lines)


def write(path: Path, lines: list) -> None:
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')


def load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


GEN = ('# 자동 생성 (amr_gazebo/tools/make_restaurant_wide.py) - 손으로 고치지 않는다. 넓은 식당 월드 restaurant_wide.sdf용.\n'
       '# 원래 식당 좌표를 90° 돌린 서쪽 홀 + 거울 복사본(이름 끝 _e) 동쪽 홀. 원래 설명의 방위는 돌기 전 기준이다\n')


def both(items: list, fn) -> list:
    return [fn(i, False) for i in items] + [fn(i, True) for i in items]


def counter_obstacles(people: list) -> list:
    """카운터 앞 정지 손님 → 그 자리 둘레에서 서성이는 걷는 손님 (원래 좌표)."""
    out = []
    for p in people:
        if p['id'] not in COUNTER_PEOPLE:
            continue
        x0, x1 = round(p['x'] - COUNTER_HALF_X, 4), round(p['x'] + COUNTER_HALF_X, 4)
        y0, y1 = COUNTER_Y
        out.append({'id': p['id'].replace('standing', 'counter'), 'type': 'customer', 'pattern': 'random',
                    'speed_range_mps': COUNTER_SPEED, 'spawn': {'x': p['x'], 'y': p['y'], 'yaw': -math.pi / 2},
                    'waypoints': [{'x': x0, 'y': y0}, {'x': x1, 'y': y0}, {'x': x1, 'y': y1}, {'x': x0, 'y': y1}],
                    'note': '카운터 앞에서 서성인다 (원래 정지 모델 ' + p['id'] + '). 위 4점은 활동 범위'})
    return out


def make_obstacles() -> None:
    cfg = GZ / 'config'
    normal = load(cfg / 'dynamic_obstacles_restaurant.yaml')['obstacles']
    peak = load(cfg / 'dynamic_obstacles_restaurant_peak.yaml')['obstacles']
    people = load(cfg / 'static_people_restaurant.yaml')['people']
    counter = counter_obstacles(people)
    dump_obstacles(cfg / 'dynamic_obstacles_restaurant_wide.yaml', GEN + (
        f'# 평상시 손님 {2 * (len(normal) + len(counter))}명: 양쪽 홀에 {len(normal) + len(counter)}명씩 고르게 '
        '(dynamic_obstacles_restaurant.yaml + 카운터 앞 서성이는 손님\n'
        '# counter_N, 동쪽 복사 _e). 손님만 담는다 (혼잡도 노드가 전부 사람으로 센다). 주방 직원은 월드의 정지 모델 staff_N\n'),
        both(normal + counter, place_obstacle))
    east = [place_obstacle(o, True) for o in normal + counter if o['id'] in PEAK_EAST or o in counter]
    west = [place_obstacle(o, False) for o in peak + counter]
    dump_obstacles(cfg / 'dynamic_obstacles_restaurant_wide_peak.yaml', GEN + (
        f'# 피크 손님 {len(west) + len(east)}명: 서쪽 홀에 {len(west)}명 몰리고 (dynamic_obstacles_restaurant_peak.yaml 그대로 - '
        '셀프 코너 앞 통로·배식대 앞 몰림\n'
        f'# + 카운터 앞 {len(counter)}명) 동쪽 홀은 {len(east)}명 ({", ".join(PEAK_EAST)} 복사 + 카운터 앞). 같은 메뉴를 붐비는\n'
        '# 서쪽(stall_1)과 한산한 동쪽(stall_1_e) 중 고르는 장면이 생긴다 (혼잡도 반영 할당)\n'),
        west + east)

    def person(p, east):
        x, y, _ = place(p['x'], p['y'], 0.0, east)
        suffix = SUFFIX if east else ''
        return {**p, 'id': p['id'] + suffix, 'x': round(x, 4), 'y': round(y, 4), 'zone': p['zone'] + suffix}

    standing = [p for p in people if p['id'] not in COUNTER_PEOPLE]
    write(cfg / 'static_people_restaurant_wide.yaml',
          [GEN.rstrip(), '# 서 있는 손님 (정지 모델 standing_N, 동쪽 복사 standing_N_e). 형식은 static_people_restaurant.yaml.',
           '# 카운터 앞 standing_1 ~ 5는 걷는 손님 counter_N으로 바꿔 손님 파일에 있다', 'people:']
          + [f'  - {_flow(p)}' for p in both(standing, person)])


def make_fleet() -> None:
    base = load(FLEET / 'stations_restaurant.yaml')['stations']
    stations = {}
    for east in (False, True):
        for n, (x, y, yaw) in base.items():
            stations[n + (SUFFIX if east else '')] = place(x, y, yaw, east)
    write(FLEET / 'stations_restaurant_wide.yaml',
          [GEN.rstrip(),
           '# 스테이션 [x, y, yaw] (world = map). 서쪽 홀은 stations_restaurant.yaml 이름 그대로, 동쪽 홀은 이름 끝에 _e',
           '# (stall_1_e = 동쪽 한식). 작업 생성기 패턴 stall_*·t*·charger_*·wait_*는 양쪽 다 고른다', 'stations:']
          + [f'  {n}: [{fmt(x)}, {fmt(y)}, {fmt(yaw)}]' for n, (x, y, yaw) in stations.items()])

    def zone(z, east):
        suffix = SUFFIX if east else ''
        return {**z, 'id': z['id'] + suffix, 'area': place_area(z['area'], east),
                'description': ('(동쪽 복사) ' if east else '') + z['description']}

    lines = [GEN.rstrip(), '# 혼잡 구역 (congestion_zones_restaurant.yaml + 동쪽 복사 _e).',
             '# area [x_min, x_max, y_min, y_max]', 'zones:']
    for z in both(load(FLEET / 'congestion_zones_restaurant.yaml')['zones'], zone):
        lines += [f'  - id: {z["id"]}', f'    area: {_v(z["area"])}', f'    capacity: {z["capacity"]}',
                  f'    description: {_v(z["description"])}']
    write(FLEET / 'congestion_zones_restaurant_wide.yaml', lines)

    # 로봇 5대: 북쪽 대기 줄 양쪽 홀에서 번갈아
    names = ['wait_1', 'wait_1_e', 'wait_2', 'wait_2_e', 'wait_3']
    write(BRINGUP / 'robots_restaurant_wide.yaml',
          [GEN.rstrip(), f'# 서빙 로봇 스폰 자세 [x, y, yaw] (Gazebo world). 북쪽 대기 줄 양쪽 {"·".join(names)}',
           '# (stations_restaurant_wide.yaml과 같다). 앞에서부터 num_robots대를 쓴다', 'robots:']
          + [f'  amr_{i:02d}: [{fmt(stations[n][0])}, {fmt(stations[n][1])}, {fmt(stations[n][2])}]'
             for i, n in enumerate(names, start=1)])


def main() -> int:
    make_world(GZ / 'worlds/restaurant.sdf', GZ / 'worlds/restaurant_wide.sdf')
    make_obstacles()
    make_fleet()
    print('restaurant_wide 파일을 다시 썼다. 지도는 amr_mapping restaurant_map.py로 따로 만든다 (머리말)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
