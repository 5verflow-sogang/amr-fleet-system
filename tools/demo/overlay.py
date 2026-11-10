"""식당 시연 끔·켬 비교 영상의 화면 배치와 강조 표시. ROS·ffmpeg 없이 돈다 (compose.py가 영상·bag을 읽고 쓴다).

화면 배치는 「식당 시연 영상 편집 가이드」(시나리오 3 최종본 기준, 1600×900 픽셀, 원점 왼쪽 위)를 따른다:
시나리오명 (40,22) 26 px, 컷 제목 (40,83) 41 px, 끔·켬 패널 (32,189)·(808,189) 760×494, 원본 영상 (36,243)·(812,243)
752×424, 설명 자막 (40,745) 28 px, 진행선 (40,845)~(1560,845). 끔 #FF9292, 켬 #83B4FF.

강조 표시(layers)는 녹화 기록으로 그린다. 로봇 위치는 /fleet/robot_states(map = world), 혼잡 구역은 /fleet/congestion,
주문 시각은 /demo/t0, 배정은 /fleet/task_table의 assignee_id, 도착은 /fleet/task_status 단계 보고다 (timeline.json).
월드 좌표를 카메라 화면으로 옮기는 식은 demo_cameras_restaurant.yaml의 카메라 자세·화각(핀홀)이다:
Gazebo 카메라는 자기 +x를 보고 화면 오른쪽이 −y, 위가 +z다.

    zones       혼잡 구역 반투명 빨강 (혼잡도에 따라 진하기)
    candidates  cut sheet의 후보 로봇(1-1: 1호·3호)에 흰 동그라미 + 주문 자리까지 직선 거리
    request     주문 자리에 주황 고리가 퍼지고 "주문 · 배식대 1 → 1번 테이블" 배지 (/demo/t0부터)
    assign      배정 순간부터 주문 자리 → 맡은 로봇 점선, 그 로봇 동그라미(끔 빨강·켬 파랑) + "1호 배정"
    hero        맡은 로봇을 따라다니는 동그라미 + 이름
    dim         hero 동그라미 밖을 어둡게 (다른 로봇에 눈이 덜 가게)
    card        패널 오른쪽 아래 작업 알림 (요청 뒤 시각, 작업, 맡은 로봇)
    clock       패널 왼쪽 위 "요청 후 12.3 s"
    arrived     주문 자리에 닿은 뒤 "도착 · 요청 후 23.4 s"
"""

from __future__ import annotations

import bisect
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CAMERAS_YAML = REPO / 'src' / 'simulation' / 'amr_gazebo' / 'config' / 'demo_cameras_restaurant.yaml'
STATIONS_YAML = REPO / 'src' / 'fleet' / 'amr_fleet_manager' / 'config' / 'stations_restaurant.yaml'

sys.path.insert(0, str(REPO / 'experiments' / 'restaurant_demo'))
import demo_metrics as dm  # noqa: E402  (작업표 접기·도착·멈춤 정의를 지표 표와 같게 쓴다)

# ---------------------------------------------------------------- 가이드 화면 배치 (1600×900)
CANVAS = (1600, 900)
BG = (13, 17, 23)
PANEL_BG = (22, 27, 34)
PANEL_BORDER = (52, 61, 73)
TEXT = (232, 237, 243)
SCENE_TEXT = (195, 204, 214)
MUTED = (162, 173, 186)
SIDE_COLOR = {'off': (255, 146, 146), 'on': (131, 180, 255)}
REQUEST_COLOR = (255, 196, 84)
ZONE_COLOR = (255, 84, 84)
BAR_BG = (52, 64, 77)
BAR_FILL = (110, 168, 255)
PANEL_XY = {'off': (32, 189), 'on': (808, 189)}
PANEL_SIZE = (760, 494)
VIDEO_OFFSET = (4, 54)          # 패널 왼쪽 위 → 원본 영상 왼쪽 위 ((36,243) − (32,189))
VIDEO_SIZE = (752, 424)
SIDES = ('off', 'on')

FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',      # 가이드: 녹화 컨테이너 Chromium이 쓴 글꼴, 최종본 전부 이것
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    'C:/Windows/Fonts/malgunbd.ttf',
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
)

# 표시 높이 (m): 로봇 동그라미는 몸통 가운데쯤, 주문 고리는 배식대 상판 높이
ROBOT_Z = 1.0
STATION_Z = 0.9
RING_M = 0.45


class Fonts:
    """크기마다 한 번만 연다. path가 없으면 FONT_CANDIDATES에서 처음 있는 것."""

    def __init__(self, path: Optional[str] = None):
        self.path = path or next((p for p in FONT_CANDIDATES if Path(p).is_file()), None)
        if self.path is None:
            raise FileNotFoundError('한글 글꼴이 없다: --font로 경로를 준다 (가이드: wqy-zenhei.ttc)')
        self._cache: dict[int, ImageFont.FreeTypeFont] = {}

    def __call__(self, size: int) -> ImageFont.FreeTypeFont:
        if size not in self._cache:
            self._cache[size] = ImageFont.truetype(self.path, size)
        return self._cache[size]


# ---------------------------------------------------------------- 카메라 (월드 → 화면)
@dataclass(frozen=True)
class Camera:
    name: str
    pose: tuple            # x, y, z, roll, pitch, yaw (SDF: R = Rz(yaw)·Ry(pitch)·Rx(roll))
    width: int
    height: int
    fov_deg: float         # 가로 화각

    @property
    def fx(self) -> float:
        return self.width / 2 / math.tan(math.radians(self.fov_deg) / 2)

    def _rotation(self):
        _, _, _, r, p, y = self.pose
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr))

    def project(self, x: float, y: float, z: float = 0.0) -> Optional[tuple[float, float, float]]:
        """월드 점 → (u, v, 깊이 m). 카메라 뒤(깊이 0.1 m 밑)면 None. 화면 밖이어도 좌표는 돌려준다."""
        R = self._rotation()
        d = (x - self.pose[0], y - self.pose[1], z - self.pose[2])
        fwd, left, up = (sum(R[i][k] * d[i] for i in range(3)) for k in range(3))   # Rᵀ·d
        if fwd < 0.1:
            return None
        return self.width / 2 - self.fx * left / fwd, self.height / 2 - self.fx * up / fwd, fwd

    def inside(self, u: float, v: float, margin: float = 0.0) -> bool:
        return margin <= u <= self.width - margin and margin <= v <= self.height - margin


def load_cameras(path: Path = CAMERAS_YAML) -> dict[str, Camera]:
    """amr_gazebo demo_cameras.load_cameras와 같은 규칙 (defaults에 카메라마다 덮어쓴다)."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    defaults = data.get('defaults', {})
    out = {}
    for name, spec in data['cameras'].items():
        m = {**defaults, **spec}
        out[name] = Camera(name, tuple(float(v) for v in m['pose']), int(m['width']), int(m['height']),
                           float(m['fov_deg']))
    return out


def load_stations(path: Path = STATIONS_YAML) -> dict[str, tuple[float, float]]:
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return {k: (float(v[0]), float(v[1])) for k, v in data['stations'].items()}


def robot_label(robot_id: str) -> str:
    """amr_03 → 3호."""
    m = re.search(r'(\d+)$', robot_id or '')
    return f'{int(m[1])}호' if m else (robot_id or '-')


# ---------------------------------------------------------------- 녹화 기록 (timeline.json)
class Track:
    """[(t, x, y)] → 시각 t의 위치 (사이는 직선 보간, 앞뒤는 끝값)."""

    def __init__(self, points: Sequence[Sequence[float]]):
        pts = sorted((float(t), float(x), float(y)) for t, x, y in points)
        self.t = [p[0] for p in pts]
        self.xy = [(p[1], p[2]) for p in pts]

    def at(self, t: float) -> Optional[tuple[float, float]]:
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.xy[0]
        if i >= len(self.t):
            return self.xy[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        (x0, y0), (x1, y1) = self.xy[i - 1], self.xy[i]
        return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a


def build_timeline(table, statuses, poses, congestion, t0_msgs, videos, key_task: str = '',
                   stop_kw: Optional[dict] = None) -> dict:
    """bag에서 읽은 원자료 → timeline.json 내용 (compose.py extract).

    table: [(t, [행 dict])] (analyze.read_bag와 같은 행), statuses: [{'t','task_id','state','phase'}],
    poses: {robot: [(t, x, y)]}, congestion: [(t, [(zone_id, [x0,x1,y0,y1], value)])],
    t0_msgs: [(t, frame_id)] (/demo/t0), videos: video.json 내용.
    핵심 작업: key_task > /demo/t0의 frame_id. 0초: /demo/t0 시각 > 핵심 작업 요청 시각.
    """
    tasks = dm.fold_table(table)
    t0_msg = t0_msgs[0] if t0_msgs else None
    key = key_task or (t0_msg[1] if t0_msg else '')
    if key not in tasks:
        raise ValueError(f'핵심 작업을 작업표에서 못 찾았다: {key!r} (/demo/t0이 없으면 --key-task)')
    task = tasks[key]
    t0 = t0_msg[0] if t0_msg and (not key_task or t0_msg[1] == key_task) else task.request
    end = max([t for t, _ in table] + [s['t'] for s in statuses]
              + [p[0] for pts in poses.values() for p in pts[-1:]] + [t0])
    arrive = dm.arrival_time(task, statuses, task.pickup)
    metrics = {'robot': task.assignee, 'assign_s': None if task.assign is None else round(task.assign - t0, 2),
               'arrive_s': None if arrive is None else round(arrive - t0, 2)}
    track = poses.get(task.assignee)
    if track:
        stop_end = arrive if arrive is not None else end
        if stop_end > t0:
            metrics.update(dm.stops(track, t0, stop_end, arrived=arrive is not None, **(stop_kw or {})))
    return {
        'key_task': key, 'pickup': task.pickup, 'dropoff': task.dropoff, 't0': t0, 'end': end,
        'assignee': task.assignee, 'assign_t': task.assign, 'arrive_t': arrive,
        'robots': {r: [list(p) for p in sorted(pts)] for r, pts in poses.items()},
        'congestion': [[t, [[z, list(a), v] for z, a, v in zones]] for t, zones in sorted(congestion, key=lambda c: c[0])],
        'videos': videos, 'metrics': metrics,
    }


@dataclass
class Run:
    """한쪽(끔/켬) 실행. 시각은 모두 시뮬레이션 s."""
    side: str
    t0: float
    pickup: str
    dropoff: str
    assignee: str
    assign_t: Optional[float]
    arrive_t: Optional[float]
    end: float
    tracks: dict[str, Track]
    congestion: list
    videos: dict
    metrics: dict
    _ct: list = field(default_factory=list)

    @classmethod
    def from_timeline(cls, side: str, tl: dict) -> 'Run':
        cong = [(float(t), zones) for t, zones in tl.get('congestion', [])]
        return cls(side, float(tl['t0']), tl['pickup'], tl['dropoff'], tl.get('assignee', ''), tl.get('assign_t'),
                   tl.get('arrive_t'), float(tl.get('end', tl['t0'])),
                   {r: Track(p) for r, p in tl.get('robots', {}).items()}, cong, tl.get('videos', {}),
                   tl.get('metrics', {}), [c[0] for c in cong])

    def rel(self, event: str) -> Optional[float]:
        """요청(0초) 기준 사건 시각: request·assign·arrive. 없으면 None."""
        value = {'request': self.t0, 'assign': self.assign_t, 'arrive': self.arrive_t}[event]
        return None if value is None else value - self.t0

    def zones_at(self, t: float) -> list:
        """시각 t에 마지막으로 받은 혼잡도 [(zone_id, area, value)]."""
        i = bisect.bisect_right(self._ct, t) - 1
        return self.congestion[i][1] if i >= 0 else []

    def pose(self, robot: str, t: float) -> Optional[tuple[float, float]]:
        tr = self.tracks.get(robot)
        return tr.at(t) if tr else None


# ---------------------------------------------------------------- 컷시트
LAYERS = {'zones', 'candidates', 'request', 'assign', 'hero', 'dim', 'card', 'clock', 'arrived'}
EVENT = re.compile(r'^\s*(?P<name>[a-z_]+)\s*(?:(?P<sign>[+-])\s*(?P<off>\d+(?:\.\d+)?))?\s*$')


@dataclass
class Cut:
    title: str = ''
    caption: str = ''
    camera: str = ''
    start: str = 'request'
    length: float = 4.0         # 완성본 길이 (s, hold 빼고)
    speed: float = 1.0          # 배속 (원본 s / 완성본 s)
    hold: float = 0.0           # 끝 화면을 멈춰 두는 시간 (s)
    layers: tuple = ()
    check: str = ''             # 이 컷 문구가 맞으려면 성립해야 하는 사건 순서 (예: arrive_on < arrive_off)
    result: bool = False

    @property
    def total(self) -> float:
        return self.length + self.hold


@dataclass
class CutSheet:
    scene: str
    feature: str
    candidates: list
    request_label: str
    station_label: str
    camera_names: dict
    cuts: list
    result: dict
    fps: int = 20
    crf: int = 17

    @classmethod
    def load(cls, path) -> 'CutSheet':
        d = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        cuts = []
        for c in d['cuts']:
            cut = Cut(**{**c, 'layers': tuple(c.get('layers', ()))})
            unknown = set(cut.layers) - LAYERS
            if unknown:
                raise ValueError(f'모르는 layer: {sorted(unknown)} (가능: {sorted(LAYERS)})')
            cuts.append(cut)
        return cls(d['scene'], d['feature'], list(d.get('candidates', [])), d['request_label'],
                   d['station_label'], dict(d.get('camera_names', {})), cuts, dict(d.get('result', {})),
                   int(d.get('fps', 20)), int(d.get('crf', 17)))


def events(runs: dict[str, Run]) -> dict[str, Optional[float]]:
    """요청 기준 사건 시각. X_off·X_on은 그쪽 실행, X는 둘 다 지난 시각(늦은 쪽)."""
    out: dict[str, Optional[float]] = {}
    for name in ('request', 'assign', 'arrive'):
        vals = {s: runs[s].rel(name) for s in SIDES}
        for s, v in vals.items():
            out[f'{name}_{s}'] = v
        out[name] = None if None in vals.values() else max(vals.values())
    out['end'] = min(r.end - r.t0 for r in runs.values())
    return out


def resolve(expr: str, ev: dict) -> float:
    """'arrive_on-6' → 요청 기준 s. 사건이 없으면 ValueError."""
    m = EVENT.match(str(expr))
    if not m or m['name'] not in ev:
        raise ValueError(f'시각 식을 못 읽는다: {expr!r} (가능: {sorted(ev)} ± s)')
    base = ev[m['name']]
    if base is None:
        raise ValueError(f'{m["name"]} 사건이 녹화에 없다 ({expr})')
    off = float(m['off'] or 0.0)
    return base + off if m['sign'] != '-' else base - off


def check_cut(cut: Cut, ev: dict) -> Optional[str]:
    """check 식(a < b)이 녹화와 맞지 않으면 경고 문구."""
    if not cut.check:
        return None
    a, op, b = re.match(r'^\s*(\S+)\s*(<|>)\s*(\S+)\s*$', cut.check).groups()
    try:
        va, vb = resolve(a, ev), resolve(b, ev)
    except ValueError as e:
        return f'{cut.title}: {e}'
    ok = va < vb if op == '<' else va > vb
    return None if ok else f'"{cut.title}": {cut.check}가 아니다 ({a} {va:.1f} s, {b} {vb:.1f} s) - 문구를 녹화에 맞춘다'


def text_values(runs: dict[str, Run], stations: dict, candidates: Sequence[str]) -> dict:
    """제목·자막의 {이름} 자리: robot_off·robot_on, arrive_off·arrive_on(s), stops_off·stops_on, dist_<로봇>."""
    v: dict[str, str] = {}
    for s, r in runs.items():
        v[f'robot_{s}'] = robot_label(r.assignee)
        a = r.rel('arrive')
        v[f'arrive_{s}'] = '-' if a is None else f'{a:.0f}'
        v[f'stops_{s}'] = str(r.metrics.get('stops', '-'))
    st = stations.get(runs['off'].pickup)
    for rid in candidates:
        p = runs['off'].pose(rid, runs['off'].t0)
        v[f'dist_{rid}'] = '-' if (p is None or st is None) else f'{math.dist(p, st):.1f}'
    return v


def fill(text: str, values: dict) -> str:
    return re.sub(r'\{(\w+)\}', lambda m: values.get(m[1], m[0]), text or '')


# ---------------------------------------------------------------- 그리기 (패널 좌표)
def _dashed(g: ImageDraw.ImageDraw, a, b, color, width=4, dash=12):
    n = max(int(math.dist(a, b) // dash), 1)
    for i in range(0, n, 2):
        t0, t1 = i / n, min((i + 1) / n, 1.0)
        g.line([a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0,
                a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1], fill=color, width=width)


def _badge(g: ImageDraw.ImageDraw, xy, text, color, font, box=VIDEO_SIZE, text_color=BG):
    tw = g.textlength(text, font=font)
    h = font.size + 12
    x = min(max(xy[0], 4), box[0] - tw - 20)
    y = min(max(xy[1], 4), box[1] - h - 4)
    g.rounded_rectangle([x, y, x + tw + 16, y + h], 6, fill=color + (235,))
    g.text((x + 8, y + 5), text, font=font, fill=text_color + (255,))


def _ring(g: ImageDraw.ImageDraw, u, v, r, color, width=5):
    """검은 테두리를 두른 동그라미 (빨간 혼잡 구역·밝은 바닥 위에서도 보이게)."""
    g.ellipse([u - r - 2, v - r - 2, u + r + 2, v + r + 2], outline=(13, 17, 23, 220), width=width + 4)
    g.ellipse([u - r, v - r, u + r, v + r], outline=color + (255,), width=width)


class Painter:
    """원본 영상 한 장(카메라 해상도) → 패널 영상 자리(752×424)에 강조 표시까지 그린 RGB."""

    def __init__(self, sheet: CutSheet, cameras: dict[str, Camera], stations: dict, fonts: Fonts):
        self.sheet, self.cameras, self.stations, self.fonts = sheet, cameras, stations, fonts

    def _pt(self, cam: Camera, x, y, z):
        p = cam.project(x, y, z)
        if p is None:
            return None
        s = VIDEO_SIZE[0] / cam.width
        r = min(max(cam.fx * RING_M / p[2] * s, 16.0), 60.0)
        return p[0] * s, p[1] * (VIDEO_SIZE[1] / cam.height), r

    def video(self, frame: Image.Image, cam_name: str, run: Run, t: float, layers: Sequence[str]) -> Image.Image:
        cam = self.cameras[cam_name]
        base = frame.convert('RGB').resize(VIDEO_SIZE, Image.BILINEAR).convert('RGBA')
        ov = Image.new('RGBA', VIDEO_SIZE, (0, 0, 0, 0))
        g = ImageDraw.Draw(ov)
        f = self.fonts
        color = SIDE_COLOR[run.side]
        rel = t - run.t0
        station = self.stations.get(run.pickup)
        hero_pos = run.pose(run.assignee, t) if run.assignee else None
        hero_pt = self._pt(cam, *hero_pos, ROBOT_Z) if hero_pos else None
        assigned = run.assign_t is not None and t >= run.assign_t

        if 'zones' in layers:
            for zone_id, area, value in run.zones_at(t):
                if value <= 0.05:
                    continue
                x0, x1, y0, y1 = area
                pts = [self._pt(cam, x, y, 0.0) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
                if None in pts:
                    continue
                a = int(20 + 50 * min(value, 1.0))
                poly = [(p[0], p[1]) for p in pts]
                g.polygon(poly, fill=ZONE_COLOR + (a,))
                g.line(poly + poly[:1], fill=ZONE_COLOR + (int(120 + 135 * min(value, 1.0)),), width=3)
                lx, ly = min(p[0] for p in pts), max(p[1] for p in pts)
                label = f'혼잡 {value:.1f}'
                g.rounded_rectangle([lx + 4, ly - 26, lx + 14 + g.textlength(label, font=f(15)), ly - 4], 4,
                                    fill=(120, 20, 20, 210))
                g.text((lx + 9, ly - 25), label, font=f(15), fill=(255, 235, 235, 255))
        if 'dim' in layers and hero_pt and assigned:
            mask = Image.new('L', VIDEO_SIZE, 140)
            u, v, r = hero_pt
            ImageDraw.Draw(mask).ellipse([u - r - 10, v - r - 10, u + r + 10, v + r + 10], fill=0)
            ov.paste((0, 0, 0, 255), (0, 0), mask)
            g = ImageDraw.Draw(ov)
        if 'candidates' in layers and station:
            for rid in self.sheet.candidates:
                p = run.pose(rid, t)
                pt = self._pt(cam, *p, ROBOT_Z) if p else None
                if not pt:
                    continue
                u, v, r = pt
                _ring(g, u, v, r, (255, 255, 255), 4)
                label = f'{robot_label(rid)} · {self.sheet.station_label}까지 {math.dist(p, station):.1f} m'
                _badge(g, (u - 60, v + r + 6), label, (255, 255, 255), f(17))
        if 'request' in layers and station and rel >= 0:
            sp = self._pt(cam, *station, STATION_Z)
            if sp:
                u, v, _ = sp
                phase = (rel % 1.2) / 1.2 if rel < 3.6 else 1.0      # 처음 3.6 s는 퍼지는 고리, 뒤로는 고정 고리
                for k in range(3):
                    rr = 14 + 34 * ((phase + k / 3) % 1.0) if rel < 3.6 else 14 + 12 * k
                    alpha = int(255 * (1 - (rr - 14) / 40)) if rel < 3.6 else 255 - 70 * k
                    g.ellipse([u - rr, v - rr, u + rr, v + rr], outline=REQUEST_COLOR + (max(alpha, 40),), width=4)
                # 주문 자리가 화면 위 끝(탑뷰의 배식대)이면 고리 아래에 붙인다
                _badge(g, (u + 30, v - 44 if v > 90 else v + 26), self.sheet.request_label, REQUEST_COLOR, f(18))
        if 'assign' in layers and assigned and station and hero_pt:
            sp = self._pt(cam, *station, STATION_Z)
            if sp:
                _dashed(g, sp[:2], hero_pt[:2], color + (255,))
            u, v, r = hero_pt
            _ring(g, u, v, r, color)
            _badge(g, (u + r + 4, v - 12), f'{robot_label(run.assignee)} 배정', color, f(18))
        elif 'hero' in layers and hero_pt and assigned:
            u, v, r = hero_pt
            _ring(g, u, v, r, color)
            _badge(g, (u + r + 4, v - 12), robot_label(run.assignee), color, f(18))
        if 'arrived' in layers and run.arrive_t is not None and t >= run.arrive_t and hero_pt:
            u, v, r = hero_pt
            _badge(g, (u - 70, v + r + 6), f'도착 · 요청 후 {run.arrive_t - run.t0:.1f} s', color, f(17))
        if 'clock' in layers:
            txt = f'요청 후 {rel:.1f} s' if rel >= 0 else f'요청 {-rel:.1f} s 전'
            g.rounded_rectangle([8, 8, 20 + g.textlength(txt, font=f(18)), 40], 6, fill=(13, 17, 23, 200))
            g.text((14, 11), txt, font=f(18), fill=TEXT + (255,))
        if 'card' in layers and rel >= 0:
            who = f'▶ {robot_label(run.assignee)}' if assigned else '배정 대기'
            lines = (f'요청 후 {rel:.1f} s · 새 작업', f'{self.sheet.request_label.split("·")[-1].strip()}  {who}')
            w = max(g.textlength(lines[0], font=f(15)), g.textlength(lines[1], font=f(19))) + 28
            # 오른쪽 아래 (탑뷰 위쪽은 배식대·매장 앞이라 비워 둔다)
            x, y = VIDEO_SIZE[0] - w - 10, VIDEO_SIZE[1] - 74
            g.rounded_rectangle([x, y, VIDEO_SIZE[0] - 10, y + 64], 8, fill=PANEL_BG + (235,),
                                outline=PANEL_BORDER + (255,), width=2)
            g.text((x + 14, y + 6), lines[0], font=f(15), fill=MUTED + (255,))
            g.text((x + 14, y + 30), lines[1], font=f(19), fill=TEXT + (255,))
        return Image.alpha_composite(base, ov).convert('RGB')

    # ---- 한 장 전체
    def frame(self, cut: Cut, index: int, count: int, progress: float, panels: dict[str, Image.Image],
              values: dict) -> Image.Image:
        """compose 한 장: 머리글·제목·패널 둘·자막·진행선. panels = {'off': 752×424, 'on': 752×424}."""
        img = Image.new('RGB', CANVAS, BG)
        g = ImageDraw.Draw(img)
        f = self.fonts
        g.text((40, 22), self.sheet.scene, font=f(26), fill=SCENE_TEXT)
        g.text((40, 83), fill(cut.title, values), font=f(41), fill=TEXT)
        cam_name = self.sheet.camera_names.get(cut.camera, cut.camera)
        for side in SIDES:
            x, y = PANEL_XY[side]
            g.rounded_rectangle([x, y, x + PANEL_SIZE[0], y + PANEL_SIZE[1]], 9, fill=PANEL_BG, outline=PANEL_BORDER,
                                width=2)
            g.ellipse([x + 18, y + 14, x + 32, y + 28], fill=SIDE_COLOR[side])
            g.text((x + 40, 194), f'{self.sheet.feature} {"끔" if side == "off" else "켬"}', font=f(23), fill=TEXT)
            g.text((x + 570, 197), cam_name, font=f(20), fill=MUTED)
            img.paste(panels[side], (x + VIDEO_OFFSET[0], y + VIDEO_OFFSET[1]))
        g.text((40, 745), fill(cut.caption, values), font=f(28), fill=TEXT)
        _progress(g, f, index, count, progress)
        return img


def _progress(g, f, index, count, progress):
    g.line([40, 845, 1560, 845], fill=BAR_BG, width=2)
    g.line([40, 845, 40 + 1520 * min(max(progress, 0.0), 1.0), 845], fill=BAR_FILL, width=3)
    txt = f'{index} / {count}'
    g.text((1560 - g.textlength(txt, font=f(18)), 856), txt, font=f(18), fill=MUTED)


def result_frame(sheet: CutSheet, runs: dict[str, Run], fonts: Fonts, index: int, count: int,
                 values: dict) -> Image.Image:
    """마지막 결과 장표: 대표 문장, 끔·켬 핵심 수치, 요청 → 도착 막대 그래프, 시간 기준·지표 정의."""
    res = sheet.result
    img = Image.new('RGB', CANVAS, BG)
    g = ImageDraw.Draw(img)
    f = fonts
    g.text((50, 42), fill(res.get('title', ''), values), font=f(48), fill=TEXT)
    rows = {}
    for side, x in (('off', 50), ('on', 838)):
        r = runs[side]
        a = r.rel('arrive')
        m = r.metrics
        head = f'{sheet.feature} {"끔" if side == "off" else "켬"} · {robot_label(r.assignee)}'
        g.ellipse([x, 155, x + 16, 171], fill=SIDE_COLOR[side])
        g.text((x + 28, 145), head, font=f(30), fill=TEXT)
        arrive = f'{a:.1f} s' if a is not None else f'도착 못 함 (녹화 끝 {r.end - r.t0:.0f} s)'
        g.text((x + 28, 195), f'요청 → {sheet.station_label} 도착  {arrive}', font=f(26), fill=SIDE_COLOR[side])
        if 'stops' in m:
            g.text((x + 28, 236), f'가다 멈춤 {m["stops"]}회 · {m.get("stop_s", 0):.1f} s', font=f(24), fill=MUTED)
        rows[side] = a if a is not None else r.end - r.t0
    # 막대: 요청 → 도착 (s)
    g.text((50, 330), res.get('graph_title', f'요청 → {sheet.station_label} 도착 (s, 짧을수록 좋다)'), font=f(24),
           fill=SCENE_TEXT)
    top, h, gap, x0, span = 385, 70, 34, 210, 1250
    vmax = max(rows.values()) or 1.0
    for i, side in enumerate(SIDES):
        y = top + i * (h + gap)
        g.text((50, y + 18), f'{"끔" if side == "off" else "켬"} · {robot_label(runs[side].assignee)}', font=f(26),
               fill=TEXT)
        g.rectangle([x0, y, x0 + span, y + h], fill=PANEL_BG)
        w = span * rows[side] / vmax
        g.rectangle([x0, y, x0 + w, y + h], fill=SIDE_COLOR[side])
        label = f'{rows[side]:.1f} s' + ('' if runs[side].rel('arrive') is not None else ' (도착 못 함)')
        lx = x0 + w + 12 if w < span - 220 else x0 + w - g.textlength(label, font=f(28)) - 14
        g.text((lx, y + 17), label, font=f(28), fill=TEXT if w < span - 220 else BG)
    y = top + 2 * (h + gap) + 30
    for line in res.get('notes', []):
        g.text((50, y), fill(line, values), font=f(20), fill=MUTED)
        y += 32
    _progress(g, f, index, count, 1.0)
    return img
