"""식당 시연 끔·켬 비교 영상의 화면 배치와 강조 표시. ROS·ffmpeg 없이 돈다 (compose.py가 영상·bag을 읽고 쓴다).

화면 배치는 「식당 시연 영상 편집 가이드」(시나리오 3 최종본 기준, 1600×900 픽셀, 원점 왼쪽 위)를 따른다:
시나리오명 (40,22) 26 px, 컷 제목 (40,83) 41 px, 끔·켬 패널 (32,189)·(808,189) 760×494, 원본 영상 (36,243)·(812,243)
752×424, 설명 자막 (40,745) 28 px, 진행선 (40,845)~(1560,845). 끔 #FF9292, 켬 #83B4FF.

강조 표시(layers)는 녹화 기록으로 그린다. 로봇 위치는 /fleet/robot_states(map = world), 혼잡 구역은 /fleet/congestion,
주문 시각은 /demo/t0, 배정은 /fleet/task_table의 assignee_id, 도착은 /fleet/task_status 단계 보고다 (timeline.json).
월드 좌표를 카메라 화면으로 옮기는 식은 demo_cameras_restaurant.yaml의 카메라 자세·화각(핀홀)이다:
Gazebo 카메라는 자기 +x를 보고 화면 오른쪽이 −y, 위가 +z다.

    zones       혼잡도 0.6 이상 구역 반투명 청록 (빨강 보색: 끔 쪽 빨간 동그라미·홀 바닥 붉은 톤과 섞이지 않게, 혼잡도에 따라 진하기)
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
ZONE_COLOR = (0, 214, 230)          # 청록 (빨강 보색). 식당 바닥이 붉은 나무라 빨강은 묻힌다
ZONE_CHIP = (0, 84, 96)
ZONE_MIN = 0.6                      # 이 혼잡도 이상인 구역만 칠한다 (낮은 구역까지 칠하면 홀 전체가 덮인다)
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
    str(Path.home() / 'AppData/Local/Microsoft/Windows/Fonts/Pretendard-Medium.otf'),   # Windows: 얇은 글꼴 먼저
    'C:/Windows/Fonts/malgun.ttf',
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
)

# 표시 높이 (m): 로봇 동그라미는 몸통 가운데쯤, 주문 고리는 배식대 상판 높이
ROBOT_Z = 1.0
STATION_Z = 0.9
RING_M = 0.45
REASON_SS = 3       # 배정 이유 층을 몇 배로 그려 줄일지 (사선이 계단지지 않게)


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
LAYERS = {'zones', 'candidates', 'request', 'reason', 'assign', 'hero', 'dim', 'card', 'clock', 'arrived'}
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
    zones: tuple = ()     # 칠할 혼잡 구역 id (비우면 전부). 장면과 상관없는 구역이 잠깐 0.6을 넘어 깜박이지 않게
    congestion_weight: float = 3.0   # 켬 배정 비용의 혼잡 가중치 (하네스 params congestion_weight와 같게)

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
                   int(d.get('fps', 20)), int(d.get('crf', 17)), tuple(d.get('zones', ())),
                   float(d.get('congestion_weight', 3.0)))


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


def text_values(runs: dict[str, Run], stations: dict, candidates: Sequence[str],
                weight: Optional[float] = None) -> dict:
    """제목·자막의 {이름} 자리: robot_off·robot_on, arrive_off·arrive_on(s), stops_off·stops_on, dist_<로봇>,
    weight를 주면 켬 실행 0초 배정 비용 cost_<로봇>·혼잡 몫 jam_<로봇>."""
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
    if weight is not None:
        for rid, c in reason_costs(runs['on'], candidates, stations.get(runs['on'].pickup), weight).items():
            v[f'cost_{rid}'] = f'{c["cost"]:.1f}'
            v[f'jam_{rid}'] = f'{c["congestion"]:.1f}'
    return v


def clip_segment(a, b, area) -> Optional[tuple]:
    """선분 a→b 중 사각형 area [x_min, x_max, y_min, y_max] 안 구간 (t0, t1), 없으면 None (Liang-Barsky,
    amr_fleet_core cost.segment_length_in_rect와 같은 식)."""
    x0, x1, y0, y1 = area
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p == 0.0:
            if q < 0.0:
                return None
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return None
    return (t0, t1) if t1 > t0 else None


def assign_cost(robot, station, zones, weight: float) -> dict:
    """배정 비용 (fleet cost_fn): 직선 거리, 켬 = 직선 + weight × Σ(혼잡도 × 로봇 → 픽업 선분이 구역 안을 지나는 길이).
    반환 {'dist', 'congestion', 'cost', 'spans': [(zone_id, value, t0, t1)]}."""
    dist = math.dist(robot, station)
    spans, extra = [], 0.0
    for zone_id, area, value in zones:
        c = clip_segment(robot, station, area)
        if c and value > 0.0:
            spans.append((zone_id, value, c[0], c[1]))
            extra += value * (c[1] - c[0]) * dist
    return {'dist': dist, 'congestion': weight * extra, 'cost': dist + weight * extra, 'spans': spans}


def reason_costs(run: 'Run', candidates, station, weight: float) -> dict:
    """요청 순간(0초) 후보마다 배정 비용. 끔은 거리만 비교한다."""
    out = {}
    zones = run.zones_at(run.t0)
    for rid in candidates:
        p = run.pose(rid, run.t0)
        if p is not None and station is not None:
            out[rid] = assign_cost(p, station, zones, weight)
    return out


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
    g.text((x + 8, y + h / 2), text, font=font, fill=text_color + (255,), anchor='lm')


def _round_line(g: ImageDraw.ImageDraw, a, b, color, width):
    """끝이 둥근 직선."""
    g.line([*a, *b], fill=color, width=int(width))
    h = width / 2
    for x, y in (a, b):
        g.ellipse([x - h, y - h, x + h, y + h], fill=color)


def _merge_spans(spans):
    """겹치거나 맞닿은 (t0, t1) 구간을 합친다 (구역 경계에서 선이 끊겨 보이지 않게)."""
    out = []
    for t0, t1 in sorted(spans):
        if out and t0 <= out[-1][1] + 1e-3:
            out[-1] = (out[-1][0], max(out[-1][1], t1))
        else:
            out.append((t0, t1))
    return out


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
                # 컷시트가 정한 구역은 값이 내려가도 계속 그린다 (0.6 밑으로 잠깐 내려가 박스가 깜박이지 않게).
                # 정한 구역이 없으면 ZONE_MIN 이상만
                if (zone_id not in self.sheet.zones) if self.sheet.zones else value < ZONE_MIN:
                    continue
                x0, x1, y0, y1 = area
                pts = [self._pt(cam, x, y, 0.0) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
                if None in pts:
                    continue
                a = int(25 + 45 * min(value, 1.0))
                poly = [(p[0], p[1]) for p in pts]
                g.polygon(poly, fill=ZONE_COLOR + (a,))
                g.line(poly + poly[:1], fill=ZONE_COLOR + (int(120 + 135 * min(value, 1.0)),), width=3)
                lx, ly = min(p[0] for p in pts), max(p[1] for p in pts)
                label = f'혼잡 {value:.1f}'
                g.rounded_rectangle([lx + 4, ly - 26, lx + 14 + g.textlength(label, font=f(15)), ly - 4], 4,
                                    fill=ZONE_CHIP + (225,))
                g.text((lx + 9, ly - 15), label, font=f(15), fill=(230, 252, 255, 255), anchor='lm')
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
        if 'reason' in layers and station:
            ov = Image.alpha_composite(ov, self._reason(cam, run, station))
            g = ImageDraw.Draw(ov)
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
            _badge(g, (u - 110, v + r + 8), f'도착 · 요청 후 {run.arrive_t - run.t0:.1f} s', color, f(26))
        if 'clock' in layers:
            txt = f'요청 후 {rel:.1f} s' if rel >= 0 else f'요청 {-rel:.1f} s 전'
            g.rounded_rectangle([8, 8, 20 + g.textlength(txt, font=f(18)), 40], 6, fill=(13, 17, 23, 200))
            g.text((14, 24), txt, font=f(18), fill=TEXT + (255,), anchor='lm')
        if 'card' in layers and rel >= 0:
            who = f'▶ {robot_label(run.assignee)}' if assigned else '배정 대기'
            lines = (f'요청 후 {rel:.1f} s · 새 작업', f'{self.sheet.request_label.split("·")[-1].strip()}  {who}')
            w = max(g.textlength(lines[0], font=f(15)), g.textlength(lines[1], font=f(19))) + 28
            # 오른쪽 아래 (탑뷰 위쪽은 배식대·매장 앞이라 비워 둔다)
            x, y = VIDEO_SIZE[0] - w - 10, VIDEO_SIZE[1] - 74
            g.rounded_rectangle([x, y, VIDEO_SIZE[0] - 10, y + 64], 8, fill=PANEL_BG + (235,),
                                outline=PANEL_BORDER + (255,), width=2)
            g.text((x + 14, y + 18), lines[0], font=f(15), fill=MUTED + (255,), anchor='lm')
            g.text((x + 14, y + 44), lines[1], font=f(19), fill=TEXT + (255,), anchor='lm')
        return Image.alpha_composite(base, ov).convert('RGB')

    def _reason(self, cam: Camera, run: Run, station) -> Image.Image:
        """배정 이유 (투명 층): 후보 → 주문 자리 직선과 비용. 끔은 거리만 크게, 켬은 혼잡 구역 안 구간을 칠하고 비용.
        선이 계단지지 않게 REASON_SS배로 그려 줄인다."""
        S = REASON_SS
        layer = Image.new('RGBA', (VIDEO_SIZE[0] * S, VIDEO_SIZE[1] * S), (0, 0, 0, 0))
        g = ImageDraw.Draw(layer)
        f = lambda size: self.fonts(size * S)  # noqa: E731
        on = run.side == 'on'
        costs = reason_costs(run, self.sheet.candidates, station, self.sheet.congestion_weight)
        sp = self._pt(cam, *station, STATION_Z)
        if not costs or not sp:
            return layer.resize(VIDEO_SIZE, Image.LANCZOS)
        key = 'cost' if on else 'dist'
        best = min(costs, key=lambda rid: costs[rid][key])
        b = (sp[0] * S, sp[1] * S)
        labels = []
        for rid, c in costs.items():
            rp = self._pt(cam, *run.pose(rid, run.t0), ROBOT_Z)
            if not rp:
                continue
            a = (rp[0] * S, rp[1] * S)
            win = rid == best
            line = SIDE_COLOR[run.side] if win else (210, 214, 220)
            # 선 굵기는 한 가지로 (혼잡 구간만 색을 바꾼다). 굵기가 바뀌거나 구간 끝이 각지면 울퉁불퉁해 보인다
            seg = lambda t0, t1, col, w: _round_line(g, (a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0),  # noqa: E731
                                                     (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1), col, w)
            seg(0.0, 1.0, (13, 17, 23, 210), 11 * S)
            # 켬: 밑선은 흰색, 혼잡 구역 안 구간만 청록 (어느 로봇을 골랐는지는 동그라미·비용 글자 색으로)
            seg(0.0, 1.0, ((255, 255, 255) if on else line) + (255,), 6 * S)
            if on:
                for t0, t1 in _merge_spans([(t0, t1) for _, value, t0, t1 in c['spans'] if value >= 0.3]):
                    seg(t0, t1, ZONE_COLOR + (255,), 6 * S)
            r = rp[2] * S
            for rr, col, w in ((r + 2 * S, (13, 17, 23, 220), (9 if win else 7) * S), (r, line + (255,), (5 if win else 3) * S)):
                g.ellipse([a[0] - rr, a[1] - rr, a[0] + rr, a[1] + rr], outline=col, width=w)
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            labels.append((rid, c, win, line, mid, abs(b[0] - a[0]) > abs(b[1] - a[1])))
        for rid, c, win, line, mid, flat in labels:
            # 큰 줄: 끔은 거리, 켬은 비용. 켬 작은 줄: 거리 + 혼잡
            big = f'{robot_label(rid)}  {c["dist"]:.1f} m' if not on else f'{robot_label(rid)}  비용 {c["cost"]:.1f}'
            if win:
                big += '  ← 선택'
            small = f'거리 {c["dist"]:.1f} + 혼잡 {c["congestion"]:.1f}' if on else ''
            fb, fs = f(30 if win else 26), f(18)
            w = max(g.textlength(big, font=fb), g.textlength(small, font=fs) if small else 0) + 24 * S
            h = (fb.size + 16 * S) + ((fs.size + 8 * S) if small else 0)
            # 가로로 누운 선(매장 앞 통로)은 선 아래로, 세로 선(서쪽 통로)은 선 오른쪽에
            x = mid[0] - w / 2 if flat else mid[0] + 18 * S
            y = mid[1] + 24 * S if flat else mid[1] - h / 2
            x = min(max(x, 4 * S), layer.width - w - 4 * S)
            y = min(max(y, 4 * S), layer.height - h - 4 * S)
            g.rounded_rectangle([x, y, x + w, y + h], 8 * S, fill=line + (240,))
            row1 = fb.size + 16 * S
            g.text((x + 12 * S, y + row1 / 2), big, font=fb, fill=BG + (255,), anchor='lm')
            if small:
                g.text((x + 12 * S, y + row1 + (fs.size + 8 * S) / 2 - 4 * S), small, font=fs, fill=(40, 46, 56, 255),
                       anchor='lm')
        return layer.resize(VIDEO_SIZE, Image.LANCZOS)

    # ---- 한 장 전체
    def frame(self, cut: Cut, index: int, count: int, progress: float, panels: dict[str, Image.Image],
              values: dict) -> Image.Image:
        """compose 한 장: 머리글·설명 한 줄·패널 둘·진행선. panels = {'off': 752×424, 'on': 752×424}."""
        img = Image.new('RGB', CANVAS, BG)
        g = ImageDraw.Draw(img)
        f = self.fonts
        g.text((40, 38), self.sheet.scene, font=f(26), fill=SCENE_TEXT, anchor='lm')
        # 위 한 줄: 그 컷의 설명 (caption, 없으면 title). 길면 한 줄에 들어가게 글자를 줄인다
        line = fill(cut.caption or cut.title, values)
        size = 38
        while size > 24 and g.textlength(line, font=f(size)) > CANVAS[0] - 80:
            size -= 1
        g.text((40, 112), line, font=f(size), fill=TEXT, anchor='lm')
        for side in SIDES:
            x, y = PANEL_XY[side]
            g.rounded_rectangle([x, y, x + PANEL_SIZE[0], y + PANEL_SIZE[1]], 9, fill=PANEL_BG, outline=PANEL_BORDER,
                                width=2)
            g.ellipse([x + 18, y + 20, x + 32, y + 34], fill=SIDE_COLOR[side])
            g.text((x + 42, y + 27), f'{self.sheet.feature} {"끔" if side == "off" else "켬"}', font=f(23), fill=TEXT,
                   anchor='lm')
            img.paste(panels[side], (x + VIDEO_OFFSET[0], y + VIDEO_OFFSET[1]))
        _progress(g, f, index, count, progress)
        return img


def _progress(g, f, index, count, progress):
    g.line([40, 845, 1560, 845], fill=BAR_BG, width=2)
    g.line([40, 845, 40 + 1520 * min(max(progress, 0.0), 1.0), 845], fill=BAR_FILL, width=3)
    txt = f'{index} / {count}'
    g.text((1560 - g.textlength(txt, font=f(18)), 856), txt, font=f(18), fill=MUTED)


def result_frame(sheet: CutSheet, runs: dict[str, Run], fonts: Fonts, index: int, count: int,
                 values: dict) -> Image.Image:
    """마지막 결과 장표: 대표 문장, 요청 → 도착 타임라인(먼저 도착한 차이), 시간 기준·지표 정의."""
    res = sheet.result
    img = Image.new('RGB', CANVAS, BG)
    g = ImageDraw.Draw(img)
    f = fonts
    g.text((50, 42), fill(res.get('title', ''), values), font=f(48), fill=TEXT)
    rows = {}
    for side in SIDES:
        r = runs[side]
        a = r.rel('arrive')
        rows[side] = a if a is not None else r.end - r.t0
    # 도착 타임라인: 요청(0초)부터 시간축, 줄마다 도착 시각에 표시. 막대는 길수록 좋아 보여서 쓰지 않는다
    g.text((50, 160), res.get('graph_title', f'요청 → {sheet.station_label} 도착 시간'), font=f(28), fill=SCENE_TEXT)
    x0, x1, top, lane = 250, 1480, 280, 120
    vmax = math.ceil((max(rows.values()) + 5) / 20.0) * 20
    sx = lambda v: x0 + (x1 - x0) * v / vmax  # noqa: E731
    axis_y = top + 2 * lane - 20
    for v in range(0, int(vmax) + 1, 20):
        g.line([sx(v), top - 30, sx(v), axis_y], fill=(40, 47, 57), width=1)
        lab = f'{v} s'
        g.text((sx(v) - g.textlength(lab, font=f(18)) / 2, axis_y + 8), lab, font=f(18), fill=MUTED)
    for i, side in enumerate(SIDES):
        y = top + i * lane + 20
        arrived = runs[side].rel('arrive') is not None
        g.text((50, y - 18), f'{"끔" if side == "off" else "켬"} · {robot_label(runs[side].assignee)}', font=f(30),
               fill=TEXT)
        g.line([x0, y, x1, y], fill=PANEL_BG, width=6)
        xa = sx(rows[side])
        g.line([x0, y, xa, y], fill=SIDE_COLOR[side], width=4)
        g.ellipse([x0 - 7, y - 7, x0 + 7, y + 7], fill=MUTED)
        r = 17
        g.ellipse([xa - r, y - r, xa + r, y + r], fill=SIDE_COLOR[side], outline=BG, width=3)
        label = f'{rows[side]:.1f} s 도착' if arrived else f'{rows[side]:.0f} s 까지 도착 못 함'
        lw = g.textlength(label, font=f(28))
        g.text((min(xa - lw / 2, x1 + 60 - lw), y - 58), label, font=f(28), fill=SIDE_COLOR[side])
    # 먼저 도착한 쪽과의 차이
    ta, tb = rows['on'], rows['off']
    if runs['on'].rel('arrive') is not None and ta < tb:
        ya, yb = top + 20, top + lane + 20
        for t, y_from in ((ta, yb + 22), (tb, ya + 22)):          # 도착 점 밑에서 차이 괄호까지
            for yy in range(int(y_from), axis_y + 40, 12):
                g.line([sx(t), yy, sx(t), yy + 6], fill=SIDE_COLOR['on'], width=2)
        yb2 = axis_y + 50
        g.line([sx(ta), yb2, sx(tb), yb2], fill=SIDE_COLOR['on'], width=3)
        for t in (ta, tb):
            g.line([sx(t), yb2 - 10, sx(t), yb2 + 10], fill=SIDE_COLOR['on'], width=3)
        msg = f'켬 {robot_label(runs["on"].assignee)}가 {tb - ta:.1f}초 먼저 도착'
        mx = (sx(ta) + sx(tb)) / 2 - g.textlength(msg, font=f(30)) / 2
        mx = min(mx, x1 - g.textlength(msg, font=f(30)))
        g.text((mx, yb2 + 16), msg, font=f(30), fill=SIDE_COLOR['on'])
    y = axis_y + 130
    for line in res.get('notes', []):
        g.text((50, y), fill(line, values), font=f(20), fill=MUTED)
        y += 32
    _progress(g, f, index, count, 1.0)
    return img
