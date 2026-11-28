"""장면 시작 작업 고르기 - 스테이션 예약 + 부스·손님 근처 제외 + 경로 겹침(마주 옴) 최소 (11/9 녹화 반영).

    python3 search_routes.py act1|act2 --ws /ros2_ws/src --astar ./astar_cli [--iterations N]

- 자리: 같이 잡힐 수 있는 작업끼리 모두 다르다 (서버 스테이션 예약 1 m, demo_scene.station_conflicts)
- 빼는 자리: 부스 b01~b03(11/9 녹화에서 도착 실패로 재큐), 장면이 쓰는 손님 파일(평상시 손님)의 경로 1 m 안 자리,
  막마다 핵심 자리
- 누가 맡나: 1번 작업 픽업이 로봇 줄(x 11.2)보다 북쪽이면 서버 최소거리 탐욕 배정이 낸 순서대로 05, 04, 03, 02, 01에 준다.
  실제로 그 순서인지 직선 거리 탐욕으로 확인한다
- 경로 겹침: A* 경로를 0.5 m 칸으로 찍어 두 경로가 같은 칸을 지나는 길이(close)와 그중 진행 방향이 반대인 길이(head_on).
  11/9 2막 녹화에서 세 대가 윗줄 통로에서 마주 와 서로 막혔다 (head_on 최대 10.8 m)
- 0초: 0초 단계는 after.near로 "먼저 비는 로봇이 하역 2 m 앞"에 맞추므로 실제 속도와 상관없이 그 로봇이 0초 직후 빈다.
  다만 2막은 2인석 배달이 60 s 기다린 뒤라야 하므로 4호 작업이 그보다 먼저 끝나면 안 된다 - 관측 평균(약 0.2 m/s)의
  평균보다 조금 빠른 V_CHECK 0.25 m/s로도 0초(약 66 s) 뒤 10 s 넘게 남는 길이를 요구한다 (2막만 쓴다)
"""
import argparse
import itertools
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detour import costmap, load_map, plan  # noqa: E402

ORDER = ['amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01']
BOOTHS = ['b01', 'b02', 'b03']
# 동쪽 좌석 좁은 곳: 4인석-6인석 사이 1.0 m 세로 통로(x 약 4.74)와 부스 앞 1.09 m 통로·부스 칸.
# 11/9 녹화(2막 4호가 x 4.3, y 0.1에서 멈춤)와 로컬 재현(추적·안전 켬: b03에서 못 빠져나오고 t05로 가다 x 4.49, y 0.25에서
# 멈춤 - 손님 없이도 가구를 '움직이는 장애물'로 추적해 DWA 회피 모드 + 안전 저속)에서 막혔다. 시작 작업 경로가 이 칸을
# 지나지 않게 한다. 6인석 L01~L04는 같은 재현에서 도착했다(L03 57.7 s) - 자리로는 쓰되 세로 통로·부스 칸은 못 지난다
TIGHT_BOXES = [(4.0, 5.6, -1.3, 4.0), (9.3, 12.0, -1.5, 4.6)]
# 2막 켬 녹화 3번(11/11 on_v3·v5a·v5b)에서 남동쪽 구석이 막혔다: t10~L03 사이 1 m 통로(x 3.8~5.6, y -1.2~1.2)에 갇히고
# c01·c02 둘레에 몰렸다. 2막 시작 작업과 2인석 배달 경로는 이 통로를 지나지 않고, 그 둘레 자리를 쓰지 않는다
ACT2_AISLE = (3.8, 5.6, -1.2, 1.2)
ACT2_NO_STATIONS = ['t10', 'L03', 'L04', 'c02']
ACT2_C01_RADIUS = 3.0
C01_PICKUPS = ['stall_1', 'stall_2', 'stall_3', 'stall_4', 'buffet_1']
# 11/12 켬 녹화 on_v6b: 2막 시작 뒤 4호(stall_2 픽업)·5호(stall_3 하역)·3호(stall_4 하역)가 매장 앞에 몰려 약 90 s 서로
# 막혀 2인석 배달이 영상 안에 안 끝났다. 매장(stall_1~4)에는 4호만 간다: 다른 시작 작업은 매장을 쓰지 않고 4호 일터(t03 하역,
# stall_2 픽업) 2.5 m 안에 서지 않는다. 3호·5호 하역은 아래쪽·왼쪽 테이블. 남은 아래쪽·왼쪽 테이블이 t07·t13·t15뿐이고
# t07·t13은 평상시 손님 경로 1 m 안이라, 2막만 이 둘과 buffet_2(1-2 핵심 자리, 2막은 안 쓴다)를 쓴다 - 안 그러면 자리가
# 모자란다 (서로 다른 자리 8곳이 필요한데 남는 곳이 8곳, 그중 픽업이 되는 곳 5곳)
STALLS = ['stall_1', 'stall_2', 'stall_3', 'stall_4']
ACT2_LOW = ['t07', 't13', 't15']
ACT2_GUEST_OK = ['t07', 't13']
ACT2_KEY4, ACT2_KEY4_M = ['stall_2', 't03'], 2.5
ACT2_MIN_M = 25.0
ACT2_SPEEDS = (0.15, 0.2, 0.25)
TIGHT = BOOTHS
V_CHECK, DWELL, T0_ACT2 = 0.25, 2.0, 66.0
CELL = 0.5
NEAR_GUEST_M = 1.0


class Routes:
    def __init__(self, ws, astar):
        self.ws, self.astar = Path(ws), astar
        self.img, self.res, self.origin = load_map(self.ws / 'localization/amr_mapping/maps/restaurant_gt.yaml')
        self.cost, _ = costmap(self.img, self.res)
        # Nav2와 같은 costmap으로 경로를 잡는다 (좁은 칸을 비싸게 하면 실제 Nav2 경로와 달라진다). 좁은 칸을 지나는
        # 경로는 trip()에서 버린다
        self.st = {k: tuple(v[:2]) for k, v in yaml.safe_load(
            open(self.ws / 'fleet/amr_fleet_manager/config/stations_restaurant.yaml'))['stations'].items()}
        self.robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(
            open(self.ws / 'bringup/amr_bringup/config/robots_restaurant.yaml'))['robots'].items()}
        self.cache = {}
        self.boxes = list(TIGHT_BOXES)

    def xy(self, name):
        return self.robots.get(name) or self.st[name]

    def leg(self, a, b):
        if (a, b) not in self.cache:
            st, pts = plan(self.astar, self.cost, self.res, self.origin, self.xy(a), self.xy(b), '/tmp/sr.bin')
            self.cache[(a, b)] = np.array(pts) if st == 0 else None
        return self.cache[(a, b)]

    def trip(self, robot, p, d):
        a, b = self.leg(robot, p), self.leg(p, d)
        if a is None or b is None:
            return None, None
        path = np.vstack([a, b])
        if passes(path, self.boxes):
            return None, None
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        return length, resample(path)

    def penalized(self, a, b, box, cost=250):
        """box 칸을 크게 비싸게(통과 불가 253 바로 밑) 한 costmap으로 a → b A*. 그래도 box를 지나면 돌아갈 길이 없다."""
        h, w = self.cost.shape
        ys, xs = np.mgrid[0:h, 0:w]
        cx, cy = self.origin[0] + (xs + 0.5) * self.res, self.origin[1] + (ys + 0.5) * self.res
        x0, x1, y0, y1 = box
        grid = self.cost.copy()
        inside = (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)
        grid[inside] = np.maximum(grid[inside], cost)
        st, pts = plan(self.astar, grid, self.res, self.origin, self.xy(a), self.xy(b), '/tmp/sr_pen.bin')
        return np.array(pts) if st == 0 else None


def passes(path, boxes):
    return any(np.any((path[:, 0] >= x0) & (path[:, 0] <= x1) & (path[:, 1] >= y0) & (path[:, 1] <= y1))
               for x0, x1, y0, y1 in boxes)


def path_len(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


def cells(path):
    """경로 → {칸: [방향 단위벡터, ...]}. 같은 칸을 갔다가 돌아오면(픽업까지 갔다가 되짚어 옴) 방향이 둘 남는다."""
    out = {}
    d = np.gradient(path, axis=0)
    for (x, y), v in zip(path, d):
        k = (int(math.floor(x / CELL)), int(math.floor(y / CELL)))
        n = float(np.hypot(*v))
        if n > 0:
            u = v / n
            dirs = out.setdefault(k, [])
            if not any(float(np.dot(u, w)) > 0.9 for w in dirs):
                dirs.append(u)
    return out


def overlap(a, b):
    """a 경로 칸 중 b 경로 칸이 둘레 3 x 3(약 0.75 m) 안에 있는 길이(close)와 그중 방향이 반대인 길이(head_on).
    route_overlap.py(점 사이 0.8 m, 방향 반대)와 같은 뜻의 칸 근사다. 같은 칸을 오가는 경로는 방향을 모두 본다."""
    close = head = 0
    for (i, j), vs in a.items():
        near = [w for di in (-1, 0, 1) for dj in (-1, 0, 1) for w in b.get((i + di, j + dj), [])]
        if near:
            close += 1
            if any(float(np.dot(v, w)) < 0 for v in vs for w in near):
                head += 1
    return close * CELL, head * CELL


def resample(path, step=0.25):
    """경로를 길이 step 간격 점으로 다시 찍는다 (같은 속도면 i번째 점 = 같은 시각의 자리)."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.arange(0.0, s[-1], step)
    return np.stack([np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])], axis=1)


def meet(a, b, near_m=1.0):
    """두 로봇이 같은 속도로 동시에 출발할 때 near_m 안에서 마주 오는(진행 방향이 반대) 구간 길이 (m).
    11/9 2막 녹화처럼 한 통로에서 엇갈려 서로 막히는 것을 본다. 같은 통로를 다른 때 지나는 것은 세지 않는다."""
    n = min(len(a), len(b)) - 1
    if n <= 0:
        return 0.0
    da, db = np.diff(a[:n + 1], axis=0), np.diff(b[:n + 1], axis=0)
    close = np.linalg.norm(a[:n] - b[:n], axis=1) <= near_m
    opposite = np.einsum('ij,ij->i', da, db) < 0
    return float(np.sum(close & opposite)) * 0.25


def traj(legs, t0, v, dwell=DWELL):
    """구간들을 속도 v로 차례로 가고 끝마다 dwell 쉰다 → 0.5 s 간격 (시각, 자리)."""
    T, P = [], []
    t = t0
    for leg in legs:
        for q in resample(leg, v * 0.5):
            T.append(t)
            P.append(q)
            t += 0.5
        for _ in range(int(dwell / 0.5)):
            T.append(t)
            P.append(leg[-1])
            t += 0.5
    return np.array(T), np.array(P)


def block(a, b, near_m=1.5, after_s=60.0):
    """a 궤적이 b 궤적과 같은 때 near_m 안에 있고 둘 중 하나가 서 있거나 같은 쪽으로 가지 않는 시간 (s).
    같은 쪽으로 뒤따라가는 것은 세지 않는다. 처음 after_s는 뺀다 (동쪽 벽 출발 줄에서 나란히 나가는 것은 장면마다 같다)."""
    (T, P), (T2, P2) = a, b
    V, V2 = np.gradient(P, axis=0), np.gradient(P2, axis=0)
    k = np.clip(np.searchsorted(T2, T), 0, len(T2) - 1)
    d = np.linalg.norm(P - P2[k], axis=1)
    na, nb = np.linalg.norm(V, axis=1), np.linalg.norm(V2[k], axis=1)
    cos = np.einsum('ij,ij->i', V, V2[k]) / np.maximum(na * nb, 1e-9)
    hit = (T > after_s) & (T <= T2[-1]) & (d <= near_m) & ((na < 1e-6) | (nb < 1e-6) | (cos < 0.5))
    return float(np.sum(hit)) * 0.5


def act2(R, normal, c01_pickup='stall_2'):
    """2막 시작 작업: 4호 t09 → t03 고정(0초 직후 가장 먼저 빈다), 2인석 배달 stall_2 → c01. 나머지 4대는
    - 매장 안 감, 4호 일터 2.5 m 안에 안 섬, 3호·5호 하역은 아래쪽·왼쪽 테이블, A* 25 m 이상
    - 남동쪽 구석(ACT2_*), 좁은 칸, 부스, 핵심 자리를 안 씀, 자리가 모두 다름, 낸 순서대로 05, 04, 03, 02, 01에 배정
    순위: (1) 픽업 → 하역 경로가 4호 일하는 길(t09 → t03 → stall_2 → c01)과 겹치는 길이 합,
    (2) 4호와 막히는 시간 합(같은 때 1.5 m 안에서 서 있거나 마주침, 0.15·0.2·0.25 m/s 중 속도 차 0.05 이하 조합의 최대),
    (3) 4호 말고 로봇끼리 막히는 시간, (4) 마주침"""
    R.boxes = list(TIGHT_BOXES) + [ACT2_AISLE]
    near_c01 = [n for n in R.st if math.dist(R.st[n], R.st['c01']) <= ACT2_C01_RADIUS]
    forbid = set(['t11', 'c01', 't01', 't06', 't08', 't12', 't14', 'return_1', 'return_2', 't09', 't03', c01_pickup]
                 + TIGHT + ACT2_NO_STATIONS + near_c01 + STALLS + [n for n in normal if n not in ACT2_GUEST_OK])
    names = [n for n in R.st if n.startswith(('buffet_', 't', 'L')) and n not in forbid
             and all(math.dist(R.st[n], R.st[f]) > 1.0 for f in forbid if f not in STALLS)
             and min(math.dist(R.st[n], R.st[z]) for z in ACT2_KEY4) > ACT2_KEY4_M]
    a4 = [R.leg('amr_04', 't09'), R.leg('t09', 't03'), R.leg('t03', c01_pickup), R.leg(c01_pickup, 'c01')]
    work4 = cells(resample(np.vstack(a4[1:])))
    t4 = {v: traj(a4, 0.5, v) for v in ACT2_SPEEDS}
    start = {r: 0.5 * i for i, r in enumerate(ORDER)}
    north = max(y for _, y in R.robots.values())
    cand = {}
    for r in ORDER:
        if r == 'amr_04':
            continue
        out = []
        for p in names:
            if R.st[p][1] < north:
                continue
            for d in (ACT2_LOW if r in ('amr_05', 'amr_03') else names):
                if d not in names or p == d or math.dist(R.st[p], R.st[d]) <= 1.0:
                    continue
                length, rs = R.trip(r, p, d)
                if length is None or length < ACT2_MIN_M:
                    continue
                legs = [R.leg(r, p), R.leg(p, d)]
                b4 = max(block(traj(legs, start[r], v), t4[v4]) for v in ACT2_SPEEDS for v4 in ACT2_SPEEDS
                         if abs(v - v4) < 0.06)
                out.append({'p': p, 'd': d, 'L': round(length, 1), 'rs': rs, 'legs': legs, 'b4': b4,
                            'work4': overlap(cells(resample(legs[1])), work4)[0]})
        cand[r] = out
    others = [r for r in ORDER if r != 'amr_04']
    best = None
    for combo in itertools.product(*(cand[r] for r in others)):
        ch = dict(zip(others, combo))
        used = [n for c in combo for n in (c['p'], c['d'])]
        if any(math.dist(R.st[a], R.st[b]) <= 1.0 for a, b in itertools.combinations(used, 2)):
            continue
        pk = {r: ch[r]['p'] for r in others}
        pk['amr_04'] = 't09'
        if greedy(R.robots, R.st, [pk[r] for r in ORDER]) != ORDER:
            continue
        rs = {r: ch[r]['rs'] for r in others}
        rs['amr_04'] = resample(np.vstack(a4[:2]))
        meets = max(meet(rs[a], rs[b]) for a, b in itertools.combinations(ORDER, 2))
        bo = max(sum(block(traj(ch[a]['legs'], start[a], v), traj(ch[b]['legs'], start[b], v))
                     for a, b in itertools.combinations(others, 2)) for v in ACT2_SPEEDS)
        score = (sum(c['work4'] for c in combo), sum(c['b4'] for c in combo), bo, meets)
        if best is None or score < best[0]:
            best = (score, ch)
    if best is None:
        return {'names': names, 'best': None}
    rows = []
    for r in ORDER:
        if r == 'amr_04':
            p, d, length, real = 't09', 't03', round(path_len(np.vstack(a4[:2])), 1), np.vstack(a4[:2])
        else:
            c = best[1][r]
            p, d, length, real = c['p'], c['d'], c['L'], np.vstack(c['legs'])
        pen = np.vstack([R.penalized(r, p, ACT2_AISLE), R.penalized(p, d, ACT2_AISLE)])
        rows.append({'robot': r, 'pickup': p, 'dropoff': d, 'astar_m': length,
                     'work4_overlap_m': best[1][r]['work4'] if r != 'amr_04' else None,
                     'block4_s': best[1][r]['b4'] if r != 'amr_04' else None,
                     'real_passes_aisle': passes(real, [ACT2_AISLE]),
                     'penalized_passes_aisle': passes(pen, [ACT2_AISLE]),
                     'c01_m': round(math.dist(R.st[d], R.st['c01']), 2)})
    keys = ['work4_overlap_m', 'block4_s_sum', 'block_others_s', 'meet_max_m']
    return {'names': names, 'score': dict(zip(keys, best[0])), 'detail': rows}


def guest_near(st, guest_file):
    def seg(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = 0 if dx == dy == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        return math.dist(p, (a[0] + t * dx, a[1] + t * dy))
    out = set()
    for o in yaml.safe_load(open(guest_file, encoding='utf-8'))['obstacles']:
        pts = [(w['x'], w['y']) for w in o.get('waypoints', [])]
        for n, p in st.items():
            if o['pattern'] == 'random':
                xs, ys = [q[0] for q in pts], [q[1] for q in pts]
                d = math.hypot(max(min(xs) - p[0], 0, p[0] - max(xs)), max(min(ys) - p[1], 0, p[1] - max(ys)))
            else:
                d = min(seg(p, a, b) for a, b in zip(pts, pts[1:]))
            if d <= NEAR_GUEST_M:
                out.add(n)
    return out


def greedy(robots, st, pickups):
    idle, out = dict(robots), []
    for p in pickups:
        r = min(idle, key=lambda k: (math.dist(idle[k], st[p]), k))
        out.append(r)
        idle.pop(r)
    return out


def pool(R, forbid):
    names = [n for n in R.st if n.startswith(('stall_', 'buffet_', 't', 'L', 'c0'))]
    return [n for n in names if all(math.dist(R.st[n], R.st[f]) > 1.0 for f in forbid) and n not in forbid]


def search(R, key_robot, forbid, key_min, key_max, gap, iterations, seed=1):
    names = pool(R, forbid)
    north = max(y for _, y in R.robots.values())
    picks = [n for n in names if R.st[n][1] >= north]
    trips = {}
    for r in ORDER:
        for p in picks:
            for d in names:
                if p != d:
                    length, cs = R.trip(r, p, d)
                    if length is not None:
                        trips[(r, p, d)] = (length, cs)
    rnd = random.Random(seed)
    keys = [(p, d) for (r, p, d), (L, _) in trips.items() if r == key_robot and key_min <= L <= key_max]
    best = None
    if not keys:
        return None
    for _ in range(iterations):
        p, d = rnd.choice(keys)
        used = {p, d}
        chosen = {key_robot: (p, d)}
        lk = trips[(key_robot, p, d)][0]
        ok = True
        for r in rnd.sample([x for x in ORDER if x != key_robot], 4):
            c = [(pp, dd) for (rr, pp, dd), (L, _) in trips.items()
                 if rr == r and L >= lk + gap and pp not in used and dd not in used]
            if not c:
                ok = False
                break
            chosen[r] = rnd.choice(c)
            used |= set(chosen[r])
        if not ok or greedy(R.robots, R.st, [chosen[r][0] for r in ORDER]) != ORDER:
            continue
        rs = {r: trips[(r, *chosen[r])][1] for r in ORDER}
        meets = [meet(rs[a], rs[b]) for a, b in itertools.combinations(ORDER, 2)]
        heads = [overlap(cells(rs[a]), cells(rs[b]))[1] for a, b in itertools.combinations(ORDER, 2)]
        lengths = {r: trips[(r, *chosen[r])][0] for r in ORDER}
        score = (max(meets), sum(meets), max(heads), sum(lengths.values()))
        if best is None or score < best[0]:
            best = (score, {r: (chosen[r][0], chosen[r][1], round(lengths[r], 1)) for r in ORDER},
                    {'meet_max_m': max(meets), 'meet_sum_m': sum(meets), 'head_on_static_max_m': max(heads),
                     'gap_m': round(min(v for r, v in lengths.items() if r != key_robot) - lengths[key_robot], 1)})
    return best


def s12(R):
    """1-2 시작 작업 2건: 1호보다 동쪽 로봇이 가까운 픽업, 좁은 동쪽 좌석을 안 지나고, 1-2 화면(서쪽, x < -6)과 핵심 자리
    (buffet_2, t11)에서 떨어진 자리. 1-2 손님 파일(peak_scene12) 경로 1 m 안 자리는 뺀다."""
    peak = guest_near(R.st, R.ws / 'simulation/amr_gazebo/config/dynamic_obstacles_restaurant_peak_scene12.yaml')
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(
        open(R.ws / 'bringup/amr_bringup/config/robots_restaurant_s12.yaml'))['robots'].items()}
    R.robots = robots
    forbid = ['buffet_2', 't11'] + TIGHT + sorted(peak)
    names = [n for n in pool(R, forbid) if R.st[n][0] > -6.0]
    best = None
    for (p1, d1), (p2, d2) in itertools.permutations(
            [(p, d) for p in names for d in names if p != d], 2):
        if len({p1, d1, p2, d2}) < 4:
            continue
        owners = greedy(robots, R.st, [p1, p2])
        if 'amr_01' in owners:
            continue
        t1, t2 = R.trip(owners[0], p1, d1), R.trip(owners[1], p2, d2)
        if t1[0] is None or t2[0] is None:
            continue
        head = meet(t1[1], t2[1])
        score = (head, overlap(cells(t1[1]), cells(t2[1]))[1], -min(t1[0], t2[0]))
        if best is None or score < best[0]:
            best = (score, [(owners[0], p1, d1, round(t1[0], 1)), (owners[1], p2, d2, round(t2[0], 1))])
    return {'peak_scene12_near': sorted(peak), 'best': best}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('act', choices=['act1', 'act2', 's12'])
    ap.add_argument('--ws', required=True)
    ap.add_argument('--astar', required=True)
    ap.add_argument('--iterations', type=int, default=20000)
    a = ap.parse_args()
    R = Routes(a.ws, a.astar)
    normal = guest_near(R.st, R.ws / 'simulation/amr_gazebo/config/dynamic_obstacles_restaurant.yaml')
    if a.act == 's12':
        print(json.dumps(s12(R), indent=1))
        return
    if a.act == 'act1':
        queue = ['stall_3', 't09', 'stall_4', 't15', 'stall_1', 't04', 'stall_2', 't02']
        forbid = ['t03', 'return_1'] + queue + TIGHT + sorted(normal)
        best = search(R, 'amr_02', forbid, 8.0, 22.0, 10.0, a.iterations)
    else:
        # 2인석 배달(→ c01) 픽업: 경로가 남동쪽 1 m 통로를 안 지나는 매장·배식대 중 가장 가까운 곳
        R.boxes = list(TIGHT_BOXES) + [ACT2_AISLE]
        c01 = []
        for p in C01_PICKUPS:
            real = R.leg(p, 'c01')
            pen = R.penalized(p, 'c01', ACT2_AISLE)
            c01.append({'pickup': p, 'real_m': round(path_len(real), 1), 'real_passes': passes(real, [ACT2_AISLE]),
                        'penalized_m': round(path_len(pen), 1), 'penalized_passes': passes(pen, [ACT2_AISLE])})
        ok = [r for r in c01 if not r['real_passes']]
        pickup = min(ok, key=lambda r: r['real_m'])['pickup'] if ok else None
        out = act2(R, normal, pickup)
        print(json.dumps({'c01_pickups': c01, 'c01_pickup': pickup, **out}, indent=1))
        return
    print(json.dumps({'normal_guest_near': sorted(normal), 'best': best}, indent=1))


if __name__ == '__main__':
    main()
