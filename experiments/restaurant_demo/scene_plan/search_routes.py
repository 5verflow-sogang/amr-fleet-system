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
  1.5배인 V_CHECK 0.3 m/s로도 0초(약 66 s) 뒤 10 s 넘게 남는 길이를 요구한다
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
TIGHT = BOOTHS
V_CHECK, DWELL, T0_ACT2 = 0.3, 2.0, 66.0
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
        for x0, x1, y0, y1 in TIGHT_BOXES:
            if np.any((path[:, 0] >= x0) & (path[:, 0] <= x1) & (path[:, 1] >= y0) & (path[:, 1] <= y1)):
                return None, None
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        return length, resample(path)


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
        forbid = ['buffet_2', 't11', 'c01', 't01', 't06', 't08', 't12', 't14', 'return_1', 'return_2', 'stall_3'] \
            + TIGHT + sorted(normal)
        key_min = (T0_ACT2 + 10 - 2 * DWELL) * V_CHECK
        best = search(R, 'amr_04', forbid, key_min, key_min + 12, 10.0, a.iterations)
    print(json.dumps({'normal_guest_near': sorted(normal), 'best': best}, indent=1))


if __name__ == '__main__':
    main()
