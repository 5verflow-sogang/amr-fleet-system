#!/usr/bin/env python3
"""전역 planner 비교 (navigation.md 3-6절). planner_compare.launch.py를 띄운 뒤 실행한다.

    python3 experiments/planner_compare/compare.py --out /tmp/planner_compare

nav_kpi와 같은 출발·목표 50쌍(experiments/nav_kpi/waypoints.yaml, 쌍 i = 점 i → 점 i+1, world 좌표)을 planner마다
ComputePathToPose(use_start=True)로 계획한다. 쌍마다 --repeats번 불러 계획 시간을 모으고, 경로 지표는 첫 성공 경로로 잰다.
- 경로 길이: 경로 점을 이은 길이 (m)
- 계획 시간: Nav2가 돌려준 planning_time (ms, planner_server 안에서 잰 createPlan 시간)
- 장애물 여유 거리: 경로를 0.0125 m 간격으로 짚어, 지도에서 가장 가까운 점유 셀까지 거리(EDT)의 최솟값 (로봇 중심 기준, m)
- 총 회전각: 경로를 0.1 m 간격으로 다시 나눠 이웃 구간 방향 차이의 절댓값 합 (도) - 격자 지그재그가 많을수록 크다
결과: pairs.csv(쌍 x planner), summary.md(표), summary.json
"""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from PIL import Image
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
PLANNERS = ['NavFn', 'NavFnAStar', 'Smac2D', 'AStarRaw', 'AStar']


class ClearanceMap:
    """지도 점유 셀까지 거리 (map_server trinary 규칙: 점유 확률 >= occupied_thresh)."""

    def __init__(self, map_yaml: Path):
        meta = yaml.safe_load(map_yaml.read_text())
        img = np.asarray(Image.open(map_yaml.parent / meta['image']).convert('L'), dtype=float)
        occ = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
        occupied = occ >= meta['occupied_thresh']
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = occupied.shape
        self.dist = distance_transform_edt(~occupied) * self.res

    def at(self, x: float, y: float) -> float:
        col = int(math.floor((x - self.ox) / self.res))
        row = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        if not (0 <= col < self.w and 0 <= row < self.h):
            return 0.0
        return float(self.dist[row, col])


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def resample(path, step):
    """경로를 step(m) 간격 점으로 (처음·끝 포함)."""
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    to_next = step  # 다음 점까지 남은 거리
    for a, b in zip(path, path[1:]):
        seg = math.dist(a, b)
        pos = 0.0
        while seg - pos >= to_next:
            pos += to_next
            out.append((a[0] + (b[0] - a[0]) * pos / seg, a[1] + (b[1] - a[1]) * pos / seg))
            to_next = step
        to_next -= seg - pos
    if math.dist(out[-1], path[-1]) > 1e-9:
        out.append(path[-1])
    return out


def min_clearance(path, cmap: ClearanceMap):
    return min(cmap.at(x, y) for x, y in resample(path, cmap.res / 2.0))


def total_turn_deg(path, step=0.1):
    pts = resample(path, step)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:]) if math.dist(a, b) > 1e-6]
    turn = 0.0
    for h0, h1 in zip(headings, headings[1:]):
        turn += abs(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)))
    return math.degrees(turn)


class Client(Node):
    def __init__(self):
        super().__init__('planner_compare')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def spin_until(self, future, timeout_s):
        end = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def plan(self, planner_id, start, goal):
        """(성공, planning_time ms, 경로 [(x, y)])."""
        req = ComputePathToPose.Goal()
        req.planner_id = planner_id
        req.use_start = True
        req.start, req.goal = start, goal
        fut = self.client.send_goal_async(req)
        if not self.spin_until(fut, 30.0) or not fut.result().accepted:
            return False, math.nan, []
        res_fut = fut.result().get_result_async()
        if not self.spin_until(res_fut, 60.0):
            return False, math.nan, []
        res = res_fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            return False, math.nan, []
        pt = res.result.planning_time
        return True, pt.sec * 1000 + pt.nanosec * 1e-6, [(p.pose.position.x, p.pose.position.y)
                                                          for p in res.result.path.poses]


def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


def to_map(x, y, origin):
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = x - ox, y - oy
    return c * dx + s * dy, -s * dx + c * dy


def summarize(rows, planners):
    """planner별 지표. 길이·여유·회전각 평균은 모든 planner가 성공한 쌍만으로 낸다(같은 쌍끼리 비교)."""
    pairs = sorted({r['pair'] for r in rows})
    ok = {(r['pair'], r['planner']): r for r in rows if r['ok']}
    common = [p for p in pairs if all((p, name) in ok for name in planners)]
    out = {}
    for name in planners:
        mine = [r for r in rows if r['planner'] == name]
        times = [t for r in mine for t in r['times_ms']]
        sel = [ok[(p, name)] for p in common]
        base = [ok[(p, planners[0])] for p in common]
        out[name] = {
            'success': sum(1 for r in mine if r['ok']),
            'pairs': len(mine),
            'length_m_mean': float(np.mean([r['length_m'] for r in sel])) if sel else math.nan,
            'length_vs_first': float(np.mean([r['length_m'] / b['length_m'] for r, b in zip(sel, base)]))
            if sel else math.nan,
            'time_ms_mean': float(np.mean(times)) if times else math.nan,
            'time_ms_p95': float(np.percentile(times, 95)) if times else math.nan,
            'time_ms_max': float(np.max(times)) if times else math.nan,
            'clearance_m_mean_of_min': float(np.mean([r['clearance_m'] for r in sel])) if sel else math.nan,
            'clearance_m_min': float(np.min([r['clearance_m'] for r in sel])) if sel else math.nan,
            'turn_deg_mean': float(np.mean([r['turn_deg'] for r in sel])) if sel else math.nan,
            'common_pairs': len(common),
        }
    return out


def markdown(summary):
    lines = ['| planner | 성공 | 경로 길이 평균 (m) | NavFn 대비 | 계획 시간 평균 / p95 / 최대 (ms) | '
             '최소 여유 평균 / 최소 (m) | 총 회전각 평균 (°) |',
             '|---|---|---|---|---|---|---|']
    for name, s in summary.items():
        lines.append(
            f"| {name} | {s['success']}/{s['pairs']} | {s['length_m_mean']:.2f} | {s['length_vs_first']:.3f} | "
            f"{s['time_ms_mean']:.1f} / {s['time_ms_p95']:.1f} / {s['time_ms_max']:.1f} | "
            f"{s['clearance_m_mean_of_min']:.3f} / {s['clearance_m_min']:.3f} | {s['turn_deg_mean']:.0f} |")
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--waypoints', default=str(REPO / 'experiments' / 'nav_kpi' / 'waypoints.yaml'))
    ap.add_argument('--map', default=str(REPO / 'src' / 'localization' / 'amr_mapping' / 'maps' /
                                         'warehouse_v2_0025.yaml'))
    ap.add_argument('--origin', type=lambda s: [float(v) for v in s.strip('[]').split(',')],
                    default=[-0.0439, 1.4712, -0.00088],
                    help='map_origin_in_world [x, y, yaw] (nav_kpi 시나리오·system.launch.py와 같은 값)')
    ap.add_argument('--planners', nargs='+', default=PLANNERS)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--pairs', type=int, default=0, help='앞에서부터 이만큼의 쌍만 (0 = 전부)')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cmap = ClearanceMap(Path(a.map))
    wps = [to_map(x, y, a.origin) for x, y in yaml.safe_load(Path(a.waypoints).read_text())['waypoints']]
    pairs = list(zip(wps, wps[1:]))
    if a.pairs:
        pairs = pairs[:a.pairs]

    rclpy.init()
    node = Client()
    if not node.client.wait_for_server(timeout_sec=60.0):
        raise SystemExit('compute_path_to_pose 액션 서버가 없음 (planner_compare.launch.py를 먼저 띄운다)')

    rows = []
    for i, ((sx, sy), (gx, gy)) in enumerate(pairs):
        yaw = math.atan2(gy - sy, gx - sx)
        for name in a.planners:
            times, first = [], None
            for _ in range(a.repeats):
                ok, ms, path = node.plan(name, pose(node, sx, sy, yaw), pose(node, gx, gy, yaw))
                if ok:
                    times.append(ms)
                    first = first or path
            row = {'pair': i, 'planner': name, 'ok': first is not None, 'times_ms': times,
                   'length_m': path_length(first) if first else math.nan,
                   'clearance_m': min_clearance(first, cmap) if first else math.nan,
                   'turn_deg': total_turn_deg(first) if first else math.nan,
                   'points': len(first) if first else 0}
            rows.append(row)
            print(f"쌍 {i:2d} {name:10s} {'OK' if row['ok'] else '실패'} "
                  f"{row['length_m']:7.2f} m {np.mean(times) if times else math.nan:8.1f} ms "
                  f"여유 {row['clearance_m']:.3f} m 회전 {row['turn_deg']:.0f}°", flush=True)

    with (out / 'pairs.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pair', 'planner', 'ok', 'length_m', 'time_ms_mean', 'time_ms_max', 'clearance_m',
                    'turn_deg', 'points'])
        for r in rows:
            t = r['times_ms']
            w.writerow([r['pair'], r['planner'], int(r['ok']), f"{r['length_m']:.3f}",
                        f"{np.mean(t):.2f}" if t else '', f"{np.max(t):.2f}" if t else '',
                        f"{r['clearance_m']:.4f}", f"{r['turn_deg']:.1f}", r['points']])
    summary = summarize(rows, a.planners)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    (out / 'summary.md').write_text(markdown(summary))
    print(markdown(summary))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
