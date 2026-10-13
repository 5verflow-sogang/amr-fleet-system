#!/usr/bin/env python3
"""Nav KPI 측정: 웨이포인트 투어(출발·목표 50쌍)를 Nav2로 돌며 경로 계획·재계획·주행시간 예측·CTE를 잰다.

하네스 nav_kpi 시나리오의 주행 명령(drive.py command 모드)이다. Nav2·위치추정이 떠 있는 상태에서:

    python3 nav_kpi.py --out-dir <실행 폴더>

쌍 i (점 i → 점 i+1, waypoints.yaml, world 좌표)마다
  1. 계획: planner_server ComputePathToPose(현재 위치 → 목표). 성공 여부, Nav2가 잰 planning_time, 경로 길이
  2. 주행: NavigateToPose. 도는 동안 replan_period_s마다 ComputePathToPose를 다시 불러 재계획 시간을 잰다
     (Nav2 BT도 1 Hz로 다시 계획하지만 그 시간을 밖에서 볼 수 없어서 같은 planner_server를 직접 부른다)
  3. 예측 주행시간 = 계획 경로 길이를 순항 속도·가속도의 사다리꼴로 간 시간. 실제 = 목표 수락 → 결과 (시뮬레이션 시간)
  4. CTE: GT 위치(world → map)를 그 시각의 전역 경로(/<ns>/plan, BT가 1 Hz로 다시 계획)에 투영.
     투영점의 경로 곡률이 curve_kappa 넘으면 곡선. KPI 평균은 도착한 쌍만 (#145 nav_measure.py와 같은 기준)
계획이 실패하면 주행하지 않고 다음 쌍으로 간다(출발은 그대로). 도착 못 하면(aborted·시간 초과) 그 자리에서 다음 쌍.

출력 (out-dir): nav_pairs.csv(쌍마다), replan.csv, cte.csv, paths.csv(쌍마다 처음 계획한 경로, map),
nav_kpi.json(요약·KPI 판정, 하네스 요약이 읽는다).
정의는 docs/algorithms/navigation.md에 맞춘다 (이진 판정, 4주차 민지 작업 2).
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, str(Path(__file__).resolve().parent))
from navkpi_core import (cumulative, curvature, path_length, predicted_time, prediction_error,  # noqa: E402
                         project, summarize)

HERE = Path(__file__).resolve().parent


def yaw_q(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavKpi(Node):

    def __init__(self, args):
        super().__init__('nav_kpi', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        ns = args.namespace
        self.nav = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, f'/{ns}/compute_path_to_pose')
        self.gt = None
        self.create_subscription(Odometry, f'/{ns}/ground_truth/odom', self._on_gt, qos_profile_sensor_data)
        self.plan_now = None   # (경로, 누적 길이, 곡률) - 지금 따라가는 전역 경로
        self.create_subscription(PathMsg, f'/{ns}/plan', self._on_plan, 10)

    def _on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)

    def _on_plan(self, msg):
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(path) >= 2:
            self.plan_now = (path, cumulative(path), curvature(path, self.args.kappa_window_m))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, done, timeout_s):
        end = time.monotonic() + timeout_s
        while not done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return done()

    def to_map(self, x, y):
        ox, oy, oyaw = self.args.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        dx, dy = x - ox, y - oy
        return c * dx + s * dy, -s * dx + c * dy

    def goal_pose(self, mx, my, yaw):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = mx, my
        g.pose.orientation.z, g.pose.orientation.w = yaw_q(yaw)
        return g

    def compute_path(self, goal):
        """(성공, Nav2 planning_time ms, 왕복 ms, 경로 [(x, y)] map)."""
        req = ComputePathToPose.Goal()
        req.goal = goal
        req.planner_id = self.args.planner_id
        req.use_start = False
        t0 = time.monotonic()
        fut = self.plan.send_goal_async(req)
        if not self.spin_until(fut.done, 10.0) or not fut.result().accepted:
            return False, math.nan, math.nan, []
        res_fut = fut.result().get_result_async()
        if not self.spin_until(res_fut.done, 10.0):
            return False, math.nan, math.nan, []
        wall_ms = (time.monotonic() - t0) * 1000
        res = res_fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            return False, math.nan, wall_ms, []
        pt = res.result.planning_time
        path = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        return True, pt.sec * 1000 + pt.nanosec * 1e-6, wall_ms, path

    def run(self):
        a = self.args
        wps = yaml.safe_load(Path(a.waypoints).read_text())['waypoints']
        narrow = set(yaml.safe_load(Path(a.waypoints).read_text()).get('narrow_pairs', []))
        out = Path(a.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        if not self.nav.wait_for_server(timeout_sec=60) or not self.plan.wait_for_server(timeout_sec=60):
            print('Nav2 액션 서버가 없음', file=sys.stderr)
            return 2
        self.spin_until(lambda: self.gt is not None and self.now() > 0, 60)

        pairs, replans, cte_rows, path_rows = [], [], [], []
        replan_rows = []
        n_pairs = min(len(wps) - 1, a.pairs) if a.pairs else len(wps) - 1
        for i in range(1, n_pairs + 1):
            gx, gy = wps[i]
            px, py = wps[i - 1]
            mx, my = self.to_map(gx, gy)
            yaw = math.atan2(gy - py, gx - px)
            goal = self.goal_pose(mx, my, yaw)
            row = {'pair': i, 'start_x': round(self.gt[0], 3), 'start_y': round(self.gt[1], 3),
                   'goal_x': gx, 'goal_y': gy, 'narrow': int(i in narrow)}
            ok, plan_ms, plan_wall_ms, path = self.compute_path(goal)
            length = path_length(path) if ok else math.nan
            pred = predicted_time(length, a.cruise_mps, a.accel_mps2) if ok else math.nan
            row.update(plan_ok=int(ok), plan_ms=round(plan_ms, 1), plan_wall_ms=round(plan_wall_ms, 1),
                       path_len_m=round(length, 3), predicted_s=round(pred, 2))
            if not ok:
                row.update(result='no_plan', nav_success=0, actual_s='', pred_err='', replan_n=0)
                pairs.append({**row, 'nav_success': False, 'plan_ok': False, 'pred_err': None})
                print(f'[{i}/{n_pairs}] 계획 실패', file=sys.stderr, flush=True)
                continue
            kappa = curvature(path, a.kappa_window_m)
            path_rows += [(i, round(x, 3), round(y, 3), round(k, 3)) for (x, y), k in zip(path, kappa)]
            self.plan_now = None

            req = NavigateToPose.Goal()
            req.pose = goal
            fut = self.nav.send_goal_async(req)
            self.spin_until(fut.done, 10.0)
            handle = fut.result() if fut.done() else None
            if handle is None or not handle.accepted:
                row.update(result='rejected', nav_success=0, actual_s='', pred_err='', replan_n=0)
                pairs.append({**row, 'nav_success': False, 'plan_ok': True, 'pred_err': None})
                continue
            t0 = self.now()
            res_fut = handle.get_result_async()
            timeout_s = max(a.min_timeout_s, a.timeout_factor * pred)
            next_replan = t0 + a.replan_period_s
            next_cte = t0
            n_replan = 0
            pair_cte = []
            while not res_fut.done():
                rclpy.spin_once(self, timeout_sec=0.02)
                t = self.now()
                if t - t0 > timeout_s:
                    handle.cancel_goal_async()
                    self.spin_until(res_fut.done, 5.0)
                    break
                if t >= next_cte and self.gt is not None and self.plan_now is not None:
                    next_cte = t + 1.0 / a.cte_hz
                    # 전역 경로는 계획 순간의 로봇 위치에서 시작하므로 앞 3 m(선분 120개)만 찾는다
                    cur, cur_cum, cur_kappa = self.plan_now
                    pr = project(self.to_map(*self.gt), cur, cur_cum, 0, search=120)
                    if pr is not None:
                        seg = 'curve' if cur_kappa[pr.index] > a.curve_kappa else 'straight'
                        pair_cte.append((round(t, 3), i, round(pr.distance, 4), seg, round(cur_cum[-1] - pr.s, 3)))
                if t >= next_replan:
                    next_replan = t + a.replan_period_s
                    r_ok, r_ms, r_wall, _ = self.compute_path(self.goal_pose(mx, my, yaw))
                    replan_rows.append((round(t, 3), i, int(r_ok), round(r_ms, 1), round(r_wall, 1)))
                    if r_ok:
                        replans.append(r_ms)
                    n_replan += 1
            status = res_fut.result().status if res_fut.done() else None
            result = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                      GoalStatus.STATUS_CANCELED: 'timeout'}.get(status, f'status_{status}')
            actual = self.now() - t0
            success = result == 'succeeded'
            cte_rows += [(*r, int(success)) for r in pair_cte]
            err = prediction_error(actual, pred) if success else None
            row.update(result=result, nav_success=int(success), actual_s=round(actual, 2) if success else '',
                       pred_err=round(err, 4) if err is not None else '', replan_n=n_replan)
            pairs.append({**row, 'nav_success': success, 'plan_ok': True, 'pred_err': err})
            print(f'[{i}/{n_pairs}] {result} 길이 {length:.1f} m 예측 {pred:.1f} s 실제 {actual:.1f} s',
                  file=sys.stderr, flush=True)

        fields = ['pair', 'start_x', 'start_y', 'goal_x', 'goal_y', 'narrow', 'plan_ok', 'plan_ms', 'plan_wall_ms',
                  'path_len_m', 'predicted_s', 'result', 'nav_success', 'actual_s', 'pred_err', 'replan_n']
        with (out / 'nav_pairs.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            for p in pairs:
                w.writerow({**p, 'plan_ok': int(bool(p['plan_ok'])), 'nav_success': int(bool(p['nav_success'])),
                            'pred_err': '' if p['pred_err'] is None else round(p['pred_err'], 4)})
        with (out / 'replan.csv').open('w', newline='') as f:
            csv.writer(f).writerows([('time', 'pair', 'ok', 'planning_ms', 'wall_ms'), *replan_rows])
        with (out / 'paths.csv').open('w', newline='') as f:
            csv.writer(f).writerows([('pair', 'x', 'y', 'kappa'), *path_rows])
        with (out / 'cte.csv').open('w', newline='') as f:
            csv.writer(f).writerows([('time', 'pair', 'cte_m', 'segment', 'remaining_m', 'pair_success'), *cte_rows])
        # CTE 판정은 도착한 쌍만 (못 간 쌍은 성공률로 따로 본다)
        summary = summarize(pairs, replans, [(r[2], r[3]) for r in cte_rows if r[5]])
        summary['definition'] = {k: getattr(a, k) for k in (
            'cruise_mps', 'accel_mps2', 'curve_kappa', 'kappa_window_m', 'replan_period_s', 'planner_id')}
        tmp = out / 'nav_kpi.json.tmp'
        tmp.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        os.replace(tmp, out / 'nav_kpi.json')
        print(json.dumps(summary['verdict'], ensure_ascii=False), file=sys.stderr)
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--waypoints', default=str(HERE / 'waypoints.yaml'))
    ap.add_argument('--namespace', default='amr_01')
    ap.add_argument('--origin', type=lambda s: [float(v) for v in s.strip('[]').split(',')],
                    default=[-0.0562, 1.5317, 0.00009], help='map_origin_in_world [x, y, yaw]')
    ap.add_argument('--pairs', type=int, default=0, help='앞에서부터 이만큼만 (0이면 전부)')
    ap.add_argument('--planner-id', default='GridBased')
    ap.add_argument('--cruise-mps', type=float, default=0.5, help='예측 주행시간의 순항 속도')
    ap.add_argument('--accel-mps2', type=float, default=1.0, help='예측 주행시간의 가감속')
    ap.add_argument('--curve-kappa', type=float, default=0.2, help='이 곡률 [1/m] 넘으면 곡선 구간')
    ap.add_argument('--kappa-window-m', type=float, default=1.0,
                    help='곡률을 잴 앞뒤 거리. NavFn 경로는 격자 계단이라 짧으면 직선도 곡선으로 나온다')
    ap.add_argument('--replan-period-s', type=float, default=1.0)
    ap.add_argument('--cte-hz', type=float, default=10.0)
    ap.add_argument('--timeout-factor', type=float, default=3.0, help='쌍 시간 제한 = 예측 × 이 값 (최소 아래 값)')
    ap.add_argument('--min-timeout-s', type=float, default=60.0)
    args = ap.parse_args()
    rclpy.init()
    node = NavKpi(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
