#!/usr/bin/env python3
"""(10) DWA vs TEB 비교: 목표 1회 주행을 재서 nav_result.json에 쓴다. 하네스 nav_compare 시나리오의 주행 명령.

    python3 nav_measure.py --scenario obstacle --out <실행 폴더>/nav_result.json

데모(nav2_dwa_demo.launch.py send_goal:=false)가 떠 있는 상태에서:
  1. NavigateToPose로 목표(map (3.0, 1.5), 데모 기본값)를 보내고 결과(succeeded/aborted/...)를 받는다.
  2. 지표는 전부 GT(ground_truth/odom)로 잰다 (docs/algorithms/navigation.md 7-3절).
     - 도착 시간: 목표 수락 → SUCCEEDED (시뮬레이션 시간). Nav2가 목표에 닿은 실행만(충돌과 따로)
     - 경로 길이: 목표 수락부터 결과까지 GT 위치 증분의 합
     - CTE: GT 위치를 그 시각의 전역 경로(/<ns>/plan, map)에 투영한 거리의 평균·최대
     - 최소 장애물 거리(obstacle만): 사각 풋프린트(0.6 x 0.4 m, GT yaw 반영)와 장애물 원(반지름
       0.25 m, dynamic_worker 충돌 실린더) 표면 사이 거리의 최솟값. 1 cm 미만이면 충돌(COLLISION_GAP_M)
     - 성공: SUCCEEDED이고 충돌 없음 (result는 Nav2 결과 그대로, collision은 따로 남긴다)
  3. scenario=obstacle이면 장애물 GT를 perception/obstacles(TrackedObstacleArray)로도 내보낸다 - DWA가
     보는 입력이다(LiDAR 군집화 노드가 아직 없어서, README "장애물·측정 방식"). TEB는 이 토픽을 안 보고
     실제 LiDAR가 그 물체에 찍힌 costmap으로 본다.

실패(aborted·timeout 등)도 nav_result.json에 success=false로 남기고 종료 코드 0으로 끝낸다 - 성공률에
들어가야 하므로 하네스 실행 자체를 실패로 만들지 않는다. 결과 파일을 못 쓴 경우만 0이 아니다.
좌표: map->odom 항등 데모라 map = world - (0, 1.5) (warehouse_v1 map_origin_in_world, 데모 docstring).
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

NAMESPACE = 'amr_01'
WORLD_TO_MAP_OFFSET = (0.0, 1.5)  # map = world - offset
GOAL_MAP_XY = (3.0, 1.5)          # 데모 goal_x/goal_y 기본값
OBSTACLE_ID = 'crosser'           # obstacle.yaml
OBSTACLE_RADIUS_M = 0.25          # dynamic_worker 충돌 실린더 반지름
OBSTACLE_NORTH_END_MAP_Y = 2.25   # obstacle.yaml 왕복 북쪽 끝 world y 3.75 = map 2.25
FOOTPRINT_HALF = (0.3, 0.2)       # 로봇 600 x 400 mm (nav2_params.yaml footprint)
PUBLISH_RATE_HZ = 10.0
# 풋프린트와 장애물 사이가 이보다 가까우면 충돌로 본다. Gazebo에서 닿은 몸체는 겹치지 않고 밀려나서 접촉 중에도
# 거리가 0이 아닌 0.1~1 mm로 나온다(10/12 진단: 0.2 mm로 15초 붙어 있음) - 정확히 0으로 판정하면 같은 접촉이 실행마다 갈린다
COLLISION_GAP_M = 0.01


def point_to_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def point_to_polyline(p, line):
    return min(point_to_segment(p, line[i], line[i + 1]) for i in range(len(line) - 1))


def footprint_to_circle(robot_xy, robot_yaw, center, radius):
    """로봇 사각 풋프린트와 원 표면 사이 거리 (겹치면 0)."""
    dx, dy = center[0] - robot_xy[0], center[1] - robot_xy[1]
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    lx, ly = c * dx + s * dy, -s * dx + c * dy  # 로봇 좌표계의 원 중심
    ox = max(abs(lx) - FOOTPRINT_HALF[0], 0.0)
    oy = max(abs(ly) - FOOTPRINT_HALF[1], 0.0)
    return max(math.hypot(ox, oy) - radius, 0.0)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMeasure(Node):

    def __init__(self, scenario: str, goal_timeout_s: float):
        super().__init__('nav_measure', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scenario = scenario
        self.goal_timeout_s = goal_timeout_s
        self.robot = None          # (map_xy, yaw)
        self.obstacle = None       # map_xy
        self.plan = []
        self.measuring = False
        self.path_length = 0.0
        self.prev_xy = None
        self.cte = []
        self.min_obstacle = math.inf
        self.series = []  # (sim_t, robot x, y, yaw, obstacle x, y, 풋프린트-장애물 거리) - nav_series.csv
        self.create_subscription(Odometry, f'/{NAMESPACE}/ground_truth/odom', self.on_robot, 50)
        self.create_subscription(Path, f'/{NAMESPACE}/plan', self.on_plan, 10)
        if scenario == 'obstacle':
            self.create_subscription(
                Odometry, f'/dynamic_obstacles/{OBSTACLE_ID}/ground_truth/odom', self.on_obstacle, 50)
            self.obstacle_pub = self.create_publisher(
                TrackedObstacleArray, f'/{NAMESPACE}/perception/obstacles', 10)
            self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_obstacle)
        self.client = ActionClient(self, NavigateToPose, f'/{NAMESPACE}/navigate_to_pose')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_plan(self, msg: Path):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_obstacle(self, msg: Odometry):
        p = msg.pose.pose.position
        self.obstacle = (p.x - WORLD_TO_MAP_OFFSET[0], p.y - WORLD_TO_MAP_OFFSET[1])

    def on_robot(self, msg: Odometry):
        p = msg.pose.pose.position
        xy = (p.x - WORLD_TO_MAP_OFFSET[0], p.y - WORLD_TO_MAP_OFFSET[1])
        yaw = yaw_of(msg.pose.pose.orientation)
        self.robot = (xy, yaw)
        if not self.measuring:
            return
        if self.prev_xy is not None:
            self.path_length += math.dist(xy, self.prev_xy)
        self.prev_xy = xy
        if len(self.plan) >= 2:
            self.cte.append(point_to_polyline(xy, self.plan))
        gap = None
        if self.obstacle is not None:
            gap = footprint_to_circle(xy, yaw, self.obstacle, OBSTACLE_RADIUS_M)
            self.min_obstacle = min(self.min_obstacle, gap)
        ob = self.obstacle or (None, None)
        self.series.append((self.now(), xy[0], xy[1], yaw, ob[0], ob[1], gap))

    def publish_obstacle(self):
        if self.obstacle is None:
            return
        msg = TrackedObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{NAMESPACE}/odom'  # map->odom 항등 데모
        o = TrackedObstacle()
        o.track_id = 1
        o.source = TrackedObstacle.SOURCE_UNKNOWN
        o.is_dynamic = True
        o.pose.pose.position.x, o.pose.pose.position.y = self.obstacle
        o.radius = OBSTACLE_RADIUS_M
        msg.obstacles = [o]
        self.obstacle_pub.publish(msg)

    def spin_until(self, cond, timeout_wall_s):
        end = time.monotonic() + timeout_wall_s
        while rclpy.ok() and time.monotonic() < end and not cond():
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def run(self) -> dict:
        record = {'scenario': self.scenario, 'result': None, 'success': False, 'collision': False, 'arrival_s': None,
                  'arrival_wall_s': None, 'min_obstacle_m': None, 'path_length_m': None,
                  'cte_mean_m': None, 'cte_max_m': None}
        # 시계·로봇 GT·(장애물 시나리오면) 장애물 GT가 들어온 뒤 시작한다
        ready = self.spin_until(
            lambda: self.now() > 0 and self.robot is not None
            and (self.scenario != 'obstacle' or self.obstacle is not None), 120.0)
        if not ready:
            record['result'] = 'no_ground_truth'
            return record
        if not self.client.wait_for_server(timeout_sec=120.0):
            record['result'] = 'server_unavailable'
            return record
        if self.scenario == 'obstacle':
            # 장애물은 뜨자마자 왕복(12초 주기)하고 Nav2 기동 시간은 실행마다 달라서, 그냥 보내면 만나는 시점이
            # 실행마다 다르다(예비 실행에서 TEB가 장애물을 거의 안 만남). 장애물이 북쪽 끝에 와 남쪽으로 출발할
            # 때 보내 교차 시점을 맞춘다 - 경로 중점(map y 0.75)까지 장애물은 약 3초, 로봇도 비슷하게 닿는다
            north_end = OBSTACLE_NORTH_END_MAP_Y - 0.15
            record['goal_sync'] = self.spin_until(
                lambda: self.obstacle is not None and self.obstacle[1] >= north_end, 20.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL_MAP_XY
        goal.pose.pose.orientation.w = 1.0
        send = self.client.send_goal_async(goal)
        self.spin_until(send.done, 30.0)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            record['result'] = 'rejected'
            return record
        t0_sim, t0_wall = self.now(), time.monotonic()
        self.measuring = True
        result = handle.get_result_async()
        self.spin_until(result.done, self.goal_timeout_s)
        self.measuring = False
        if result.done():
            status = result.result().status
            record['result'] = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                                GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, f'status_{status}')
        else:
            record['result'] = 'timeout'
            handle.cancel_goal_async()
            self.spin_until(lambda: False, 2.0)
        # 7-3절 정의: 성공 = SUCCEEDED이고 충돌 0. 풋프린트와 장애물이 COLLISION_GAP_M보다 가까워진 적이 있으면 충돌
        record['collision'] = self.min_obstacle < COLLISION_GAP_M
        record['success'] = record['result'] == 'succeeded' and not record['collision']
        if record['result'] == 'succeeded':
            # 도착 시간은 Nav2가 목표에 닿았으면 남긴다 (충돌 여부는 collision으로 따로 본다)
            record['arrival_s'] = round(self.now() - t0_sim, 3)
            record['arrival_wall_s'] = round(time.monotonic() - t0_wall, 3)
        record['path_length_m'] = round(self.path_length, 4)
        if self.cte:
            record['cte_mean_m'] = round(sum(self.cte) / len(self.cte), 4)
            record['cte_max_m'] = round(max(self.cte), 4)
        if self.scenario == 'obstacle' and self.min_obstacle != math.inf:
            record['min_obstacle_m'] = round(self.min_obstacle, 4)
        if self.robot is not None:
            record['final_map_xy'] = [round(v, 4) for v in self.robot[0]]
        return record


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', choices=['clear', 'obstacle'], required=True)
    ap.add_argument('--controller', default='', help='기록용 (dwa|teb)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--goal-timeout-s', type=float, default=240.0, help='목표 결과를 기다리는 실제 시간')
    args = ap.parse_args()
    rclpy.init()
    node = NavMeasure(args.scenario, args.goal_timeout_s)
    try:
        record = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    record['controller'] = args.controller
    # 시계열 (7-3절 "실행마다 최솟값 + 시계열"): 충돌·회피 모양을 나중에 다시 볼 수 있게 남긴다
    series_path = os.path.join(os.path.dirname(args.out), 'nav_series.csv')
    with open(series_path, 'w', encoding='utf-8') as f:
        f.write('t,robot_x,robot_y,robot_yaw,obstacle_x,obstacle_y,gap_m\n')
        for row in node.series:
            f.write(','.join('' if v is None else f'{v:.4f}' for v in row) + '\n')
    print(json.dumps(record, ensure_ascii=False), file=sys.stderr, flush=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
