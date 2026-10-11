#!/usr/bin/env python3
"""연구 2 RL 평가 환경: 2D 학습 환경과 같은 시나리오를 Gazebo에서 돌리고 같은 지표를 CSV로 남긴다.

에피소드마다
1. 시나리오를 뽑는다 (평가 목록 eval_list의 에피소드 번호 → 시드, S1·S2 강제는 2D evaluate.py와 같음)
2. 통로 벽(폭 corridor_width)과 돌발 장애물(S1만)을 스폰한다. 장애물은 처음엔 멀리 둔다
3. 로봇을 가속 구간 시작으로 옮기고 출발 속도까지 가속한다. 출발선(x = 0)을 지나면 에피소드 시작
4. 0.1 s마다 정책(obs → v, ω)을 부른다. 트리거 지점 place_ahead_m 앞에서 장애물을 제자리에 옮기고,
   로봇 중심이 trigger_x를 지나는 순간(GT 100 Hz) 움직이기 시작한다 (가로지름: 바로, 멈춰 섬: dwell 뒤)
5. 도착·충돌·지나침·시간 초과로 끝나면 지표 한 줄 (rl_eval.EpisodeTracker)

로봇 적재 질량은 스폰할 때 정해진다 (URDF payload_mass). 질량마다 launch를 다시 띄운다 (rl_eval.launch.py).
"""

import csv
import math
import os
import subprocess
import tempfile
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from amr_gazebo.rl_eval import (
    Sample, EpisodeTracker, default_research2_dir, eval_seed, layout_for, load_research2,
    obstacle_sdf, obstacle_spot, policy_obs, scenario_for, walls_sdf, DT)
from amr_gazebo.rl_policies import load_policy

SCENARIOS = {'S1': True, 'S2': False, 'mixed': None}
V_MAX, W_MAX = 2.0, 1.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RlEval(Node):

    def __init__(self):
        super().__init__('rl_eval')
        p = self.declare_parameter
        root = p('research2_dir', '').value or default_research2_dir()
        if not root:
            raise RuntimeError('experiments/research2를 못 찾았다: research2_dir 파라미터나 AMR_RESEARCH2_DIR')
        self.r2 = load_research2(root)
        self.world = p('world', 'rl_eval').value
        self.robot = p('robot', 'amr_01').value
        # launch에서 '25'처럼 정수로 와도 받게 dynamic_typing
        self.payload = float(p('payload_kg', 0.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.eval_list = p('eval_list', 0).value
        self.episodes = p('episodes', 5).value
        self.start_episode = p('start_episode', 0).value
        self.scenarios = [s.strip() for s in p('scenarios', 'S1,S2').value.split(',') if s.strip()]
        self.policy_name = p('policy', 'brake').value
        self.policy = load_policy(self.policy_name)
        self.csv_path = p('csv', '/tmp/rl_eval/episodes.csv').value
        self.place_ahead = p('place_ahead_m', 1.5).value

        self.lock = threading.Lock()
        self.robot_sample = None      # (stamp, x, y, theta, v, w)
        self.obstacle = None          # (x, y, vx, vy)
        self.tracker = None
        self.t0 = None
        self.spot = None              # 돌발 장애물을 옮길 자리 (트리거 place_ahead_m 앞에서 정함)
        self.appeared_at = None
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)
        self.obs_pub = self.create_publisher(Twist, '/model/popout/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{self.robot}/ground_truth/odom', self.on_robot, 50)
        self.create_subscription(Odometry, '/model/popout/odometry', self.on_obstacle, 50)
        self.get_logger().info(
            f'rl_eval: research2 {root}, 질량 {self.payload} kg, 목록 {self.eval_list}, '
            f'에피소드 {self.start_episode}~{self.start_episode + self.episodes - 1}, {self.scenarios}, 정책 {self.policy_name}')

    # ---------------------------------------------------------------- GT
    def on_robot(self, m: Odometry):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose
        sample = (stamp, p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x,
                  m.twist.twist.angular.z)
        with self.lock:
            self.robot_sample = sample
            tracker, t0, obstacle = self.tracker, self.t0, self.obstacle
            if tracker is None or t0 is None:
                return
            s = Sample(stamp - t0, *sample[1:])
            if tracker.popout_triggered(s):
                # 옮기기(set_pose)가 아직 안 끝났으면 GT는 멀리 둔 자리다 → 옮길 자리(spot)로 판정한다
                xy = self.spot
                if obstacle and self.spot and math.hypot(obstacle[0] - self.spot[0], obstacle[1] - self.spot[1]) < 0.3:
                    xy = obstacle[:2]
                tracker.appear(s, xy)
                self.appeared_at = s.t
                po = tracker.scenario.popout
                if po.dwell_s <= 0.0:
                    self._move_obstacle(po.direction * po.speed)
            tracker.update(s, obstacle[:2] if (obstacle and tracker.popout_appeared) else None)

    def on_obstacle(self, m: Odometry):
        p = m.pose.pose
        with self.lock:
            self.obstacle = (p.position.x, p.position.y, m.twist.twist.linear.x, m.twist.twist.linear.y)

    # ---------------------------------------------------------------- Gazebo
    def gz(self, service, reqtype, req, reptype='ignition.msgs.Boolean'):
        for _ in range(3):
            out = subprocess.run(
                ['ign', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype,
                 '--reptype', reptype, '--timeout', '10000', '--req', req],
                capture_output=True, text=True, timeout=20)
            if 'true' in out.stdout:
                return True
        return False

    def spawn(self, name, sdf, x=0.0, y=0.0):
        path = os.path.join(tempfile.gettempdir(), f'rl_eval_{name}.sdf')
        with open(path, 'w') as f:
            f.write(sdf)
        ok = self.gz('create', 'ignition.msgs.EntityFactory',
                     f'sdf_filename: "{path}", name: "{name}", pose: {{position: {{x: {x}, y: {y}}}}}')
        if not ok:
            raise RuntimeError(f'{name} 스폰 실패')

    def remove(self, name):
        self.gz('remove', 'ignition.msgs.Entity', f'name: "{name}", type: MODEL')

    def set_pose(self, name, x, y, yaw=0.0, z=0.0):
        return self.gz('set_pose', 'ignition.msgs.Pose',
                       f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.3f}}}, '
                       f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')

    def _move_obstacle(self, vy):
        t = Twist()
        t.linear.y = float(vy)
        self.obs_pub.publish(t)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def now_sim(self):
        with self.lock:
            return self.robot_sample[0] if self.robot_sample else None

    def wait_sim(self, seconds):
        """시뮬레이션 시간으로 seconds 기다린다 (GT 시각 기준)."""
        start = None
        while rclpy.ok():
            now = self.now_sim()
            if now is not None:
                start = now if start is None else start
                if now - start >= seconds:
                    return now
            time.sleep(0.005)

    # ---------------------------------------------------------------- 에피소드
    def run(self):
        while self.now_sim() is None and rclpy.ok():
            time.sleep(0.1)
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        for k in range(self.start_episode, self.start_episode + self.episodes):
            for name in self.scenarios:
                seed = eval_seed(self.eval_list, k)
                row = self.episode(seed, name)
                row = {'scenario': name, 'scenario_seed': seed, 'eval_list': self.eval_list, 'episode': k,
                       'policy': self.policy_name, **row}
                self.get_logger().info(
                    f'{name} #{k} (seed {seed}): {row["outcome"]} {row["time_s"]:.1f} s, '
                    f'popout {row["popout_result"]} gap {row["popout_gap_actual"]}/{row["popout_gap"]}')
                new = not os.path.exists(self.csv_path)
                with open(self.csv_path, 'a', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=list(row))
                    if new:
                        w.writeheader()
                    w.writerow(row)
        self.get_logger().info(f'끝: {self.csv_path}')

    def episode(self, seed, scenario_name):
        sc = scenario_for(self.r2, seed, SCENARIOS[scenario_name])
        layout = layout_for(self.r2, sc, self.payload)
        with self.lock:
            self.tracker, self.t0, self.obstacle = None, None, None
        self.cmd(0.0, 0.0)
        self.remove('corridor')
        self.remove('popout')
        self.spawn('corridor', walls_sdf(layout))
        park = (layout.wall_x0 - 20.0, 0.0)
        if sc.popout is not None:
            self.spawn('popout', obstacle_sdf(sc.popout.radius), *park)
        self.set_pose(self.robot, layout.start_x, layout.start_y, layout.heading, 0.02)
        for _ in range(15):
            self.cmd(0.0, 0.0)
            self.wait_sim(0.1)
        self._move_obstacle(0.0)
        self.spot = None
        placed = False
        tracker = EpisodeTracker(self.r2, sc, self.payload, layout.half_width)

        def place_if_due(x, y):
            nonlocal placed
            if sc.popout is not None and not placed and x >= sc.popout.trigger_x - self.place_ahead:
                placed = True
                self.spot = obstacle_spot(self.r2, sc, sc.popout.trigger_x, y, layout.half_width)
                threading.Thread(target=self.set_pose, args=('popout', *self.spot), daemon=True).start()

        # 가속 구간: 출발선까지 initial_speed로, 출발 방향 선(0, initial_y)을 따라
        c, s = math.cos(sc.initial_heading), math.sin(sc.initial_heading)
        start_wall = time.monotonic()
        while rclpy.ok():
            with self.lock:
                _, x, y, th, v, w = self.robot_sample
            place_if_due(x, y)
            if x >= 0.0:
                break
            lateral = -s * x + c * (y - sc.initial_y)
            heading_err = math.atan2(math.sin(th - sc.initial_heading), math.cos(th - sc.initial_heading))
            self.cmd(sc.initial_speed, max(-W_MAX, min(W_MAX, -3.0 * lateral - 3.0 * heading_err)))
            self.wait_sim(0.02)
            if time.monotonic() - start_wall > 120:
                raise RuntimeError('가속 구간에서 출발선에 못 감')
        with self.lock:
            stamp, x, y, th, v, w = self.robot_sample
            self.t0 = stamp
            self.tracker = tracker
        start = {'start_speed_actual': round(v, 3), 'start_y_actual': round(y, 3),
                 'start_heading_deg_actual': round(math.degrees(th), 2)}

        # 에피소드: 0.1 s마다 정책
        next_t = 0.0
        self.appeared_at = None
        dwell_started = False
        while rclpy.ok():
            with self.lock:
                stamp, x, y, th, v, w = self.robot_sample
                obstacle = self.obstacle
                done = tracker.outcome is not None
            t = stamp - self.t0
            if done:
                break
            if t + 1e-6 < next_t:
                time.sleep(0.002)
                continue
            next_t += DT
            sample = Sample(t, x, y, th, v, w)
            place_if_due(x, y)
            po = sc.popout
            if po is not None and tracker.popout_appeared and po.dwell_s > 0.0 and not dwell_started \
                    and self.appeared_at is not None and t >= self.appeared_at + po.dwell_s:
                dwell_started = True
                self._move_obstacle(po.direction * po.speed)
            seen = []
            if tracker.popout_appeared and obstacle is not None:
                seen = [{'x': obstacle[0], 'y': obstacle[1], 'vx': obstacle[2], 'vy': obstacle[3],
                         'radius': po.radius}]
            cmd_v, cmd_w = self.policy(policy_obs(sample, sc, layout, self.payload, seen))
            self.cmd(max(0.0, min(V_MAX, cmd_v)), max(-W_MAX, min(W_MAX, cmd_w)))
            with self.lock:
                tracker.control_tick(sample, obstacle if tracker.popout_appeared else None)
        self.cmd(0.0, 0.0)
        self._move_obstacle(0.0)
        with self.lock:
            self.tracker = None
        m = tracker.metrics()
        m.update(start)
        m['popout_gap'] = None if sc.popout is None else round(sc.popout.gap, 3)
        m['popout_gap_actual'] = None if tracker.popout_gap_actual is None else round(tracker.popout_gap_actual, 3)
        m['initial_speed'] = round(sc.initial_speed, 3)
        m['goal_distance'] = round(sc.goal_distance, 3)
        return m


def main():
    rclpy.init()
    node = RlEval()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    finally:
        node.cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
