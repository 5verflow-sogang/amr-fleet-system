"""순간 이동(Kidnapped) 시험 한 번: 로봇이 움직였다 멈추면 Gazebo에서 옮기고, 복구되는지 판정해 파일로 남긴다.

    ros2 run amr_localization kidnap_trial --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \\
        -p mode:=near -p seed:=1 -p output_dir:=/tmp/kidnap

하네스 시나리오 `kidnap`이 부른다. 판정 기준은 docs/algorithms/localization.md 4-4절 (kidnap/trial.py).

순서: 로봇이 moved_min_m 넘게 움직인 뒤 still_s 동안 멈추면 → pick_target(mode, seed)으로 고른 world 자세로
`ign service .../set_pose` → 정답이 그 자리로 옮겨진 시각을 순간 이동 시각으로 → 복구되거나 시간이 다 되면 저장하고 끝난다.

구독: ground_truth/odom (world), odometry/filtered (map, EKF 전역), kidnap_recovery/state·match_ratio, amcl_pose
저장: <output_dir>/kidnap.json (판정), kidnap_samples.csv (순간 이동 뒤 오차 + 그때의 일치율·AMCL 표준편차),
      kidnap_states.csv (복구 노드 상태)
"""

import json
import math
import os
import subprocess

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from .eval.trajectory import Pose2D, quaternion_from_yaw, transform_to_map, yaw_from_quaternion
from .kidnap.trial import Criteria, Sample, evaluate, pick_target, robot_frame_error

WAIT_MOVE, WAIT_STILL, TELEPORTING, OBSERVE, DONE = 'wait_move', 'wait_still', 'teleporting', 'observe', 'done'


def _pose(msg: Odometry) -> Pose2D:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return Pose2D(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y,
                  yaw_from_quaternion(q.x, q.y, q.z, q.w))


class KidnapTrialNode(Node):

    def __init__(self):
        super().__init__('kidnap_trial')
        d = self.declare_parameter
        self.mode = d('mode', 'near').value
        self.seed = d('seed', 1).value
        self.world = d('world', 'warehouse').value
        ns = self.get_namespace().strip('/')
        self.model = d('model', ns or 'amr').value
        self.origin = tuple(d('map_origin_in_world', [0.0, 1.5, 0.0]).value)
        self.output_dir = d('output_dir', '').value
        self.moved_min_m = d('moved_min_m', 2.0).value
        self.still_s = d('still_s', 2.0).value
        self.criteria = Criteria(
            detect_s=d('detect_s', 5.0).value, window_s=d('window_s', 60.0).value,
            along_m=d('along_m', 0.3).value, lateral_m=d('lateral_m', 0.1).value,
            yaw_rad=d('yaw_rad', 0.1).value, hold_s=d('hold_s', 3.0).value)

        self.phase = WAIT_MOVE
        self.gt = None
        self.gt_z = 0.0
        self.gt_speed = 0.0
        self.start = None
        self.still_since = None
        self.target = None
        self.t_teleport_call = None
        self.t_kidnap = None
        self.states: list[tuple[float, str]] = []
        self.samples: list[Sample] = []
        # 샘플마다 그때의 복구 노드 일치율과 AMCL 위치 표준편차 (수렴 판정을 나중에 따져 보려고)
        self.extra: list[tuple[float, float]] = []
        self.ratio = float('nan')
        self.amcl_std = float('nan')
        self.error = None

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_gt, qos)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_est, 10)
        self.create_subscription(String, 'kidnap_recovery/state', self._on_state, 10)
        self.create_subscription(Float32, 'kidnap_recovery/match_ratio', self._on_ratio, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.create_timer(0.1, self._step)
        self.get_logger().info(f'순간 이동 시험 대기: mode={self.mode}, seed={self.seed}, model={self.model}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_gt(self, msg: Odometry):
        self.gt = _pose(msg)
        self.gt_z = msg.pose.pose.position.z
        self.gt_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) + abs(msg.twist.twist.angular.z)

    def _on_est(self, msg: Odometry):
        if self.phase != OBSERVE or self.gt is None:
            return
        est = _pose(msg)
        gt_map = transform_to_map([self.gt], self.origin)[0]
        along, lateral, yaw = robot_frame_error(est, gt_map)
        self.samples.append(Sample(est.t, along, lateral, yaw))
        self.extra.append((self.ratio, self.amcl_std))

    def _on_ratio(self, msg: Float32):
        self.ratio = msg.data

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl_std = math.sqrt(max(c[0], c[7], 0.0))

    def _on_state(self, msg: String):
        t = self._now()
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append((t, msg.data))

    def _step(self):
        if self.gt is None or self.phase == DONE:
            return
        t = self._now()
        if self.phase == WAIT_MOVE:
            self.start = self.start or self.gt
            if math.hypot(self.gt.x - self.start.x, self.gt.y - self.start.y) > self.moved_min_m:
                self.phase = WAIT_STILL
        elif self.phase == WAIT_STILL:
            if self.gt_speed > 0.02:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = t
            elif t - self.still_since >= self.still_s:
                self._teleport()
        elif self.phase == TELEPORTING:
            if math.hypot(self.gt.x - self.target.x, self.gt.y - self.target.y) < 0.3:
                self.t_kidnap = t
                self.phase = OBSERVE
                self.get_logger().info(f'순간 이동 완료 t={t:.2f}')
            elif t - self.t_teleport_call > 10.0:
                self._finish('순간 이동 후 정답 자세가 목표로 바뀌지 않음')
        elif self.phase == OBSERVE:
            c = self.criteria
            # 성공이 정해졌거나, 복구 노드가 포기했거나, 기준 시간이 다 지나면 끝낸다
            success = evaluate(self.t_kidnap, self.states, self.samples, c)['success']
            failed = self.states and self.states[-1][1] == 'failed' and t - self.states[-1][0] > 2.0
            if success or failed or t - self.t_kidnap > c.detect_s + c.window_s + c.hold_s + 1.0:
                self._finish()

    def _teleport(self):
        self.target = pick_target(self.mode, self.seed, self.gt)
        _, _, qz, qw = quaternion_from_yaw(self.target.yaw)
        req = (f'name: "{self.model}", position: {{x: {self.target.x:.4f}, y: {self.target.y:.4f}, '
               f'z: {self.gt_z + 0.01:.4f}}}, orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
        cmd = ['ign', 'service', '-s', f'/world/{self.world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
               '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', req]
        self.get_logger().warn(f'순간 이동: ({self.gt.x:.2f}, {self.gt.y:.2f}) → '
                               f'({self.target.x:.2f}, {self.target.y:.2f}, {self.target.yaw:.2f} rad)')
        self.t_teleport_call = self._now()
        self.phase = TELEPORTING
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired) as e:
            self._finish(f'set_pose 호출 실패: {e}')
            return
        if 'true' not in res.stdout:
            self._finish(f'set_pose 응답이 성공이 아님: {res.stdout.strip()} {res.stderr.strip()}')

    def _finish(self, error=None):
        self.phase = DONE
        self.error = error
        result = {'mode': self.mode, 'seed': self.seed, 'error': error, 'target_world': None}
        if self.target is not None:
            result['target_world'] = [round(self.target.x, 3), round(self.target.y, 3), round(self.target.yaw, 3)]
        if error is None:
            result.update(evaluate(self.t_kidnap, self.states, self.samples, self.criteria))
        self._save(result)
        self.get_logger().info(f'시험 끝: {json.dumps(result, ensure_ascii=False)}')
        raise SystemExit(0)

    def _save(self, result: dict):
        if not self.output_dir:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        with open(os.path.join(self.output_dir, 'kidnap_samples.csv'), 'w', encoding='utf-8') as f:
            f.write('time,along_m,lateral_m,yaw_rad,match_ratio,amcl_std_m\n')
            for s, (ratio, std) in zip(self.samples, self.extra):
                f.write(f'{s.t:.3f},{s.along:.4f},{s.lateral:.4f},{s.yaw:.4f},{ratio:.3f},{std:.3f}\n')
        with open(os.path.join(self.output_dir, 'kidnap_states.csv'), 'w', encoding='utf-8') as f:
            f.write('time,state\n')
            for t, s in self.states:
                f.write(f'{t:.3f},{s}\n')
        # 판정 파일은 마지막에 쓴다 (하네스가 이 파일이 생기기를 기다린다)
        path = os.path.join(self.output_dir, 'kidnap.json')
        with open(path + '.tmp', 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        os.replace(path + '.tmp', path)


def main(args=None):
    rclpy.init(args=args)
    node = KidnapTrialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        if node.phase != DONE:
            # 시험이 끝나기 전에 멈췄다 (하네스 주행 시간 초과 등). 판정 없이 어디까지 갔는지만 남긴다
            node._save({'mode': node.mode, 'seed': node.seed, 'error': f'시험이 끝나기 전에 멈춤 ({node.phase})'})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
