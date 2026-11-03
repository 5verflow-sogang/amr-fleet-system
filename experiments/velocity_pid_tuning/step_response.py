"""속도 계단 응답 1회 측정: 직선 경로를 내고 path_tracker_node(velocity_pid 포함)가 실제로
Gazebo DiffDrive를 통해 0 -> desired_speed로 움직이는 과정을 GT로 기록한다.

    python3 step_response.py --out <결과 폴더> --mass 10.0 --label default
        [--ns amr_01] [--frame amr_01/odom] [--start-x 0 --start-y 1.5] [--target 0.5]

run_one.sh가 sim.launch.py + path_tracker_node를 이미 띄운 뒤 이 스크립트를 부른다
(cte_baseline/path_publisher.py와 같은 역할 분리). path_tracker_node는 max_linear_accel을
크게 줘서(run_one.sh) 추종기 자체의 가속 램프가 PID 계단 응답을 가리지 않게 한다 - 그래야
odom으로 들어오는 속도 변화가 (추종기 램프가 아니라) PID + Gazebo DiffDrive 플랜트의 응답이다.

GT 속도는 plugin Twist 필드(이동평균이라 늦게 반응, payload_braking README 참고)가 아니라
위치를 직접 미분해서 쓴다. 측정 윈도(--duration, 시뮬레이션 시각 기준) 동안 샘플을 모은 뒤
오버슈트·정착 시간(2% 밴드)·정상상태 오차를 계산해 <out>/raw_<label>_<mass>kg.csv에 샘플을,
results.csv에 한 행을 (append) 남긴다.
"""

import argparse
import csv
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

SETTLE_BAND = 0.02  # 목표 대비 ±2%
SS_WINDOW_S = 1.0  # 정상상태 오차를 잴 마지막 구간 [s]


class StepHarness(Node):

    def __init__(self, args):
        super().__init__('velocity_pid_step_harness')
        self.args = args
        self.pub = self.create_publisher(Path, f'/{args.ns}/plan', 10)
        # 깊이를 넉넉히 둔다 - Python 콜백 처리가 잠깐 늦어져도(GIL·컨테이너 CPU 경합)
        # 과도기(가속 구간) 메시지가 큐에서 밀려 사라지지 않게(얕은 큐 때문에 "이미 정착한" 뒤의
        # 메시지만 보고 계단 응답을 놓치는 버그를 겪었다)
        self.create_subscription(Odometry, f'/{args.ns}/ground_truth/odom', self._on_gt, 2000)
        self.samples = []  # [(t, v)], t는 경로를 낸 시각 기준 상대 시간
        self._prev = None  # (t, x, y)
        self._t0 = None
        self.sent = 0
        self.create_timer(0.5, self._publish_path)

    def _publish_path(self):
        if self.sent >= 3:
            return
        msg = Path()
        msg.header.frame_id = self.args.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(401):  # 20 m, 0.05 m 간격 - 측정 윈도 안에서는 절대 끝에 안 닿는다
            p = PoseStamped()
            p.header.frame_id = self.args.frame
            p.pose.position.x = i * 0.05
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub.publish(msg)
        if self.sent == 0:
            self._t0 = None  # 첫 발행 시각을 기준으로 다시 잡는다(그 전 GT는 버림)
        self.sent += 1

    def _on_gt(self, msg: Odometry):
        if self.sent == 0:
            return  # 경로를 아직 안 냈다 - 추종기가 아직 설 상태
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._t0 is None:
            self._t0 = t
            self._prev = (t, x, y)
            return
        pt, px, py = self._prev
        dt = t - pt
        if dt > 1e-6:
            v = math.hypot(x - px, y - py) / dt
            self.samples.append((t - self._t0, v))
        self._prev = (t, x, y)


def analyze(samples, target):
    if not samples:
        return {'peak_v': 0.0, 'overshoot_pct': 0.0, 'settling_time_s': -1.0, 'ss_error': target}
    peak_v = max(v for _, v in samples)
    overshoot_pct = max(0.0, (peak_v - target) / target * 100.0) if target > 0 else 0.0

    band = SETTLE_BAND * target
    settling_time_s = -1.0
    for i, (t, v) in enumerate(samples):
        if abs(v - target) <= band:
            if all(abs(v2 - target) <= band for _, v2 in samples[i:]):
                settling_time_s = t
                break

    t_end = samples[-1][0]
    tail = [v for t, v in samples if t >= t_end - SS_WINDOW_S]
    ss_error = (sum(tail) / len(tail) - target) if tail else target
    return {
        'peak_v': round(peak_v, 4),
        'overshoot_pct': round(overshoot_pct, 2),
        'settling_time_s': round(settling_time_s, 3) if settling_time_s >= 0 else -1.0,
        'ss_error': round(ss_error, 4),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--ns', default='amr_01')
    parser.add_argument('--frame', default='amr_01/odom')
    parser.add_argument('--mass', type=float, required=True)
    parser.add_argument('--label', required=True, help='게인 세트 이름 (results.csv 식별용)')
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=0.0)
    parser.add_argument('--kd', type=float, default=0.0)
    parser.add_argument('--enabled', type=str, default='false')
    parser.add_argument('--target', type=float, default=0.5)
    parser.add_argument('--duration', type=float, default=6.0, help='측정 윈도 [s] (시뮬레이션 시각)')
    parser.add_argument('--timeout', type=float, default=120.0, help='실제 시간 안전망 [s]')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = StepHarness(args)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.samples and node.samples[-1][0] >= args.duration:
            break
    node.destroy_node()
    rclpy.shutdown()

    raw_path = os.path.join(args.out, f'raw_{args.label}_{args.mass:g}kg.csv')
    with open(raw_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'v'])
        w.writerows([[f'{t:.4f}', f'{v:.5f}'] for t, v in node.samples] if node.samples else [])

    metrics = analyze(node.samples, args.target)
    print(f'[{args.label}] mass={args.mass}kg n={len(node.samples)} {metrics}', flush=True)

    results_csv = os.path.join(args.out, 'results.csv')
    is_new = not os.path.exists(results_csv)
    with open(results_csv, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'label', 'mass_kg', 'enabled', 'kp', 'ki', 'kd', 'target_mps',
            'peak_v', 'overshoot_pct', 'settling_time_s', 'ss_error', 'n_samples'])
        if is_new:
            writer.writeheader()
        writer.writerow({
            'label': args.label, 'mass_kg': args.mass, 'enabled': args.enabled,
            'kp': args.kp, 'ki': args.ki, 'kd': args.kd, 'target_mps': args.target,
            'n_samples': len(node.samples), **metrics,
        })
    return 0 if node.samples else 1


if __name__ == '__main__':
    raise SystemExit(main())
