"""추적 결과(perception/obstacles)를 Gazebo 정답(GT)과 비교해 정확도·동적 판정 지연을 기록한다 (평가 전용).

    ros2 run amr_tracking tracking_eval --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
        -p duration_s:=300.0 -p out_dir:=/tmp/amr_eval/tracking_run1

구독 (상대 이름은 로봇 네임스페이스 아래):
    perception/obstacles                       amr_msgs/TrackedObstacleArray  트랙 (odom 좌표계)
    ground_truth/odom                          nav_msgs/Odometry  로봇 GT (world → base_footprint, twist는 몸체 기준)
    /dynamic_obstacles/<id>/ground_truth/odom  nav_msgs/Odometry  장애물 GT (world, twist는 몸체 기준)
    /tf                                         odom → base_footprint (트랙 시각 값)

트랙은 odom, GT는 world 좌표계다. 프레임마다 같은 시각의 로봇 GT 자세(world → base)와
TF(odom → base)로 world → odom을 구해 트랙을 world로 옮긴다: T_world_odom = T_world_base · T_odom_base⁻¹.
odom이 world에서 천천히 미끄러져도(휠 오도메트리 누적 오차) 프레임마다 다시 구하므로 상쇄된다.
로봇 GT 자세·속도와 트랙 반경도 넘겨, 가짜 동적 트랙 때문에 늘어난 감속·정지를 안전 노드 규칙으로 잰다.
로봇 속도는 GT다 (추적 노드 TTC는 odom 속도를 쓰지만 시뮬레이터에서는 거의 같다).

트랙 메시지를 바로 처리하지 않고 큐에 넣었다가, 그 시각 뒤의 로봇 GT가 들어오면(보간) 처리한다.
duration_s가 지나거나(첫 트랙 프레임 기준, 시뮬레이션 시간) 종료(Ctrl+C)하면 out_dir에 저장한다:
    frames.csv     프레임별 짝(match)·놓침(miss, 센서 범위 안)·짝 없는 트랙(track_only)
    episodes.csv   GT 물체별 "움직이며 센서 범위 안" 구간과 동적 판정 지연
    summary.json   지표 요약 (amr_perception_core.tracking_eval)
지표 정의는 docs/algorithms/perception.md 1-7절.
"""

import csv
import json
import math
import os
import signal
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from amr_msgs.msg import TrackedObstacleArray
from amr_perception_core.tracking_eval import (
    EvalConfig,
    format_summary_ko,
    TimeSeries,
    TrackingEvaluator,
    TrackObs,
    world_from_odom,
)

# amr_gazebo config/dynamic_obstacles.yaml을 못 읽을 때 쓰는 id (같은 파일의 값)
DEFAULT_OBSTACLE_IDS = ['worker_1', 'worker_2', 'forklift_1', 'forklift_2', 'robot_1']
# 로봇 GT 보간 시 두 샘플 사이 최대 간격 [s] (GT 100 Hz, best effort라 몇 개 빠져도 보간)
ROBOT_GT_MAX_GAP = 0.2


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def obstacle_ids_from_gazebo_config() -> list:
    """amr_gazebo의 dynamic_obstacles.yaml에서 id 목록을 읽는다. 실패하면 DEFAULT_OBSTACLE_IDS."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        path = os.path.join(get_package_share_directory('amr_gazebo'), 'config',
                            'dynamic_obstacles.yaml')
        with open(path, encoding='utf-8') as f:
            ids = [str(o['id']) for o in yaml.safe_load(f)['obstacles']]
        return ids or list(DEFAULT_OBSTACLE_IDS)
    except Exception:  # noqa: BLE001 패키지·yaml이 없어도 기본 id로 돈다
        return list(DEFAULT_OBSTACLE_IDS)


def track_from_msg(obstacle) -> TrackObs:
    """TrackedObstacle → TrackObs (메시지 좌표계 그대로). 공분산은 6x6에서 xy 블록만."""
    p, v = obstacle.pose.pose.position, obstacle.twist.twist.linear
    pc = np.asarray(obstacle.pose.covariance, dtype=float).reshape(6, 6)[:2, :2]
    vc = np.asarray(obstacle.twist.covariance, dtype=float).reshape(6, 6)[:2, :2]
    return TrackObs(int(obstacle.track_id), float(p.x), float(p.y), float(v.x), float(v.y),
                    bool(obstacle.is_dynamic),
                    pc if np.any(pc) else None, vc if np.any(vc) else None, float(obstacle.radius))


def robot_velocity_in_world(sample) -> tuple:
    """로봇 GT 샘플 (x, y, yaw, 몸체 vx, 몸체 vy) → world 축 속도 (vx, vy)."""
    c, s = math.cos(sample[2]), math.sin(sample[2])
    return (c * sample[3] - s * sample[4], s * sample[3] + c * sample[4])


def write_csv(path: str, rows: list) -> None:
    keys = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=keys or ['empty'])
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ('' if row.get(k) is None else
                                 (f'{row[k]:.4f}' if isinstance(row[k], float) else row[k]))
                             for k in keys})


class TrackingEvalNode(Node):

    def __init__(self):
        super().__init__('tracking_eval')
        p = self.declare_parameter
        ns = self.get_namespace().strip('/')
        ids = [s.strip() for s in p('obstacle_ids', '').value.split(',') if s.strip()]
        self.obstacle_ids = ids or obstacle_ids_from_gazebo_config()
        gt_format = p('obstacle_gt_topic_format',
                      '/dynamic_obstacles/{id}/ground_truth/odom').value
        base_frame = p('base_frame', '').value
        self.base_frame = base_frame or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self.duration_s = float(p('duration_s', 0.0).value)
        out = p('out_dir', '').value
        self.out_dir = out or os.path.join(
            '/tmp/amr_eval', f"tracking_{ns or 'robot'}_{time.strftime('%Y%m%d-%H%M%S')}")
        # 큐에 넣은 트랙 프레임을 이 시간 [s, wall] 안에 GT·TF로 처리하지 못하면 버린다
        self.pending_timeout = float(p('pending_timeout', 3.0).value)
        self.config = EvalConfig(
            gate=p('gate', 1.0).value,
            moving_speed=p('moving_speed', 0.1).value,
            sensor_range=p('sensor_range', 12.0).value,
            horizons=tuple(float(h) for h in p('horizons', [1.0, 2.0]).value),
            max_gt_gap=p('max_gt_gap', 0.2).value,
            heading_min_speed=p('heading_min_speed', 0.2).value,
            episode_merge_gap=p('episode_merge_gap', 1.0).value,
        )
        self.evaluator = TrackingEvaluator(self.config, self.obstacle_ids)
        self.robot_gt = TimeSeries(angle_columns=(2,))
        self.pending = []           # (t, 받은 wall 시각, msg)
        self.first_t = None
        self._last_t = None
        self.done = False
        self.saved = False
        self.counts = {'msgs': 0, 'frames': 0, 'no_robot_gt': 0, 'tf_fail': 0, 'frame_world': 0}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # best effort 구독은 reliable·best effort 발행자 모두와 호환된다 (Gazebo 브리지)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TrackedObstacleArray,
                                 p('tracks_topic', 'perception/obstacles').value,
                                 self.on_tracks, qos)
        self.create_subscription(Odometry, p('robot_gt_topic', 'ground_truth/odom').value,
                                 self.on_robot_gt, qos)
        for gid in self.obstacle_ids:
            self.create_subscription(Odometry, gt_format.format(id=gid),
                                     lambda msg, gid=gid: self.on_obstacle_gt(gid, msg), qos)
        self.create_timer(0.1, self.process_pending)
        self.create_timer(10.0, self.log_counts)
        self.get_logger().info(
            f'추적 평가 시작: 장애물 {self.obstacle_ids}, base {self.base_frame}, '
            f'duration {self.duration_s or "종료까지"} s, 저장 {self.out_dir}')

    # ------------------------------------------------------------------ 입력
    def on_robot_gt(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self.robot_gt.add(stamp_to_sec(msg.header.stamp), (pos.x, pos.y, yaw, v.x, v.y))

    def on_obstacle_gt(self, gid: str, msg: Odometry):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        # twist는 몸체 기준 (amr_gazebo README) → world 축
        bx, by = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        c, s = math.cos(yaw), math.sin(yaw)
        self.evaluator.add_gt(gid, stamp_to_sec(msg.header.stamp),
                              pose.position.x, pose.position.y,
                              c * bx - s * by, s * bx + c * by)

    def on_tracks(self, msg: TrackedObstacleArray):
        if self.done:
            return
        self.counts['msgs'] += 1
        self.pending.append((stamp_to_sec(msg.header.stamp), time.monotonic(), msg))

    # ------------------------------------------------------------------ 처리
    def process_pending(self):
        keep = []
        for item in self.pending:
            if not self._process(*item):
                if time.monotonic() - item[1] < self.pending_timeout:
                    keep.append(item)
        self.pending = keep
        if (self.duration_s > 0.0 and self.first_t is not None and self.evaluator.num_frames
                and self._last_t - self.first_t >= self.duration_s):
            self.get_logger().info(f'{self.duration_s} s 기록 완료')
            self.done = True

    def _process(self, t: float, received: float, msg) -> bool:
        """처리했거나 버릴 프레임이면 True, GT·TF를 더 기다려야 하면 False."""
        expired = time.monotonic() - received >= self.pending_timeout
        last_gt = self.robot_gt.last_time
        if last_gt is None or last_gt < t:
            if expired:
                self.counts['no_robot_gt'] += 1
            return expired
        robot = self.robot_gt.at(t, ROBOT_GT_MAX_GAP)
        if robot is None:
            self.counts['no_robot_gt'] += 1
            return True
        frame = msg.header.frame_id
        tracks = [track_from_msg(o) for o in msg.obstacles]
        if frame == 'world':
            self.counts['frame_world'] += 1
        else:
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame,
                                                     Time.from_msg(msg.header.stamp))
            except TransformException:
                if expired:
                    self.counts['tf_fail'] += 1
                return expired
            tr, q = tf.transform.translation, tf.transform.rotation
            world_odom = world_from_odom(tuple(robot[:3]), (tr.x, tr.y, yaw_from_quaternion(q)))
            tracks = [track.transformed(world_odom) for track in tracks]
        self.evaluator.add_frame(t, tracks, robot_xy=(robot[0], robot[1]), robot_yaw=robot[2],
                                 robot_velocity=robot_velocity_in_world(robot))
        self.counts['frames'] += 1
        if self.first_t is None:
            self.first_t = t
        self._last_t = t
        return True

    def log_counts(self):
        c = self.counts
        gt = ', '.join(f'{gid} {self.evaluator.gt_count(gid)}' for gid in self.obstacle_ids)
        self.get_logger().info(
            f'트랙 메시지 {c["msgs"]}, 처리 프레임 {c["frames"]} (대기 {len(self.pending)}, '
            f'로봇 GT 없음 {c["no_robot_gt"]}, TF 실패 {c["tf_fail"]}), 로봇 GT {len(self.robot_gt)}, '
            f'장애물 GT: {gt}')
        missing = [gid for gid in self.obstacle_ids if self.evaluator.gt_count(gid) == 0]
        if missing and len(self.robot_gt):
            self.get_logger().warn(f'GT가 안 들어오는 장애물: {missing} (dynamic_obstacles:=true인지 확인)',
                                   throttle_duration_sec=30.0)

    # ------------------------------------------------------------------ 저장
    def save(self) -> None:
        if self.saved:
            return
        self.saved = True
        for item in self.pending:           # 남은 프레임도 처리할 수 있으면 처리
            self._process(*item)
        self.pending = []
        rows, episodes, summary = self.evaluator.evaluate()
        summary['input'] = dict(self.counts, obstacle_ids=self.obstacle_ids,
                                robot_gt_samples=len(self.robot_gt),
                                obstacle_gt_samples={gid: self.evaluator.gt_count(gid)
                                                     for gid in self.obstacle_ids})
        os.makedirs(self.out_dir, exist_ok=True)
        write_csv(os.path.join(self.out_dir, 'frames.csv'), rows)
        write_csv(os.path.join(self.out_dir, 'episodes.csv'), episodes)
        with open(os.path.join(self.out_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        text = format_summary_ko(summary)
        print(text, flush=True)
        self.get_logger().info(f'저장: {self.out_dir} (frames.csv, episodes.csv, summary.json)')


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvalNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C(SIGINT)와 launch·docker stop(SIGTERM) 모두 저장하고 끝낸다
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.save()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
