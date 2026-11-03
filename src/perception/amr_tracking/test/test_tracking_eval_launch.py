"""추적 KPI 측정 도구: tracking_eval 노드를 띄우고 합성 GT·TF·트랙을 넣어 summary.json을 확인한다 (launch_testing).

GCP 본 측정의 모든 결과가 이 노드에 기대므로, 좌표 맞추기와 저장까지 노드 그대로 돌려 본다 (Gazebo 없이).
- odom은 world에서 (5, −3, 0.7 rad)에 있고, 로봇은 odom (1.0, 0.5, 0.3 rad)에서 0.5 m/s로 직진한다.
  평가 노드는 프레임마다 로봇 GT(world)와 TF(odom → base)로 world → odom을 다시 구해야 한다
- 작업자 1명: 로봇 출발 자세의 앞 2 m·왼쪽 3 m에서 로봇과 같은 방향으로 1.0 m/s. 트랙은 GT를 odom으로 옮긴 값 그대로
- 가짜 동적 트랙 1개: 2.0~2.5 s 동안 로봇 앞(차체 거리 0.45 m 이하)에 world에서 멈춰 있다(반경 0.3 m).
  로봇이 다가가므로 상대 속도로 TTC ≤ 0.9 s → 안전 노드 규칙으로 정지. 로봇 속도·반경을 노드가 제대로 넘겨야 나온다

확인: 작업자 위치·속도 오차 ≈ 0 (좌표 변환), 가짜 때문에 정지 1번 이상, 가짜 행의 차체 거리·TTC가 장면 값 그대로,
저장, 정상 종료.
"""

import csv
import json
import math
import os
import tempfile
import time
import unittest

from geometry_msgs.msg import TransformStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from tf2_ros import TransformBroadcaster

from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from amr_perception_core.tracking_eval import se2_compose, se2_inverse

# 다른 패키지 테스트와 병렬로 돌아도 토픽·TF 프레임이 겹치지 않게 전용 네임스페이스·GT 토픽을 쓴다
NS = 'teval_amr'
GT_FORMAT = '/teval_obstacles/{id}/ground_truth/odom'
OUT_DIR = tempfile.mkdtemp(prefix='tracking_eval_launch_')
DURATION_S = 5.0                    # 평가 노드 기록 시간 (첫 트랙 프레임부터)
GT_RATE = 50.0                      # Hz. 로봇·작업자 GT와 odom → base TF
TRACK_RATE = 10.0                   # Hz. 트랙 (실제 추적 주기)

WORLD_ODOM = (5.0, -3.0, 0.7)       # world에서 본 odom 원점
ODOM_START = (1.0, 0.5, 0.3)        # odom에서 본 로봇 출발 자세
ROBOT_SPEED = 0.5                   # m/s, 몸체 앞으로
WORKER_START = (2.0, 3.0)           # 로봇 출발 몸체 좌표 (앞, 왼쪽)
WORKER_SPEED = 1.0                  # m/s, 로봇과 같은 방향
WORKER_RADIUS = 0.25
FRAGMENT_BODY = (1.05, 0.0)         # FRAGMENT_T[0]의 몸체 좌표. 차체 거리 1.05 − 0.3 − 0.3 = 0.45 m
FRAGMENT_T = (2.0, 2.5)             # 가짜가 보이는 시간 [s] (장면 시각)
FRAGMENT_RADIUS = 0.3
HALF_LENGTH = 0.3                   # 차체 반길이 (safety.yaml robot_length 0.6)


def odom_base(t):
    x, y, th = ODOM_START
    return (x + ROBOT_SPEED * t * math.cos(th), y + ROBOT_SPEED * t * math.sin(th), th)


def world_base(t):
    return se2_compose(WORLD_ODOM, odom_base(t))


def worker_world(t):
    """작업자 (x, y, yaw). yaw 방향으로 걷는다."""
    x, y, th = se2_compose(world_base(0.0), (WORKER_START[0], WORKER_START[1], 0.0))
    return (x + WORKER_SPEED * t * math.cos(th), y + WORKER_SPEED * t * math.sin(th), th)


FRAGMENT_WORLD = se2_compose(world_base(FRAGMENT_T[0]), (FRAGMENT_BODY[0], FRAGMENT_BODY[1], 0.0))


def to_odom(x, y, vx, vy):
    """world 위치·속도 → odom (트랙 메시지 좌표계)."""
    ox, oy, _ = se2_compose(se2_inverse(WORLD_ODOM), (x, y, 0.0))
    c, s = math.cos(-WORLD_ODOM[2]), math.sin(-WORLD_ODOM[2])
    return ox, oy, c * vx - s * vy, s * vx + c * vy


def odometry(stamp, frame, child, pose, body_speed):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.child_frame_id = child
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(pose[0]), float(pose[1])
    msg.pose.pose.orientation.z = math.sin(0.5 * pose[2])
    msg.pose.pose.orientation.w = math.cos(0.5 * pose[2])
    msg.twist.twist.linear.x = float(body_speed)       # GT twist는 몸체 기준 (amr_gazebo README)
    return msg


def tracked(track_id, x, y, vx, vy, radius):
    o = TrackedObstacle()
    o.track_id = track_id
    o.is_dynamic = True
    o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
    o.pose.pose.orientation.w = 1.0
    o.twist.twist.linear.x, o.twist.twist.linear.y = float(vx), float(vy)
    o.radius = float(radius)
    return o


@pytest.mark.launch_test
def generate_test_description():
    evaluator = launch_ros.actions.Node(
        package='amr_tracking',
        executable='tracking_eval',
        name='tracking_eval',
        namespace=NS,
        parameters=[{'use_sim_time': False, 'duration_s': DURATION_S, 'out_dir': OUT_DIR,
                     'obstacle_ids': 'w1', 'obstacle_gt_topic_format': GT_FORMAT}],
        output='screen',
    )
    return launch.LaunchDescription([evaluator, launch_testing.actions.ReadyToTest()])


class TestTrackingEvalSynthetic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_tracking_eval_launch')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_evaluator(self, topic):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and self.node.count_subscribers(topic) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreater(self.node.count_subscribers(topic), 0, f'평가 노드가 {topic}을 구독하지 않는다')

    def test_summary_from_synthetic_scene(self):
        tracks_topic = f'/{NS}/perception/obstacles'
        track_pub = self.node.create_publisher(TrackedObstacleArray, tracks_topic, 10)
        robot_pub = self.node.create_publisher(Odometry, f'/{NS}/ground_truth/odom', 10)
        worker_pub = self.node.create_publisher(Odometry, GT_FORMAT.format(id='w1'), 10)
        tf_broadcaster = TransformBroadcaster(self.node)
        # 장면 시각은 평가 노드가 뜬 뒤부터 잰다 (CI에서 노드가 늦게 떠도 가짜가 기록 안에 들어오게)
        self._wait_for_evaluator(tracks_topic)
        t0 = self.node.get_clock().now().nanoseconds * 1e-9

        def scene_time(now):
            return now.nanoseconds * 1e-9 - t0

        def publish_gt_and_tf():
            now = self.node.get_clock().now()
            t, stamp = scene_time(now), now.to_msg()
            robot_pub.publish(odometry(stamp, 'world', f'{NS}/base_footprint', world_base(t),
                                       ROBOT_SPEED))
            worker_pub.publish(odometry(stamp, 'world', 'w1/base_footprint', worker_world(t),
                                        WORKER_SPEED))
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = f'{NS}/odom'
            tf.child_frame_id = f'{NS}/base_footprint'
            x, y, th = odom_base(t)
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(0.5 * th), math.cos(0.5 * th)
            tf_broadcaster.sendTransform(tf)

        def publish_tracks():
            now = self.node.get_clock().now()
            t = scene_time(now)
            msg = TrackedObstacleArray()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = f'{NS}/odom'
            wx, wy, wth = worker_world(t)
            msg.obstacles.append(tracked(1, *to_odom(wx, wy, WORKER_SPEED * math.cos(wth),
                                                     WORKER_SPEED * math.sin(wth)), WORKER_RADIUS))
            if FRAGMENT_T[0] <= t < FRAGMENT_T[1]:
                msg.obstacles.append(tracked(2, *to_odom(FRAGMENT_WORLD[0], FRAGMENT_WORLD[1], 0.0, 0.0),
                                             FRAGMENT_RADIUS))
            track_pub.publish(msg)

        self.node.create_timer(1.0 / GT_RATE, publish_gt_and_tf)
        self.node.create_timer(1.0 / TRACK_RATE, publish_tracks)

        # 기록이 끝나면 평가 노드가 summary.json을 쓰고 끝난다. 쓰는 도중이면 읽기가 실패하므로 다시 읽는다
        path = os.path.join(OUT_DIR, 'summary.json')
        summary = None
        deadline = time.monotonic() + DURATION_S + 30.0
        while time.monotonic() < deadline and summary is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if os.path.exists(path):
                try:
                    with open(path, encoding='utf-8') as f:
                        summary = json.load(f)
                except ValueError:
                    pass
        self.assertIsNotNone(summary, f'{path}가 저장되지 않았다')
        for name in ('frames.csv', 'episodes.csv'):
            self.assertTrue(os.path.exists(os.path.join(OUT_DIR, name)), name)

        self.assertGreaterEqual(summary['frames'], 30, summary['input'])
        self.assertEqual(summary['input']['tf_fail'], 0)
        # 좌표 맞추기: 트랙은 GT를 odom으로 옮긴 값 그대로라 world로 되돌리면 오차가 없다
        worker = summary['per_object']['w1']
        self.assertGreaterEqual(worker['match_rate_in_range'], 0.9)
        self.assertLess(worker['pos']['rmse'], 0.01)
        self.assertLess(worker['vel']['rmse'], 0.01)
        # 가짜 동적 트랙 때문에 정지: 로봇 속도(GT, world 축)와 트랙 반경이 넘어와야 TTC ≤ 1 s가 된다
        fs = summary['false_dynamic_safety']
        self.assertIsNotNone(fs, '로봇 자세·속도가 평가에 넘어오지 않았다')
        self.assertGreaterEqual(fs['frames'], 30)
        self.assertGreaterEqual(fs['stop_events'], 1, fs)
        self.assertGreaterEqual(fs['stop_s'], 0.5, fs)               # 단계를 내리기 전 0.5 s 유지
        self.assertTrue(0.15 < fs['false_min_clearance'] <= 0.46, fs)
        self.assertTrue(0.3 < fs['false_min_ttc'] <= 0.91, fs)
        # 가짜 행마다 차체 거리·TTC가 장면 값 그대로다. 로봇 방향·속도, 트랙 반경 중 하나라도 틀리면 어긋난다
        # (정면이라 방향이 틀려도 정지는 그대로 나와서 위 확인만으로는 못 잡는다)
        with open(os.path.join(OUT_DIR, 'frames.csv'), encoding='utf-8') as f:
            rows = [r for r in csv.DictReader(f) if r['kind'] == 'track_only' and r['track_id'] == '2']
        self.assertGreaterEqual(len(rows), 2)
        for row in rows:
            dt = float(row['t']) - t0 - FRAGMENT_T[0]
            expected = FRAGMENT_BODY[0] - ROBOT_SPEED * dt - HALF_LENGTH - FRAGMENT_RADIUS
            self.assertAlmostEqual(float(row['clearance']), expected, delta=0.002, msg=row)
            self.assertAlmostEqual(float(row['ttc']), expected / ROBOT_SPEED, delta=0.002, msg=row)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
