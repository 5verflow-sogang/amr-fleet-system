"""분산 모드 통합: 가짜 로봇 5대 + 로봇별 경매 노드, 서버 없이 작업 60건 -> 모두 할당·완료, 중복 할당 0.

amr_fleet_manager의 가짜 로봇(fake_robot)을 relayed 구독(server_topic_prefix: 'fleet' ->
/<robot_id>/fleet/task_assignments)으로 띄우고, 같은 네임스페이스에 task_auction_node를 띄운다.
Fleet 서버·장애 주입기·작업 생성기는 안 띄운다 - 이 테스트가 /fleet/task_requests에 작업을 직접
방송한다(작업 생성기는 서버 구독을 기다려 일정을 시작하므로 서버 없는 분산 모드에서는 바로 못 쓴다).

시나리오 파일(tools/harness/scenarios)은 건드리지 않는다. 가짜 로봇은 빠르게(20 m/s, 대기 0.1초)
돌려 60건을 1분 안에 끝낸다 - 할당·완료·중복 여부만 보는 테스트라 이동 시간은 상관없다.

재는 것: 작업마다 TaskAssignment를 받은 로봇(중계본 토픽 5개), TaskStatus COMPLETED, 첫 공고
(TaskAuction) -> ACCEPT(TaskBid) 응답 시간(header.stamp 차, 같은 기계라 같은 시계).

ROS_DOMAIN_ID는 이 파일 전용 값으로 둔다(test_task_auction_launch.py와 같은 이유, 값은 다르게).
"""

import random
import statistics
import time
import unittest
import zlib

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from amr_msgs.msg import RobotState, Task, TaskAssignment, TaskAuction, TaskBid, TaskStatus
from launch.actions import SetEnvironmentVariable
from rclpy.qos import QoSProfile, ReliabilityPolicy

_TEST_ROS_DOMAIN_ID = str(zlib.crc32(b'amr_fleet_decentralized/test_auction_fleet') % 102)

ROBOT_IDS = [f'amr_{i:02d}' for i in range(1, 6)]
TASK_COUNT = 60
TASK_INTERVAL_S = 0.25


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


@pytest.mark.launch_test
def generate_test_description():
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', _TEST_ROS_DOMAIN_ID)]
    for i, robot_id in enumerate(ROBOT_IDS):
        actions.append(launch_ros.actions.Node(
            package='amr_fleet_manager', executable='fake_robot', namespace=robot_id,
            parameters=[{
                'robot_id': robot_id, 'server_topic_prefix': 'fleet',
                'start_xy': [2.0 + 2.0 * i, 2.0], 'speed_mps': 20.0, 'dwell_s': 0.1,
                'state_rate_hz': 5.0, 'step_hz': 20.0,
            }]))
        actions.append(launch_ros.actions.Node(
            package='amr_fleet_decentralized', executable='task_auction_node', namespace=robot_id,
            parameters=[{
                'robot_id': robot_id, 'bid_window_s': 0.5, 'update_rate_hz': 20.0,
                'no_bid_retry_s': 0.5, 'alive_timeout_s': 3.0,
            }]))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


class TestDecentralizedFleet(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auction_fleet')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def test_sixty_tasks_all_assigned_and_completed_without_duplicates(self):
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        robots_seen = set()
        assigned = {}       # task_id -> {robot_id}
        assignment_ids = []
        completed = {}      # task_id -> robot_id
        first_announce = {}
        first_accept = {}

        def on_state(msg):
            robots_seen.add(msg.robot_id)

        def on_assignment(msg):
            if msg.action == TaskAssignment.ACTION_ASSIGN:
                assigned.setdefault(msg.task.task_id, set()).add(msg.robot_id)
                assignment_ids.append((msg.robot_id, msg.assignment_id))

        def on_status(msg):
            if msg.state == TaskStatus.STATE_COMPLETED:
                completed.setdefault(msg.task_id, msg.robot_id)

        def on_auction(msg):
            first_announce.setdefault(msg.task.task_id, _stamp(msg))

        def on_bid(msg):
            if msg.type == TaskBid.TYPE_ACCEPT:
                first_accept.setdefault(msg.task_id, _stamp(msg))

        self.node.create_subscription(RobotState, '/fleet/robot_states', on_state, qos)
        for robot_id in ROBOT_IDS:
            self.node.create_subscription(
                TaskAssignment, f'/{robot_id}/fleet/task_assignments', on_assignment, qos)
        self.node.create_subscription(TaskStatus, '/fleet/task_status', on_status, qos)
        self.node.create_subscription(TaskAuction, '/fleet/task_auctions', on_auction, qos)
        self.node.create_subscription(TaskBid, '/fleet/task_bids', on_bid, qos)
        task_pub = self.node.create_publisher(Task, '/fleet/task_requests', qos)

        # 가짜 로봇 5대의 RobotState가 돌고, 경매 노드 5개가 작업 방송을 구독할 때까지
        self.assertTrue(self.spin_until(
            lambda: robots_seen >= set(ROBOT_IDS) and task_pub.get_subscription_count() >= 5, 60.0),
            f'준비 안 됨: 로봇 {sorted(robots_seen)}, 작업 방송 구독 {task_pub.get_subscription_count()}')
        # 경매 노드들이 서로 같은 "살아 있는 로봇" 목록을 갖도록 RobotState 몇 주기를 더 기다린다
        self.spin_until(lambda: False, 2.0)

        rng = random.Random(7)
        for i in range(TASK_COUNT):
            task = Task()
            task.task_id = f'task_{i:04d}'
            task.task_type = Task.TYPE_DELIVERY
            task.pickup_station_id = 'dock_1'
            task.pickup_pose.position.x = rng.uniform(0.0, 12.0)
            task.pickup_pose.position.y = rng.uniform(0.0, 6.0)
            task.dropoff_station_id = 'rack_1'
            task.dropoff_pose.position.x = rng.uniform(0.0, 12.0)
            task.dropoff_pose.position.y = rng.uniform(0.0, 6.0)
            task_pub.publish(task)
            self.spin_until(lambda: False, TASK_INTERVAL_S)

        task_ids = {f'task_{i:04d}' for i in range(TASK_COUNT)}
        self.spin_until(lambda: set(completed) >= task_ids, 120.0)

        duplicates = {t: r for t, r in assigned.items() if len(r) > 1}
        response = [first_accept[t] - first_announce[t] for t in task_ids
                    if t in first_accept and t in first_announce]
        print(
            f'[분산 통합] 할당 {len(set(assigned) & task_ids)}/{TASK_COUNT}, '
            f'완료 {len(set(completed) & task_ids)}/{TASK_COUNT}, 중복 할당 {len(duplicates)}, '
            f'응답 시간(공고->ACCEPT) 평균 {statistics.mean(response):.3f}s '
            f'중앙값 {statistics.median(response):.3f}s 최대 {max(response):.3f}s (n={len(response)}), '
            f'로봇별 완료 {sorted((r, list(completed.values()).count(r)) for r in ROBOT_IDS)}',
            flush=True)

        self.assertEqual(set(assigned) & task_ids, task_ids, f'할당 안 된 작업: {sorted(task_ids - set(assigned))}')
        self.assertEqual(duplicates, {}, '같은 작업이 두 로봇에게 할당됨')
        self.assertEqual(set(completed) & task_ids, task_ids, f'완료 안 된 작업: {sorted(task_ids - set(completed))}')
        # 경매 쪽 번호 공간(1,000,000 + 로봇 번호 x 100,000 + 순번)이고 로봇별로 겹치지 않는다
        for robot_id, assignment_id in assignment_ids:
            index = int(robot_id[-2:])
            self.assertEqual(assignment_id // 100_000, 10 + index, (robot_id, assignment_id))
        self.assertEqual(len(response), TASK_COUNT)
