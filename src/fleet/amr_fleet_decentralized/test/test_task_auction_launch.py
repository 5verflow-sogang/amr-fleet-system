"""task_auction_node을 실제로 띄워 경매 전체 흐름·토픽 접두어·실패 재시도를 확인한다
(launch_testing).

amr_01(경매인)·amr_02(낙찰자)는 기본값(절대 경로 `/fleet/task_auctions`,
`/fleet/task_bids`)을 그대로 쓴다 - relay_auction 없이 두 노드가 직접 구독·발행한다.
amr_03은 `auction_topic_prefix: 'fleet'`(= `relayed:=true`와 동일)로 띄워, 상대
이름(장애 주입기 중계본과 같은 네임스페이스 하위 경로)만 구독하고 절대 경로는
무시하는지 따로 확인한다.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import RobotState, TaskAuction, TaskBid, TaskStatus


@pytest.mark.launch_test
def generate_test_description():
    common_params = {
        'bid_window_s': 1.0,
        'update_rate_hz': 20.0,
        'min_battery_percent': 20.0,
    }
    amr_01 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_01', parameters=[{**common_params, 'robot_id': 'amr_01'}])
    amr_02 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_02', parameters=[{**common_params, 'robot_id': 'amr_02'}])
    # relayed:=true와 동일(상대 이름 구독) - 절대 경로 기본값과 다르게 동작하는지 따로 본다.
    amr_03 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_03',
        parameters=[{**common_params, 'robot_id': 'amr_03', 'auction_topic_prefix': 'fleet'}])
    return launch.LaunchDescription([
        amr_01, amr_02, amr_03, launch_testing.actions.ReadyToTest(),
    ])


class TestTaskAuctionLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_task_auction_live')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def publish_robot_state(self, pub, robot_id: str, x: float, y: float, battery: float):
        msg = RobotState()
        msg.robot_id = robot_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.mode = RobotState.MODE_IDLE
        msg.battery_percent = battery
        pub.publish(msg)

    def make_auction(self, auctioneer_id: str, auction_id: int, task_id: str,
                      pickup_xy=(10.0, 0.0)):
        auction = TaskAuction()
        auction.header.stamp = self.node.get_clock().now().to_msg()
        auction.auctioneer_id = auctioneer_id
        auction.auction_id = auction_id
        auction.task.task_id = task_id
        auction.task.pickup_station_id = 'station_a'
        auction.task.pickup_pose.position.x = pickup_xy[0]
        auction.task.pickup_pose.position.y = pickup_xy[1]
        auction.task.dropoff_station_id = 'station_b'
        auction.task.dropoff_pose.position.x = 20.0
        auction.task.dropoff_pose.position.y = 0.0
        auction.bid_window.sec = 1
        return auction

    def test_auction_bid_award_accept_default_absolute_topics(self):
        # amr_01·amr_02는 파라미터를 안 바꿨다 - 기본값(절대 경로 원본)을 직접
        # 구독·발행하는지 이 전체 흐름이 통과하는 것 자체로 확인한다.
        auction_pub = self.node.create_publisher(TaskAuction, '/fleet/task_auctions', 10)
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        status_pub = self.node.create_publisher(TaskStatus, '/fleet/task_status', 10)
        bids = []
        auctions = []
        self.node.create_subscription(TaskBid, '/fleet/task_bids', bids.append, 10)
        self.node.create_subscription(TaskAuction, '/fleet/task_auctions', auctions.append, 10)

        self.assertTrue(self.spin_until(
            lambda: auction_pub.get_subscription_count() >= 2
            and state_pub.get_subscription_count() >= 2,
            15.0))

        # amr_01은 pickup(10, 0)에서 멀고(비용 10), amr_02는 가깝다(비용 1) - amr_02가
        # 이겨야 한다. 두 로봇 상태를 주기적으로 계속 흘려준다(노드가 최신 상태를 쓴다).
        def refresh_states():
            self.publish_robot_state(state_pub, 'amr_01', x=0.0, y=0.0, battery=80.0)
            self.publish_robot_state(state_pub, 'amr_02', x=9.0, y=0.0, battery=80.0)

        refresh_states()
        rclpy.spin_once(self.node, timeout_sec=0.1)

        auction = self.make_auction('amr_01', auction_id=1, task_id='task_1')

        end = time.monotonic() + 3.0
        while time.monotonic() < end and not bids:
            refresh_states()
            auction_pub.publish(auction)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(bids, 'TaskBid가 전혀 발행되지 않음')

        def bid_from(robot_id, msg_type, task_id=None):
            return [
                b for b in bids if b.robot_id == robot_id and b.type == msg_type
                and (task_id is None or b.task_id == task_id)
            ]

        def has_bid_from_both():
            refresh_states()
            return bid_from('amr_01', TaskBid.TYPE_BID) and bid_from('amr_02', TaskBid.TYPE_BID)
        self.assertTrue(
            self.spin_until(has_bid_from_both, 5.0),
            f'둘 다 입찰하지 않음: {[(b.robot_id, b.type, b.cost) for b in bids]}')

        amr_02_bid = bid_from('amr_02', TaskBid.TYPE_BID)[0]
        amr_01_bid = bid_from('amr_01', TaskBid.TYPE_BID)[0]
        self.assertAlmostEqual(amr_02_bid.cost, 1.0, places=3)
        self.assertAlmostEqual(amr_01_bid.cost, 10.0, places=3)

        def awarded_to_amr_02():
            refresh_states()
            awards = bid_from('amr_01', TaskBid.TYPE_AWARD)
            return any(a.winner_id == 'amr_02' for a in awards)
        self.assertTrue(
            self.spin_until(awarded_to_amr_02, 8.0),
            f'amr_02에게 낙찰되지 않음: {[(b.robot_id, b.type, b.winner_id) for b in bids]}')

        def accepted_by_amr_02():
            refresh_states()
            return bid_from('amr_02', TaskBid.TYPE_ACCEPT)
        self.assertTrue(
            self.spin_until(accepted_by_amr_02, 5.0),
            f'amr_02가 수락하지 않음: {[(b.robot_id, b.type) for b in bids]}')

        # --- ACCEPT 뒤 TaskStatus FAILED: 1회(기본 failed_retries=1)는 재경매 ---

        def report_failed():
            msg = TaskStatus()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.task_id = 'task_1'
            msg.robot_id = 'amr_02'
            msg.state = TaskStatus.STATE_FAILED
            status_pub.publish(msg)

        def reauctioned_once():
            refresh_states()
            report_failed()
            return [a for a in auctions if a.task.task_id == 'task_1' and a.auction_id != 1]
        self.assertTrue(
            self.spin_until(reauctioned_once, 5.0),
            'FAILED 보고 뒤 재경매(새 auction_id)가 발행되지 않음')
        second_auction_id = max(
            a.auction_id for a in auctions if a.task.task_id == 'task_1' and a.auction_id != 1)

        # 재경매된 두 번째 경매도 같은 조건이라 amr_02가 다시 낙찰·수락해야 한다.
        def accepted_second_round():
            refresh_states()
            return [
                b for b in bids if b.robot_id == 'amr_02' and b.type == TaskBid.TYPE_ACCEPT
                and b.auction_id == second_auction_id
            ]
        self.assertTrue(
            self.spin_until(accepted_second_round, 8.0),
            f'재경매(auction_id={second_auction_id}) 뒤 amr_02가 수락하지 않음: '
            f'{[(b.robot_id, b.type, b.auction_id) for b in bids]}')

        # --- 두 번째 FAILED는 failed_retries(=1)를 다 썼으니 더 재경매하지 않고 실패로 남김 ---

        before_count = len([a for a in auctions if a.task.task_id == 'task_1'])

        def no_third_auction():
            refresh_states()
            report_failed()
            rclpy.spin_once(self.node, timeout_sec=0.1)
            return False  # 끝까지 기다렸다가 아래에서 직접 판정한다
        self.spin_until(no_third_auction, 3.0)
        after_count = len([a for a in auctions if a.task.task_id == 'task_1'])
        self.assertEqual(
            before_count, after_count,
            f'두 번째 FAILED 뒤에도 재경매가 발행됨(실패로 남아야 함): '
            f'{[a.auction_id for a in auctions if a.task.task_id == "task_1"]}')

    def test_relayed_prefix_subscribes_relative_not_absolute(self):
        # amr_03은 auction_topic_prefix='fleet'(relayed:=true와 동일)로 떴다 -
        # 절대 경로 /fleet/task_auctions는 무시하고, 자기 네임스페이스 밑
        # fleet/task_auctions(중계본과 같은 상대 이름)만 구독해야 한다.
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        absolute_auction_pub = self.node.create_publisher(TaskAuction, '/fleet/task_auctions', 10)
        relative_auction_pub = self.node.create_publisher(
            TaskAuction, '/amr_03/fleet/task_auctions', 10)
        bids = []
        self.node.create_subscription(TaskBid, '/fleet/task_bids', bids.append, 10)

        def refresh_state():
            self.publish_robot_state(state_pub, 'amr_03', x=9.0, y=0.0, battery=80.0)

        self.assertTrue(self.spin_until(
            lambda: relative_auction_pub.get_subscription_count() >= 1, 15.0))
        refresh_state()
        rclpy.spin_once(self.node, timeout_sec=0.1)

        def bid_from_amr_03(task_id):
            return [
                b for b in bids if b.robot_id == 'amr_03' and b.type == TaskBid.TYPE_BID
                and b.task_id == task_id
            ]

        # 절대 경로로 보낸 공고에는 반응하지 않아야 한다(상대 이름만 구독하므로).
        absolute_auction = self.make_auction(
            'amr_relay_probe', auction_id=1, task_id='absolute_probe')
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            refresh_state()
            absolute_auction_pub.publish(absolute_auction)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertFalse(
            bid_from_amr_03('absolute_probe'),
            '절대 경로 공고에 amr_03이 반응함(relayed 접두어가 적용되지 않음)')

        # 상대 이름(자기 네임스페이스 하위, 중계본과 같은 경로)으로 보낸 공고에는 반응해야 한다.
        relative_auction = self.make_auction(
            'amr_relay_probe', auction_id=2, task_id='relay_task')

        def got_relayed_bid():
            refresh_state()
            relative_auction_pub.publish(relative_auction)
            return bid_from_amr_03('relay_task')
        self.assertTrue(
            self.spin_until(got_relayed_bid, 5.0),
            '상대 이름(중계본) 공고에 amr_03이 반응하지 않음')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
