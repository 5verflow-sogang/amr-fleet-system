"""task_auction_node 2대를 실제로 띄워 경매 전체 흐름을 확인한다 (launch_testing).

공고(TaskAuction) → 입찰(TaskBid BID, 둘 다 가능하면 둘 다) → 낙찰(TaskBid AWARD,
pickup에 더 가까운 로봇) → 수락(TaskBid ACCEPT) 순서로 메시지가 나오는지 본다.

장애 주입기(relay_auction) 없이 절대 경로 /fleet/task_auctions, /fleet/task_bids를
두 노드가 직접 구독·발행하게 한다(relay_auction을 끈 실험과 같은 설정, 파라미터
task_auctions_sub_topic·task_bids_sub_topic 참고) - 테스트에 중계 노드까지 띄울
필요가 없다.
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
from amr_msgs.msg import RobotState, TaskAuction, TaskBid


@pytest.mark.launch_test
def generate_test_description():
    common_params = {
        'bid_window_s': 1.0,
        'update_rate_hz': 20.0,
        'min_battery_percent': 20.0,
        # relay_auction을 끈 것과 같다 - 절대 경로를 그대로 구독한다.
        'task_auctions_sub_topic': '/fleet/task_auctions',
        'task_bids_sub_topic': '/fleet/task_bids',
    }
    amr_01 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_01', parameters=[{**common_params, 'robot_id': 'amr_01'}])
    amr_02 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_02', parameters=[{**common_params, 'robot_id': 'amr_02'}])
    return launch.LaunchDescription([
        amr_01, amr_02, launch_testing.actions.ReadyToTest(),
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

    def test_auction_bid_award_accept(self):
        auction_pub = self.node.create_publisher(TaskAuction, '/fleet/task_auctions', 10)
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        bids = []
        self.node.create_subscription(TaskBid, '/fleet/task_bids', bids.append, 10)

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

        auction = TaskAuction()
        auction.header.stamp = self.node.get_clock().now().to_msg()
        auction.auctioneer_id = 'amr_01'
        auction.auction_id = 1
        auction.task.task_id = 'task_1'
        auction.task.pickup_station_id = 'station_a'
        auction.task.pickup_pose.position.x = 10.0
        auction.task.pickup_pose.position.y = 0.0
        auction.task.dropoff_station_id = 'station_b'
        auction.task.dropoff_pose.position.x = 20.0
        auction.task.dropoff_pose.position.y = 0.0
        auction.bid_window.sec = 1

        end = time.monotonic() + 3.0
        while time.monotonic() < end and not bids:
            refresh_states()
            auction_pub.publish(auction)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(bids, 'TaskBid가 전혀 발행되지 않음')

        def bid_from(robot_id, msg_type):
            return [b for b in bids if b.robot_id == robot_id and b.type == msg_type]

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


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
