"""task_auction_node을 실제로 띄워 경매 전체 흐름·토픽 접두어·실패 재시도를 확인한다
(launch_testing).

amr_01(경매인)·amr_02(낙찰자)는 기본값(절대 경로 `/fleet/task_auctions`,
`/fleet/task_bids`)을 그대로 쓴다 - relay_auction 없이 두 노드가 직접 구독·발행한다.
amr_03은 `auction_topic_prefix: 'fleet'`(= `relayed:=true`와 동일)로 띄워, 상대
이름(장애 주입기 중계본과 같은 네임스페이스 하위 경로)만 구독하고 절대 경로는
무시하는지 따로 확인한다. amr_04는 "내 상태(RobotState)보다 공고가 먼저 오는" 회귀
시나리오 전용이다(10/10 연우 리뷰, #109) - 다른 테스트가 먼저 상태를 흘려주지 않게
독립된 네임스페이스를 쓴다.

ROS_DOMAIN_ID를 이 패키지 전용 값으로 고정한다 - CI에서 다른 패키지의 launch
테스트(예: amr_fleet_manager)와 같은 도메인에서 동시에 돌면 `/fleet/*` 토픽이
섞여 양쪽 다 흔들린다(연우가 #109 코멘트에서 보고한 CI 간헐적 실패 원인). 이 파일만
격리하고 다른 패키지 테스트는 그대로 둔다.
"""

import time
import unittest
import zlib

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import RobotState, TaskAuction, TaskBid, TaskStatus
from launch.actions import SetEnvironmentVariable

# 패키지 이름으로부터 고정된 도메인 ID를 끌어낸다(실행마다 같은 값, 다른 패키지가
# 같은 방식을 써도 패키지 이름이 다르면 거의 항상 다른 값) - 리눅스에서 안전한
# 범위 0~101(포트 계산이 65535를 안 넘는 범위) 안으로 맞춘다.
_TEST_ROS_DOMAIN_ID = str(zlib.crc32(b'amr_fleet_decentralized') % 102)


@pytest.mark.launch_test
def generate_test_description():
    common_params = {
        'bid_window_s': 1.0,
        'update_rate_hz': 20.0,
        'min_battery_percent': 20.0,
        # 넉넉하게 둬서(기본 5회=5초 대신) 테스트가 ACCEPT를 확인하는 동안 AWARD
        # 재전송이 먼저 다 돼 버려 엉뚱한 재경매(FAILED 재시도와 무관)가 끼어들지
        # 않게 한다 - 안 그러면 10/10 반복 실행 검증에서 가끔 재경매 횟수 집계가
        # 어긋났다(연우 리뷰, #109).
        'award_max_attempts': 50,
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
    # 공고가 RobotState보다 먼저 오는 회귀 시나리오 전용 - 다른 테스트와 섞이지 않게 분리.
    amr_04 = launch_ros.actions.Node(
        package='amr_fleet_decentralized', executable='task_auction_node',
        namespace='amr_04', parameters=[{**common_params, 'robot_id': 'amr_04'}])
    return launch.LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', _TEST_ROS_DOMAIN_ID),
        amr_01, amr_02, amr_03, amr_04, launch_testing.actions.ReadyToTest(),
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
                      pickup_xy=(10.0, 0.0), bid_window_s: int = 1):
        # header.stamp는 여기서 안 찍는다 - 호출 쪽이 재전송 루프에 들어가기 직전(처음 실제
        # publish 바로 전)에 찍는다. 여기서 찍으면 그 뒤로 몇 초씩 같은 메시지 객체를 그대로
        # 반복 publish하는 호출부(마감 판정과는 무관하지만, stamp는 "언제 보냈는지"를
        # 뜻해야 한다) 전부에서 stamp가 생성 시점에 멈춰 버린다 (#155·#171 CI).
        auction = TaskAuction()
        auction.auctioneer_id = auctioneer_id
        auction.auction_id = auction_id
        auction.task.task_id = task_id
        auction.task.pickup_station_id = 'station_a'
        auction.task.pickup_pose.position.x = pickup_xy[0]
        auction.task.pickup_pose.position.y = pickup_xy[1]
        auction.task.dropoff_station_id = 'station_b'
        auction.task.dropoff_pose.position.x = 20.0
        auction.task.dropoff_pose.position.y = 0.0
        auction.bid_window.sec = bid_window_s
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

        # amr_04도 기본값(절대 경로)을 쓰므로 /fleet/task_auctions 구독자는 amr_01·02·04
        # 셋이다 - 2만 보면 amr_02가 아직 디스커버리 전인데도(amr_01·04만 잡혀도) 통과해
        # 버려서, 그 상태로 공고를 먼저 보내면 amr_02가 입찰 마감을 놓칠 수 있었다(10/10
        # 반복 실행 검증, #109). 셋·넷 다 잡힌 뒤에 시작한다.
        self.assertTrue(self.spin_until(
            lambda: auction_pub.get_subscription_count() >= 3
            and state_pub.get_subscription_count() >= 4,
            15.0))

        # amr_01은 pickup(10, 0)에서 멀고(비용 10), amr_02는 가깝다(비용 1) - amr_02가
        # 이겨야 한다. 두 로봇 상태를 주기적으로 계속 흘려준다(노드가 최신 상태를 쓴다).
        def refresh_states():
            self.publish_robot_state(state_pub, 'amr_01', x=0.0, y=0.0, battery=80.0)
            self.publish_robot_state(state_pub, 'amr_02', x=9.0, y=0.0, battery=80.0)

        refresh_states()
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # bid_window를 넉넉히(3초) 둔다 - amr_02가 RobotState보다 공고를 먼저 받아
        # 입찰을 미뤘다가(_pending_bids) 다시 보는 경우, 반복 실행 중 CPU 경쟁으로
        # 재시도가 늦어져도 마감을 못 맞추는 일이 없게 한다(10/10 반복 실행 검증, #109).
        auction = self.make_auction('amr_01', auction_id=1, task_id='task_1', bid_window_s=3)
        auction.header.stamp = self.node.get_clock().now().to_msg()  # 처음 publish 바로 전에 찍는다

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
            # 다른 패키지 launch 테스트와 같은 도메인에서 겹쳐 돌 때(CI) 상태가 공고보다
            # 늦게 도착하면 로봇이 처음엔 입찰을 못 한다 - 기다리는 동안 공고도 다시
            # 보내서, 늦게 상태를 받은 로봇도 그제서야 볼 수 있게 한다(10/10 연우 리뷰, #109).
            auction_pub.publish(auction)
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
        # auction_id를 1로 못박지 않고 그때그때 "이미 본 것" 집합과 비교해서 새로
        # 생긴 것만 본다 - AWARD 재전송 다 써서 생기는 재경매(이 테스트가 보는
        # failed_retries와는 무관) 등 예상 밖의 경매가 먼저 섞여도 안정적으로
        # 통과하게 한다(10/10 반복 실행 검증, #109).

        def task_1_auction_ids():
            return {a.auction_id for a in auctions if a.task.task_id == 'task_1'}

        def report_failed():
            msg = TaskStatus()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.task_id = 'task_1'
            msg.robot_id = 'amr_02'
            msg.state = TaskStatus.STATE_FAILED
            status_pub.publish(msg)

        ids_before_first_failure = task_1_auction_ids()

        def reauctioned_once():
            refresh_states()
            report_failed()
            return task_1_auction_ids() - ids_before_first_failure
        self.assertTrue(
            self.spin_until(reauctioned_once, 5.0),
            'FAILED 보고 뒤 재경매(새 auction_id)가 발행되지 않음')
        second_auction_id = max(task_1_auction_ids() - ids_before_first_failure)

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

        ids_before_second_failure = task_1_auction_ids()

        def no_third_auction():
            refresh_states()
            report_failed()
            rclpy.spin_once(self.node, timeout_sec=0.1)
            return False  # 끝까지 기다렸다가 아래에서 직접 판정한다
        self.spin_until(no_third_auction, 3.0)
        new_ids = task_1_auction_ids() - ids_before_second_failure
        self.assertFalse(
            new_ids,
            f'두 번째 FAILED 뒤에도 재경매가 발행됨(실패로 남아야 함): new_ids={new_ids}')

    def test_winner_lost_reauctioned(self):
        # ACCEPT한 낙찰 로봇(amr_02)의 RobotState가 reclaim_timeout_s(기본 5초) 끊기면 경매인(amr_01)이
        # 새 auction_id로 재경매한다 - 중앙 #132의 작업 회수와 같은 규칙
        auction_pub = self.node.create_publisher(TaskAuction, '/fleet/task_auctions', 10)
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        bids, auctions = [], []
        self.node.create_subscription(TaskBid, '/fleet/task_bids', bids.append, 10)
        self.node.create_subscription(TaskAuction, '/fleet/task_auctions', auctions.append, 10)
        self.assertTrue(self.spin_until(
            lambda: auction_pub.get_subscription_count() >= 3
            and state_pub.get_subscription_count() >= 4,
            15.0))

        def amr_01_alive():
            self.publish_robot_state(state_pub, 'amr_01', x=0.0, y=0.0, battery=80.0)

        def both_alive():
            amr_01_alive()
            self.publish_robot_state(state_pub, 'amr_02', x=9.0, y=0.0, battery=80.0)

        # bid_window를 5초로 넉넉히 둔다 - 경매인(amr_01)의 마감 판정은 amr_01이 이 공고를
        # 처음 받은 자기 시각(announce_time) 기준이라, amr_02가 공고를 받고 입찰을 보내
        # amr_01이 그 입찰을 받기까지의 왕복이 CI에서 CPU 경쟁으로 늘어지면 2초로는
        # 간헐적으로 못 맞췄다(#155·#171) - amr_02가 입찰만 하고 정작 amr_01 자기
        # 입찰로 낙찰되는 증상이었다. 기본값(yaml)은 그대로 두고 이 테스트만 넉넉히 둔다.
        auction = self.make_auction('amr_01', auction_id=100, task_id='task_reclaim', bid_window_s=5)
        auction.header.stamp = self.node.get_clock().now().to_msg()  # 처음 publish 바로 전에 찍는다

        def accepted_by_amr_02():
            both_alive()
            auction_pub.publish(auction)
            return [b for b in bids if b.robot_id == 'amr_02' and b.type == TaskBid.TYPE_ACCEPT
                    and b.task_id == 'task_reclaim']
        self.assertTrue(
            self.spin_until(accepted_by_amr_02, 10.0),
            f'amr_02가 수락하지 않음: {[(b.robot_id, b.type, b.task_id) for b in bids]}')

        # 이제 amr_02 상태를 끊는다 (amr_01 것만 계속 보낸다)
        silent_since = time.monotonic()

        def reauctioned():
            amr_01_alive()
            return [a for a in auctions if a.task.task_id == 'task_reclaim' and a.auction_id != 100]
        self.assertTrue(
            self.spin_until(reauctioned, 10.0),
            '낙찰 로봇 상태가 끊겼는데 재경매(새 auction_id)가 발행되지 않음')
        # 끊긴 지 reclaim_timeout_s(5초) 전에 회수하면 안 된다 (마지막 상태가 끊기기 직전에 갔으므로 약간 여유)
        self.assertGreaterEqual(time.monotonic() - silent_since, 4.5)

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
        absolute_auction.header.stamp = self.node.get_clock().now().to_msg()
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
        relative_auction.header.stamp = self.node.get_clock().now().to_msg()

        def got_relayed_bid():
            refresh_state()
            relative_auction_pub.publish(relative_auction)
            return bid_from_amr_03('relay_task')
        self.assertTrue(
            self.spin_until(got_relayed_bid, 5.0),
            '상대 이름(중계본) 공고에 amr_03이 반응하지 않음')

    def test_bid_deferred_until_robot_state_arrives(self):
        # amr_04는 이 테스트 전용 - 공고가 내 RobotState보다 먼저 와도(기동 직후 등)
        # 입찰 결정을 미뤄 뒀다가, 상태를 처음 받으면 마감 전인 것만 다시 판단해야
        # 한다(10/10 연우 리뷰, #109) - 그러지 않으면 그 경매에 영원히 입찰을 못 한다.
        auction_pub = self.node.create_publisher(TaskAuction, '/fleet/task_auctions', 10)
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        bids = []
        self.node.create_subscription(TaskBid, '/fleet/task_bids', bids.append, 10)

        self.assertTrue(self.spin_until(
            lambda: auction_pub.get_subscription_count() >= 3, 15.0))

        def bid_from_amr_04(task_id):
            return [
                b for b in bids if b.robot_id == 'amr_04' and b.type == TaskBid.TYPE_BID
                and b.task_id == task_id
            ]

        # amr_04의 RobotState를 아직 한 번도 보내지 않은 채로 공고를 먼저 보낸다
        # (bid_window를 5초로 넉넉히 둬 아래 지연 뒤에도 마감 전이게 한다).
        auction = self.make_auction(
            'amr_relay_probe', auction_id=1, task_id='deferred_task', bid_window_s=5)
        auction.header.stamp = self.node.get_clock().now().to_msg()
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            auction_pub.publish(auction)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertFalse(
            bid_from_amr_04('deferred_task'),
            '아직 RobotState도 안 보냈는데 amr_04가 입찰함')

        # 이제서야 amr_04의 상태를 처음 보낸다 - 미뤄 둔 공고를 다시 보고 입찰해야 한다.
        def got_deferred_bid():
            self.publish_robot_state(state_pub, 'amr_04', x=9.0, y=0.0, battery=80.0)
            return bid_from_amr_04('deferred_task')
        self.assertTrue(
            self.spin_until(got_deferred_bid, 5.0),
            'RobotState를 처음 받은 뒤에도 미뤄 둔 공고에 입찰하지 않음')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
