"""task_auction_node.py의 BID 재전송(#155·#171)을 노드 메서드를 직접 불러서 확인하는
가벼운 rclpy 단위 테스트다 - launch_testing 없이, test_zone_client_node.py와 같은 방식.
"first BID 유실 -> 재전송으로 구제"·"중복 BID 안 세기"는 ROS 비의존인 auction/쪽
(test_auction.py)에서 이미 확인했으므로, 여기서는 "AWARD를 받으면 재전송이 멈추는지"만
노드 상태로 확인한다.
"""

import rclpy
from amr_msgs.msg import RobotState, TaskAuction, TaskBid

from amr_fleet_decentralized.task_auction_node import TaskAuctionNode


def _make_auction(auctioneer_id: str, auction_id: int, task_id: str, bid_window_s: float = 2.0):
    msg = TaskAuction()
    msg.auctioneer_id = auctioneer_id
    msg.auction_id = auction_id
    msg.task.task_id = task_id
    msg.task.dropoff_pose.position.x = 3.0
    msg.task.dropoff_pose.position.y = 4.0
    msg.bid_window.sec = int(bid_window_s)
    msg.bid_window.nanosec = int(round((bid_window_s - msg.bid_window.sec) * 1e9))
    return msg


def test_award_stops_bid_resend():
    rclpy.init()
    try:
        node = TaskAuctionNode()
        try:
            # amr_01(기본 robot_id)이 IDLE·배터리 충분 - 다른 로봇(amr_02)의 공고에 입찰한다
            node._on_robot_state(RobotState(
                robot_id='amr_01', mode=RobotState.MODE_IDLE, battery_percent=80.0))

            key = ('amr_02', 1)
            auction = _make_auction('amr_02', 1, 'task_1')
            node._on_task_auction(auction)

            # BID를 보냈고, 마감까지 재전송하려고 대기 중이다
            assert key in node._bid_resends

            award = TaskBid()
            award.auctioneer_id = 'amr_02'
            award.auction_id = 1
            award.task_id = 'task_1'
            award.type = TaskBid.TYPE_AWARD
            award.winner_id = 'amr_01'
            node._on_task_bid(award)

            # AWARD를 받았으니 더 이상 입찰을 재전송하지 않는다
            assert key not in node._bid_resends
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_bid_resend_entry_cleared_at_deadline():
    rclpy.init()
    try:
        node = TaskAuctionNode()
        try:
            node._on_robot_state(RobotState(
                robot_id='amr_01', mode=RobotState.MODE_IDLE, battery_percent=80.0))

            key = ('amr_02', 1)
            node._on_task_auction(_make_auction('amr_02', 1, 'task_1', bid_window_s=2.0))
            assert key in node._bid_resends

            # 마감(공고를 받은 시각 + 2.0s)이 지나면 재전송을 멈춘다(AWARD 없이도)
            deadline = node._bid_deadline[key]
            node._maybe_resend_bids(deadline + 0.01)
            assert key not in node._bid_resends
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
