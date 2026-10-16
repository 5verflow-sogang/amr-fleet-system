"""task_auction_node.py의 BID 재전송(#155·#171)을 노드 메서드를 직접 불러서 확인하는
가벼운 rclpy 단위 테스트다 - launch_testing 없이, test_zone_client_node.py와 같은 방식.
"first BID 유실 -> 재전송으로 구제"·"중복 BID 안 세기"는 ROS 비의존인 auction/쪽
(test_auction.py)에서 이미 확인했으므로, 여기서는 "AWARD를 받으면 재전송이 멈추는지"만
노드 상태로 확인한다.

동시 낙찰 경합 테스트는 노드 여러 개를 한 프로세스에 만들고, 발행을 가짜 버스(_Bus)로 바꿔
모든 노드에 차례로 넘긴다 - DDS 디스커버리·타이밍 없이 메시지 순서를 정해 놓고 본다.
시계(_now)도 가짜로 바꿔 입찰 마감을 직접 넘긴다.
"""

import rclpy
from amr_msgs.msg import RobotState, TaskAuction, TaskBid, TaskStatus

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


# --- 동시 낙찰 경합 ---


class _Bus:
    """노드들의 발행을 가로채 모든 노드의 콜백에 차례로 넘긴다(자기 자신 포함 - 실제 토픽과 같음)."""

    def __init__(self):
        self.nodes = []
        self.queue = []
        self.log = []

    def publisher(self):
        bus = self

        class _Pub:
            def publish(self, msg):
                bus.queue.append(msg)
        return _Pub()

    def attach(self, node, clock):
        node._bid_pub = self.publisher()
        node._auction_pub = self.publisher()
        node._now = lambda: clock[0]
        self.nodes.append(node)

    def publish(self, msg):
        self.queue.append(msg)
        self.pump()

    def pump(self):
        while self.queue:
            msg = self.queue.pop(0)
            self.log.append(msg)
            for node in self.nodes:
                if isinstance(msg, TaskAuction):
                    node._on_task_auction(msg)
                else:
                    node._on_task_bid(msg)

    def bids(self, type_):
        return [m for m in self.log if isinstance(m, TaskBid) and m.type == type_]


def _make_node(robot_id, bus, clock, xy, mode):
    node = TaskAuctionNode()
    node._robot_id = robot_id
    bus.attach(node, clock)
    node._on_robot_state(RobotState(
        robot_id=robot_id, mode=mode, battery_percent=80.0,
        pose=_pose(*xy)))
    return node


def _pose(x, y):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose


def _run_concurrent_auctions(robots):
    """경매인 amr_10·amr_11(BUSY, 입찰 안 함)이 작업 2개를 동시에 공고하고 마감까지 진행한다.
    robots: [(robot_id, xy)]. 두 작업 모두 amr_01(3,0)이 가장 가깝다(거리 4), amr_02(0,0)는 5."""
    bus = _Bus()
    clock = [100.0]
    nodes = {
        'amr_10': _make_node('amr_10', bus, clock, (20.0, 20.0), RobotState.MODE_BUSY),
        'amr_11': _make_node('amr_11', bus, clock, (20.0, 20.0), RobotState.MODE_BUSY),
    }
    for robot_id, xy in robots:
        nodes[robot_id] = _make_node(robot_id, bus, clock, xy, RobotState.MODE_IDLE)

    auction_a = _make_auction('amr_10', 1, 'task_a')
    auction_a.task.dropoff_pose.position.x, auction_a.task.dropoff_pose.position.y = 3.0, 4.0
    auction_b = _make_auction('amr_11', 1, 'task_b')
    auction_b.task.dropoff_pose.position.x, auction_b.task.dropoff_pose.position.y = 3.0, -4.0
    bus.publish(auction_a)
    bus.publish(auction_b)

    # 두 경매의 마감이 거의 같이 지나 AWARD 두 개가 연달아 나간다
    clock[0] += 2.1
    for node in nodes.values():
        node._on_timer()
    bus.pump()
    return bus, clock, nodes


def _accepted_robots_by_task(bus):
    """작업별로 ACCEPT를 보낸 로봇 - 중복 할당이면 한 작업에 둘 이상."""
    by_task = {}
    for msg in bus.bids(TaskBid.TYPE_ACCEPT):
        by_task.setdefault(msg.task_id, set()).add(msg.robot_id)
    return by_task


def test_concurrent_awards_same_robot_accepts_one_and_second_goes_to_runner_up():
    rclpy.init()
    nodes = {}
    try:
        bus, _, nodes = _run_concurrent_auctions([('amr_01', (3.0, 0.0)), ('amr_02', (0.0, 0.0))])

        # amr_01이 두 경매 모두 최저가로 낙찰됐다
        awards_to_01 = {m.task_id for m in bus.bids(TaskBid.TYPE_AWARD) if m.winner_id == 'amr_01'}
        assert awards_to_01 == {'task_a', 'task_b'}

        # 하나(먼저 받은 task_a)만 수락하고 다른 하나는 포기, 경매인은 2순위 amr_02에게 줬다
        assert [(m.task_id, m.robot_id) for m in bus.bids(TaskBid.TYPE_WITHDRAW)] == [
            ('task_b', 'amr_01')]
        state_a = nodes['amr_10']._auctions[('amr_10', 1)]
        state_b = nodes['amr_11']._auctions[('amr_11', 1)]
        assert (state_a.phase, state_a.winner_id) == ('done', 'amr_01')
        assert (state_b.phase, state_b.winner_id) == ('done', 'amr_02')
        assert nodes['amr_01']._active_task == (('amr_10', 1), 'task_a')
        assert nodes['amr_02']._active_task == (('amr_11', 1), 'task_b')
        # 재경매(새 auction_id)는 없었다
        assert [m for m in bus.log if isinstance(m, TaskAuction) and m.auction_id > 1] == []
    finally:
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()


def test_concurrent_awards_single_bidder_reauctions():
    rclpy.init()
    nodes = {}
    try:
        bus, clock, nodes = _run_concurrent_auctions([('amr_01', (3.0, 0.0))])

        # task_b는 입찰자가 amr_01 하나뿐 - 포기하자 새 auction_id(2)로 재경매한다
        reauctions = [m for m in bus.log if isinstance(m, TaskAuction) and m.auction_id > 1]
        assert [(m.auctioneer_id, m.auction_id, m.task.task_id) for m in reauctions] == [
            ('amr_11', 2, 'task_b')]
        assert ('amr_11', 1) not in nodes['amr_11']._auctions
        assert nodes['amr_11']._auctions[('amr_11', 2)].phase == 'collecting'
        # task_a를 맡은 amr_01은 재경매에 입찰하지 않는다(IDLE 로봇만 입찰)
        assert [m for m in bus.bids(TaskBid.TYPE_BID) if m.auction_id == 2] == []

        # 작업을 끝내면(내 TaskStatus COMPLETED) 다시 입찰할 수 있다
        status = TaskStatus(task_id='task_a', robot_id='amr_01', state=TaskStatus.STATE_COMPLETED)
        nodes['amr_01']._on_task_status(status)
        assert nodes['amr_01']._active_task is None
        later = _make_auction('amr_11', 3, 'task_c')
        bus.publish(later)
        assert [m.robot_id for m in bus.bids(TaskBid.TYPE_BID) if m.auction_id == 3] == ['amr_01']
    finally:
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()


def test_concurrent_awards_no_duplicate_assignment():
    rclpy.init()
    nodes = {}
    try:
        bus, clock, nodes = _run_concurrent_auctions([('amr_01', (3.0, 0.0)), ('amr_02', (0.0, 0.0))])

        # 경매인이 포기 전에 보낸 task_b AWARD가 늦게 다시 도착해도 amr_01은 다시 포기한다 -
        # task_a를 끝낸 뒤라도(맡은 작업이 없어도) 이미 2순위에게 넘어간 작업을 수락하지 않는다.
        nodes['amr_01']._on_task_status(
            TaskStatus(task_id='task_a', robot_id='amr_01', state=TaskStatus.STATE_COMPLETED))
        stale = TaskBid(
            auctioneer_id='amr_11', auction_id=1, task_id='task_b', robot_id='amr_11',
            type=TaskBid.TYPE_AWARD, winner_id='amr_01')
        bus.publish(stale)
        # AWARD 재전송 주기를 여러 번 넘겨도 확정된 경매는 다시 안 보낸다
        for _ in range(6):
            clock[0] += 1.0
            for node in nodes.values():
                node._on_timer()
            bus.pump()

        by_task = _accepted_robots_by_task(bus)
        assert by_task == {'task_a': {'amr_01'}, 'task_b': {'amr_02'}}
        # 로봇별로도 맡은 작업은 하나씩
        winners = [
            state.winner_id for node in nodes.values() for state in node._auctions.values()
            if state.phase == 'done']
        assert sorted(winners) == ['amr_01', 'amr_02']
        assert nodes['amr_11']._auctions[('amr_11', 1)].winner_id == 'amr_02'
    finally:
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()
