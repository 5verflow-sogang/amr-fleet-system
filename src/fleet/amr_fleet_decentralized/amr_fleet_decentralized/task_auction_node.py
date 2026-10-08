"""분산 경매(TaskAuction/TaskBid) 노드. 탐색 로직은 전부 auction/(ROS 비의존)에 위임하고,
이 노드는 메시지 <-> auction 타입 변환, 경매별 상태(진행 중/낙찰/완료) 관리, 재전송·재경매
타이밍만 맡는다 - amr_nav2_plugins::DwaController가 amr_planning_core에 위임하는 것과
같은 패턴.

구독(상대 이름, 장애 주입기 relay_auction 중계본 - 접두어 파라미터로 바꾸면 relay_auction을
끈 /fleet/... 원본을 직접 구독할 수도 있다):
    fleet/task_auctions   TaskAuction   -> 공고를 보고 입찰할지 정한다(decide_bid)
    fleet/task_bids       TaskBid       -> 내가 경매인인 입찰 수집, 내가 낙찰자인 AWARD, 수락·포기

발행(절대 경로, 중계되기 전 원본 - 장애 주입기가 여기서 읽어 로봇별로 다시 보낸다):
    /fleet/task_auctions  TaskAuction
    /fleet/task_bids      TaskBid

보낸 로봇 자기 메시지(중계 에코)는 robot_id로 걸러 무시한다 - 보낼 때 이미 처리했으므로
또 처리하면 중복이다.

이 노드는 "경매를 언제 새로 시작할지"(서버 장애·미할당 작업 감지)는 아직 안 다룬다 -
README '확인 필요' 절 참고. 재경매(AAWARD 재전송 다 썼는데 ACCEPT 없음)는 다룬다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from amr_msgs.msg import RobotState, Task, TaskAuction, TaskBid
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .auction import (
    AuctionParams,
    AuctionResult,
    Bid,
    RobotInfo,
    TaskInfo,
    award_exhausted,
    can_withdraw,
    decide_bid,
    decide_winner,
    should_resend_award,
)

AuctionKey = Tuple[str, int]  # (auctioneer_id, auction_id)


def _time_to_epoch(t: TimeMsg) -> Optional[float]:
    """Task.deadline 변환. "0이면 마감 없음"(Task.msg 주석)."""
    if t.sec == 0 and t.nanosec == 0:
        return None
    return t.sec + t.nanosec * 1e-9


def _epoch_to_time(epoch_s: float) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(epoch_s)
    msg.nanosec = int(round((epoch_s - msg.sec) * 1e9))
    return msg


def _duration_to_s(d: DurationMsg) -> float:
    return d.sec + d.nanosec * 1e-9


def _task_from_msg(task: Task) -> TaskInfo:
    # CHARGE·RETURN은 pickup을 비운다(Task.msg 주석) - pickup_station_id가 빈 문자열인지로 본다.
    pickup_xy = (
        (task.pickup_pose.position.x, task.pickup_pose.position.y)
        if task.pickup_station_id else None
    )
    dropoff_xy = (task.dropoff_pose.position.x, task.dropoff_pose.position.y)
    return TaskInfo(
        task_id=task.task_id, dropoff_xy=dropoff_xy, pickup_xy=pickup_xy,
        deadline=_time_to_epoch(task.deadline),
    )


@dataclass
class _AuctionState:
    """내가 경매인인 경매 1건의 진행 상태. phase: collecting(입찰 수집 중) ->
    awarded(낙찰 통보, ACCEPT 기다림) -> done(확정, 더 안 건드림)."""

    task: TaskInfo
    announce_time: float
    bid_window_s: float
    phase: str = 'collecting'
    bids: List[Bid] = field(default_factory=list)
    winner_id: Optional[str] = None
    winning_cost: Optional[float] = None
    award_attempts: int = 0
    award_last_sent: Optional[float] = None


class TaskAuctionNode(Node):

    def __init__(self):
        super().__init__('task_auction')
        p = self.declare_parameter
        default_robot_id = self.get_namespace().strip('/') or 'amr_01'
        self._robot_id = p('robot_id', default_robot_id).value

        robot_states_topic = p('robot_states_topic', '/fleet/robot_states').value
        task_auctions_sub_topic = p('task_auctions_sub_topic', 'fleet/task_auctions').value
        task_bids_sub_topic = p('task_bids_sub_topic', 'fleet/task_bids').value
        task_auctions_pub_topic = p('task_auctions_pub_topic', '/fleet/task_auctions').value
        task_bids_pub_topic = p('task_bids_pub_topic', '/fleet/task_bids').value
        update_rate_hz = p('update_rate_hz', 10.0).value

        self._params = AuctionParams(
            bid_window_s=p('bid_window_s', AuctionParams.bid_window_s).value,
            min_battery_percent=p('min_battery_percent', AuctionParams.min_battery_percent).value,
            robot_speed_mps=p('robot_speed_mps', AuctionParams.robot_speed_mps).value,
            award_resend_interval_s=p(
                'award_resend_interval_s', AuctionParams.award_resend_interval_s).value,
            award_max_attempts=p('award_max_attempts', AuctionParams.award_max_attempts).value,
        )

        self._robot_info: Optional[RobotInfo] = None
        # 내가 경매인인 경매들 (auctioneer_id는 항상 나 자신이라 auction_id만 키로 써도 되지만,
        # TaskBid 처리 쪽과 자료형을 맞추려고 (auctioneer_id, auction_id) 그대로 쓴다).
        self._auctions: Dict[AuctionKey, _AuctionState] = {}
        # 입찰 여부 판단을 한 번만 하려고(같은 공고를 중계·재구독 등으로 다시 받아도 중복 입찰 방지).
        self._bid_decided: set = set()
        # 입찰할 때 본 작업 내용 - AWARD를 받았을 때 재확인(decide_bid)하려면 필요하다
        # (TaskBid AWARD에는 task_id만 있고 좌표·마감이 없다).
        self._tasks_seen: Dict[AuctionKey, TaskInfo] = {}
        # 이미 ACCEPT를 보낸 경매 - WITHDRAW는 ACCEPT 전까지만(10/7 결정), 재전송된 AWARD에
        # 또 ACCEPT를 보내지 않는 데도 쓴다(멱등).
        self._accepted: set = set()
        self._next_reauction_id: Dict[str, int] = {}

        self.create_subscription(RobotState, robot_states_topic, self._on_robot_state, 10)
        self.create_subscription(TaskAuction, task_auctions_sub_topic, self._on_task_auction, 10)
        self.create_subscription(TaskBid, task_bids_sub_topic, self._on_task_bid, 10)
        self._auction_pub = self.create_publisher(TaskAuction, task_auctions_pub_topic, 10)
        self._bid_pub = self.create_publisher(TaskBid, task_bids_pub_topic, 10)
        self.create_timer(1.0 / update_rate_hz, self._on_timer)

        self.get_logger().info(
            f'task_auction({self._robot_id}): 구독 {task_auctions_sub_topic} (공고), '
            f'{task_bids_sub_topic} (입찰/낙찰/수락), {robot_states_topic} (내 상태) · '
            f'발행 {task_auctions_pub_topic}, {task_bids_pub_topic}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- 내 상태 ---

    def _on_robot_state(self, msg: RobotState):
        if msg.robot_id != self._robot_id:
            return
        self._robot_info = RobotInfo(
            robot_id=msg.robot_id,
            xy=(msg.pose.position.x, msg.pose.position.y),
            battery_percent=msg.battery_percent,
            mode=msg.mode,
        )

    # --- 참여 로봇 쪽: 공고를 보고 입찰할지 정한다 ---

    def _on_task_auction(self, msg: TaskAuction):
        key: AuctionKey = (msg.auctioneer_id, msg.auction_id)
        if msg.auctioneer_id == self._robot_id and key not in self._auctions:
            # 내가 경매인인 공고 - 직접 시작했든(_announce) 외부에서 내 이름으로 왔든(이
            # 노드는 아직 "언제 새로 경매를 열지"는 안 다룬다, README 참고) 똑같이
            # 입찰 수집을 시작한다. 이미 추적 중이면(재전송·중계 에코) 다시 안 만든다.
            self._auctions[key] = _AuctionState(
                task=_task_from_msg(msg.task), announce_time=self._now(),
                bid_window_s=_duration_to_s(msg.bid_window))

        self._tasks_seen[key] = _task_from_msg(msg.task)
        self._maybe_bid(key, msg)

    def _maybe_bid(self, key: AuctionKey, msg: TaskAuction):
        if key in self._bid_decided:
            return
        self._bid_decided.add(key)
        if self._robot_info is None:
            return
        task = self._tasks_seen[key]
        now = self._now()
        bid = decide_bid(self._robot_info, task, now, self._params)
        if bid is None:
            return

        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = msg.auctioneer_id
        out.auction_id = msg.auction_id
        out.task_id = bid.task_id
        out.robot_id = self._robot_id
        out.type = TaskBid.TYPE_BID
        out.cost = bid.cost
        out.estimated_finish_time = _epoch_to_time(bid.estimated_finish_time)
        self._bid_pub.publish(out)

        # 내 경매에 나도 입찰했으면, 중계 에코를 기다리지 않고(자기 메시지는 걸러 무시하므로
        # 에코로는 절대 등록되지 않는다) 바로 내 입찰 목록에 넣는다.
        if key in self._auctions:
            self._register_bid(key, bid, now)

    # --- 경매인 쪽: 입찰 수집·낙찰·수락/포기 처리 ---

    def _register_bid(self, key: AuctionKey, bid: Bid, received_time: float):
        state = self._auctions.get(key)
        if state is None or state.phase != 'collecting':
            return
        state.bids.append(bid)

    def _on_task_bid(self, msg: TaskBid):
        if msg.robot_id == self._robot_id:
            return  # 자기 메시지(중계 에코) - 보낼 때 이미 처리했다
        key: AuctionKey = (msg.auctioneer_id, msg.auction_id)

        if msg.type == TaskBid.TYPE_BID:
            if key not in self._auctions:
                return  # 내가 경매인이 아닌 경매의 입찰은 안 본다
            # 10/7 결정: 입찰 마감은 경매인(나)이 "받은" 시각 기준 - 보낸 로봇의 자기 신고
            # 시각(header.stamp)을 믿지 않고 내 수신 시각을 쓴다(types.Bid 참고).
            received_time = self._now()
            bid = Bid(
                robot_id=msg.robot_id, task_id=msg.task_id, cost=msg.cost,
                estimated_finish_time=_time_to_epoch(msg.estimated_finish_time) or 0.0,
                bid_time=received_time,
            )
            self._register_bid(key, bid, received_time)
        elif msg.type == TaskBid.TYPE_AWARD:
            if msg.winner_id == self._robot_id:
                self._on_awarded(key, msg)
        elif msg.type == TaskBid.TYPE_ACCEPT:
            self._on_accept(key, msg.robot_id)
        elif msg.type == TaskBid.TYPE_WITHDRAW:
            self._on_withdraw(key, msg.robot_id)

    def _on_awarded(self, key: AuctionKey, msg: TaskBid):
        if key in self._accepted:
            return  # 재전송된 AWARD - 이미 수락을 보냈다(멱등)
        task = self._tasks_seen.get(key)
        now = self._now()
        # 입찰했을 때와 사정이 바뀌었을 수 있으니(배터리·IDLE 여부) 다시 확인한다.
        still_eligible = (
            self._robot_info is not None and task is not None
            and decide_bid(self._robot_info, task, now, self._params) is not None
        )

        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = msg.auctioneer_id
        out.auction_id = msg.auction_id
        out.task_id = msg.task_id
        out.robot_id = self._robot_id

        if still_eligible:
            out.type = TaskBid.TYPE_ACCEPT
            self._accepted.add(key)
        elif can_withdraw(accepted=False):
            # 10/7 결정: WITHDRAW는 ACCEPT 전까지만 - 아직 ACCEPT를 안 보냈으니 가능하다.
            out.type = TaskBid.TYPE_WITHDRAW
        else:
            return
        self._bid_pub.publish(out)

    def _on_accept(self, key: AuctionKey, robot_id: str):
        state = self._auctions.get(key)
        if state is None or state.phase != 'awarded' or state.winner_id != robot_id:
            return
        state.phase = 'done'
        self.get_logger().info(f'경매 {key}: {robot_id} 수락, 작업 {state.task.task_id} 확정')

    def _on_withdraw(self, key: AuctionKey, robot_id: str):
        state = self._auctions.get(key)
        if state is None:
            return
        if state.phase == 'collecting':
            state.bids = [b for b in state.bids if b.robot_id != robot_id]
        elif state.phase == 'awarded' and state.winner_id == robot_id:
            self.get_logger().warn(f'경매 {key}: 낙찰자 {robot_id} 포기 - 재경매')
            self._reauction(key, state)

    # --- 주기 처리: 마감 판정, AWARD 재전송·재경매 ---

    def _on_timer(self):
        now = self._now()
        for key, state in list(self._auctions.items()):
            if state.phase == 'collecting':
                self._maybe_decide_winner(key, state, now)
            elif state.phase == 'awarded':
                self._maybe_resend_or_reauction(key, state, now)

    def _maybe_decide_winner(self, key: AuctionKey, state: _AuctionState, now: float):
        if now < state.announce_time + state.bid_window_s:
            return
        result: AuctionResult = decide_winner(
            state.task.task_id, state.bids, state.announce_time, state.bid_window_s)
        if result.winner_id is None:
            self.get_logger().warn(f'경매 {key}: 입찰 없음 - 종료')
            state.phase = 'done'
            return
        state.winner_id = result.winner_id
        state.winning_cost = result.winning_cost
        state.phase = 'awarded'
        self._send_award(key, state, now)

    def _maybe_resend_or_reauction(self, key: AuctionKey, state: _AuctionState, now: float):
        if should_resend_award(
            state.award_attempts, state.award_last_sent, now,
            self._params.award_resend_interval_s, self._params.award_max_attempts,
        ):
            self._send_award(key, state, now)
        elif award_exhausted(state.award_attempts, self._params.award_max_attempts):
            self.get_logger().warn(f'경매 {key}: ACCEPT 없음({state.award_attempts}회 재전송) - 재경매')
            self._reauction(key, state)

    def _send_award(self, key: AuctionKey, state: _AuctionState, now: float):
        auctioneer_id, auction_id = key
        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = auctioneer_id
        out.auction_id = auction_id
        out.task_id = state.task.task_id
        out.robot_id = self._robot_id
        out.type = TaskBid.TYPE_AWARD
        out.winner_id = state.winner_id
        out.winning_cost = state.winning_cost
        self._bid_pub.publish(out)
        state.award_attempts += 1
        state.award_last_sent = now

    def _reauction(self, key: AuctionKey, state: _AuctionState):
        auctioneer_id, old_auction_id = key
        next_id = max(old_auction_id + 1, self._next_reauction_id.get(auctioneer_id, 0))
        self._next_reauction_id[auctioneer_id] = next_id + 1
        new_key = (auctioneer_id, next_id)
        self._auctions.pop(key, None)
        new_state = _AuctionState(
            task=state.task, announce_time=self._now(), bid_window_s=state.bid_window_s)
        self._auctions[new_key] = new_state
        self._tasks_seen[new_key] = state.task

        out = TaskAuction()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = auctioneer_id
        out.auction_id = next_id
        out.task.task_id = state.task.task_id
        out.task.dropoff_pose.position.x, out.task.dropoff_pose.position.y = state.task.dropoff_xy
        if state.task.pickup_xy is not None:
            out.task.pickup_station_id = state.task.task_id  # 재경매용 - 실제 스테이션 id는 모른다
            out.task.pickup_pose.position.x, out.task.pickup_pose.position.y = state.task.pickup_xy
        if state.task.deadline is not None:
            out.task.deadline = _epoch_to_time(state.task.deadline)
        out.bid_window.sec = int(new_state.bid_window_s)
        out.bid_window.nanosec = int(round((new_state.bid_window_s - out.bid_window.sec) * 1e9))
        self._auction_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAuctionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
