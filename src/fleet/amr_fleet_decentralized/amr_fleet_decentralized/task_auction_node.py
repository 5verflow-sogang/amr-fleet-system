"""분산 경매(TaskAuction/TaskBid) 노드. 탐색 로직은 전부 auction/(ROS 비의존)에 위임하고,
이 노드는 메시지 <-> auction 타입 변환, 경매별 상태(진행 중/낙찰/완료) 관리, 재전송·재경매
타이밍만 맡는다 - amr_nav2_plugins::DwaController가 amr_planning_core에 위임하는 것과
같은 패턴.

구독 기본값(절대 경로, 중계되기 전 원본 - BT·가짜 로봇과 같은 relayed 방식, 10/9
하연·이진 결정 #107·#108):
    /fleet/task_auctions  TaskAuction   -> 공고를 보고 입찰할지 정한다(decide_bid)
    /fleet/task_bids      TaskBid       -> 내가 경매인인 입찰 수집, 내가 낙찰자인 AWARD, 수락·포기
    /fleet/task_status    TaskStatus    -> ACCEPT 뒤 낙찰자가 보낸 FAILED를 보고 재경매 여부 판단
launch 인자 relayed:=true(또는 파라미터 auction_topic_prefix:=fleet)면 상대 이름
fleet/task_auctions, fleet/task_bids로 - 장애 주입기 relay_auction 중계본을 구독한다
(amr_bt·가짜 로봇의 relayed 인자와 같은 이름·동작). task_status는 장애 주입기 relay
목록에 없어 relayed 여부와 무관하게 항상 절대 경로 그대로 구독한다.

발행(항상 절대 경로 원본 - 장애 주입기가 여기서 읽어 로봇별로 다시 보낸다):
    /fleet/task_auctions  TaskAuction
    /fleet/task_bids      TaskBid

보낸 로봇 자기 메시지(중계 에코)는 robot_id로 걸러 무시한다 - 보낼 때 이미 처리했으므로
또 처리하면 중복이다.

분산 모드 경매 트리거: 모든 로봇의 노드가 작업 방송(/fleet/task_requests, Task)을 구독하고,
살아 있는 로봇(RobotState를 최근 alive_timeout_s 안에 받은 로봇, robot_id 순) 중
crc32(task_id) mod 로봇 수 번째 로봇만 그 작업의 경매를 연다(auction.auction_opener). 한 번 본
task_id(작업 방송·다른 로봇의 공고)는 다시 열지 않는다. 로봇마다 살아 있는 목록이 잠깐 달라 두
로봇이 같은 작업을 동시에 열면, 입찰 수집 중에 더 작은 robot_id의 공고를 본 쪽이 자기 경매를
접는다. 이렇게 연 경매에 입찰이 하나도 없으면(모두 작업 중) no_bid_retry_s 뒤 새 auction_id로
다시 연다 - 서버가 미할당 작업을 큐에 두고 다시 할당하는 것과 같은 역할이다(외부에서 들어온
공고는 기존처럼 입찰이 없으면 끝낸다).

ACCEPT 뒤 작업 실행: ACCEPT한 로봇의 노드가 자기에게 TaskAssignment를 보낸다 - amr_bt·가짜
로봇이 relayed:=true로 구독하는 중계본 이름(/<robot_id>/fleet/task_assignments)으로 바로 보내
장애 주입이 두 번 걸리지 않게 한다. assignment_id는 1,000,000 + 로봇 번호 x 100,000 + 로봇별
순번(서버 번호와 안 겹침). Ack(/fleet/task_assignment_acks)가 올 때까지 AWARD와 같은 간격·횟수로
다시 보내고, 거부(REJECTED)면 TaskStatus FAILED를 보내 경매인이 재경매하게 한다. 경매인은 ACCEPT를
받으면 "공고 → ACCEPT" 응답 시간을 로그로 남긴다(분산 응답 시간).

재경매는 두 가지 - AWARD 재전송 다 썼는데 ACCEPT 없음,
그리고 ACCEPT 뒤 TaskStatus FAILED(최대 failed_retries번, 그래도 실패하면 실패로 남김
- 중앙 assignment.failed_retries와 같은 규칙). 취소(ErrorInfo.ASSIGN_CANCELED)로 온 FAILED는
재경매하지 않는다(중앙 #107과 같음). 그리고 ACCEPT한 낙찰 로봇의 RobotState가
reclaim_timeout_s(실제 시간) 동안 끊기면 로봇이 죽은 것으로 보고 재경매한다 - 중앙 #132의
작업 회수와 같은 규칙(FAILED 재시도와 따로 세고 상한 없음). 회수된 옛 낙찰 로봇의 늦은
보고는 그 경매가 이미 새 경매로 바뀌어 맞는 경매가 없으므로 무시된다.

내 상태(RobotState)를 아직 모를 때 공고를 받으면(기동 직후 등) 입찰 여부 결정을 미룬다 -
상태를 먼저 확인하기 전에 "결정 끝"으로 기록해 버리면 그 경매에 영원히 입찰을 못 한다
(10/10 연우 리뷰, #109). RobotState를 처음 받으면 미뤄 둔 공고를 마감 전인 것만 다시 본다.

BID는 1회성 발행이라 DDS 디스커버리가 아직 안 끝난 상태에서 나가면 그대로 유실되고
다시 안 온다(#155·#171) - 마감(공고를 받은 시각 + bid_window_s)까지 bid_resend_period_s
간격으로 같은 입찰(같은 cost)을 다시 보낸다. AWARD를 받거나 마감되면 멈춘다. 경매인
쪽은 같은 로봇의 재전송 BID를 중복으로 세지 않는다(auction.register_bid).

동시 낙찰 경합: 경매가 동시에 여러 개 열리면 IDLE 로봇 하나가 여러 경매에 낙찰될 수 있다.
이미 ACCEPT한 작업이 있는 로봇은 새 AWARD에 WITHDRAW를 보내고(auction.should_withdraw_award),
그 작업이 풀릴 때까지 새 공고에 입찰하지 않는다. 맡은 작업은 (a) 내 TaskStatus COMPLETED/FAILED,
(b) 내 RobotState가 BUSY였다가 다시 IDLE, (c) ACCEPT 뒤 commit_timeout_s(실제 시간) 동안 한 번도
BUSY가 안 됨(작업이 시작 안 됨, 경고 로그) 중 먼저 오는 것으로 푼다. 경매인은
낙찰자의 WITHDRAW를 받으면 같은 경매의 다음 순위 입찰자에게 AWARD하고, 남은 입찰자가 없으면
새 auction_id로 재경매한다.
"""
from __future__ import annotations

import signal
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from amr_msgs.msg import (
    RobotState, Task, TaskAssignment, TaskAssignmentAck, TaskAuction, TaskBid, TaskStatus,
)
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .auction import (
    ERROR_CANCELED,
    AuctionParams,
    AuctionResult,
    Bid,
    RobotInfo,
    TaskInfo,
    TaskRetryLedger,
    assignment_id_for,
    assignment_topic,
    auction_opener,
    award_exhausted,
    can_withdraw,
    commit_timed_out,
    decide_bid,
    decide_winner,
    live_robots,
    register_bid,
    robot_index,
    should_resend_award,
    should_resend_bid,
    should_withdraw_award,
    task_finished_by_state,
    winner_lost,
)

# 발행은 항상 이 절대 경로 원본 - 장애 주입기가 여기서 읽어 로봇별로 다시 보낸다(파라미터로
# 안 바꾼다, amr_bt의 로봇->서버 발행 토픽과 같은 방식).
TASK_AUCTIONS_PUB_TOPIC = '/fleet/task_auctions'
TASK_BIDS_PUB_TOPIC = '/fleet/task_bids'
# TaskStatus는 장애 주입기 relay 목록에 없어 relayed 여부와 무관하게 항상 이 절대 경로다.
TASK_STATUS_TOPIC = '/fleet/task_status'
# ACCEPT 뒤 보낸 TaskAssignment의 Ack - 로봇(amr_bt·가짜 로봇)이 서버에 보내는 것과 같은 토픽이다.
# 서버는 모르는 assignment_id의 Ack를 무시한다(amr_fleet_manager Dispatcher.on_ack).
TASK_ASSIGNMENT_ACKS_TOPIC = '/fleet/task_assignment_acks'

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
    awarded(낙찰 통보, ACCEPT 기다림) -> done(확정, 낙찰 로봇 장애 감시) -> completed(COMPLETED
    보고, 감시 끝) 또는 failed(ACCEPT 뒤 FAILED, 재경매 다 쓰고 실패로 남김 - 더 안 건드림)."""

    task: TaskInfo
    announce_time: float
    bid_window_s: float
    phase: str = 'collecting'
    bids: List[Bid] = field(default_factory=list)
    winner_id: Optional[str] = None
    winning_cost: Optional[float] = None
    award_attempts: int = 0
    award_last_sent: Optional[float] = None
    # ACCEPT를 받은 시각 (실제 시간, 낙찰 로봇 장애 판정 기준 - winner_lost)
    accepted_at: Optional[float] = None
    # 이 경매를 WITHDRAW한 로봇 - 다음 순위를 고를 때 빼고, 그 뒤 온 재전송 BID도 안 센다.
    withdrawn: set = field(default_factory=set)
    # 공고에 실린 작업 원본(Task) - 재경매·TaskAssignment에 그대로 싣는다(종류·물품·스테이션 id).
    task_msg: Optional[Task] = None
    # 작업 방송(/fleet/task_requests)으로 내가 연 경매인지 - 입찰이 없으면 다시 연다(no_bid_retry_s).
    from_request: bool = False
    # 입찰 없이 마감된 뒤 다시 열 시각(_now 기준)
    retry_at: Optional[float] = None


class TaskAuctionNode(Node):

    def __init__(self):
        super().__init__('task_auction')
        p = self.declare_parameter
        default_robot_id = self.get_namespace().strip('/') or 'amr_01'
        self._robot_id = p('robot_id', default_robot_id).value

        robot_states_topic = p('robot_states_topic', '/fleet/robot_states').value
        # 구독 기본값은 절대 경로 원본. relayed:=true(launch) 또는 이 파라미터를 직접
        # 'fleet'로 주면 상대 이름(장애 주입기 relay_auction 중계본)을 구독한다 - amr_bt의
        # server_topic_prefix와 같은 방식(10/9 결정).
        auction_topic_prefix = p('auction_topic_prefix', '/fleet').value
        task_auctions_sub_topic = auction_topic_prefix + '/task_auctions'
        task_bids_sub_topic = auction_topic_prefix + '/task_bids'
        update_rate_hz = p('update_rate_hz', 10.0).value

        self._params = AuctionParams(
            bid_window_s=p('bid_window_s', AuctionParams.bid_window_s).value,
            min_battery_percent=p('min_battery_percent', AuctionParams.min_battery_percent).value,
            robot_speed_mps=p('robot_speed_mps', AuctionParams.robot_speed_mps).value,
            award_resend_interval_s=p(
                'award_resend_interval_s', AuctionParams.award_resend_interval_s).value,
            award_max_attempts=p('award_max_attempts', AuctionParams.award_max_attempts).value,
            bid_resend_period_s=p(
                'bid_resend_period_s', AuctionParams.bid_resend_period_s).value,
            failed_retries=p('failed_retries', AuctionParams.failed_retries).value,
            reclaim_timeout_s=p('reclaim_timeout_s', AuctionParams.reclaim_timeout_s).value,
            max_reclaims=p('max_reclaims', AuctionParams.max_reclaims).value,
            commit_timeout_s=p('commit_timeout_s', AuctionParams.commit_timeout_s).value,
        )

        # 분산 모드 경매 트리거 · ACCEPT 뒤 작업 실행
        task_requests_topic = p('task_requests_topic', '/fleet/task_requests').value
        self._open_auctions = p('open_auctions', True).value
        self._alive_timeout_s = p('alive_timeout_s', 3.0).value
        self._no_bid_retry_s = p('no_bid_retry_s', 1.0).value
        self._run_accepted_tasks = p('run_accepted_tasks', True).value
        relayed_assignments = p('relayed_assignments', True).value
        index = p('robot_index', -1).value
        self._robot_index = index if index >= 0 else (
            robot_index(self._robot_id) if self._run_accepted_tasks else 0)
        self._assignment_topic = assignment_topic(self._robot_id, relayed_assignments)

        self._robot_info: Optional[RobotInfo] = None
        # 내가 경매인인 경매들 (auctioneer_id는 항상 나 자신이라 auction_id만 키로 써도 되지만,
        # TaskBid 처리 쪽과 자료형을 맞추려고 (auctioneer_id, auction_id) 그대로 쓴다).
        self._auctions: Dict[AuctionKey, _AuctionState] = {}
        # 입찰 여부 판단을 한 번만 하려고(같은 공고를 중계·재구독 등으로 다시 받아도 중복 입찰 방지).
        self._bid_decided: set = set()
        # 내 상태(RobotState)를 아직 몰라 입찰 여부를 못 정한 공고 - (msg, 받은 시각).
        # RobotState를 처음 받으면 마감 전인 것만 다시 본다(10/10 연우 리뷰, #109).
        self._pending_bids: Dict[AuctionKey, Tuple[TaskAuction, float]] = {}
        # 입찰 마감(실제 시간) - 공고를 처음 받은 시각 + bid_window_s. #155·#171 BID
        # 재전송이 멈춰야 할 때(마감)를 판단하는 기준이라, 공고를 받을 때(_on_task_auction)
        # 마다 한 번만 찍는다(재전송·중계 에코로 다시 받아도 안 바꾼다).
        self._bid_deadline: Dict[AuctionKey, float] = {}
        # 재전송 중인 내 BID - key -> [TaskBid, 마감, 마지막 전송 시각]. AWARD를 받거나
        # 마감되면 꺼진다(_on_awarded·_maybe_resend_bids).
        self._bid_resends: Dict[AuctionKey, list] = {}
        # 입찰할 때 본 작업 내용 - AWARD를 받았을 때 재확인(decide_bid)하려면 필요하다
        # (TaskBid AWARD에는 task_id만 있고 좌표·마감이 없다).
        self._tasks_seen: Dict[AuctionKey, TaskInfo] = {}
        # 이미 ACCEPT를 보낸 경매 - WITHDRAW는 ACCEPT 전까지만(10/7 결정), 재전송된 AWARD에
        # 또 ACCEPT를 보내지 않는 데도 쓴다(멱등).
        self._accepted: set = set()
        # 동시 낙찰 경합: 내가 지금 맡고 있는(ACCEPT한) 작업 - (경매 key, task_id). 있으면 새
        # AWARD는 WITHDRAW하고 새 공고에는 입찰하지 않는다. 내 TaskStatus COMPLETED/FAILED로 푼다.
        self._active_task: Optional[Tuple[AuctionKey, str]] = None
        # 맡은 작업을 ACCEPT한 시각(실제 시간)과, 그 뒤 내 RobotState에서 BUSY를 봤는지 - 해제
        # 안전장치(b)(c)에 쓴다.
        self._active_accepted_at: float = 0.0
        self._active_seen_busy: bool = False
        # 내가 WITHDRAW한 경매 - 재전송된 AWARD가 와도 다시 WITHDRAW한다(뒤늦은 수락으로 다음
        # 순위 낙찰자와 같은 작업을 맡지 않게).
        self._withdrawn: set = set()
        self._next_reauction_id: Dict[str, int] = {}
        # 작업별 FAILED 재시도·장애 회수 횟수 - task_id로 세어 재경매로 auction_id가 바뀌어도
        # 이어서 센다(중앙 TaskEntry.failures·reclaims와 같은 역할, 둘은 따로 센다).
        self._retries = TaskRetryLedger(self._params.failed_retries, self._params.max_reclaims)
        # 로봇별 마지막 RobotState 수신 시각 (실제 시간). 낙찰 로봇 장애 판정에 쓴다 - 중앙 #132처럼
        # 로봇 생존은 실제 시간으로 잰다(메시지 stamp는 시뮬레이션 시간일 수 있다)
        self._robot_last_seen: Dict[str, float] = {}
        # 작업 방송·공고로 이미 본 task_id - 같은 작업으로 경매를 두 번 열지 않는다.
        self._task_ids_seen: set = set()
        # 내가 연 작업의 첫 공고 시각(_now) - "공고 -> ACCEPT" 응답 시간 기준.
        self._first_announce: Dict[str, float] = {}
        # 공고에 실린 작업 원본(Task) - ACCEPT 뒤 TaskAssignment에 싣는다.
        self._task_msgs: Dict[AuctionKey, Task] = {}
        # 내가 보낸 TaskAssignment 중 Ack를 기다리는 것 - assignment_id -> [msg, 보낸 횟수, 마지막 전송(실제 시간)].
        self._assignment_seq = 0
        self._assignment_resends: Dict[int, list] = {}

        self.create_subscription(RobotState, robot_states_topic, self._on_robot_state, 10)
        self.create_subscription(TaskAuction, task_auctions_sub_topic, self._on_task_auction, 10)
        self.create_subscription(TaskBid, task_bids_sub_topic, self._on_task_bid, 10)
        self.create_subscription(TaskStatus, TASK_STATUS_TOPIC, self._on_task_status, 10)
        self.create_subscription(Task, task_requests_topic, self._on_task_request, 10)
        self.create_subscription(
            TaskAssignmentAck, TASK_ASSIGNMENT_ACKS_TOPIC, self._on_assignment_ack, 10)
        self._auction_pub = self.create_publisher(TaskAuction, TASK_AUCTIONS_PUB_TOPIC, 10)
        self._bid_pub = self.create_publisher(TaskBid, TASK_BIDS_PUB_TOPIC, 10)
        self._assignment_pub = self.create_publisher(TaskAssignment, self._assignment_topic, 10)
        self._status_pub = self.create_publisher(TaskStatus, TASK_STATUS_TOPIC, 10)
        self.create_timer(1.0 / update_rate_hz, self._on_timer)

        self.get_logger().info(
            f'task_auction({self._robot_id}): 구독 {task_auctions_sub_topic} (공고), '
            f'{task_bids_sub_topic} (입찰/낙찰/수락), {TASK_STATUS_TOPIC} (실패 보고), '
            f'{robot_states_topic} (내 상태), {task_requests_topic} (작업 방송) · 발행 '
            f'{TASK_AUCTIONS_PUB_TOPIC}, {TASK_BIDS_PUB_TOPIC}, {self._assignment_topic} (수락한 작업)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- 내 상태 ---

    def _on_robot_state(self, msg: RobotState):
        self._robot_last_seen[msg.robot_id] = time.monotonic()
        if msg.robot_id != self._robot_id:
            return
        if self._active_task is not None:
            if msg.mode == RobotState.MODE_BUSY:
                self._active_seen_busy = True
            elif task_finished_by_state(self._active_seen_busy, msg.mode):
                # (b) BUSY였다가 다시 IDLE - TaskStatus가 안 와도 작업은 끝났다
                self._release_active_task('RobotState BUSY -> IDLE')
        had_info = self._robot_info is not None
        self._robot_info = RobotInfo(
            robot_id=msg.robot_id,
            xy=(msg.pose.position.x, msg.pose.position.y),
            battery_percent=msg.battery_percent,
            mode=msg.mode,
        )
        if had_info or not self._pending_bids:
            return
        # 내 상태를 처음 알았다 - 그걸 몰라서 미뤄 둔 공고들을 지금 다시 본다. 마감
        # (bid_window_s)이 지났으면 이제 입찰해도 소용없으니 버린다 (10/10 연우 리뷰, #109).
        now = self._now()
        for key, (auction_msg, received_time) in list(self._pending_bids.items()):
            del self._pending_bids[key]
            if now - received_time < _duration_to_s(auction_msg.bid_window):
                self._maybe_bid(key, auction_msg)

    # --- 참여 로봇 쪽: 공고를 보고 입찰할지 정한다 ---

    def _on_task_auction(self, msg: TaskAuction):
        key: AuctionKey = (msg.auctioneer_id, msg.auction_id)
        if key not in self._bid_deadline:
            # 마감은 "내가 이 공고를 받은 시각" 기준(10/7 결정과 같은 원칙 - 보낸 쪽
            # 시각이 아니라 내 수신 시각을 쓴다). 재전송·중계 에코로 또 받아도 안 바꾼다.
            self._bid_deadline[key] = self._now() + _duration_to_s(msg.bid_window)
        if msg.auctioneer_id == self._robot_id and key not in self._auctions:
            # 내가 경매인인 공고 - 직접 시작했든(_announce) 외부에서 내 이름으로 왔든(이
            # 노드는 아직 "언제 새로 경매를 열지"는 안 다룬다, README 참고) 똑같이
            # 입찰 수집을 시작한다. 이미 추적 중이면(재전송·중계 에코) 다시 안 만든다.
            self._auctions[key] = _AuctionState(
                task=_task_from_msg(msg.task), announce_time=self._now(),
                bid_window_s=_duration_to_s(msg.bid_window), task_msg=msg.task)
        elif msg.auctioneer_id != self._robot_id:
            self._drop_duplicate_auction(msg)

        self._task_ids_seen.add(msg.task.task_id)
        self._tasks_seen[key] = _task_from_msg(msg.task)
        self._task_msgs[key] = msg.task
        self._maybe_bid(key, msg)

    def _drop_duplicate_auction(self, msg: TaskAuction):
        """같은 작업을 다른 로봇도 열었다(살아 있는 로봇 목록이 잠깐 달랐을 때). 입찰 수집 중이면
        robot_id가 작은 쪽의 경매만 남긴다 - 두 경매가 각자 낙찰하면 같은 작업이 두 로봇에게 간다."""
        if msg.auctioneer_id > self._robot_id:
            return
        for key, state in list(self._auctions.items()):
            if state.task.task_id == msg.task.task_id and state.phase == 'collecting':
                del self._auctions[key]
                self.get_logger().warn(
                    f'경매 {key}: 같은 작업 {msg.task.task_id}을 {msg.auctioneer_id}도 열어 내 경매를 접는다')

    # --- 경매 트리거: 작업 방송을 보고 내가 열 차례면 경매를 연다 ---

    def _on_task_request(self, msg: Task):
        task_id = msg.task_id
        if not self._open_auctions or not task_id or task_id in self._task_ids_seen:
            return
        self._task_ids_seen.add(task_id)
        robots = live_robots(self._robot_last_seen, time.monotonic(), self._alive_timeout_s)
        opener = auction_opener(task_id, robots)
        if opener is None:
            self.get_logger().warn(f'작업 {task_id}: 살아 있는 로봇이 없어 경매를 못 연다')
            return
        if opener == self._robot_id:
            self._open_auction(msg)

    def _next_auction_id(self) -> int:
        next_id = self._next_reauction_id.get(self._robot_id, 1)
        self._next_reauction_id[self._robot_id] = next_id + 1
        return next_id

    def _open_auction(self, task: Task):
        out = TaskAuction()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = self._robot_id
        out.auction_id = self._next_auction_id()
        out.task = task
        out.bid_window.sec = int(self._params.bid_window_s)
        out.bid_window.nanosec = int(round(
            (self._params.bid_window_s - out.bid_window.sec) * 1e9))
        self._first_announce[task.task_id] = self._now()
        self._auction_pub.publish(out)
        self.get_logger().info(
            f'경매 공고 ({self._robot_id}, {out.auction_id}): 작업 {task.task_id} '
            f't={self._first_announce[task.task_id]:.3f}')
        # 내 경매 상태를 만들고 나도 입찰한다(자기 공고 에코를 기다리지 않는다 - 에코는 이미 있는
        # 경매라 다시 안 만든다).
        self._on_task_auction(out)
        self._auctions[(self._robot_id, out.auction_id)].from_request = True

    def _maybe_bid(self, key: AuctionKey, msg: TaskAuction):
        if key in self._bid_decided:
            return
        if self._robot_info is None:
            # 내 상태(RobotState)를 아직 모른다 - "결정 끝"으로 기록하지 않고 미룬다.
            # 그대로 기록해 버리면 상태를 공고보다 늦게 받은 로봇은 이 경매에 영원히
            # 입찰하지 못한다(10/10 연우 리뷰, #109) - _on_robot_state가 상태를 처음
            # 받으면 마감 전인지 보고 다시 판단한다.
            self._pending_bids.setdefault(key, (msg, self._now()))
            return
        self._bid_decided.add(key)
        if self._active_task is not None:
            # 이미 ACCEPT한 작업이 있다 - RobotState가 아직 IDLE이어도 IDLE 로봇이 아니다
            # (IDLE 로봇만 입찰, 10/7 결정). 입찰하면 낙찰돼도 WITHDRAW할 수밖에 없다.
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

        # #155·#171: BID는 1회성 발행이라 디스커버리가 안 끝난 상태에서 나가면 유실될 수
        # 있다 - 마감(_bid_deadline)까지 bid_resend_period_s 간격으로 같은 메시지를
        # 다시 보낸다. AWARD를 받으면(_on_awarded) 바로 멈춘다.
        self._bid_resends[key] = [out, self._bid_deadline[key], now]

        # 내 경매에 나도 입찰했으면, 중계 에코를 기다리지 않고(자기 메시지는 걸러 무시하므로
        # 에코로는 절대 등록되지 않는다) 바로 내 입찰 목록에 넣는다.
        if key in self._auctions:
            self._register_bid(key, bid, now)

    # --- 경매인 쪽: 입찰 수집·낙찰·수락/포기 처리 ---

    def _register_bid(self, key: AuctionKey, bid: Bid, received_time: float):
        state = self._auctions.get(key)
        if state is None or state.phase != 'collecting' or bid.robot_id in state.withdrawn:
            return
        # register_bid: 재전송(bid_resend_period_s)으로 같은 로봇의 BID가 또 와도
        # 중복으로 세지 않는다(#155·#171).
        state.bids = register_bid(state.bids, bid)

    def _on_task_bid(self, msg: TaskBid):
        key: AuctionKey = (msg.auctioneer_id, msg.auction_id)

        if msg.type == TaskBid.TYPE_BID:
            if msg.robot_id == self._robot_id:
                return  # 자기 입찰 에코 - 보낼 때 이미 _register_bid로 처리했다
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
            # robot_id(보낸 경매인)로 자기 메시지를 거르지 않는다 - 경매인이 자기 경매에
            # 자기 입찰이 낙찰되면(제일 싸서) robot_id==winner_id==나인 메시지도 와야
            # 하는데, BID·ACCEPT·WITHDRAW와 같은 자기 필터를 여기 적용하면 그 경매는
            # 영원히 ACCEPT를 못 보내고 AWARD만 자기한테 계속 재전송한다(10/10 발견, #109).
            # winner_id만으로 정확히 거른다.
            if msg.winner_id == self._robot_id:
                self._on_awarded(key, msg)
        elif msg.type == TaskBid.TYPE_ACCEPT:
            # 위 AWARD와 같은 이유로 자기 필터를 안 쓴다 - 경매인이 자기 경매에 자기가
            # 낙찰된 경우, 자기가 보낸 ACCEPT 에코를 받아야 경매인 역할로 확정
            # 처리(phase='done')할 수 있다. _on_accept는 winner_id와 비교해 걸러서
            # 다른 로봇·다른 경매의 ACCEPT에는 영향이 없다.
            self._on_accept(key, msg.robot_id)
        elif msg.type == TaskBid.TYPE_WITHDRAW:
            self._on_withdraw(key, msg.robot_id)

    def _on_awarded(self, key: AuctionKey, msg: TaskBid):
        # AWARD를 받았다는 건 경매인이 이미 낙찰자(나)를 정했다는 뜻 - 더 입찰을
        # 재전송할 이유가 없다(#155·#171, ACCEPT·WITHDRAW 어느 쪽이든 마찬가지).
        self._bid_resends.pop(key, None)
        if key in self._accepted:
            return  # 재전송된 AWARD - 이미 수락을 보냈다(멱등)
        task = self._tasks_seen.get(key)
        now = self._now()
        # 입찰했을 때와 사정이 바뀌었을 수 있으니(배터리·IDLE 여부) 다시 확인한다. 이미 다른
        # 작업을 ACCEPT했거나 이 경매를 한 번 포기했으면 수락하지 않는다(동시 낙찰 경합).
        holding_other_task = self._active_task is not None and self._active_task[0] != key
        still_eligible = (
            not should_withdraw_award(holding_other_task, key in self._withdrawn)
            and self._robot_info is not None and task is not None
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
            self._active_task = (key, msg.task_id)
            self._active_accepted_at = time.monotonic()
            self._active_seen_busy = False
            self._bid_pub.publish(out)
            self.get_logger().info(
                f'경매 {key}: 작업 {msg.task_id} ACCEPT t={self._now():.3f}')
            if self._run_accepted_tasks:
                self._send_assignment(key, msg.task_id)
            return
        elif can_withdraw(accepted=False):
            # 10/7 결정: WITHDRAW는 ACCEPT 전까지만 - 이 경매에는 아직 ACCEPT를 안 보냈으니 가능하다.
            out.type = TaskBid.TYPE_WITHDRAW
            self._withdrawn.add(key)
        else:
            return
        self._bid_pub.publish(out)

    def _send_assignment(self, key: AuctionKey, task_id: str):
        """ACCEPT한 작업을 나(amr_bt·가짜 로봇)에게 TaskAssignment로 보낸다. Ack가 올 때까지 다시 보낸다."""
        task_msg = self._task_msgs.get(key)
        if task_msg is None:
            self.get_logger().error(f'경매 {key}: 작업 {task_id} 원본이 없어 TaskAssignment를 못 보낸다')
            return
        self._assignment_seq += 1
        out = TaskAssignment()
        out.header.stamp = self.get_clock().now().to_msg()
        out.assignment_id = assignment_id_for(self._robot_index, self._assignment_seq)
        out.robot_id = self._robot_id
        out.action = TaskAssignment.ACTION_ASSIGN
        out.task = task_msg
        self._assignment_pub.publish(out)
        self._assignment_resends[out.assignment_id] = [out, 1, time.monotonic()]
        self.get_logger().info(
            f'작업 {task_id} 실행: TaskAssignment {out.assignment_id} -> {self._assignment_topic}')

    def _on_assignment_ack(self, msg: TaskAssignmentAck):
        if msg.robot_id != self._robot_id:
            return
        entry = self._assignment_resends.pop(msg.assignment_id, None)
        if entry is None:
            return  # 서버가 보낸 할당의 Ack이거나 이미 받은 Ack
        if msg.result == TaskAssignmentAck.RESULT_ACCEPTED:
            return
        task = entry[0].task
        self.get_logger().warn(
            f'TaskAssignment {msg.assignment_id}(작업 {task.task_id}) 거부: code={msg.error.code} '
            f'{msg.error.message} - FAILED로 보고해 경매인이 재경매하게 한다')
        status = TaskStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.task_id = task.task_id
        status.robot_id = self._robot_id
        status.state = TaskStatus.STATE_FAILED
        status.error = msg.error
        self._status_pub.publish(status)

    def _maybe_resend_assignments(self):
        now = time.monotonic()
        for assignment_id, entry in list(self._assignment_resends.items()):
            msg, attempts, last_sent = entry
            if now - last_sent < self._params.award_resend_interval_s:
                continue
            if attempts >= self._params.award_max_attempts:
                del self._assignment_resends[assignment_id]
                self.get_logger().warn(
                    f'TaskAssignment {assignment_id}(작업 {msg.task.task_id}) Ack 없음({attempts}회) - '
                    f'재전송 그만(작업이 시작 안 되면 commit_timeout_s 뒤 맡은 작업을 푼다)')
                continue
            self._assignment_pub.publish(msg)
            entry[1] = attempts + 1
            entry[2] = now

    def _on_accept(self, key: AuctionKey, robot_id: str):
        state = self._auctions.get(key)
        if state is None or state.phase != 'awarded' or state.winner_id != robot_id:
            return
        state.phase = 'done'
        state.accepted_at = time.monotonic()
        task_id = state.task.task_id
        now = self._now()
        first = self._first_announce.get(task_id, state.announce_time)
        # 분산 응답 시간 = 첫 공고 -> ACCEPT (재경매·입찰 대기 포함). 이 경매만의 값도 같이 남긴다.
        self.get_logger().info(
            f'경매 {key}: {robot_id} 수락, 작업 {task_id} 확정 - 응답 시간(공고->ACCEPT) '
            f'{now - first:.3f}s (이 경매 {now - state.announce_time:.3f}s)')

    def _on_withdraw(self, key: AuctionKey, robot_id: str):
        state = self._auctions.get(key)
        if state is None:
            return
        if state.phase == 'collecting':
            state.withdrawn.add(robot_id)
            state.bids = [b for b in state.bids if b.robot_id != robot_id]
        elif state.phase == 'awarded' and state.winner_id == robot_id:
            # 동시 낙찰 경합: 같은 입찰 목록에서 포기한 로봇들을 빼고 다음 순위에게 준다.
            # 남은 입찰자가 없으면 새 auction_id로 재경매(기존 재경매 규칙과 같음).
            state.withdrawn.add(robot_id)
            result = decide_winner(
                state.task.task_id, state.bids, state.announce_time, state.bid_window_s,
                excluded=state.withdrawn)
            if result.winner_id is None:
                self.get_logger().warn(f'경매 {key}: 낙찰자 {robot_id} 포기, 남은 입찰 없음 - 재경매')
                self._reauction(key, state)
                return
            self.get_logger().info(
                f'경매 {key}: 낙찰자 {robot_id} 포기 - 다음 순위 {result.winner_id}에게 낙찰')
            state.winner_id = result.winner_id
            state.winning_cost = result.winning_cost
            state.award_attempts = 0
            self._send_award(key, state, self._now())

    # --- 확정된 작업이 수행 중 실패했다는 보고(amr_bt가 TaskStatus로 발행) ---

    def _on_task_status(self, msg: TaskStatus):
        if msg.state not in (TaskStatus.STATE_FAILED, TaskStatus.STATE_COMPLETED):
            return
        if (
            msg.robot_id == self._robot_id and self._active_task is not None
            and self._active_task[1] == msg.task_id
        ):
            # (a) 내가 맡은 작업이 끝났다 - 다시 입찰·수락할 수 있다(동시 낙찰 경합).
            self._release_active_task('TaskStatus COMPLETED/FAILED')
        for key, state in list(self._auctions.items()):
            if (
                state.phase == 'done' and state.task.task_id == msg.task_id
                and state.winner_id == msg.robot_id
            ):
                if msg.state == TaskStatus.STATE_COMPLETED:
                    # 끝난 작업은 낙찰 로봇이 그 뒤 꺼져도 회수하지 않는다
                    state.phase = 'completed'
                else:
                    self._on_task_failed(key, state, msg.error.code)
                return

    def _on_task_failed(self, key: AuctionKey, state: _AuctionState, error_code: int = 0):
        task_id = state.task.task_id
        if self._retries.on_failed(task_id, error_code):
            self.get_logger().warn(
                f'경매 {key}: 작업 {task_id} 실패 보고({self._retries.failures(task_id)}/'
                f'{self._params.failed_retries}) - 재경매')
            self._reauction(key, state)
        elif error_code == ERROR_CANCELED:
            # 취소로 멈춘 작업: 재경매하지 않고 재시도 횟수에도 세지 않는다 (중앙 #107과 같음)
            state.phase = 'failed'
            self.get_logger().info(f'경매 {key}: 작업 {task_id} 취소(ASSIGN_CANCELED) - 재경매 안 함')
        else:
            state.phase = 'failed'
            self.get_logger().error(
                f'경매 {key}: 작업 {task_id} 재경매({self._params.failed_retries}회) 후에도 '
                f'실패 - 실패로 남김')

    # --- 주기 처리: 마감 판정, AWARD 재전송·재경매 ---

    def _release_active_task(self, reason: str):
        self.get_logger().info(f'맡은 작업 {self._active_task[1]} 해제({reason}) - 다시 입찰한다')
        self._active_task = None
        self._active_seen_busy = False

    def _maybe_release_uncommitted_task(self):
        """(c) ACCEPT 뒤 commit_timeout_s 동안 RobotState가 한 번도 BUSY가 안 됐으면 맡은 작업을 푼다."""
        if self._active_task is None or not commit_timed_out(
            self._active_accepted_at, time.monotonic(), self._active_seen_busy,
            self._params.commit_timeout_s,
        ):
            return
        self.get_logger().warn(
            f'작업 {self._active_task[1]}을 ACCEPT하고 {self._params.commit_timeout_s:g}초 동안 '
            f'RobotState가 BUSY가 안 됨(작업이 시작 안 됨) - 맡은 작업 해제')
        self._active_task = None

    def _on_timer(self):
        now = self._now()
        self._maybe_resend_bids(now)
        self._maybe_resend_assignments()
        self._maybe_release_uncommitted_task()
        for key, state in list(self._auctions.items()):
            if state.phase == 'collecting':
                self._maybe_decide_winner(key, state, now)
            elif state.phase == 'awarded':
                self._maybe_resend_or_reauction(key, state, now)
            elif state.phase == 'done':
                self._maybe_reclaim(key, state)

    def _maybe_resend_bids(self, now: float):
        """내가 보낸 BID들 중 아직 마감 전인 것을 bid_resend_period_s 간격으로 다시 보낸다
        (#155·#171 - 1회성 발행이 디스커버리 레이스로 유실되는 문제). AWARD를 받으면
        (_on_awarded) 이미 꺼져 있다."""
        for key, entry in list(self._bid_resends.items()):
            msg, deadline, last_sent = entry
            if now >= deadline:
                del self._bid_resends[key]
                continue
            if should_resend_bid(last_sent, now, self._params.bid_resend_period_s, deadline):
                self._bid_pub.publish(msg)
                entry[2] = now

    def _maybe_reclaim(self, key: AuctionKey, state: _AuctionState):
        """ACCEPT한 낙찰 로봇의 RobotState가 reclaim_timeout_s 동안 없으면 재경매한다 (중앙 #132와 같음)."""
        if state.winner_id is None or state.accepted_at is None:
            return  # 입찰 없이 끝난 경매
        if not winner_lost(
            self._robot_last_seen.get(state.winner_id), state.accepted_at, time.monotonic(),
            self._params.reclaim_timeout_s,
        ):
            return
        task_id = state.task.task_id
        if self._retries.on_winner_lost(task_id):
            self.get_logger().warn(
                f'경매 {key}: 낙찰 로봇 {state.winner_id}의 RobotState가 '
                f'{self._params.reclaim_timeout_s:g}초 끊김 - 작업 {task_id} 회수해 재경매 '
                f'(회수 {self._retries.reclaims(task_id)}회)')
            self._reauction(key, state)
        else:
            state.phase = 'failed'
            self.get_logger().error(
                f'경매 {key}: 낙찰 로봇 {state.winner_id} 끊김, 회수 상한'
                f'({self._params.max_reclaims}회) 다 씀 - 실패로 남김')

    def _maybe_decide_winner(self, key: AuctionKey, state: _AuctionState, now: float):
        if now < state.announce_time + state.bid_window_s:
            return
        result: AuctionResult = decide_winner(
            state.task.task_id, state.bids, state.announce_time, state.bid_window_s)
        if result.winner_id is None:
            if state.from_request and self._no_bid_retry_s > 0:
                # 작업 방송으로 연 경매: 모두 작업 중이라 입찰이 없으면 잠시 뒤 다시 연다
                # (서버가 미할당 작업을 큐에 두고 다시 할당하는 것과 같은 역할)
                if state.retry_at is None:
                    state.retry_at = now + self._no_bid_retry_s
                elif now >= state.retry_at:
                    self._reauction(key, state)
                return
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
            task=state.task, announce_time=self._now(), bid_window_s=state.bid_window_s,
            task_msg=state.task_msg, from_request=state.from_request)
        self._auctions[new_key] = new_state
        self._tasks_seen[new_key] = state.task
        if state.task_msg is not None:
            self._task_msgs[new_key] = state.task_msg

        out = TaskAuction()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = auctioneer_id
        out.auction_id = next_id
        if state.task_msg is not None:
            # 원본 작업 그대로(종류·물품·스테이션 id) - TaskAssignment로 로봇에게 그대로 간다
            out.task = state.task_msg
        else:
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
        # Ctrl+C는 터미널과 launch에서 SIGINT가 두 번 온다. 정리 중에 끊기지 않게 이후 신호는 무시한다
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        # ok() 확인과 shutdown() 사이에 SIGINT 처리기가 먼저 종료하면 "already called" 예외로
        # 종료 코드 1이 난다 - try_shutdown은 이미 종료됐으면 그냥 넘어간다 (amr_fleet_manager와 같음)
        rclpy.try_shutdown()
