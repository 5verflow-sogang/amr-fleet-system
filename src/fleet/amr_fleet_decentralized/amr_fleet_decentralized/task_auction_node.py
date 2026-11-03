"""분산 경매(TaskAuction/TaskBid) 노드. 탐색 로직은 전부 auction/(ROS 비의존)에 위임하고,
이 노드는 메시지 <-> auction 타입 변환, 경매별 상태(진행 중/낙찰/완료) 관리, 재전송·재경매
타이밍만 맡는다 - amr_nav2_plugins::DwaController가 amr_planning_core에 위임하는 것과
같은 패턴.

하이브리드 (fleet_mode:=hybrid, architecture.md 5-2절 B안): 평시에는 중앙 모드라 경매를 열지도 입찰하지도
않는다. 자기 heartbeat 중계본(<server_topic_prefix>/heartbeat)이 fallback_timeout_s 동안 안 오면 분산 모드로
바꾸고, 마지막 작업표(<server_topic_prefix>/task_table)의 미할당 작업과 새 작업 방송을 분산 모드 로봇끼리
경매한다(경매를 여는 로봇도 분산 모드 로봇 중에서 고른다). heartbeat가 return_stable_s 동안 안정되면 중앙으로
돌아와 아직 낙찰 전인 경매를 접는다(서버가 다시 맡긴다). 맡은 작업은 끝까지 한다. 모드는 /fleet/robot_modes
(RobotFleetMode)로 1초마다·바뀔 때 알린다 - 서버는 분산 모드 로봇에게 작업을 맡기지 않는다.
fleet_mode:=decentral(기본)은 기존 분산 비교군 그대로 늘 분산이다.

먼저 수락한 쪽이 주인(amr_fleet_core.claims, 두 모드 공통): 다른 로봇이 그 작업을 맡았다고 TaskStatus로
보고했으면 그 작업의 경매를 열지 않고, 입찰 수집 중인 경매도 접는다. 다른 로봇이 먼저 맡아 물러난
FAILED·거부(ErrorInfo.ASSIGN_TASK_TAKEN)는 재경매하지 않는다.

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

처리 순서(#223 연구 1 처리 순서 수정, 하연·민지): 중앙은 대기열을 priority 내림차순 ->
deadline 오름차순 -> created 오름차순 -> task_id로 정렬해 꺼낸다(amr_fleet_core.assignment.
common.sort_by_priority). 공고 시각(header.stamp)은 순서에 안 쓴다 - 재경매하면 새로 찍혀서
밀린 작업이 새 작업처럼 보인다(created_time은 원본 그대로 실려 간다).

처리 순서 - 입찰은 열린 경매 전부에(#227 민지 리뷰 1): 처음에는(#223) IDLE 로봇이 입찰
가능한 열린 경매 중 순서가 가장 앞선 것 하나에만 입찰했다. 그런데 한가한 로봇 여럿이 거의 같은
순간 비게 되면(시작 직후, 포화에서 두 로봇이 비슷하게 끝날 때, 하이브리드 폴백 이어받기로 밀린
작업이 한꺼번에 열릴 때) 전부 똑같이 "지금 1순위" 작업 하나에만 몰려 그 한 건만 배정되고,
나머지 작업은 입찰이 없어 마감 -> no_bid_retry_s 뒤 재공고 -> 또 몰림을 반복하며 한 건씩만
배정된다(중앙은 한 틱에 다 배정). 그래서 지금은 IDLE 로봇이 입찰 가능한 열린 경매 전부에
입찰한다(_consider_bids) - 동시에 여러 경매에 낙찰될 수 있지만, 그 처리는 AWARD를 받는 쪽
(아래)에서 한다.

처리 순서 - 가장 앞선 AWARD만 수락(#227 민지 리뷰 2): 경매마다 결정 시점이 서로 다른
경매인(로봇) 프로세스의 타이머에서 따로 돌아, 우선순위와 무관하게 어느 AWARD가 먼저
도착하느냐가 뒤섞일 수 있다(네트워크 지연·손실이면 더 심하다 - 더 급한 작업의 AWARD가
재전송 간격만큼 늦게 와서, 그사이 받은 다른 AWARD를 먼저 수락해 버리는 식). 그래서 AWARD를
받아도 바로 수락하지 않고 일단 보류한다(_pending_awards). 아직 결과를 모르는(마감 전이거나
마감은 지났지만 AWARD를 못 받은, _bid_resends·_awaiting_award) 더 급한 입찰이 있으면
계속 보류하고, 그런 게 없으면 보류 중인 것 중 가장 앞선 것만 ACCEPT하고 나머지는 WITHDRAW한다
(_resolve_pending_awards). "아직 결과를 모른다"는 다른 로봇이 그 작업을 맡은 걸 알면
(_taken_by_other) 바로 끝나고, 아니면 award_wait_s(경매인이 ACCEPT 없이 재경매하기까지
걸리는 시간보다 짧게 둔다, params.py 참고) 뒤에 "졌다"로 보고 끝난다 - 그래야 정말로 이겼는데
AWARD가 늦게 오는 경우와, 져서 AWARD가 영영 안 오는 경우를 유한한 시간 안에 구분할 수 있다.
한 번 ACCEPT한 뒤에는(이미 실행 중일 수 있어 WITHDRAW 못 함, 10/7 결정) 그 뒤 온 다른 AWARD는
무조건 WITHDRAW한다 - 또 그 뒤 온 AWARD가 다시 오면(_withdrawn에 남아 있으면) 다시 WITHDRAW한다.
"""
from __future__ import annotations

import signal
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from amr_fleet_core.assignment.common import sort_by_priority
from amr_fleet_core.claims import TaskClaims
from amr_msgs.msg import (
    FleetHeartbeat, FleetTaskTable, RobotFleetMode, RobotState, Task, TaskAssignment, TaskAssignmentAck,
    TaskAuction, TaskBid, TaskStatus,
)
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .auction import (
    ERROR_CANCELED,
    ERROR_TASK_TAKEN,
    MODE_CENTRALIZED,
    MODE_DECENTRALIZED,
    AuctionParams,
    AuctionResult,
    Bid,
    FallbackMode,
    FallbackParams,
    RobotInfo,
    TaskInfo,
    TaskRetryLedger,
    assignment_id_for,
    assignment_topic,
    auction_opener,
    award_exhausted,
    backlog_task_ids,
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
# 하이브리드 로봇별 모드 (로봇 → 모두, 장애 주입 없음)
ROBOT_MODES_TOPIC = '/fleet/robot_modes'
FLEET_MODES = ('decentral', 'hybrid')
MODE_NAMES = {MODE_CENTRALIZED: '중앙', MODE_DECENTRALIZED: '분산'}


def _stamp_to_s(t: TimeMsg) -> float:
    return t.sec + t.nanosec * 1e-9


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
        priority=task.priority, created=_stamp_to_s(task.created_time),
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
            award_wait_s=p('award_wait_s', AuctionParams.award_wait_s).value,
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

        # 하이브리드 (architecture.md 5-2절). decentral = 늘 분산(기존 분산 비교군), hybrid = 평시 중앙
        self._fleet_mode = p('fleet_mode', 'decentral').value
        if self._fleet_mode not in FLEET_MODES:
            raise ValueError(f'fleet_mode는 {FLEET_MODES} 중 하나: {self._fleet_mode!r}')
        self._fallback = FallbackMode(FallbackParams(
            fallback_timeout_s=p('fallback_timeout_s', FallbackParams.fallback_timeout_s).value,
            return_stable_s=p('return_stable_s', FallbackParams.return_stable_s).value))
        # 서버 → 로봇 토픽 앞부분. 장애 주입기 중계본이면 'fleet'(→ /<robot_id>/fleet/...), 가짜 로봇과 같은 이름
        server_topic_prefix = p('server_topic_prefix', '/fleet').value
        self._last_table: Optional[FleetTaskTable] = None
        # 중앙 모드에서 받은 작업 방송 중 아직 작업표에 안 나온 것. 작업표는 1초마다, 분산 전환은 2초 끊긴 뒤라
        # 마지막 작업표 뒤 ~ 전환 사이에 들어온 작업은 표에 없다 - 분산으로 바뀌면 표의 미할당분과 같이 이어받는다
        self._central_requests: Dict[str, Task] = {}
        # 다른 로봇의 모드 (RobotFleetMode). 하이브리드에서 분산 모드 로봇끼리만 경매를 연다
        self._robot_modes: Dict[str, int] = {}
        # 먼저 수락한 쪽이 주인 - 다른 로봇이 맡은 작업은 경매를 열거나 이어가지 않는다. 죽은 로봇의 주장은
        # 낙찰 로봇 장애 회수와 같은 시간 뒤에 뺀다
        claim_timeout = self._params.reclaim_timeout_s if self._params.reclaim_timeout_s > 0 else float('inf')
        self._claims = TaskClaims(claim_timeout)

        self._robot_info: Optional[RobotInfo] = None
        # 내가 경매인인 경매들 (auctioneer_id는 항상 나 자신이라 auction_id만 키로 써도 되지만,
        # TaskBid 처리 쪽과 자료형을 맞추려고 (auctioneer_id, auction_id) 그대로 쓴다).
        self._auctions: Dict[AuctionKey, _AuctionState] = {}
        # 입찰 가능한(아직 마감 전, 아직 안 입찰한) 경매 후보 - 작업 원문(TaskInfo)을 들고 있다가
        # _consider_bids가 작업 순서(sort_by_priority)로 가장 앞선 것 하나만 고른다(#223 연구 1
        # 처리 순서 수정). 로봇 상태를 아직 몰라도·맡은 작업이 있어도 그냥 쌓인다 - _consider_bids가
        # 그 조건(_robot_info is not None, _active_task is None)을 보고 아무것도 안 할 뿐이다.
        self._bid_candidates: Dict[AuctionKey, TaskInfo] = {}
        # 입찰 마감(실제 시간) - 공고를 처음 받은 시각 + bid_window_s. #155·#171 BID
        # 재전송이 멈춰야 할 때(마감)를 판단하는 기준이라, 공고를 받을 때(_on_task_auction)
        # 마다 한 번만 찍는다(재전송·중계 에코로 다시 받아도 안 바꾼다).
        self._bid_deadline: Dict[AuctionKey, float] = {}
        # 재전송 중인 내 BID(마감 전, 결과를 아직 모른다) - key -> [TaskBid, 마감, 마지막 전송
        # 시각]. AWARD를 받거나(_on_awarded) 마감되면(_maybe_resend_bids, _awaiting_award로
        # 옮긴다) 꺼진다.
        self._bid_resends: Dict[AuctionKey, list] = {}
        # 마감은 지났지만 AWARD를 못 받은 입찰(#227 민지 리뷰 2) - key -> (작업, 마감이 지난
        # 시각). 졌는지 AWARD가 늦는지 모른다 - 다른 로봇이 그 작업을 맡은 걸 알거나
        # (_taken_by_other) award_wait_s가 지나면 "졌다"로 보고 뺀다(_resolve_pending_awards).
        self._awaiting_award: Dict[AuctionKey, Tuple[TaskInfo, float]] = {}
        # AWARD를 받았지만 더 급한 입찰 결과를 기다리느라 아직 ACCEPT·WITHDRAW를 안 보낸 경매
        # (#227 민지 리뷰 2) - key -> TaskBid(AWARD). _resolve_pending_awards가 더 급한 입찰이
        # 다 끝나면(이기거나 지거나) 그중 가장 앞선 것만 ACCEPT, 나머지는 WITHDRAW한다.
        self._pending_awards: Dict[AuctionKey, TaskBid] = {}
        # 보류 시작 시각(_now 기준, 로그용) - 얼마나 기다리다 수락했는지 남긴다.
        self._pending_since: Dict[AuctionKey, float] = {}
        # 입찰할 때 본 작업 내용 - AWARD를 받았을 때 재확인(decide_bid)하려면 필요하다
        # (TaskBid AWARD에는 task_id만 있고 좌표·마감이 없다).
        self._tasks_seen: Dict[AuctionKey, TaskInfo] = {}
        # 이미 ACCEPT를 보낸 경매 - WITHDRAW는 ACCEPT 전까지만(10/7 결정), 재전송된 AWARD에
        # 또 ACCEPT를 보내지 않는 데도 쓴다(멱등).
        self._accepted: set = set()
        # 보낸 ACCEPT - 같은 경매의 AWARD가 다시 오면(내 ACCEPT 유실) 그대로 다시 보낸다.
        self._accept_msgs: Dict[AuctionKey, TaskBid] = {}
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
        if self._fleet_mode == 'hybrid':
            self.create_subscription(
                FleetHeartbeat, server_topic_prefix + '/heartbeat', self._on_heartbeat, 10)
            self.create_subscription(
                FleetTaskTable, server_topic_prefix + '/task_table', self._on_task_table, 10)
            self.create_subscription(RobotFleetMode, ROBOT_MODES_TOPIC, self._on_robot_mode, 10)
            self._mode_pub = self.create_publisher(RobotFleetMode, ROBOT_MODES_TOPIC, 10)
            self.create_timer(1.0, self._publish_mode)

        self.get_logger().info(
            f'task_auction({self._robot_id}): 구독 {task_auctions_sub_topic} (공고), '
            f'{task_bids_sub_topic} (입찰/낙찰/수락), {TASK_STATUS_TOPIC} (실패 보고), '
            f'{robot_states_topic} (내 상태), {task_requests_topic} (작업 방송) · 발행 '
            f'{TASK_AUCTIONS_PUB_TOPIC}, {TASK_BIDS_PUB_TOPIC}, {self._assignment_topic} (수락한 작업), '
            f'fleet_mode={self._fleet_mode}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- 내 상태 ---

    def _on_robot_state(self, msg: RobotState):
        self._robot_last_seen[msg.robot_id] = time.monotonic()
        self._claims.robot_state(msg.robot_id, msg.current_task_id, time.monotonic())
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
        if not had_info:
            # 내 상태를 처음 알았다 - 그걸 몰라서 못 고르던 입찰 후보를 지금 다시 본다.
            # _bid_candidates는 로봇 상태를 몰라도 쌓여 있었다(마감 전인 것은 _consider_bids가
            # 알아서 거른다, 10/10 연우 리뷰 #109와 같은 취지).
            self._consider_bids()

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
        # 입찰 후보로 쌓아 두고 지금 가장 급한 것 하나를 고른다(_consider_bids가 이미 입찰
        # 중이거나(_accepted)·포기한(_withdrawn) 경매는 거른다) - 로봇 상태를 몰라도·맡은
        # 작업이 있어도 그냥 쌓인다(10/10 연우 리뷰 #109와 같은 취지, 둘 다 _consider_bids가
        # 조건으로 거를 뿐 후보 자체는 잃지 않는다).
        self._bid_candidates[key] = self._tasks_seen[key]
        self._consider_bids()

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
        if not self._decentral():
            # 중앙 모드(하이브리드)면 서버가 맡긴다. 본 것으로 적지 않고 모아 둔다 - 작업표에 나오면 지우고, 그 전에
            # 분산으로 바뀌면 이어받는다(_take_over_backlog)
            self._central_requests[task_id] = msg
            return
        self._task_ids_seen.add(task_id)
        if self._taken_by_other(task_id):
            return
        robots = self._auction_robots()
        opener = auction_opener(task_id, robots)
        if opener is None:
            self.get_logger().warn(f'작업 {task_id}: 살아 있는 로봇이 없어 경매를 못 연다')
            return
        if opener == self._robot_id:
            self._open_auction(msg)

    def _decentral(self) -> bool:
        return self._fleet_mode == 'decentral' or self._fallback.mode == MODE_DECENTRALIZED

    def _auction_robots(self) -> List[str]:
        """경매를 열 로봇 후보: 살아 있는 로봇. 하이브리드면 그중 분산 모드인 로봇만 (중앙 모드 로봇은 서버를 따른다)."""
        robots = live_robots(self._robot_last_seen, time.monotonic(), self._alive_timeout_s)
        if self._fleet_mode == 'decentral':
            return robots
        return [r for r in robots
                if r == self._robot_id or self._robot_modes.get(r) == MODE_DECENTRALIZED]

    def _taken_by_other(self, task_id: str) -> bool:
        return self._claims.owner(task_id, time.monotonic(), exclude=self._robot_id) is not None

    # --- 하이브리드: 모드 전환 ---

    def _on_heartbeat(self, msg: FleetHeartbeat):
        self._on_mode(self._fallback.on_heartbeat(time.monotonic(), msg.mode))

    def _on_task_table(self, msg: FleetTaskTable):
        self._last_table = msg
        # 서버가 아는 작업은 작업표로 이어받는다 (미할당이면 경매, 다른 로봇에게 맡겼으면 서버에 맡김)
        for r in msg.records:
            self._central_requests.pop(r.task.task_id, None)

    def _on_robot_mode(self, msg: RobotFleetMode):
        if msg.robot_id != self._robot_id:
            self._robot_modes[msg.robot_id] = msg.mode

    def _publish_mode(self):
        out = RobotFleetMode()
        out.header.stamp = self.get_clock().now().to_msg()
        out.robot_id = self._robot_id
        out.mode = self._fallback.mode
        out.transitions = self._fallback.transitions
        self._mode_pub.publish(out)

    def _on_mode(self, mode: Optional[int]):
        if mode is None:
            return
        self.get_logger().warn(
            f'{MODE_NAMES[mode]} 모드로 전환 ({self._fallback.transitions}번째) t={self._now():.3f}')
        self._publish_mode()
        if mode == MODE_DECENTRALIZED:
            self._take_over_backlog()
        else:
            self._hand_back_to_server()

    def _take_over_backlog(self):
        """분산으로 바뀌었다: 마지막 작업표의 미할당 작업과 그 뒤 들어온 작업 방송 중 내가 열 차례인 것의 경매를 연다."""
        if not self._open_auctions:
            return
        robots = self._auction_robots()
        records = self._last_table.records if self._last_table is not None else []
        tasks = {r.task.task_id: r.task for r in records}
        backlog = backlog_task_ids(records, self._robot_id)
        for task_id, task in self._central_requests.items():
            if task_id not in tasks:
                tasks[task_id] = task
                backlog.append(task_id)
        self._central_requests.clear()
        opened = 0
        for task_id in backlog:
            if task_id in self._task_ids_seen or self._taken_by_other(task_id):
                continue
            self._task_ids_seen.add(task_id)
            if auction_opener(task_id, robots) == self._robot_id:
                self._open_auction(tasks[task_id])
                opened += 1
        seq = self._last_table.seq if self._last_table is not None else '-'
        self.get_logger().info(
            f'이어받기(작업표 seq {seq} + 그 뒤 방송): 후보 {len(backlog)}건, 경매 {opened}건 열음, 경매 참여 로봇 {robots}')

    def _hand_back_to_server(self):
        """중앙으로 돌아왔다: 아직 낙찰 전인 경매는 접고(서버가 다시 맡긴다), 확정된 경매는 감시를 끝낸다.
        맡은 작업은 끝까지 한다 - 서버는 그 로봇의 TaskStatus를 보고 장부에 올린다."""
        for key, state in list(self._auctions.items()):
            if state.phase in ('collecting', 'awarded'):
                del self._auctions[key]
                # 다음에 또 분산으로 바뀌면 작업표에서 다시 이어받을 수 있게
                self._task_ids_seen.discard(state.task.task_id)
            elif state.phase == 'done':
                state.phase = 'completed'
        self._bid_candidates.clear()
        # 남의 경매에 넣은 입찰도 거둔다 - 중앙 로봇이 낙찰되면 WITHDRAW하느라 그 경매가 늦어진다 (#215 리뷰 2)
        self._bid_resends.clear()
        self._awaiting_award.clear()
        self._pending_awards.clear()
        self._pending_since.clear()

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

    def _consider_bids(self):
        """입찰 후보(_bid_candidates) 중 지금 입찰 가능한 것 전부에 입찰한다(#227 민지 리뷰 1 -
        열린 경매 중 하나에만 입찰하면, 한가한 로봇 여럿이 거의 같은 순간 비었을 때 전부 같은
        1순위 작업에만 몰려 동시에 열린 N건이 한 라운드에 1건씩만 배정된다). 여러 경매에 동시에
        낙찰될 수 있지만, 그중 어느 AWARD를 수락할지는 여기서 정하지 않는다 - AWARD가 올 때마다
        _on_awarded·_resolve_pending_awards가 작업 순서(amr_fleet_core.assignment.common.
        sort_by_priority와 같은 키)로 가장 앞선 것만 고른다.

        로봇 상태를 아직 모르거나(_robot_info) 맡은 작업이 있으면(_active_task, 10/7
        결정: IDLE 로봇만 입찰) 아무것도 안 한다 - 후보는 그대로 쌓여 있다가 로봇 상태를
        처음 알거나(_on_robot_state) 맡은 작업이 풀리면(_release_active_task) 이 함수가
        다시 불려 그때 입찰한다(10/10 연우 리뷰 #109와 같은 취지 - 몰라서 못 고른 걸
        "결정 끝"으로 잃지 않는다). 분산 모드가 아니어도(하이브리드 중앙) 아무것도 안 한다 -
        후보는 쌓이지만 중앙으로 있는 동안은 입찰하지 않는다.
        """
        if not self._decentral() or self._robot_info is None or self._active_task is not None:
            return
        now = self._now()
        for key, task in list(self._bid_candidates.items()):
            if now > self._bid_deadline.get(key, 0.0):
                # 마감을 지난 것만 지운다(now == deadline은 아직 유효 - 경매인도 bid_time <=
                # deadline으로 받아 준다, decision.decide_winner). 틱 간격이 굵으면 "경매인이
                # 막 마감을 넘겨 결정하는 순간"과 "이 틱에서 처음 입찰할 수 있는 순간"이 같은
                # 틱에 겹칠 수 있는데, 여기서 >=로 먼저 지워 버리면 그 순간에 아직 입찰 안 한
                # 후보가 영영 입찰 기회를 못 얻고 매번 재공고로 밀린다.
                del self._bid_candidates[key]
            elif (key in self._accepted or key in self._withdrawn or key in self._bid_resends
                  or key in self._awaiting_award or key in self._pending_awards
                  or self._taken_by_other(task.task_id)):
                # 이미 입찰을 보냈거나(_bid_resends·_awaiting_award) 낙찰을 보류 중이면
                # (_pending_awards) 또 보내지 않는다 - _on_task_auction은 공고 에코·재공고
                # 구분 없이 후보를 다시 쌓으므로, 여기서 걸러야 같은 경매에 BID를 두 번 안 보낸다.
                del self._bid_candidates[key]
            elif decide_bid(self._robot_info, task, now, self._params) is not None:
                del self._bid_candidates[key]
                self._send_bid(key, task, now)
            # else: 지금은 자격 없음(마감을 못 맞춤 등) - 후보로 남겨 다음 틱에 사정이
            # 바뀌었는지 다시 본다(10/10 연우 리뷰 #109와 같은 취지).

    def _send_bid(self, key: AuctionKey, task: TaskInfo, now: float):
        bid = decide_bid(self._robot_info, task, now, self._params)
        if bid is None:
            return  # 호출 쪽이 이미 거른다 - 방어적으로만 둔다

        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id, out.auction_id = key
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

    def _withdraw_bid(self, key: AuctionKey):
        """아직 AWARD 전인 내 입찰을 거둔다 - 다른 로봇이 그 작업을 먼저 맡은 걸 알았을 때
        (_consider_bids·_maybe_resend_bids), 또는 다른 경매에 이미 ACCEPT해 더는 이 경매를
        맡을 수 없을 때(_accept_award) 쓴다. 한 번 거둔 경매는 _withdrawn에 남아, 뒤늦게
        AWARD가 와도(네트워크 경쟁) should_withdraw_award가 다시 포기하게 한다(#227 민지
        리뷰 2 - AWARD가 여러 개 와도 가장 급한 쪽만 수락)."""
        self._bid_resends.pop(key, None)
        self._awaiting_award.pop(key, None)
        self._withdrawn.add(key)
        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id, out.auction_id = key
        out.task_id = self._tasks_seen[key].task_id
        out.robot_id = self._robot_id
        out.type = TaskBid.TYPE_WITHDRAW
        self._bid_pub.publish(out)
        if key[0] == self._robot_id:
            self._on_withdraw(key, self._robot_id)

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
        elif self._is_own_loop(msg):
            # 내 경매에 내가 보낸 AWARD·ACCEPT·WITHDRAW의 중계 에코 - 보낼 때 바로 처리했다(_send_award·
            # _on_awarded). 자기 자신과는 네트워크 없이 주고받는다: 끊긴 로봇도 혼자 경매해 스스로 맡을 수 있어야
            # 하고(하이브리드), 중계 지연이 자기 경매에 두 번 걸리지 않게 한다
            return
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

    def _is_own_loop(self, msg: TaskBid) -> bool:
        if msg.auctioneer_id != self._robot_id:
            return False
        if msg.type == TaskBid.TYPE_AWARD:
            return msg.winner_id == self._robot_id
        return msg.type in (TaskBid.TYPE_ACCEPT, TaskBid.TYPE_WITHDRAW) and msg.robot_id == self._robot_id

    def _on_awarded(self, key: AuctionKey, msg: TaskBid):
        # AWARD를 받았다는 건 경매인이 이미 낙찰자(나)를 정했다는 뜻 - 더 입찰을
        # 재전송할 이유가 없다(#155·#171, ACCEPT·WITHDRAW 어느 쪽이든 마찬가지). 결과를
        # 몰라 기다리던 입찰이었으면(_awaiting_award) 이제 결과를 안다.
        self._bid_resends.pop(key, None)
        self._awaiting_award.pop(key, None)
        if key in self._accepted:
            # 재전송된 AWARD - 이미 수락했다. 경매인이 AWARD를 다시 보낸 건 내 ACCEPT를 못 받았다는
            # 뜻일 수 있으니 같은 ACCEPT를 다시 보낸다(안 보내면 경매인은 재전송을 다 쓰고 재경매해
            # 내가 이미 실행 중인 작업을 다른 로봇도 맡는다). 작업 실행(TaskAssignment)은 다시 하지
            # 않는다 - 처음 보낸 것이 같은 assignment_id로 Ack까지 재전송 중이다.
            self._bid_pub.publish(self._accept_msgs[key])
            if msg.auctioneer_id == self._robot_id:
                self._on_accept(key, self._robot_id)
            return
        task = self._tasks_seen.get(key)
        now = self._now()
        # 입찰했을 때와 사정이 바뀌었을 수 있으니(배터리·IDLE 여부) 다시 확인한다. 이미 다른
        # 작업을 ACCEPT했거나 이 경매를 한 번 포기했으면 수락하지 않는다(동시 낙찰 경합) -
        # 이미 ACCEPT한 작업이 있으면 더 기다릴 이유가 없다(못 바꾼다, 10/7 결정).
        holding_other_task = self._active_task is not None and self._active_task[0] != key
        still_eligible = (
            self._decentral()
            and not should_withdraw_award(holding_other_task, key in self._withdrawn)
            and self._robot_info is not None and task is not None
            and decide_bid(self._robot_info, task, now, self._params) is not None
        )
        if not still_eligible:
            self._decline_award(key, msg)
            return
        # 바로 수락하지 않는다 - 더 급한(sort_by_priority로 앞선) 입찰 결과를 아직 모르면
        # 그걸 기다린다(#227 민지 리뷰 2, _resolve_pending_awards).
        self._pending_awards[key] = msg
        self._pending_since.setdefault(key, now)
        self._resolve_pending_awards(now)

    def _decline_award(self, key: AuctionKey, msg: TaskBid):
        """AWARD에 ACCEPT 대신 WITHDRAW를 보낸다(10/7 결정: WITHDRAW는 ACCEPT 전까지만 -
        이 경매에는 아직 ACCEPT를 안 보냈으니 항상 가능하다)."""
        self._withdrawn.add(key)
        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = msg.auctioneer_id
        out.auction_id = msg.auction_id
        out.task_id = msg.task_id
        out.robot_id = self._robot_id
        out.type = TaskBid.TYPE_WITHDRAW
        self._bid_pub.publish(out)
        if msg.auctioneer_id == self._robot_id:
            self._on_withdraw(key, self._robot_id)

    def _accept_award(self, key: AuctionKey, msg: TaskBid):
        """보류 없이 바로 ACCEPT한다(_resolve_pending_awards가 가장 급한 보류 AWARD를 고른
        뒤에만 부른다)."""
        out = TaskBid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.auctioneer_id = msg.auctioneer_id
        out.auction_id = msg.auction_id
        out.task_id = msg.task_id
        out.robot_id = self._robot_id
        out.type = TaskBid.TYPE_ACCEPT
        self._accepted.add(key)
        self._accept_msgs[key] = out
        self._active_task = (key, msg.task_id)
        self._active_accepted_at = time.monotonic()
        self._active_seen_busy = False
        self._bid_pub.publish(out)
        held_s = self._now() - self._pending_since.pop(key, self._now())
        self.get_logger().info(
            f'경매 {key}: 작업 {msg.task_id} ACCEPT t={self._now():.3f}'
            + (f' (더 급한 입찰 결과를 {held_s:.3f}s 기다림)' if held_s > 1e-6 else ''))
        if msg.auctioneer_id == self._robot_id:
            self._on_accept(key, self._robot_id)
        if self._run_accepted_tasks:
            self._send_assignment(key, msg.task_id)
        # 동시에 다른 경매에도 입찰해 둔 게 있으면(#227 민지 리뷰 1 - 열린 경매 전부에
        # 입찰한다) 이제 맡을 수 없으니 거둔다. 결과를 기다리던 것도 더는 안 궁금하다.
        for other_key in list(self._bid_resends):
            if other_key != key:
                self._withdraw_bid(other_key)
        self._awaiting_award.clear()

    def _resolve_pending_awards(self, now: Optional[float] = None):
        """보류 중인 AWARD(_pending_awards) 중 가장 앞선(sort_by_priority) 것만 ACCEPT하고
        나머지는 WITHDRAW한다(#227 민지 리뷰 2). 아직 결과를 모르는(마감 전이거나 마감은
        지났지만 AWARD를 못 받은, _bid_resends·_awaiting_award) 더 급한 입찰이 있으면
        기다린다 - 다른 로봇이 그 작업을 맡은 걸 알면(_taken_by_other) 바로 끝난다.

        시간으로 끝나는 기준은 "AWARD를 받은 시점"부터 award_wait_s다(_pending_since) -
        더 급한 입찰의 마감 시점(_awaiting_award 진입 시각)부터 재면, 그 입찰의 마감이 멀거나
        (입찰 창이 길거나 아직 안 닫혔으면 _bid_resends에 그대로 있다) 이미 한참 지나 있으면
        AWARD를 받고도 award_wait_s보다 훨씬 오래 보류할 수 있어, 그사이 이 AWARD의 경매인이
        ACCEPT 없이 먼저 재경매해 버릴 수 있다(그러면 award_wait_s를 경매인의 재경매
        타임아웃보다 짧게 둔 의미가 없어진다). 그래서 보류 중인 것 중 가장 앞선 것이 award_wait_s
        보다 오래 기다렸으면, 더 급한 입찰 결과를 몰라도 포기하고 그걸 수락한다 - 포기한 입찰은
        나중에 AWARD가 와도 이미 다른 작업을 ACCEPT한 뒤라 거절된다(holding_other_task).

        AWARD가 올 때(_on_awarded)뿐 아니라 매 틱(_on_timer)·TaskStatus를 받을 때
        (_on_task_status, 다른 로봇이 그 작업을 맡았다고 보고하는 경우)도 불러, 시간이
        지나거나 다른 로봇이 맡은 걸 알게 되는 것만으로도(새 AWARD 없이) 보류가 풀리게 한다.
        """
        if now is None:
            now = self._now()
        for key, (task, deadline_passed_at) in list(self._awaiting_award.items()):
            if self._taken_by_other(task.task_id) or now - deadline_passed_at > self._params.award_wait_s:
                del self._awaiting_award[key]
        if not self._pending_awards or self._active_task is not None:
            return

        pending = {key: self._tasks_seen[key] for key in self._pending_awards}
        best_pending_task = sort_by_priority(list(pending.values()))[0]
        best_key = next(k for k, t in pending.items() if t is best_pending_task)

        if now - self._pending_since[best_key] <= self._params.award_wait_s:
            unresolved = [
                self._tasks_seen[key] for key in self._bid_resends
                if not self._taken_by_other(self._tasks_seen[key].task_id)]
            unresolved += [task for task, _ in self._awaiting_award.values()]
            if any(sort_by_priority([task, best_pending_task])[0] is task for task in unresolved):
                return  # 더 급한 입찰 결과를 아직 모르고, 기다린 지 award_wait_s도 안 됐다

        self._accept_award(best_key, self._pending_awards.pop(best_key))
        for key, msg in list(self._pending_awards.items()):
            del self._pending_awards[key]
            self._decline_award(key, msg)

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
        start = _stamp_to_s(msg.start_time) or _stamp_to_s(msg.header.stamp)
        self._claims.observe(msg.task_id, msg.robot_id, msg.state, start, time.monotonic())
        # 다른 로봇이 이 작업을 맡은 걸 지금 알았을 수 있다 - 그 작업을 기다리느라 보류 중인
        # AWARD가 있으면 다음 틱까지 안 기다리고 바로 풀어 준다(#227 민지 리뷰 2).
        self._resolve_pending_awards()
        if msg.robot_id != self._robot_id and msg.state != TaskStatus.STATE_FAILED:
            # 다른 로봇이 맡았다(서버 할당이든 다른 경매든) - 낙찰 전인 내 경매는 접는다
            for key, state in list(self._auctions.items()):
                if state.task.task_id == msg.task_id and state.phase == 'collecting':
                    del self._auctions[key]
                    self.get_logger().info(f'경매 {key}: {msg.robot_id}가 작업 {msg.task_id}을 맡아 접는다')
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
        elif error_code == ERROR_TASK_TAKEN:
            # 다른 로봇이 먼저 맡았다 - 주인이 하고 있으니 재경매하지 않는다 (architecture.md 5-2절)
            state.phase = 'completed'
            self.get_logger().info(f'경매 {key}: 작업 {task_id}을 다른 로봇이 먼저 맡음(ASSIGN_TASK_TAKEN) - 재경매 안 함')
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
        # 작업 중에도 _bid_candidates는 계속 쌓여 있었다 - 지금 다시 골라 입찰한다
        # (마감 지난 것은 _consider_bids가 거른다).
        self._consider_bids()

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
        self._release_active_task(f'commit_timeout_s {self._params.commit_timeout_s:g}초')

    def _on_timer(self):
        if self._fleet_mode == 'hybrid':
            self._on_mode(self._fallback.update(time.monotonic()))
        now = self._now()
        self._maybe_resend_bids(now)
        self._maybe_resend_assignments()
        self._maybe_release_uncommitted_task()
        # 새 공고 없이도 마감으로 입찰 후보가 늘 수 있다(마감 지난 후보가 사라지는 경우) - 매 틱 다시 본다.
        self._consider_bids()
        # 시간이 지나거나(award_wait_s) 다른 로봇이 그 작업을 맡은 걸 알게 되는 것만으로도
        # (새 AWARD 없이) 보류 중인 AWARD가 풀릴 수 있다 - 매 틱 다시 본다(#227 민지 리뷰 2).
        self._resolve_pending_awards(now)
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
        (_on_awarded) 이미 꺼져 있다. 마감이 지나도 AWARD를 못 받았으면 지우지 않고
        _awaiting_award로 옮긴다(#227 민지 리뷰 2) - 졌는지 AWARD가 늦는지 몰라서, 더 급한
        보류 중인 AWARD가 있으면 그 결과를 기다리게 한다(_resolve_pending_awards)."""
        for key, entry in list(self._bid_resends.items()):
            msg, deadline, last_sent = entry
            task = self._tasks_seen[key]
            if self._taken_by_other(task.task_id):
                # 마감 전에 이미 다른 로봇이 맡은 걸 알았다 - 기다릴 것도 없이 거둔다.
                self._withdraw_bid(key)
                continue
            if now >= deadline:
                self._awaiting_award[key] = (task, now)
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
        if state.winner_id == self._robot_id:
            # 내 경매에 내가 낙찰 - 중계를 거치지 않고 바로 받는다 (_is_own_loop)
            self._on_awarded(key, out)

    def _reauction(self, key: AuctionKey, state: _AuctionState):
        owner = self._claims.owner(state.task.task_id, time.monotonic(), exclude=self._robot_id)
        if owner is not None and owner != state.winner_id:
            # 그 사이 다른 로봇(서버 할당·다른 경매)이 맡았다 - 다시 경매하면 같은 작업을 두 로봇이 한다
            self._auctions.pop(key, None)
            self.get_logger().info(f'경매 {key}: 작업 {state.task.task_id}을 {owner}가 맡아 재경매하지 않는다')
            return
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
            # 원본 작업 그대로(종류·물품·스테이션 id, priority·created_time도) - TaskAssignment로
            # 로봇에게 그대로 가고, 처리 순서(#223)도 재경매 전과 똑같이 매겨진다.
            out.task = state.task_msg
        else:
            out.task.task_id = state.task.task_id
            out.task.dropoff_pose.position.x, out.task.dropoff_pose.position.y = state.task.dropoff_xy
            out.task.priority = state.task.priority
            out.task.created_time = _epoch_to_time(state.task.created)
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
