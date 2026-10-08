"""경매(TaskAuction/TaskBid) 로직의 입출력 타입. ROS 비의존 - amr_msgs를 import하지 않는다.

필드 이름은 amr_fleet_core/assignment/types.py(하연, 중앙 할당)와 되도록 맞췄다
(robot_id, battery_percent 등). 다만 도메인이 달라 공통 타입을 import하지는
않는다 - CONTRIBUTING: *_core는 ROS 비의존 라이브러리이고 도메인별로 독립이다.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class RobotInfo:
    """경매에 입찰할 로봇 1대의 현재 상태.

    진행 중인 작업이 있으면 그 작업을 마치는 지점(`current_task_dropoff_xy`)과
    남은 거리(`current_task_remaining_m`)를 넣는다 - 입찰가는 "지금 위치"가
    아니라 "현재 큐 끝"에서부터 계산한다(TaskBid.msg 주석과 동일한 정의).
    """

    robot_id: str
    xy: Point
    battery_percent: float
    current_task_dropoff_xy: Optional[Point] = None
    current_task_remaining_m: float = 0.0


@dataclass(frozen=True)
class TaskInfo:
    """경매에 부쳐진 작업 1건."""

    task_id: str
    pickup_xy: Point
    dropoff_xy: Point
    deadline: Optional[float] = None  # epoch 초. None이면 마감 없음


@dataclass(frozen=True)
class Bid:
    """로봇 1대가 작업 1건에 낸 입찰(TaskBid TYPE_BID 1건에 대응)."""

    robot_id: str
    task_id: str
    cost: float
    estimated_finish_time: float
    bid_time: float  # 입찰을 낸 시각(epoch 초) - 마감 판정에 쓴다


@dataclass(frozen=True)
class AuctionResult:
    """경매 1건의 낙찰 결과(TaskBid TYPE_AWARD에 대응). 입찰이 없으면 winner_id=None."""

    task_id: str
    winner_id: Optional[str] = None
    winning_cost: Optional[float] = None
    winning_bid: Optional[Bid] = None
