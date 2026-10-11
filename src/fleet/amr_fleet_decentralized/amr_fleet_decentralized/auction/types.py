"""경매(TaskAuction/TaskBid) 로직의 입출력 타입. ROS 비의존 - amr_msgs를 import하지 않는다.

필드 이름·MODE_* 값은 amr_fleet_core/assignment/types.py(하연, 중앙 할당)와 되도록
맞췄다(robot_id, battery_percent, mode 등 - amr_msgs/msg/RobotState.MODE_*와 동일한
값). 다만 도메인이 달라 공통 타입을 import하지는 않는다 - CONTRIBUTING: *_core는
ROS 비의존 라이브러리이고 도메인별로 독립이다.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

Point = Tuple[float, float]

# amr_msgs/msg/RobotState.MODE_* 와 같은 값
MODE_IDLE = 0
MODE_BUSY = 1
MODE_CHARGING = 2
MODE_ERROR = 3
MODE_ESTOP = 4


@dataclass(frozen=True)
class RobotInfo:
    """경매에 입찰할 로봇 1대의 현재 상태.

    IDLE 로봇만 입찰한다(#91 리뷰 제안을 10/7 하연·이진이 결정: 명세 3장 "가장 가까운
    로봇 선점"에 맞춘다) - 그래서 "진행 중인 작업을 마치는 지점"같은 큐 개념은 없다.
    """

    robot_id: str
    xy: Point
    battery_percent: float
    mode: int = MODE_IDLE


@dataclass(frozen=True)
class TaskInfo:
    """경매에 부칠 작업 1건. amr_fleet_core.assignment.types.TaskInfo와 같은 패턴
    (pickup_xy 없으면 dropoff_xy를 시작점으로 본다 - CHARGE·RETURN처럼 pickup이 없는
    작업)."""

    task_id: str
    dropoff_xy: Point
    pickup_xy: Optional[Point] = None
    deadline: Optional[float] = None  # epoch 초. None이면 마감 없음

    @property
    def start_xy(self) -> Point:
        return self.pickup_xy if self.pickup_xy is not None else self.dropoff_xy


@dataclass(frozen=True)
class Bid:
    """로봇 1대가 작업 1건에 낸 입찰(TaskBid TYPE_BID 1건에 대응).

    bid_time: decide_winner에 넘길 때는 "경매인이 이 입찰을 받은 시각"이어야 한다
    (10/7 결정: 입찰 마감은 경매인이 받은 시각 기준) - 입찰자가 보낸 시각(네트워크
    지연·시계 오차가 있을 수 있다)을 그대로 믿지 않는다. decide_bid가 막 만든 Bid는
    "지금 입찰하기로 정한 시각"이라 이 의미와 같다(자기 자신의 결정 시각이므로).
    노드가 다른 로봇의 TaskBid를 받아 Bid로 옮길 때는 반드시 수신 시각(자기 시계)으로
    덮어써야 한다 - TaskBid.header.stamp를 쓰면 안 된다.
    """

    robot_id: str
    task_id: str
    cost: float
    estimated_finish_time: float
    bid_time: float


@dataclass(frozen=True)
class AuctionResult:
    """경매 1건의 낙찰 결과(TaskBid TYPE_AWARD에 대응). 입찰이 없으면 winner_id=None."""

    task_id: str
    winner_id: Optional[str] = None
    winning_cost: Optional[float] = None
    winning_bid: Optional[Bid] = None
