"""작업 할당의 입출력 타입. amr_msgs를 import하지 않는다 (CONTRIBUTING: *_core는 ROS 비의존)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

Point = Tuple[float, float]

# amr_msgs/msg/Task.TYPE_* 와 같은 값
TYPE_DELIVERY = 0
TYPE_CHARGE = 1
TYPE_RETURN = 2

# amr_msgs/msg/RobotState.MODE_* 와 같은 값
MODE_IDLE = 0
MODE_BUSY = 1
MODE_CHARGING = 2
MODE_ERROR = 3
MODE_ESTOP = 4


@dataclass(frozen=True)
class TaskInfo:
    """할당 대상 작업 1건. 좌표는 map 기준 (m), 시각은 초."""

    task_id: str
    task_type: int
    dropoff_xy: Point
    priority: int
    created: float
    # CHARGE·RETURN은 픽업이 없다
    pickup_xy: Optional[Point] = None
    # 마감 없음이면 None
    deadline: Optional[float] = None

    @property
    def start_xy(self) -> Point:
        """로봇이 처음 가야 할 곳: 픽업이 있으면 픽업, 없으면 목적지."""
        return self.pickup_xy if self.pickup_xy is not None else self.dropoff_xy


@dataclass(frozen=True)
class RobotInfo:
    """할당 후보 로봇 1대."""

    robot_id: str
    xy: Point
    battery_percent: float
    mode: int
    # 부하균형용 누적 할당 수. RobotState에 없는 값이라 서버가 센다
    load: int = 0


@dataclass
class AssignmentResult:
    """할당 결과. 알고리즘 3종이 같은 형식을 돌려준다."""

    # task_id -> robot_id
    assignments: dict[str, str] = field(default_factory=dict)
    # 이번에 할당 못 한 task_id. 서버 큐에 남아 다음 호출로 넘어간다
    unassigned: list[str] = field(default_factory=list)
    # 할당된 짝의 비용 합. 알고리즘 간 최적성 비교에 쓴다
    total_cost: float = 0.0


CostFn = Callable[[RobotInfo, TaskInfo], float]
