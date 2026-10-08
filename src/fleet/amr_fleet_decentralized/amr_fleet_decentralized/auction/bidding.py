"""로봇 쪽 입찰가 계산. 참여 로봇이 TaskAuction 공고를 받으면 호출한다."""
import math
from typing import Optional

from .params import AuctionParams
from .types import MODE_IDLE, Bid, RobotInfo, TaskInfo


def compute_bid_cost(robot: RobotInfo, task: TaskInfo) -> float:
    """로봇 → 작업 시작 지점 직선 거리(m).

    10/7 하연·이진 결정(#91 리뷰 제안): 명세 3장 "가장 가까운 로봇 선점"에 맞춰
    중앙 할당(amr_fleet_core.assignment.cost.euclidean_cost, nearest 규칙)과 같은
    식을 쓴다 - 분산·중앙 비교군의 비용 계산이 같아야 공정하다(연구 1). 픽업 →
    목적지 구간은 어느 로봇이 맡아도 같으므로 안 넣는다. IDLE 로봇만 입찰하므로
    "진행 중인 작업을 마치는 지점"(큐) 보정은 필요 없다.
    """
    return math.dist(robot.xy, task.start_xy)


def estimate_finish_time(cost_m: float, now_s: float, params: AuctionParams) -> float:
    """단순 등속 추정: 지금부터 cost_m만큼 더 이동하면 끝나는 시각(epoch 초)."""
    return now_s + cost_m / params.robot_speed_mps


def decide_bid(
    robot: RobotInfo,
    task: TaskInfo,
    now_s: float,
    params: AuctionParams = AuctionParams(),
) -> Optional[Bid]:
    """이 로봇이 이 작업에 입찰할지 정한다. 입찰하지 않으면 None.

    입찰하지 않는 경우:
    - IDLE이 아님 (10/7 결정: 명세 3장 "가장 가까운 로봇 선점"에 맞춰 IDLE 로봇만 입찰).
    - 배터리가 `min_battery_percent` 미만 (10/7 결정: 중앙집중(fleet_manager)과 같은
      20% - 비교군 조건을 같게 둔다).
    - 예상 완료 시각이 `task.deadline`을 넘김 (TaskBid.msg 주석: "Task.deadline
      넘기면 무효" - 넘기는 입찰은 애초에 내지 않는다).
    """
    if robot.mode != MODE_IDLE:
        return None
    if robot.battery_percent < params.min_battery_percent:
        return None

    cost = compute_bid_cost(robot, task)
    finish = estimate_finish_time(cost, now_s, params)

    if task.deadline is not None and finish > task.deadline:
        return None

    return Bid(
        robot_id=robot.robot_id,
        task_id=task.task_id,
        cost=cost,
        estimated_finish_time=finish,
        bid_time=now_s,
    )
