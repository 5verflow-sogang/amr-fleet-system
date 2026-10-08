"""로봇 쪽 입찰가 계산. 참여 로봇이 TaskAuction 공고를 받으면 호출한다."""
import math
from typing import Optional

from .params import AuctionParams
from .types import Bid, RobotInfo, TaskInfo


def compute_bid_cost(robot: RobotInfo, task: TaskInfo) -> float:
    """이 작업을 맡으면 늘어나는 이동 거리(m) - TaskBid.cost(한계비용)와 같은 정의.

    "현재 큐 끝(진행 중인 작업을 마치는 지점, 없으면 현재 위치) -> pickup ->
    dropoff" 증가분이다. 진행 중인 작업이 있으면 그걸 마칠 때까지 남은 거리
    (`current_task_remaining_m`)도 더한다.
    """
    queue_end = robot.current_task_dropoff_xy if robot.current_task_dropoff_xy is not None else robot.xy
    return (
        robot.current_task_remaining_m
        + math.dist(queue_end, task.pickup_xy)
        + math.dist(task.pickup_xy, task.dropoff_xy)
    )


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
    - 배터리가 `min_battery_percent` 미만 (낙찰돼도 못 끝낼 로봇은 처음부터 빼둔다).
    - 예상 완료 시각이 `task.deadline`을 넘김 (TaskBid.msg 주석: "Task.deadline
      넘기면 무효" - 넘기는 입찰은 애초에 내지 않는다).
    """
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
