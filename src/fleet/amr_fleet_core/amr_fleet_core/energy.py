"""에너지 예측 배정 (배터리 인지 운영 2단계). ROS 비의존.

고정 하한(min_battery_percent) 대신 "이 작업을 끝내고 충전기까지 갈 수 있는지"를 본다.

    필요 배터리 = (현재 위치 → 픽업 → 배달지 → 가장 가까운 충전기) 직선 거리 × path_factor × 1 m당 소모 + reserve_percent

1 m당 소모는 이동 소모(move_per_m)에 그 거리를 가는 동안의 정지 소모(idle_per_s / speed_mps)를 더한 값이다.
배터리가 모자란 로봇은 그 작업에서만 빠진다 (짧은 작업은 계속 받는다). 충전 작업은 충전기까지만 본다.

충전 대기열(ChargeQueue)을 주면, 작업을 마친 뒤 배터리가 충전 기준(low_percent) 밑이라 충전 차례를 기다려야 하는
경우에 그 대기 시간 동안의 정지 소모(idle_per_s × 대기 s)를 더한다. 최소 운영 대수(N) 때문에 동시 충전이 묶이면
작업을 끝낸 로봇이 충전기로 바로 못 가고 서서 닳는다 (#312 뒤 비교 실행: 21 %로 끝내고 90초 기다려 3.6 %로 도착).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from .assignment.types import CostFn, Point, RobotInfo, TaskInfo
from .charging import Station

# amr_msgs/msg/Task.TYPE_CHARGE
TYPE_CHARGE = 1

Feasible = Callable[[RobotInfo, TaskInfo], bool]


@dataclass(frozen=True)
class EnergyModel:
    """로봇 배터리 모델 추정 (amr_bt battery.* 파라미터와 같은 값을 넣는다)."""

    move_per_m: float = 0.05
    idle_per_s: float = 0.002
    # 평균 주행 속도 [m/s]. 가는 동안의 정지 소모를 거리로 바꾸는 데만 쓴다
    speed_mps: float = 0.5
    # 직선 거리 → 실제 경로 길이 비율. 통로를 돌아가는 만큼 (가짜 로봇은 직선이라 1.0)
    path_factor: float = 1.3
    # 도착해서 남겨 둘 여유 (%). 싣기·내리기 대기, 혼잡 정지, 추정 오차
    reserve_percent: float = 5.0
    # 충전 [%/s]와 충전 목표 (%). 충전 중인 로봇이 언제 끝날지(대기 시간) 추정에 쓴다
    charge_per_s: float = 0.5
    charge_target_percent: float = 100.0

    def __post_init__(self):
        if self.move_per_m < 0 or self.idle_per_s < 0:
            raise ValueError(f'소모율은 0 이상: move_per_m={self.move_per_m}, idle_per_s={self.idle_per_s}')
        if not self.speed_mps > 0:
            raise ValueError(f'speed_mps는 0보다 커야 한다: {self.speed_mps}')
        if not self.path_factor >= 1.0:
            raise ValueError(f'path_factor는 1 이상: {self.path_factor}')
        if self.reserve_percent < 0:
            raise ValueError(f'reserve_percent는 0 이상: {self.reserve_percent}')
        if not self.charge_per_s > 0:
            raise ValueError(f'charge_per_s는 0보다 커야 한다: {self.charge_per_s}')

    @property
    def per_m(self) -> float:
        """1 m 갈 때 드는 배터리 [%] (이동 + 그동안의 정지 소모)."""
        return self.move_per_m + self.idle_per_s / self.speed_mps


@dataclass(frozen=True)
class ChargeQueue:
    """지금 충전 대기열 상태. 작업을 마치고 충전 차례를 얼마나 기다릴지 추정한다."""

    # 충전 중(충전기로 가는 중 포함)인 로봇들이 충전기를 비울 때까지 남은 시간 [s]
    remaining_s: tuple[float, ...] = ()
    # 이미 충전 차례를 기다리는 로봇 수 (작업을 마친 로봇은 그 뒤에 선다)
    waiting: int = 0
    # 지금 비어 있는 충전 자리 (최소 운영 대수와 빈 충전기 중 작은 쪽)
    free_slots: int = 0
    # 자리 하나가 한 번 도는 시간 [s] (충전 기준 → 목표 충전). 지금 충전 중인 로봇 다음 차례부터 쓴다
    cycle_s: float = 0.0

    def wait_s(self, after_s: float = 0.0) -> float:
        """after_s 뒤 대기열에 섰을 때 충전 자리가 날 때까지 [s]. 빈 자리가 있으면 0.

        충전 중인 로봇이 없는데 자리도 없으면(살아 있는 로봇이 N대 이하) 충전을 못 하므로 추정하지 않고 0.
        """
        ahead = self.waiting - self.free_slots
        if ahead < 0 or not self.remaining_s:
            return 0.0
        rem = sorted(self.remaining_s)
        rounds, index = divmod(ahead, len(rem))
        return max(0.0, rem[index] + rounds * self.cycle_s - after_s)


def charge_remaining_s(robot: RobotInfo, charger_xy: Point, model: EnergyModel) -> float:
    """충전하러 간 로봇이 충전기를 비울 때까지 [s]: 충전기까지 이동 + 목표까지 충전."""
    travel = math.dist(robot.xy, charger_xy) * model.path_factor / model.speed_mps
    return travel + max(0.0, model.charge_target_percent - robot.battery_percent) / model.charge_per_s


def trip_length(robot_xy: Point, task: TaskInfo, chargers: Sequence[Station]) -> float:
    """현재 위치 → (픽업) → 배달지 → 가장 가까운 충전기 직선 거리 합 [m]. 충전 작업은 충전기까지만.

    충전기가 없으면 배달지에서 끝난다.
    """
    length = math.dist(robot_xy, task.start_xy)
    if task.pickup_xy is not None:
        length += math.dist(task.pickup_xy, task.dropoff_xy)
    if task.task_type != TYPE_CHARGE and chargers:
        length += min(math.dist(task.dropoff_xy, c.xy) for c in chargers)
    return length


def required_percent(robot_xy: Point, task: TaskInfo, chargers: Sequence[Station], model: EnergyModel,
                     battery_percent: Optional[float] = None, queue: Optional[ChargeQueue] = None,
                     low_percent: Optional[float] = None) -> float:
    """이 작업을 끝내고 충전기까지 가는 데 필요한 배터리 [%] (여유분 포함).

    queue·low_percent·battery_percent를 주면, 작업을 마친 뒤 배터리가 low_percent 밑일 때(충전 차례를 기다려야 함)
    예상 대기 시간 동안의 정지 소모를 더한다. 대기 시간은 이 작업에 걸리는 시간만큼 줄여 잡는다.
    """
    need = trip_length(robot_xy, task, chargers) * model.path_factor * model.per_m + model.reserve_percent
    if queue is None or low_percent is None or battery_percent is None or task.task_type == TYPE_CHARGE:
        return need
    work_m = trip_length(robot_xy, task, ()) * model.path_factor
    if battery_percent - work_m * model.per_m >= low_percent:
        return need
    return need + model.idle_per_s * queue.wait_s(work_m / model.speed_mps)


def make_energy_feasible(chargers: Sequence[Station], model: EnergyModel, queue: Optional[ChargeQueue] = None,
                         low_percent: Optional[float] = None) -> Feasible:
    """배터리가 필요량 이상인 (로봇, 작업) 짝만 True. queue를 주면 충전 대기 시간도 넣는다."""
    chargers = tuple(chargers)

    def feasible(robot: RobotInfo, task: TaskInfo) -> bool:
        return robot.battery_percent >= required_percent(robot.xy, task, chargers, model, robot.battery_percent,
                                                         queue, low_percent)

    return feasible


def make_battery_cost(base: CostFn, chargers: Sequence[Station], model: EnergyModel, weight: float) -> CostFn:
    """비용 + weight × (필요 배터리 / 남은 배터리) [m]. 배터리가 많은 로봇이 먼 작업을 맡게 한다.

    필요량이 남은 배터리의 절반이면 weight/2 m를 더 간 것으로 본다. weight 0이면 base와 같다.
    """
    if weight < 0:
        raise ValueError(f'battery_weight는 0 이상: {weight}')
    if weight == 0:
        return base
    chargers = tuple(chargers)

    def cost(robot: RobotInfo, task: TaskInfo) -> float:
        need = required_percent(robot.xy, task, chargers, model)
        return base(robot, task) + weight * need / max(robot.battery_percent, 1e-6)

    return cost
