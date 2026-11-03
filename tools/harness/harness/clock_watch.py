"""시뮬레이션 시계가 멈췄는지 본다 (drive.py, ROS 불필요).

시뮬레이션 시간으로 기다리는 주행(use_sim_time: true)은 Gazebo가 죽어 /clock이 멈추면 끝나지 않는다. 그러면 시나리오
timeout_s(fleet_stack 5400 s)까지 VM이 논다 (#285 하연 리뷰). 시계가 실제 시간 limit_s 동안 늘지 않으면 멈춘 것으로 본다.
"""

from __future__ import annotations

# Gazebo가 느려도(RTF 0.3) 시뮬레이션 시간은 실제 1초에 0.3초씩 는다. 60초 동안 전혀 안 늘면 죽은 것이다
CLOCK_STALL_S = 60.0


class ClockStall:
    """stalled(시뮬레이션 시각, 실제 시각)를 주기적으로 부른다. 시뮬레이션 시각이 늘면 기준을 새로 잡는다."""

    def __init__(self, sim_now: float, wall_now: float, limit_s: float = CLOCK_STALL_S):
        self.limit_s = limit_s
        self.last_sim = sim_now
        self.last_wall = wall_now

    def stalled(self, sim_now: float, wall_now: float) -> bool:
        if sim_now > self.last_sim:
            self.last_sim, self.last_wall = sim_now, wall_now
            return False
        return wall_now - self.last_wall > self.limit_s

    def frozen_s(self, wall_now: float) -> float:
        """마지막으로 시계가 는 뒤 지난 실제 시간."""
        return wall_now - self.last_wall
