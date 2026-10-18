"""마커판 하나 → 로봇 자세 (ROS 비의존, 2D). Kidnapped 복구용 위치 (#148, 10/19 안건 ③).

판마다 마커 id가 달라(markers.py: 판 k → 2k·2k+1) 판이 보이면 어느 스테이션인지 바로 안다. 랙이 7.5 m 간격으로
반복돼 스캔만으로는 몇 번째 통로인지 모르는 경우(다른 통로 Kidnapped 0/10)를 이걸로 푼다.

좌표는 geometry.py와 같다: 왼쪽·오른쪽 마커 중심(로봇 base 기준 xy)을 이은 선 u(왼→오), 판 앞면 바깥(로봇 쪽)
n = u를 −90° 돌린 것. 판의 world 자세(앞면 중심, 바라보는 방향)를 알면 로봇 world 자세가 나온다.
"""

from dataclasses import dataclass
import math
from typing import Tuple

Point = Tuple[float, float]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def board_in_base(left: Point, right: Point) -> Pose2D:
    """마커 두 장의 중심(로봇 기준) → 판 앞면 중심과 판이 바라보는 방향(로봇 기준)."""
    ux, uy = right[0] - left[0], right[1] - left[1]
    norm = math.hypot(ux, uy)
    if norm < 1e-6:
        raise ValueError('마커 두 장의 중심이 겹친다')
    nx, ny = uy / norm, -ux / norm
    return Pose2D((left[0] + right[0]) / 2, (left[1] + right[1]) / 2, math.atan2(ny, nx))


def robot_from_board(board_base: Pose2D, board_world: Pose2D) -> Pose2D:
    """판의 로봇 기준 자세와 world 자세 → 로봇 world 자세.

    world 자세 = 로봇 자세 ∘ 판 로봇 기준 자세라서, 로봇 방향 = 판 world 방향 − 판 로봇 기준 방향,
    로봇 위치 = 판 world 위치 − R(로봇 방향)·판 로봇 기준 위치.
    """
    yaw = wrap(board_world.yaw - board_base.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    x = board_world.x - (c * board_base.x - s * board_base.y)
    y = board_world.y - (s * board_base.x + c * board_base.y)
    return Pose2D(x, y, yaw)


def transform(pose: Pose2D, frame: Pose2D) -> Pose2D:
    """frame 기준 자세 → frame이 놓인 좌표계 기준 (2D 강체 변환 합성)."""
    c, s = math.cos(frame.yaw), math.sin(frame.yaw)
    return Pose2D(frame.x + c * pose.x - s * pose.y, frame.y + s * pose.x + c * pose.y,
                  wrap(frame.yaw + pose.yaw))


def inverse(frame: Pose2D) -> Pose2D:
    c, s = math.cos(frame.yaw), math.sin(frame.yaw)
    return Pose2D(-(c * frame.x + s * frame.y), -(-s * frame.x + c * frame.y), wrap(-frame.yaw))


@dataclass(frozen=True)
class SigmaModel:
    """판까지 거리 d [m]와 판 마커 간격(spacing)에 따른 표준편차.

    판 하나로는 판이 로봇 쪽으로 얼마나 돌아 있는지(방향)가 가장 약하다. 방향 오차는 거리에 비례하고 마커 간격에
    반비례하며(두 중심의 깊이 차로 재므로), 그만큼 로봇이 판 둘레로 돌아간 자리로 위치가 틀린다 (d·σ_yaw).

        σ_yaw = yaw0 + yaw1·d·(ref_spacing / spacing),  σ_xy = xy0 + xy2·d² + d·σ_yaw

    기본값은 Gazebo 거리별 측정(experiments/aruco_localizer)에서 정했다: 비스듬히 본 최악 오차가 2σ 안팎에 들게.
    """
    xy0: float = 0.02
    xy2: float = 0.005
    yaw0: float = math.radians(1.0)
    yaw1: float = math.radians(2.0)
    ref_spacing: float = 0.32

    def sigma(self, distance: float, spacing: float = 0.32) -> Tuple[float, float]:
        d = max(0.0, distance)
        yaw = self.yaw0 + self.yaw1 * d * self.ref_spacing / max(spacing, 1e-3)
        return self.xy0 + self.xy2 * d * d + d * yaw, yaw
