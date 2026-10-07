"""매핑 주행: GT 위치를 보고 경유점 사이 직선을 따라간다 (주행용일 뿐, 지도 만들기에는 GT를 쓰지 않는다).

    ros2 run amr_mapping map_route_driver   # sim + 센서 필터 + (rosbag 녹화 또는 mapping.launch) 뒤에 실행

같은 경로로 다시 매핑하면 지도끼리 비교할 수 있다 (지도 v1, README "지도 v1").

경로 (world 좌표, 월드 v0 + #65 통로 수정):
  북쪽 루프: A–B 통로(y 3) 동쪽 → 동쪽 끝 → 메인 통로(y 12) 서쪽 → 서쪽 끝 → A–B 통로로 출발 구간 복귀 (루프 클로저)
  좁은 통로: (2.8, −1.1)에서 서쪽으로 좁은 통로 ②(분리 랙–B4) 통과 → (−4.7, −1.1)에서 남쪽으로 좁은 통로 ①(B3–B4) 통과.
            옆을 지나며 비스듬히 보면 0.6 m 폭 통로 안쪽이 미관측으로 남아서 직접 지나간다
  남쪽 루프: B–C 통로(y −4) 동쪽 → 동쪽 끝 → C열 남쪽(y −13) 서쪽 → 서쪽 끝 → B–C 통로로 복귀 (루프 클로저)
  → 출발점 (0, 1.5)
넓은 곳은 기둥 (±11.25, 7), (±15, 1.5), (±7.5, −5.5)과 랙에서 1 m 이상 떨어진 선이다.
좁은 통로는 양옆 여유가 10 cm(통로 0.6 m − 로봇 폭 0.4 m)라 천천히, 방향을 맞춘 뒤 들어간다.
"""

from dataclasses import dataclass
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass(frozen=True)
class Leg:
    """이 경유점까지 가는 구간의 주행 설정."""
    x: float
    y: float
    v_max: float = 0.6          # m/s (매핑은 천천히: 스캔 매칭 탐색 범위 0.5 m 안)
    tolerance: float = 0.3      # m. 도착 판정
    turn_in_place: float = 0.6  # rad. 이보다 틀어지면 멈춰서 돈다
    lookahead: float = 0.8      # m. 직선 위 앞쪽 점을 보고 조향한다
    narrow: bool = False        # 좁은 통로 구간 (선에서 벗어난 최대 거리를 기록)


def narrow_leg(x, y):
    return Leg(x, y, v_max=0.25, tolerance=0.05, turn_in_place=0.1, lookahead=0.3, narrow=True)


START = (0.0, 1.5)
ROUTE = [
    Leg(3.0, 3.0), Leg(27.0, 3.0), Leg(27.0, 12.0), Leg(-27.0, 12.0), Leg(-27.0, 3.0), Leg(2.8, 3.0),
    narrow_leg(2.8, -1.1),      # 좁은 통로 ② 동쪽 입구 앞 2.2 m에서 멈춘다
    narrow_leg(-4.7, -1.1),     # 좁은 통로 ② (x 0.6 → −4.4, y −1.1) 통과
    narrow_leg(-4.7, -4.0),     # 좁은 통로 ① (y −1.4 → −2.6, x −4.7) 통과해 B–C 통로로
    Leg(27.0, -4.0), Leg(27.0, -13.0), Leg(-27.0, -13.0), Leg(-27.0, -4.0), Leg(2.8, -4.0),
    Leg(2.8, 3.0), Leg(0.0, 1.5),
]
W_MAX = 0.8          # rad/s
STUCK_SECONDS = 30.0  # sim 시간 동안 1 cm도 못 가면 멈춘다 (충돌 등)


class Driver(Node):
    def __init__(self):
        super().__init__('map_route_driver')
        self.pose = None
        self.path_len = 0.0
        self.create_subscription(Odometry, '/amr_01/ground_truth/odom', self.on_gt, 50)
        self.pub = self.create_publisher(Twist, '/amr_01/cmd_vel', 10)

    def on_gt(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw,
             m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if self.pose is not None:
            self.path_len += math.hypot(p[0] - self.pose[0], p[1] - self.pose[1])
        self.pose = p

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.pub.publish(t)

    def stop(self):
        for _ in range(20):
            self.cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def follow(d: Driver, start, leg: Leg):
    """start → leg 직선을 따라간다. 선에서 벗어난 최대 거리 [m]를 돌려준다."""
    sx, sy = start
    seg = math.hypot(leg.x - sx, leg.y - sy)
    ux, uy = ((leg.x - sx) / seg, (leg.y - sy) / seg) if seg > 1e-6 else (1.0, 0.0)
    max_lateral = 0.0
    last_progress_t, last_progress_pos = d.pose[3], d.pose[:2]
    last_print = time.time()
    while True:
        rclpy.spin_once(d, timeout_sec=0.05)
        x, y, yaw, t = d.pose
        along = (x - sx) * ux + (y - sy) * uy
        lateral = -(x - sx) * uy + (y - sy) * ux
        if along > 0.3 and seg - along > 0.3:   # 회전 구간을 뺀 직진 구간
            max_lateral = max(max_lateral, abs(lateral))
        dist = math.hypot(leg.x - x, leg.y - y)
        if dist < leg.tolerance:
            return max_lateral
        # 직선 위 앞쪽 점 (목표를 넘지 않게)
        ahead = min(max(along, 0.0) + leg.lookahead, seg)
        tx, ty = sx + ux * ahead, sy + uy * ahead
        if dist < leg.lookahead:
            tx, ty = leg.x, leg.y
        err = math.atan2(ty - y, tx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        w = max(-W_MAX, min(W_MAX, 1.5 * err))
        if abs(err) > leg.turn_in_place:
            v = 0.0
        else:
            v = leg.v_max * (1.0 - abs(err) / leg.turn_in_place) * min(1.0, dist / 0.8 + 0.3)
        d.cmd(v, w)
        if math.hypot(x - last_progress_pos[0], y - last_progress_pos[1]) > 0.01 or abs(w) > 0.05:
            last_progress_t, last_progress_pos = t, (x, y)
        elif t - last_progress_t > STUCK_SECONDS:
            d.stop()
            raise RuntimeError(f'({x:.2f}, {y:.2f})에서 {STUCK_SECONDS:.0f} s 동안 못 움직임 (충돌?)')
        if time.time() - last_print > 30:
            print(f'  → ({leg.x}, {leg.y}) 남은 {dist:.1f} m', flush=True)
            last_print = time.time()


def main():
    rclpy.init()
    d = Driver()
    while d.pose is None:
        rclpy.spin_once(d, timeout_sec=0.1)
    t_start = d.pose[3]
    print(f'시작 자세 (world): x={d.pose[0]:.3f} y={d.pose[1]:.3f} yaw={d.pose[2]:.4f}', flush=True)
    prev = START
    try:
        for i, leg in enumerate(ROUTE):
            lateral = follow(d, prev, leg)
            extra = f', 선에서 최대 {lateral * 100:.1f} cm 벗어남 (좁은 통로 여유 10 cm)' if leg.narrow else ''
            print(f'경유점 {i + 1}/{len(ROUTE)} 도착 ({leg.x}, {leg.y}) sim t={d.pose[3] - t_start:.0f} s{extra}',
                  flush=True)
            prev = (leg.x, leg.y)
    except RuntimeError as e:
        print(f'중단: {e}', flush=True)
        rclpy.shutdown()
        return 1
    d.stop()
    print(f'완료: 주행 거리 {d.path_len:.1f} m, sim {d.pose[3] - t_start:.0f} s, '
          f'끝 자세 x={d.pose[0]:.3f} y={d.pose[1]:.3f} yaw={d.pose[2]:.4f}', flush=True)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
