#!/usr/bin/env python3
"""amr_nav2_plugins::DwaController의 동적 장애물 회피(perception/obstacles)를
Gazebo 데모(nav2_dwa_demo.launch.py)에서 실제로 확인하는 1회용 실험 스크립트.

로봇 경로(map 기준 (0,0) -> (3.0,1.5), 데모 기본 목표)를 가로지르는 가짜 장애물
하나를 TrackedObstacleArray로 계속 발행하면서, 로봇 ground_truth와의 최소 거리와
목표 도착 여부를 기록한다. 패키지가 아니라 단일 스크립트다 - 데모가 이미 떠 있는
컨테이너에서 `python3 publish_dynamic_obstacle.py`로 바로 돌린다(rclpy·amr_msgs는
ROS 환경 source로 이미 PYTHONPATH에 있다).

좌표계: perception/obstacles는 local costmap의 global_frame(odom)과 같은 프레임으로
봐야 하는데(DwaController::onObstacles), 이 데모는 map->odom이 항등 변환이라 map
프레임 숫자를 그대로 odom 프레임 숫자로 써도 된다 - 그래서 장애물은 "map 프레임"
숫자로 계산해서 발행한다. ground_truth/odom은 world 프레임이고 스폰 위치가
world(0, 1.5)라 world = map + (0, 1.5) 관계다(amr_mapping/maps/warehouse_v0.yaml
주석과 동일) - 거리 비교 전에 world -> map으로 뺀다.
"""
import math
import time

import rclpy
from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from nav_msgs.msg import Odometry
from rclpy.node import Node

NAMESPACE = 'amr_01'
WORLD_TO_MAP_OFFSET = (0.0, 1.5)  # map = world - offset

# 데모 기본 목표(goal_x/goal_y) - nav2_dwa_demo.launch.py
GOAL_MAP_XY = (3.0, 1.5)
GOAL_TOLERANCE_M = 0.3  # xy_goal_tolerance(0.25) + GT 샘플링 여유

# 장애물: 경로 중점(map (1.5, 0.75))을 CROSS_TIME_S에 지나가도록 역산한 시작점에서
# -y 방향으로 등속 이동한다. 이 스크립트는 데모 launch를 시작하고 몇 초 뒤에 띄우므로
# (goal은 launch 시작 15s 뒤 전송, 로봇은 목표까지 ~9~10s 걸린다), CROSS_TIME_S는
# "이 스크립트 기준" 시각이다 - 실행할 때 데모와 몇 초 차이로 띄우는지에 맞춰 조정한다.
OBSTACLE_SPEED_MPS = 0.5
OBSTACLE_RADIUS_M = 0.3
CROSSING_POINT_MAP = (1.5, 0.75)
CROSS_TIME_S = 15.0
OBSTACLE_START_MAP = (
    CROSSING_POINT_MAP[0],
    CROSSING_POINT_MAP[1] + OBSTACLE_SPEED_MPS * CROSS_TIME_S,
)
OBSTACLE_VELOCITY_MPS = (0.0, -OBSTACLE_SPEED_MPS)

RUN_DURATION_S = 40.0
PUBLISH_RATE_HZ = 10.0


class DynamicObstacleExperiment(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_experiment')
        self._obstacle_pub = self.create_publisher(
            TrackedObstacleArray, f'/{NAMESPACE}/perception/obstacles', 10)
        self.create_subscription(
            Odometry, f'/{NAMESPACE}/ground_truth/odom', self._on_ground_truth, 10)

        self._start_time = time.monotonic()
        self._robot_map_xy = None
        self._min_distance = math.inf
        self._min_distance_t = None
        self._reached_goal = False
        self._reached_goal_t = None

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)
        self.get_logger().info(
            f'동적 장애물 실험 시작: 시작 map({OBSTACLE_START_MAP[0]:.2f}, '
            f'{OBSTACLE_START_MAP[1]:.2f}), 속도 {OBSTACLE_VELOCITY_MPS} m/s, '
            f'반경 {OBSTACLE_RADIUS_M} m, 목표 교차 t={CROSS_TIME_S:.1f}s')

    def _on_ground_truth(self, msg: Odometry):
        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y
        self._robot_map_xy = (wx - WORLD_TO_MAP_OFFSET[0], wy - WORLD_TO_MAP_OFFSET[1])

    def _obstacle_map_xy(self, t: float):
        return (
            OBSTACLE_START_MAP[0] + OBSTACLE_VELOCITY_MPS[0] * t,
            OBSTACLE_START_MAP[1] + OBSTACLE_VELOCITY_MPS[1] * t,
        )

    def _on_timer(self):
        t = time.monotonic() - self._start_time
        obstacle_xy = self._obstacle_map_xy(t)

        msg = TrackedObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{NAMESPACE}/odom'  # map->odom 항등이라 map 숫자 그대로 쓴다
        obstacle = TrackedObstacle()
        obstacle.track_id = 1
        obstacle.source = TrackedObstacle.SOURCE_UNKNOWN
        obstacle.is_dynamic = True
        obstacle.pose.pose.position.x = obstacle_xy[0]
        obstacle.pose.pose.position.y = obstacle_xy[1]
        obstacle.twist.twist.linear.x = OBSTACLE_VELOCITY_MPS[0]
        obstacle.twist.twist.linear.y = OBSTACLE_VELOCITY_MPS[1]
        obstacle.radius = OBSTACLE_RADIUS_M
        msg.obstacles = [obstacle]
        self._obstacle_pub.publish(msg)

        if self._robot_map_xy is not None:
            distance = math.dist(self._robot_map_xy, obstacle_xy)
            if distance < self._min_distance:
                self._min_distance = distance
                self._min_distance_t = t
            if not self._reached_goal and math.dist(self._robot_map_xy, GOAL_MAP_XY) <= GOAL_TOLERANCE_M:
                self._reached_goal = True
                self._reached_goal_t = t

        if t >= RUN_DURATION_S:
            self._report_and_exit()

    def _report_and_exit(self):
        self.get_logger().info('=== 실험 결과 ===')
        self.get_logger().info(
            f'목표 도착: {self._reached_goal}'
            + (f' (t={self._reached_goal_t:.1f}s)' if self._reached_goal else ''))
        if self._min_distance_t is not None:
            self.get_logger().info(
                f'장애물과 최소 거리(GT 기준): {self._min_distance:.3f} m (t={self._min_distance_t:.1f}s)')
        else:
            self.get_logger().info('ground_truth/odom을 못 받아 최소 거리를 못 구함')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DynamicObstacleExperiment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
