#!/usr/bin/env python3
"""동적 장애물을 config/dynamic_obstacles.yaml의 경로대로 움직인다.

장애물마다 GT odom(/dynamic_obstacles/<id>/ground_truth/odom)을 받아 위치를 보고,
속도 명령(/dynamic_obstacles/<id>/cmd_vel)을 낸다. Gazebo와는 dynamic_obstacles.launch.py의 브리지가 잇는다.
이동 패턴은 amr_gazebo.obstacle_motion에 있다. yield_to_robots가 있는 장애물이 있으면
그 로봇들의 GT odom(/<ns>/ground_truth/odom, world 기준)도 받는다.
"""

import math
import random

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from amr_gazebo.obstacle_motion import ObstacleMotion, body_twist

# 로봇 GT가 이보다 오래 [ns] 끊기면 그 로봇을 모르는 것으로 본다 (마지막 정지 값에 계속 비켜 서 있지 않게)
ROBOT_TIMEOUT_NS = 1_000_000_000


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DynamicObstacles(Node):

    def __init__(self):
        super().__init__('dynamic_obstacles')
        config = self.declare_parameter('config', '').value
        # 0이면 실행마다 무작위 (amr_odometry encoder.seed와 같은 규칙). 반복 실험에서는 지정한다
        seed = self.declare_parameter('seed', 0).value
        # 움직일 장애물 id. 비우면 yaml의 전부
        only = [s for s in self.declare_parameter('only', ['']).value if s]
        rate = self.declare_parameter('control_rate_hz', 20.0).value
        # 로봇 중심이 이 반경 [m] 안에 있고 그쪽으로 걷는 중이면 선다 (0이면 끔). 볼 로봇은 keep_away_robots
        keep_away_m = float(self.declare_parameter('keep_away_m', 0.0).value)
        keep_away_robots = [s for s in self.declare_parameter('keep_away_robots', ['']).value if s]

        with open(config, encoding='utf-8') as f:
            specs = yaml.safe_load(f)['obstacles']
        if only:
            unknown = set(only) - {s['id'] for s in specs}
            if unknown:
                raise ValueError(f'yaml에 없는 장애물: {sorted(unknown)}')
            specs = [s for s in specs if s['id'] in only]

        master = random.Random(seed or None)
        self._motions = {}
        self._pose = {}
        self._cmd_pub = {}
        for spec in specs:
            oid = spec['id']
            self._motions[oid] = ObstacleMotion(spec, random.Random(master.random()),
                                                keep_away_m=keep_away_m if keep_away_robots else 0.0)
            self._cmd_pub[oid] = self.create_publisher(Twist, f'/dynamic_obstacles/{oid}/cmd_vel', 10)
            self.create_subscription(
                Odometry, f'/dynamic_obstacles/{oid}/ground_truth/odom',
                lambda msg, oid=oid: self._on_odom(oid, msg), 10)

        # 멈춘 로봇에 비키는 장애물이 볼 로봇: 이름 → (x, y, yaw, 속력, 받은 시각 [ns])
        self._robots = {}
        yield_to = sorted({name for m in self._motions.values() for name in m.yield_to})
        for name in sorted(set(yield_to) | set(keep_away_robots)):
            self.create_subscription(
                Odometry, f'/{name}/ground_truth/odom',
                lambda msg, name=name: self._on_robot(name, msg), 10)

        # 시뮬레이션 시간 기준 (use_sim_time). sim이 멈추면 명령도 멈춘다
        self.create_timer(1.0 / rate, self._control)
        self.get_logger().info(
            f'동적 장애물 {len(self._motions)}개: '
            + ', '.join(f'{m.id}({m.pattern})' for m in self._motions.values())
            + f', seed={seed}'
            + (f', 로봇 {keep_away_m:g} m 안이면 섬: {", ".join(keep_away_robots)}'
               if keep_away_m > 0 and keep_away_robots else '')
            + (f', 멈춘 로봇에 비킴: {", ".join(yield_to)}' if yield_to else ''))

    def _on_odom(self, oid: str, msg: Odometry):
        p = msg.pose.pose
        self._pose[oid] = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_robot(self, name: str, msg: Odometry):
        p, v = msg.pose.pose, msg.twist.twist.linear
        self._robots[name] = (p.position.x, p.position.y, _yaw(p.orientation), math.hypot(v.x, v.y),
                              self.get_clock().now().nanoseconds)

    def _control(self):
        now = self.get_clock().now().nanoseconds
        robots = {name: r[:4] for name, r in self._robots.items() if now - r[4] < ROBOT_TIMEOUT_NS}
        for oid, motion in self._motions.items():
            pose = self._pose.get(oid)
            if pose is None:
                continue
            x, y, yaw = pose
            waiting, turn_backs, keeping = motion.waiting, motion.turn_backs, motion.keeping_away
            vx, vy = motion.step((x, y), robots, now * 1e-9)
            if motion.turn_backs != turn_backs:
                self.get_logger().info(f'{oid}: 로봇이 계속 멈춰 있어 돌아선다')
            elif motion.keeping_away != keeping:
                self.get_logger().info(
                    f'{oid}: ' + ('로봇이 가까워 선다' if motion.keeping_away else '로봇이 멀어져 다시 걷는다'))
            elif motion.waiting != waiting:
                self.get_logger().info(
                    f'{oid}: ' + ('멈춘 로봇이 앞길을 막아 선다' if motion.waiting else '앞길이 비어 다시 걷는다'))
            bx, by, wz = body_twist(vx, vy, yaw)
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = bx, by, wz
            self._cmd_pub[oid].publish(t)

    def stop_all(self):
        # VelocityControl은 마지막 명령을 유지하므로 끝낼 때 0을 보낸다
        for pub in self._cmd_pub.values():
            pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
