#!/usr/bin/env python3
"""동적 장애물을 config/dynamic_obstacles.yaml의 경로대로 움직인다.

장애물마다 GT odom(/dynamic_obstacles/<id>/ground_truth/odom)을 받아 위치를 보고,
속도 명령(/dynamic_obstacles/<id>/cmd_vel)을 낸다. Gazebo와는 dynamic_obstacles.launch.py의 브리지가 잇는다.
이동 패턴은 amr_gazebo.obstacle_motion에 있다.
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
            self._motions[oid] = ObstacleMotion(spec, random.Random(master.random()))
            self._cmd_pub[oid] = self.create_publisher(Twist, f'/dynamic_obstacles/{oid}/cmd_vel', 10)
            self.create_subscription(
                Odometry, f'/dynamic_obstacles/{oid}/ground_truth/odom',
                lambda msg, oid=oid: self._on_odom(oid, msg), 10)

        # 시뮬레이션 시간 기준 (use_sim_time). sim이 멈추면 명령도 멈춘다
        self.create_timer(1.0 / rate, self._control)
        self.get_logger().info(
            f'동적 장애물 {len(self._motions)}개: '
            + ', '.join(f'{m.id}({m.pattern})' for m in self._motions.values())
            + f', seed={seed}')

    def _on_odom(self, oid: str, msg: Odometry):
        p = msg.pose.pose
        self._pose[oid] = (p.position.x, p.position.y, _yaw(p.orientation))

    def _control(self):
        for oid, motion in self._motions.items():
            pose = self._pose.get(oid)
            if pose is None:
                continue
            x, y, yaw = pose
            vx, vy = motion.step((x, y))
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
