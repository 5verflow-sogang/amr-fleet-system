"""작업 생성기: 시나리오 또는 JSON Task 파일의 일정대로 /fleet/task_requests에 Task를 발행한다.

시각은 노드 시계(use_sim_time이면 시뮬레이션 시간) 기준이다. 시뮬레이터가 느려져도 로봇 속도에 비해
작업이 몰리지 않게 하기 위해서다. Fleet 서버(server_node)가 구독을 시작한 뒤부터 시계를 잰다.
rosbag 녹화처럼 다른 구독자가 먼저 떠도 서버가 없으면 시작하지 않는다(서버 없이 보낸 작업은 사라진다).
"""

from __future__ import annotations

import os

import rclpy
import yaml
from amr_msgs.msg import Task
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .task_gen import (ITEMS, TASK_TYPES, TaskSpec, dump_tasks_json, generate, load_stations, load_tasks_json,
                       world_to_map, yaw_to_quaternion)


class TaskGenerator(Node):

    def __init__(self):
        super().__init__('task_generator')
        scenario_file = self.declare_parameter('scenario_file', '').value
        tasks_file = self.declare_parameter('tasks_file', '').value
        seed = self.declare_parameter('seed', 0).value
        stations_file = self.declare_parameter('stations_file', '').value
        self._origin = list(self.declare_parameter('map_origin_in_world', [0.0, 1.5, 0.0]).value)
        schedule_out = self.declare_parameter('schedule_out', '').value
        tick_s = self.declare_parameter('tick_s', 0.1).value
        # 이 이름의 노드가 /fleet/task_requests를 구독해야 일정을 시작한다. 비우면 아무 구독자나
        self._server_node = self.declare_parameter('server_node', 'fleet_manager').value

        self._stations = load_stations(stations_file)
        if tasks_file:
            self._tasks = load_tasks_json(tasks_file)
            source = tasks_file
        elif scenario_file:
            with open(scenario_file, encoding='utf-8') as f:
                scenario = yaml.safe_load(f)
            self._tasks = generate(scenario, self._stations, seed)
            source = f'{scenario_file} (seed={seed})'
        else:
            raise ValueError('scenario_file 또는 tasks_file 중 하나는 있어야 한다')
        unknown = {n for t in self._tasks for n in (t.pickup, t.dropoff) if n and n not in self._stations}
        if unknown:
            raise ValueError(f'stations_file에 없는 스테이션: {sorted(unknown)}')
        if schedule_out:
            dump_tasks_json(self._tasks, schedule_out, meta={
                'source': os.path.abspath(scenario_file or tasks_file), 'seed': seed,
                'stations_file': os.path.abspath(stations_file), 'map_origin_in_world': self._origin,
            })

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Task, '/fleet/task_requests', qos)
        self._next = 0
        self._start_ns: int | None = None
        self.create_timer(tick_s, self._tick)
        self.get_logger().info(f'작업 {len(self._tasks)}건 준비: {source}. Fleet 서버({self._server_node or "아무 구독자"}) 구독을 기다린다')

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self._start_ns is None:
            # 서버가 구독하기 전에 보낸 작업은 사라지므로 서버를 찾은 뒤부터 잰다
            if not self._server_ready():
                return
            self._start_ns = now_ns
            self.get_logger().info('Fleet 서버 발견, 일정 시작')
        elapsed = (now_ns - self._start_ns) * 1e-9
        while self._next < len(self._tasks) and self._tasks[self._next].at_s <= elapsed:
            self._pub.publish(self._to_msg(self._tasks[self._next]))
            self._next += 1
            if self._next == len(self._tasks):
                self.get_logger().info(f'작업 {len(self._tasks)}건 발행 완료')

    def _server_ready(self) -> bool:
        subscribers = self.get_subscriptions_info_by_topic('/fleet/task_requests')
        if not self._server_node:
            return len(subscribers) > 0
        return any(info.node_name == self._server_node for info in subscribers)

    def _to_msg(self, spec: TaskSpec) -> Task:
        now = self.get_clock().now()
        msg = Task()
        msg.task_id = spec.task_id
        msg.task_type = TASK_TYPES[spec.type]
        msg.item_type = ITEMS[spec.item]
        if spec.pickup:
            msg.pickup_station_id = spec.pickup
            self._fill_pose(msg.pickup_pose, spec.pickup)
        msg.dropoff_station_id = spec.dropoff
        self._fill_pose(msg.dropoff_pose, spec.dropoff)
        msg.priority = spec.priority
        msg.created_time = now.to_msg()
        if spec.deadline_s > 0:
            sec, nanosec = divmod(now.nanoseconds + int(round(spec.deadline_s * 1e9)), 1_000_000_000)
            msg.deadline = Time(sec=sec, nanosec=nanosec)
        return msg

    def _fill_pose(self, pose, station: str):
        x, y, yaw = world_to_map(self._stations[station], self._origin)
        pose.position.x, pose.position.y = x, y
        pose.orientation.z, pose.orientation.w = yaw_to_quaternion(yaw)


def main(args=None):
    rclpy.init(args=args)
    node = TaskGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
