"""서버 → 로봇 토픽을 로봇별로 중계하며 지연·손실을 거는 노드.

/fleet/<topic> 을 구독해 /<robot_id>/fleet/<topic> 으로 다시 발행한다.
메시지를 직렬화된 바이트 그대로 넘기므로 header.stamp와 번호 필드가 바뀌지 않는다 (api.md 4-3절).
"""

from __future__ import annotations

import time
from functools import partial

import rclpy
from amr_msgs.msg import FleetHeartbeat, FleetTaskTable, TaskAssignment, ZoneGrant
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .fault_model import DelayLine, FaultConfig, link_rng

# 중계 대상 (api.md 4-2절 서버 → 로봇 토픽)
TOPIC_TYPES = {
    'task_assignments': TaskAssignment,
    'heartbeat': FleetHeartbeat,
    'zone_grants': ZoneGrant,
    'task_table': FleetTaskTable,
}


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')
        robot_ids = self.declare_parameter(
            'robot_ids', ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']).value
        topics = self.declare_parameter('topics', list(TOPIC_TYPES)).value
        config = FaultConfig(
            delay_ms=self.declare_parameter('delay_ms', 0.0).value,
            loss_pct=self.declare_parameter('loss_pct', 0.0).value,
        )
        seed = self.declare_parameter('seed', 0).value
        tick_ms = self.declare_parameter('tick_ms', 1.0).value

        unknown = set(topics) - set(TOPIC_TYPES)
        if unknown:
            raise ValueError(f'중계할 수 없는 토픽: {sorted(unknown)}')

        # TODO(하연): api.md 6절 QoS 기본값이 정해지면 토픽별로 맞춘다
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._links: list[tuple[DelayLine, object]] = []
        for topic in topics:
            msg_type = TOPIC_TYPES[topic]
            links = [
                (DelayLine(config, link_rng(seed, robot_id, topic)),
                 self.create_publisher(msg_type, f'/{robot_id}/fleet/{topic}', qos))
                for robot_id in robot_ids
            ]
            self._links.extend(links)
            self.create_subscription(
                msg_type, f'/fleet/{topic}', partial(self._on_message, links), qos, raw=True)

        # 지연은 네트워크를 흉내 내므로 use_sim_time과 상관없이 실제 시간(단조 시계)으로 잰다
        self.create_timer(tick_ms / 1000.0, self._flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'중계 시작: topics={topics}, robots={robot_ids}, '
            f'delay_ms={config.delay_ms}, loss_pct={config.loss_pct}, seed={seed}')

    def _on_message(self, links, data: bytes):
        now = time.monotonic()
        for line, _ in links:
            line.push(data, now)
        # 지연 0이면 타이머를 기다리지 않고 바로 내보낸다
        self._flush()

    def _flush(self):
        now = time.monotonic()
        for line, publisher in self._links:
            for data in line.pop_due(now):
                publisher.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
