"""Fleet 토픽을 로봇별로 중계하며 지연·손실을 거는 노드.

/fleet/<topic> 을 구독해 /<robot_id>/fleet/<topic> 으로 다시 발행한다.
기본은 서버 → 로봇 토픽이다(D8). relay_auction이면 분산 경매 토픽(로봇 → 로봇)도 중계한다.
분산·하이브리드 비교군이 중앙집중과 같은 지연·손실을 겪어야 비교가 공정하기 때문이다.
메시지를 직렬화된 바이트 그대로 넘기므로 header.stamp와 번호 필드가 바뀌지 않는다 (api.md 4-3절).
"""

from __future__ import annotations

import time
from functools import partial

import rclpy
from amr_msgs.msg import FleetHeartbeat, FleetTaskTable, TaskAssignment, TaskAuction, TaskBid, ZoneGrant
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .fault_model import Blackout, DelayLine, FaultConfig, link_rng

# 중계 대상 (api.md 4-2절 서버 → 로봇 토픽)
SERVER_TOPICS = {
    'task_assignments': TaskAssignment,
    'heartbeat': FleetHeartbeat,
    'zone_grants': ZoneGrant,
    'task_table': FleetTaskTable,
}
# 분산 경매 (로봇 → 로봇, api.md 4-2절). 보낸 로봇에게도 중계본이 가며, 받는 로봇이 자기 것을 거른다
AUCTION_TOPICS = {
    'task_auctions': TaskAuction,
    'task_bids': TaskBid,
}
TOPIC_TYPES = {**SERVER_TOPICS, **AUCTION_TOPICS}


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')
        robot_ids = self.declare_parameter(
            'robot_ids', ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']).value
        topics = list(self.declare_parameter('topics', list(SERVER_TOPICS)).value)
        # 분산·하이브리드 비교군 실험에서 켠다. 중앙집중 실험은 끈다 (D8)
        if self.declare_parameter('relay_auction', False).value:
            topics += [t for t in AUCTION_TOPICS if t not in topics]
        # 연속 끊김: blackout_s > 0이면 시작 blackout_at_s초부터 blackout_s초 동안 (blackout_period_s마다 되풀이)
        # blackout_robot의 링크(all이면 모든 로봇)에 들어온 메시지를 버린다. 명세 밖이라 allow_out_of_spec이 필요하다
        blackout_s = self.declare_parameter('blackout_s', 0.0).value
        if blackout_s < 0:
            # 0은 끔. 음수는 확장 조건 입력 실수라 조용히 끄지 않고 시작을 거부한다
            raise ValueError(f'blackout_s는 0(끔) 이상: {blackout_s}')
        blackout = Blackout(
            at_s=self.declare_parameter('blackout_at_s', 0.0).value,
            duration_s=blackout_s,
            period_s=self.declare_parameter('blackout_period_s', 0.0).value,
        ) if blackout_s > 0 else None
        blackout_robot = self.declare_parameter('blackout_robot', 'all').value
        config = FaultConfig(
            delay_ms=self.declare_parameter('delay_ms', 0.0).value,
            loss_pct=self.declare_parameter('loss_pct', 0.0).value,
            allow_out_of_spec=self.declare_parameter('allow_out_of_spec', False).value,
            blackout=blackout,
            # delay: 다시 이어지면 쌓인 것을 순서대로 전달 (기본), drop: 끊김 동안 나갈 메시지를 버림
            blackout_mode=self.declare_parameter('blackout_mode', 'delay').value,
        )
        seed = self.declare_parameter('seed', 0).value
        tick_ms = self.declare_parameter('tick_ms', 1.0).value

        unknown = set(topics) - set(TOPIC_TYPES)
        if unknown:
            raise ValueError(f'중계할 수 없는 토픽: {sorted(unknown)}')
        if blackout_robot != 'all' and blackout_robot not in robot_ids:
            raise ValueError(f'blackout_robot은 all 또는 {robot_ids} 중 하나: {blackout_robot}')
        start = time.monotonic()

        # TODO(하연): api.md 6절 QoS 기본값이 정해지면 토픽별로 맞춘다
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._links: list[tuple[DelayLine, object]] = []
        for topic in topics:
            msg_type = TOPIC_TYPES[topic]
            links = [
                (DelayLine(config, link_rng(seed, robot_id, topic), start=start,
                           blackout=blackout_robot in ('all', robot_id)),
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
            f'delay_ms={config.delay_ms}, loss_pct={config.loss_pct}, seed={seed}'
            + (' (명세 밖 조건 허용)' if config.allow_out_of_spec else '')
            + (f', 끊김 {blackout_robot} {blackout} ({config.blackout_mode})' if blackout else ''))

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C나 launch 종료(SIGINT)는 정상 종료로 본다
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
