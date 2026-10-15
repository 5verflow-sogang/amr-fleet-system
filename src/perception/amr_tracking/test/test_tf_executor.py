"""TF를 콜백과 다른 스레드에서 받는지 (tracking_node.make_executor).

콜백 안에서 lookup_transform(timeout=)으로 기다리는 동안 TF가 들어와야 한다. 단일 스레드 executor면
TF를 받을 스레드가 기다리는 콜백 자신이라 끝내 못 받는다 (Mac Docker에서 "TF 없음, 스캔 건너뜀" 반복).
"""

import threading
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from amr_tracking.tracking_node import make_executor

PARENT, CHILD = 'tf_executor_test_odom', 'tf_executor_test_base'


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Waiter(Node):
    """타이머 콜백(기본 그룹) 안에서 TF를 기다린다. 추적 노드 on_scan과 같은 구조.

    스캔 시각이 버퍼의 최신 TF보다 앞선 상황을 만들려고 지금보다 0.3 s 뒤 시각을 요청한다. 그 시각을 넘는 TF가
    새로 들어와야 성공한다 (이미 버퍼에 있는 TF로는 안 된다).
    """

    def __init__(self):
        super().__init__('tf_executor_waiter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.result = None
        self.timer = self.create_timer(0.05, self.wait_for_tf)

    def wait_for_tf(self):
        self.timer.cancel()
        try:
            stamp = self.get_clock().now() + Duration(seconds=0.3)
            self.buffer.lookup_transform(PARENT, CHILD, stamp, timeout=Duration(seconds=2.0))
            self.result = True
        except TransformException:
            self.result = False


def _publish_tf_in_background(stop: threading.Event):
    """다른 노드·스레드에서 TF를 20 Hz로 낸다 (EKF 역할)."""
    node = Node('tf_executor_publisher')
    broadcaster = TransformBroadcaster(node)

    def run():
        while not stop.is_set():
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = PARENT, CHILD
            t.transform.rotation.w = 1.0
            broadcaster.sendTransform(t)
            time.sleep(0.05)
        node.destroy_node()

    thread = threading.Thread(target=run)
    thread.start()
    return thread


def _run(executor_factory) -> bool:
    waiter = Waiter()
    executor = executor_factory(waiter)
    stop = threading.Event()
    publisher = _publish_tf_in_background(stop)
    try:
        deadline = time.monotonic() + 5.0
        while waiter.result is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    finally:
        stop.set()
        publisher.join()
        executor.shutdown()
        waiter.destroy_node()
    return waiter.result


def _single_threaded(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    return executor


def test_lookup_inside_callback_receives_tf(ros):
    assert _run(make_executor) is True


def test_single_threaded_executor_starves_tf(ros):
    # 고치기 전 구조(rclpy.spin)의 재현: 기다리는 동안 TF를 못 받아 시간 초과
    assert _run(_single_threaded) is False
