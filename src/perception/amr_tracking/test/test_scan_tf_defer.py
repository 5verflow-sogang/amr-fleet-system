"""촬영 시각 TF보다 먼저 온 스캔을 미뤄 두었다가 처리하는지 (tracking_node.on_scan, 단일 스레드).

콜백 안에서 TF를 기다리지 않는다. 스캔 시각의 TF가 아직 없으면 스캔을 미뤄 두고, 버퍼에 그 시각을 덮는 TF가
들어오는 순간(Buffer.set_transform) 처리한다. 다음 스캔이 먼저 오면 앞 스캔은 버린다.
앞의 테스트들은 버퍼에 TF를 직접 넣고, 마지막 테스트는 실제 /tf·스캔 토픽과 단일 스레드 executor로 확인한다.
"""

import math
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from amr_msgs.msg import TrackedObstacleArray
from amr_tracking.tracking_node import TrackingNode

# 네임스페이스 없이 띄우면 fixed_frame = odom, base_frame = base_footprint
ODOM, BASE, LIDAR = 'odom', 'base_footprint', 'lidar_link'


@pytest.fixture
def node():
    rclpy.init()
    tracking = TrackingNode()
    yield tracking
    tracking.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def ready(node):
    """lidar_link 위치(정적)와 1.0 s까지의 odom → base_footprint TF가 있는 노드."""
    node.tf_buffer.set_transform_static(_tf(BASE, LIDAR, 0.0, x=0.15), 'test')
    _odom_tf(node, 1.0)
    return node


def _stamp(t: float):
    return Time(nanoseconds=int(round(t * 1e9))).to_msg()


def _tf(parent: str, child: str, t: float, x: float = 0.0) -> TransformStamped:
    m = TransformStamped()
    m.header.stamp = _stamp(t)
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x = x
    m.transform.rotation.w = 1.0
    return m


def _odom_tf(node: TrackingNode, t: float):
    node.tf_buffer.set_transform(_tf(ODOM, BASE, t, x=0.1 * t), 'test')


def _scan(t: float) -> LaserScan:
    """빈 스캔 (군집 없음). 처리 여부만 본다."""
    s = LaserScan()
    s.header.stamp = _stamp(t)
    s.header.frame_id = LIDAR
    s.angle_min, s.angle_increment = -math.pi, math.radians(0.5)
    s.angle_max = s.angle_min + 719 * s.angle_increment
    s.range_min, s.range_max = 0.12, 25.0
    s.ranges = [float('inf')] * 720
    return s


def _record(node: TrackingNode) -> list:
    """처리된 스캔의 촬영 시각 [s] 목록. 추적·발행 대신 기록만 한다."""
    seen = []
    node.track_and_publish = (
        lambda header, *args, **kwargs: seen.append(Time.from_msg(header.stamp).nanoseconds * 1e-9))
    return seen


def test_scan_with_tf_is_processed_at_once(ready):
    seen = _record(ready)
    _odom_tf(ready, 1.2)
    ready.on_scan(_scan(1.1))
    assert seen == pytest.approx([1.1])
    assert ready.stats['deferred'] == 0


def test_scan_before_tf_waits_for_it(ready):
    seen = _record(ready)
    ready.on_scan(_scan(1.1))       # TF는 1.0 s까지뿐
    assert seen == []
    assert ready.stats['deferred'] == 1
    _odom_tf(ready, 1.05)           # 아직 1.1을 못 덮는다
    assert seen == []
    _odom_tf(ready, 1.15)           # 덮는 TF가 들어오는 순간 처리
    assert seen == pytest.approx([1.1])
    assert ready._pending_scan is None


def test_older_pending_scan_is_dropped(ready):
    seen = _record(ready)
    ready.on_scan(_scan(1.1))
    ready.on_scan(_scan(1.2))       # 1.1의 TF가 오기 전에 다음 스캔 → 1.1은 버린다
    assert ready.stats['tf_drop'] == 1
    _odom_tf(ready, 1.3)
    assert seen == pytest.approx([1.2])


def test_no_waiting_check_left_in_buffer(ready):
    """버린 스캔의 TF 기다림이 버퍼에 남지 않는다. 남으면 TF가 올 때마다 헛확인을 하고, 끝내 오지 않는 TF면
    계속 쌓인다."""
    _record(ready)
    lost = _scan(1.1)
    lost.header.frame_id = 'unknown_link'     # 이 TF는 끝내 오지 않는다
    ready.on_scan(lost)
    ready.on_scan(_scan(1.2))                 # 앞 스캔을 버린다
    _odom_tf(ready, 1.3)
    _odom_tf(ready, 1.4)            # 버퍼는 끝난 기다림을 다음 TF 때 지운다
    assert ready.tf_buffer._new_data_callbacks == []


def _wait(executor, condition, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


def test_topics_single_thread_scan_before_tf(node):
    """실제 토픽: 스캔을 TF보다 먼저 보내도, 단일 스레드 executor에서 TF가 오면 그 스캔의 장애물을 낸다."""
    helper = Node('scan_defer_helper')
    static = StaticTransformBroadcaster(helper)
    broadcaster = TransformBroadcaster(helper)
    scan_pub = helper.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
    stamps = []
    helper.create_subscription(
        TrackedObstacleArray, 'perception/obstacles',
        lambda msg: stamps.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    try:
        static.sendTransform(_tf(BASE, LIDAR, 0.0, x=0.15))
        assert _wait(executor, lambda: scan_pub.get_subscription_count() > 0
                     and broadcaster.pub_tf.get_subscription_count() > 0)
        broadcaster.sendTransform(_tf(ODOM, BASE, 1.0))
        assert _wait(executor, lambda: node.tf_buffer.can_transform_core(
            ODOM, LIDAR, Time(seconds=1.0))[0])
        scan_pub.publish(_scan(1.1))
        assert _wait(executor, lambda: node.stats['deferred'] == 1)
        assert stamps == []
        broadcaster.sendTransform(_tf(ODOM, BASE, 1.2))
        assert _wait(executor, lambda: stamps)
        assert stamps == pytest.approx([1.1])
    finally:
        executor.remove_node(helper)
        executor.remove_node(node)
        executor.shutdown()
        helper.destroy_node()
