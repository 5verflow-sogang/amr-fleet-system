"""추론이 끝났을 때 촬영 시각 TF가 없으면 결과를 미뤄 두었다가 발행하는지 (detection_node, 단일 스레드).

콜백 안에서 TF를 기다리지 않는다. 촬영 시각의 TF가 아직 없으면 결과를 미뤄 두고, 버퍼에 그 시각을 덮는 TF가
들어오는 순간(Buffer.set_transform) 발행한다. 미룬 결과가 있는 동안에는 다음 추론을 쉬고, tf_timeout을 넘기면 버린다.
YOLO는 가짜 추론기로 바꿔 모델 없이 돈다. 앞의 테스트들은 버퍼에 TF를 직접 넣고, 마지막 테스트는 실제 /tf·영상
토픽과 단일 스레드 executor로 확인한다.
"""

import time

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from amr_detection import detection_node
from amr_detection.detector import Detection
from amr_msgs.msg import DetectedObject3DArray
from amr_perception_core.camera_model import PinholeCamera
from sensor_msgs.msg import Image

MAP, ODOM, OPTICAL = 'map', 'odom', 'camera_optical_frame'
W, H = 64, 48       # 작은 영상 (처리 여부만 본다)


class FakeDetector:
    """영상 가운데에 사람 하나를 내는 가짜 추론기. 부른 횟수를 센다."""

    def __init__(self, **kwargs):
        self.class_map = {'person': 'person'}
        self.kwargs = {'imgsz': W}
        self.num_threads = 1
        self.calls = 0

    def detect(self, bgr):
        self.calls += 1
        return [Detection('person', 0, 0.9, (28.0, 20.0, 36.0, 28.0))]


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(detection_node, 'YoloDetector', FakeDetector)
    rclpy.init()
    det = detection_node.DetectionNode()
    det.camera = PinholeCamera.from_k([34.0, 0.0, W / 2, 0.0, 34.0, H / 2, 0.0, 0.0, 1.0])
    yield det
    det.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def ready(node):
    """카메라 위치(odom → optical, 정적)와 1.0 s까지의 map → odom TF가 있는 노드."""
    node.tf_buffer.set_transform_static(_tf(ODOM, OPTICAL, 0.0, x=0.2), 'test')
    _map_tf(node, 1.0)
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


def _map_tf(node, t: float):
    node.tf_buffer.set_transform(_tf(MAP, ODOM, t, x=1.0), 'test')


def _images(t: float) -> tuple:
    """같은 촬영 시각의 RGB·Depth 짝 (깊이 2 m)."""
    bridge = CvBridge()
    rgb = bridge.cv2_to_imgmsg(np.zeros((H, W, 3), np.uint8), 'rgb8')
    depth = bridge.cv2_to_imgmsg(np.full((H, W), 2.0, np.float32), '32FC1')
    for m in (rgb, depth):
        m.header.stamp = _stamp(t)
        m.header.frame_id = OPTICAL
    return rgb, depth


def _record(node) -> list:
    """발행된 결과의 촬영 시각 [s] 목록. 발행 대신 기록만 한다."""
    seen = []

    class Pub:
        def publish(self, msg):
            seen.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)

    node.pub = Pub()
    return seen


def test_result_with_tf_is_published_at_once(ready):
    seen = _record(ready)
    _map_tf(ready, 1.2)
    ready.on_images(*_images(1.1))
    ready.process_latest()
    assert seen == pytest.approx([1.1])
    assert ready.stats['deferred'] == 0 and ready._pending is None


def test_result_before_tf_waits_for_it(ready):
    seen = _record(ready)
    ready.on_images(*_images(1.1))  # TF는 1.0 s까지뿐
    ready.process_latest()
    assert seen == []
    assert ready.stats['deferred'] == 1
    _map_tf(ready, 1.05)            # 아직 1.1을 못 덮는다
    assert seen == []
    _map_tf(ready, 1.15)            # 덮는 TF가 들어오는 순간 발행
    assert seen == pytest.approx([1.1])
    assert ready._pending is None


def test_no_new_inference_while_waiting(ready):
    """미룬 결과가 있는 동안 새 영상이 와도 추론하지 않는다 (/tf 콜백이 먼저 돌게 차례를 넘긴다)."""
    _record(ready)
    ready.on_images(*_images(1.1))
    ready.process_latest()
    ready.on_images(*_images(1.2))
    ready.process_latest()
    assert ready.detector.calls == 1
    assert ready.latest is not None          # 새 영상은 그대로 남아 다음 차례에 처리한다


def test_waiting_result_dropped_after_timeout(ready):
    seen = _record(ready)
    ready.on_images(*_images(1.1))
    ready.process_latest()
    # tf_timeout(0.5 s)보다 오래 기다린 것으로 만든다
    ready._pending = ready._pending[:-1] + (time.monotonic() - 1.0,)
    ready.on_images(*_images(1.2))
    ready.process_latest()                   # 앞 결과를 버리고 새 영상을 추론한다
    assert ready.stats['tf_drop'] == 1 and ready.detector.calls == 2
    _map_tf(ready, 1.3)
    assert seen == pytest.approx([1.2])


def test_no_waiting_check_left_in_buffer(ready):
    """버린 결과의 TF 기다림이 버퍼에 남지 않는다. 남으면 TF가 올 때마다 헛확인을 하고, 끝내 오지 않는 TF면
    계속 쌓인다."""
    _record(ready)
    rgb, depth = _images(1.1)
    rgb.header.frame_id = 'unknown_frame'    # 이 TF는 끝내 오지 않는다
    ready.on_images(rgb, depth)
    ready.process_latest()
    ready._pending = ready._pending[:-1] + (time.monotonic() - 1.0,)
    ready.on_images(*_images(1.2))
    ready.process_latest()                   # 앞 결과를 버린다
    _map_tf(ready, 1.3)
    _map_tf(ready, 1.4)             # 버퍼는 끝난 기다림을 다음 TF 때 지운다
    assert ready.tf_buffer._new_data_callbacks == []


def _wait(executor, condition, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


def test_topics_single_thread_image_before_tf(node):
    """실제 토픽: 영상을 TF보다 먼저 보내도, 단일 스레드 executor에서 TF가 오면 그 영상의 결과를 낸다."""
    helper = Node('detection_defer_helper')
    static = StaticTransformBroadcaster(helper)
    broadcaster = TransformBroadcaster(helper)
    rgb_pub = helper.create_publisher(Image, 'camera/color/image_raw', qos_profile_sensor_data)
    depth_pub = helper.create_publisher(Image, 'camera/depth/image_raw', qos_profile_sensor_data)
    stamps = []
    helper.create_subscription(
        DetectedObject3DArray, 'perception/objects',
        lambda msg: stamps.append((Time.from_msg(msg.header.stamp).nanoseconds * 1e-9, len(msg.objects))), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    try:
        static.sendTransform(_tf(ODOM, OPTICAL, 0.0, x=0.2))
        assert _wait(executor, lambda: rgb_pub.get_subscription_count() > 0
                     and depth_pub.get_subscription_count() > 0
                     and broadcaster.pub_tf.get_subscription_count() > 0)
        broadcaster.sendTransform(_tf(MAP, ODOM, 1.0, x=1.0))
        assert _wait(executor, lambda: node.tf_buffer.can_transform_core(
            MAP, OPTICAL, Time(seconds=1.0))[0])
        rgb, depth = _images(1.1)
        rgb_pub.publish(rgb)
        depth_pub.publish(depth)
        assert _wait(executor, lambda: node.stats['deferred'] == 1)
        assert stamps == []
        broadcaster.sendTransform(_tf(MAP, ODOM, 1.2, x=1.0))
        assert _wait(executor, lambda: stamps)
        assert stamps == [(pytest.approx(1.1), 1)]
    finally:
        executor.remove_node(helper)
        executor.remove_node(node)
        executor.shutdown()
        helper.destroy_node()
