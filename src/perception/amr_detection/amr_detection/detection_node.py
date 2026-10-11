"""카메라 RGB·Depth → YOLO 인식 → Pinhole 3D → perception/objects, perception/markers (원 명세 6장).

RGB와 Depth는 rgbd_camera 센서 하나에서 나와 시각이 같고 픽셀이 1:1이다 (amr_description sensors.yaml).
추론이 카메라 주기(30 Hz)보다 느리므로 최신 짝 하나만 보관하고 타이머가 처리한다. 밀린 프레임을
차례로 처리하면 결과가 점점 과거 장면이 된다.
위치는 영상 촬영 시각의 TF로 target_frame(기본 map)에 옮긴다. header.stamp는 촬영 시각이다 (api.md 4-3절).
"""

import signal
import time

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from amr_msgs.msg import DetectedObject3DArray
from amr_perception_core.camera_model import PinholeCamera
from amr_perception_core.depth_sampling import (DepthSamplingConfig, bbox_center_to_pixel,
                                                sample_bbox_depth)
from amr_perception_core.projection import (backproject_with_covariance, quaternion_to_matrix,
                                            transform_with_covariance)

from .detector import YoloDetector, build_class_map
from .messages import to_markers, to_object


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection')
        p = self.declare_parameter
        self.detector = YoloDetector(
            model_path=p('model_path', '/opt/models/yolov8n.pt').value,
            class_map=build_class_map(p('model_classes', ['person', 'stop sign']).value,
                                      p('output_classes', ['person', 'sign']).value),
            device=p('device', 'cpu').value,
            image_size=p('image_size', 640).value,
            confidence=p('confidence_threshold', 0.5).value,
            iou=p('iou_threshold', 0.45).value,
            num_threads=p('num_threads', 4).value,
        )
        self.depth_config = DepthSamplingConfig(
            roi_scale=p('roi_scale', 0.5).value,
            min_depth=p('min_depth', 0.3).value,
            max_depth=p('max_depth', 10.0).value,
            min_valid_ratio=p('min_valid_ratio', 0.1).value,
        )
        self.sigma_px = p('pixel_noise_std', 5.0).value
        self.sigma_z = p('depth_noise_std', 0.02).value
        self.target_frame = p('target_frame', 'map').value
        self.tf_timeout = Duration(seconds=p('tf_timeout', 0.1).value)
        self.marker_size = p('marker_size', 0.4).value
        self.stats_period = p('stats_period', 5.0).value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera = None
        self.latest = None
        self._reset_stats()

        self.pub = self.create_publisher(DetectedObject3DArray, 'perception/objects', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'perception/markers', 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_camera_info,
                                 qos_profile_sensor_data)
        rgb = message_filters.Subscriber(self, Image, 'camera/color/image_raw',
                                         qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(self, Image, 'camera/depth/image_raw',
                                           qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.TimeSynchronizer([rgb, depth], 5)
        self.sync.registerCallback(self.on_images)
        self.create_timer(1.0 / p('max_rate', 30.0).value, self.process_latest)
        self.get_logger().info(
            f'detection: camera → perception/objects ({self.target_frame}), '
            f'클래스 {self.detector.class_map}, 입력 {self.detector.kwargs["imgsz"]} px')

    def on_camera_info(self, msg: CameraInfo):
        if self.camera is None:
            self.camera = PinholeCamera.from_k(msg.k)
            self.get_logger().info(
                f'카메라 K: fx {self.camera.fx:.1f}, fy {self.camera.fy:.1f}, '
                f'cx {self.camera.cx:.1f}, cy {self.camera.cy:.1f}')

    def on_images(self, rgb: Image, depth: Image):
        self.latest = (rgb, depth)   # 처리하지 못한 이전 짝은 버린다

    def process_latest(self):
        if self.latest is None or self.camera is None:
            return
        rgb_msg, depth_msg = self.latest
        self.latest = None

        start = time.monotonic()
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')   # ultralytics는 numpy 영상을 BGR로 본다
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        detections = self.detector.detect(bgr)
        infer_s = time.monotonic() - start

        stamp = rgb_msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, rgb_msg.header.frame_id, Time.from_msg(stamp),
                timeout=self.tf_timeout)
        except TransformException as e:
            self.stats['tf_fail'] += 1
            self.get_logger().warn(f'TF 없음, 프레임 건너뜀: {e}', throttle_duration_sec=5.0)
            return
        q, t = tf.transform.rotation, tf.transform.translation
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z])

        out = DetectedObject3DArray()
        out.header.stamp = stamp                  # 촬영 시각 (추론이 끝난 시각이 아님)
        out.header.frame_id = self.target_frame
        image_size = (bgr.shape[1], bgr.shape[0])
        for det in detections:
            sample = sample_bbox_depth(depth, det.xyxy, self.depth_config, image_size=image_size)
            if not sample.ok:
                self.stats['no_depth'] += 1      # 깊이가 없으면 3D 위치를 낼 수 없어 뺀다
                continue
            u, v = bbox_center_to_pixel(det.xyxy)
            p_cam, cov_cam = backproject_with_covariance(
                self.camera, u, v, sample.depth, self.sigma_px, self.sigma_z)
            p_out, cov_out = transform_with_covariance(rotation, translation, p_cam, cov_cam)
            out.objects.append(to_object(det, p_out, cov_out, float(np.linalg.norm(p_cam))))
        self.pub.publish(out)                     # 검출이 없어도 빈 배열 (api.md 4-3절)
        self.marker_pub.publish(to_markers(out, self.marker_size))

        self.stats['frames'] += 1
        self.stats['infer_s'] += infer_s
        self.stats['objects'] += len(out.objects)
        latency = self.get_clock().now() - Time.from_msg(stamp)
        self.stats['latency_s'] += latency.nanoseconds * 1e-9
        self._log_stats()

    def _reset_stats(self):
        self.stats = {'frames': 0, 'objects': 0, 'no_depth': 0, 'tf_fail': 0,
                      'infer_s': 0.0, 'latency_s': 0.0}
        self.stats_start = time.monotonic()

    def _log_stats(self):
        """처리 FPS(원 명세 KPI: CPU 10 FPS), 추론 시간, 촬영→발행 지연을 주기적으로 남긴다."""
        elapsed = time.monotonic() - self.stats_start
        if elapsed < self.stats_period:
            return
        s, n = self.stats, max(self.stats['frames'], 1)
        self.get_logger().info(
            f'{s["frames"] / elapsed:.1f} FPS, 추론 {1000 * s["infer_s"] / n:.0f} ms, '
            f'촬영→발행 {1000 * s["latency_s"] / n:.0f} ms, 객체 {s["objects"]}개, '
            f'깊이 없어 뺀 검출 {s["no_depth"]}개, TF 실패 {s["tf_fail"]}프레임')
        self._reset_stats()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
