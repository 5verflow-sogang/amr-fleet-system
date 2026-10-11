"""도킹 액션 서버 (docking/dock, amr_msgs/action/Dock).

한 번 호출에 한 번 시도한다 (3회 재시도는 BT의 DockWithRetry). 스테이션의 마커판(config/dock_stations.yaml)을
카메라로 보며 도킹 자세까지 들어간다. 과정은 controller.py, 판 → 오차 계산은 geometry.py 참고.

구독: camera/color/image_raw, camera/camera_info (enable_camera:=true 필요)
발행: cmd_vel_topic (기본 cmd_vel_nav = Nav2 컨트롤러 자리. 구역·안전 노드를 그대로 거친다)
TF: 카메라 광학 프레임 → base_frame (URDF 고정 변환)
"""

import math
import os
import threading
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from amr_msgs.action import Dock
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .controller import ControllerConfig, DockingController
from .detector import MarkerDetector, to_gray
from .geometry import DockError, dock_error
from .markers import load_stations

DOCK_NOT_IN_DB = 901


def _rotation(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class DockingServer(Node):

    def __init__(self):
        super().__init__('docking_server')
        p = self.declare_parameter
        stations_file = p('stations_file', '').value or os.path.join(
            get_package_share_directory('amr_docking'), 'config', 'dock_stations.yaml')
        self.stations = load_stations(stations_file)
        ns = self.get_namespace().strip('/')
        self.base_frame = p('base_frame', '').value or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self.control_hz = p('control_hz', 20.0).value
        self.max_age_s = p('max_age_s', 0.3).value
        self.log_period_s = p('log_period_s', 0.0).value   # 0보다 크면 진행 상황을 이 주기로 로그 (조정용)
        self.config = ControllerConfig(
            v_max=p('v_max', 0.12).value,
            w_max=p('w_max', 0.4).value,
            position_tol=p('position_tol', 0.02).value,
            yaw_tol=math.radians(p('yaw_tol_deg', 1.0).value),
            timeout_s=p('timeout_s', 60.0).value,
            verify_samples=p('verify_samples', 15).value,
        )

        self.detector = MarkerDetector()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.active = None          # (station, controller) 진행 중인 목표
        self.measurement: Optional[DockError] = None
        self.measured_at = 0.0

        group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic', 'cmd_vel_nav').value, 10)
        self.create_subscription(CameraInfo, p('camera_info_topic', 'camera/camera_info').value,
                                 self.on_info, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Image, p('image_topic', 'camera/color/image_raw').value,
                                 self.on_image, qos_profile_sensor_data, callback_group=group)
        self.server = ActionServer(
            self, Dock, 'docking/dock', self.execute, callback_group=group,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'docking_server: 스테이션 {len(self.stations)}곳, base {self.base_frame}, '
            f'허용 {self.config.position_tol * 100:.0f} cm / {math.degrees(self.config.yaw_tol):.1f}°')

    def on_info(self, msg: CameraInfo):
        with self.lock:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def on_image(self, msg: Image):
        with self.lock:
            active, k, dist = self.active, self.camera_matrix, self.dist
        if active is None or k is None:
            return
        station = active[0]
        try:
            gray = to_gray(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
            pair = self.detector.board_centers(gray, k, station.marker_ids, station.board.marker_size,
                                               station.board.spacing, dist)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return
        if pair is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'TF {msg.header.frame_id} → {self.base_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return
        rot = _rotation(tf.transform.rotation)
        trans = np.array([tf.transform.translation.x, tf.transform.translation.y,
                          tf.transform.translation.z])

        def base_xy(c):
            v = rot @ np.array(c) + trans
            return float(v[0]), float(v[1])

        error = dock_error(base_xy(pair[0]), base_xy(pair[1]), station.standoff)
        with self.lock:
            self.measurement = error
            self.measured_at = self.now_s()

    def on_goal(self, goal_request):
        with self.lock:
            busy = self.active is not None
        return GoalResponse.REJECT if busy else GoalResponse.ACCEPT

    def now_s(self) -> float:
        """노드 시계 [s]. use_sim_time이면 시뮬레이션 시간이라 RTF가 1보다 작아도 제어 주기·시간 초과가 맞다."""
        return self.get_clock().now().nanoseconds * 1e-9

    def stop(self):
        self.cmd_pub.publish(Twist())

    def execute(self, goal_handle):
        station_id = goal_handle.request.station_id
        result = Dock.Result()
        result.attempts = 1
        station = self.stations.get(station_id)
        if station is None:
            result.success = False
            result.error_code = DOCK_NOT_IN_DB
            result.message = f'unknown station {station_id}'
            goal_handle.abort()
            return result

        controller = DockingController(ControllerConfig(**vars(self.config)))
        with self.lock:
            self.measurement = None
            self.active = (station, controller)
        self.get_logger().info(f'도킹 시작: {station_id} (마커 {station.marker_ids})')
        period = 1.0 / self.control_hz
        last = self.now_s()
        last_log = 0.0
        outcome = None
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop()
                    goal_handle.canceled()
                    result.message = 'canceled'
                    return result
                wall = time.monotonic()
                now = self.now_s()
                dt, last = now - last, now
                with self.lock:
                    fresh = self.measurement if now - self.measured_at <= self.max_age_s else None
                v, w, outcome = controller.step(fresh, dt)
                if self.log_period_s > 0 and now - last_log >= self.log_period_s:
                    last_log = now
                    e = controller.last_error
                    self.get_logger().info(
                        f'{controller.stage} {"보임" if fresh else "안 보임"} v={v:.3f} w={w:.3f} ' + (
                            f'along={e.along * 100:.1f} lat={e.lateral * 100:.1f} cm '
                            f'head={math.degrees(e.heading):.2f}°' if e else ''))
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.cmd_pub.publish(cmd)
                if outcome is not None:
                    break
                feedback = Dock.Feedback()
                feedback.attempt = 1
                if controller.last_error is not None:
                    feedback.distance_remaining = float(controller.last_error.position)
                    feedback.yaw_error = float(controller.last_error.heading)
                goal_handle.publish_feedback(feedback)
                time.sleep(max(0.0, period - (time.monotonic() - wall)))
        finally:
            self.stop()
            with self.lock:
                self.active = None

        if outcome is None:   # 종료 중
            goal_handle.abort()
            result.message = 'shutdown'
            return result
        result.success = outcome.success
        result.error_code = outcome.error_code
        result.message = outcome.message
        if outcome.error is not None:
            result.position_error = float(outcome.error.position)
            result.yaw_error = float(outcome.error.heading)
        text = (f'도킹 {"성공" if outcome.success else "실패"}: {station_id} 위치 {result.position_error * 100:.1f} cm, '
                f'방향 {math.degrees(result.yaw_error):.2f}°, {controller.elapsed:.1f} s {outcome.message}')
        if outcome.success:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        if outcome.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
