"""ArUco 위치 노드: 보이는 마커판으로 로봇 map 자세를 낸다 (Kidnapped 복구용, #148, 10/19 안건 ③).

스테이션 마커판은 판마다 id가 달라서(markers.py) 판 하나만 보여도 창고 안 어디인지 안다. 복구 노드(amr_localization
kidnap_recovery)가 이 자세로 초기 위치를 좁게 다시 잡는다 (민지). 카메라 처리가 무거워 기본은 위치를 잃었을 때
(kidnap_recovery/state가 tracking이 아닐 때)만 인식한다.

구독: camera/color/image_raw, camera/camera_info, kidnap_recovery/state (std_msgs/String)
발행: aruco_pose (geometry_msgs/PoseWithCovarianceStamped, map 기준, 영상 시각)
TF: 카메라 광학 프레임 → base_frame (URDF), world → map (system.launch.py 정렬 원점)

판이 여럿 보이면 가장 가까운 판 하나로 낸다. 공분산은 판까지 거리에 따라 커진다 (localize.SigmaModel).
같은 판을 연달아 비슷한 자리로 읽었을 때만 낸다 (localize.Agreement, 뒤집힌 PnP 해 거르기).
"""

import math
import os
import threading
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .detector import MarkerDetector, to_gray
from .docking_server_node import _rotation
from .localize import Agreement, Pose2D, SigmaModel, board_in_base, inverse, robot_from_board, transform
from .markers import load_stations


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ArucoLocalizer(Node):

    def __init__(self):
        super().__init__('aruco_localizer')
        p = self.declare_parameter
        stations_file = p('stations_file', '').value or os.path.join(
            get_package_share_directory('amr_docking'), 'config', 'dock_stations.yaml')
        self.stations = list(load_stations(stations_file).values())
        ns = self.get_namespace().strip('/')
        self.base_frame = p('base_frame', '').value or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self.map_frame = p('map_frame', 'map').value
        self.world_frame = p('world_frame', 'world').value
        # true면 kidnap_recovery/state가 tracking이 아닐 때만 인식한다. 상태가 state_stale_s 넘게 안 오면 tracking으로 본다
        self.only_when_lost = p('only_when_lost', True).value
        self.state_stale_s = p('state_stale_s', 5.0).value
        self.min_period_s = 1.0 / max(p('max_hz', 5.0).value, 1e-3)
        self.max_distance = p('max_distance', 8.0).value
        self.sigma = SigmaModel(
            xy0=p('sigma_xy0', SigmaModel.xy0).value, xy2=p('sigma_xy2', SigmaModel.xy2).value,
            yaw0=math.radians(p('sigma_yaw0_deg', math.degrees(SigmaModel.yaw0)).value),
            yaw1=math.radians(p('sigma_yaw1_deg', math.degrees(SigmaModel.yaw1)).value))
        # confirm_n이 1이면 거르지 않는다
        self.agreement = Agreement(n=p('confirm_n', 2).value, tol_m=p('confirm_tol_m', 0.4).value,
                                   window_s=p('confirm_window_s', 2.0).value)

        self.detector = MarkerDetector()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.state = 'tracking'
        self.state_at = -math.inf
        self.last_run = -math.inf
        self.was_active = False

        self.pub = self.create_publisher(PoseWithCovarianceStamped, p('pose_topic', 'aruco_pose').value, 10)
        self.create_subscription(String, p('state_topic', 'kidnap_recovery/state').value, self.on_state, 10)
        self.create_subscription(CameraInfo, p('camera_info_topic', 'camera/camera_info').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic', 'camera/color/image_raw').value,
                                 self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'aruco_localizer: 마커판 {len(self.stations)}개, {self.map_frame} 기준 aruco_pose, '
            f'{"위치를 잃었을 때만" if self.only_when_lost else "항상"} 인식 (최대 {1 / self.min_period_s:.0f} Hz, '
            f'{self.max_distance:.1f} m까지)')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg: String):
        with self.lock:
            self.state = msg.data
            self.state_at = self.now_s()

    def on_info(self, msg: CameraInfo):
        with self.lock:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def active(self, now: float) -> bool:
        if not self.only_when_lost:
            return True
        with self.lock:
            state, at = self.state, self.state_at
        return state != 'tracking' and now - at <= self.state_stale_s

    def on_image(self, msg: Image):
        now = self.now_s()
        active = self.active(now)
        if active != self.was_active:
            self.get_logger().info(
                ('인식 시작 (위치 잃음)' if self.only_when_lost else '인식 시작') if active else '인식 멈춤 (tracking)')
            self.was_active = active
        if not active or now - self.last_run < self.min_period_s:
            return
        self.last_run = now
        with self.lock:
            k, dist = self.camera_matrix, self.dist
        if k is None:
            return
        try:
            gray = to_gray(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return
        corners = self.detector.detect(gray)
        if not corners:
            return
        try:
            cam = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            origin = self.tf_buffer.lookup_transform(self.world_frame, self.map_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=5.0)
            return
        rot = _rotation(cam.transform.rotation)
        trans = np.array([cam.transform.translation.x, cam.transform.translation.y, cam.transform.translation.z])

        best = None
        for station in self.stations:
            pair = self.detector.board_from_corners(corners, k, station.marker_ids, station.board.marker_size,
                                                    station.board.spacing, dist)
            if pair is None:
                continue
            left, right = ((float(v[0]), float(v[1])) for v in (rot @ np.array(c) + trans for c in pair))
            board = board_in_base(left, right)
            distance = math.hypot(board.x, board.y)
            if distance > self.max_distance or (best is not None and distance >= best[0]):
                continue
            bx, by, _ = station.board_center
            best = (distance, station, robot_from_board(board, Pose2D(bx, by, station.board_yaw)))
        if best is None:
            return
        distance, station, robot_world = best
        o = origin.transform
        map_in_world = Pose2D(o.translation.x, o.translation.y, _yaw(o.rotation))
        robot = transform(robot_world, inverse(map_in_world))
        if not self.agreement.check(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9, station.name, robot.x, robot.y):
            return
        sxy, syaw = self.sigma.sigma(distance, station.board.spacing)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x, out.pose.pose.position.y = robot.x, robot.y
        out.pose.pose.orientation.z = math.sin(robot.yaw / 2)
        out.pose.pose.orientation.w = math.cos(robot.yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy * sxy
        cov[35] = syaw * syaw
        cov[14] = cov[21] = cov[28] = 1e6   # z·roll·pitch는 쓰지 않는다
        out.pose.covariance = cov
        self.pub.publish(out)
        self.get_logger().info(
            f'{station.name} 판 {distance:.2f} m → map ({robot.x:.2f}, {robot.y:.2f}, {math.degrees(robot.yaw):.1f}°), '
            f'σ {sxy * 100:.1f} cm / {math.degrees(syaw):.1f}°', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
