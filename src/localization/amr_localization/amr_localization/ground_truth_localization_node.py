"""시뮬레이터 정답 위치(ground_truth/odom, world 기준)를 AMCL+EKF 대신 쓴다 - 식당 시연 녹화용.

리허설에서 AMCL 오차 0.9~2.3 m 때문에 로봇이 테이블 안에 있다고 믿고 멈추거나 거짓 도착이 났다
(원인 분석·지도 분리는 pp 창에서 따로 진행). 위치추정 성능을 전혀 반영하지 않으므로 물류센터
KPI에는 쓰지 않는다 - localization.launch.py(AMCL+EKF)와 같은 토픽·TF를 내 나머지 노드(Nav2,
구역 클라이언트 등)는 바뀐 걸 모른다:
    발행: odometry/local(odom 기준), odometry/filtered(map 기준), amcl_pose
    /tf:  <ns>/odom → <ns>/base_footprint (map → <ns>/odom은 launch의 static_transform_publisher가 identity로 낸다)
    구독: ground_truth/odom (world 기준, amr_gazebo)
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .eval.trajectory import quaternion_from_yaw, yaw_from_quaternion
from .ground_truth import world_pose_to_map

# amcl_pose 공분산 대각(x, y, yaw) - 0이 아니라 아주 작은 값을 넣는다. 0은 "공분산 없음"으로
# 읽는 소비자가 있을 수 있어, "정답이라 거의 확실하다"는 뜻으로 작은 값을 쓴다
_COVARIANCE_DIAG = 1e-6


class GroundTruthLocalizationNode(Node):

    def __init__(self):
        super().__init__('ground_truth_localization')
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        self._map_frame = 'map'
        self._odom_frame = prefix + 'odom'
        self._base_frame = prefix + 'base_footprint'
        origin = self.declare_parameter(
            'map_origin_in_world', [-0.0439, 1.4712, -0.00088]).value
        self._origin = tuple(float(v) for v in origin)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._local_pub = self.create_publisher(Odometry, 'odometry/local', 10)
        self._filtered_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_ground_truth, 10)
        self.get_logger().info(
            f'정답 위치추정(demo): map_origin_in_world={list(self._origin)} - 식당 시연 녹화용, '
            '물류센터 KPI에는 쓰지 않는다')

    def _on_ground_truth(self, msg: Odometry):
        p = msg.pose.pose.position
        world_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        x, y, yaw = world_pose_to_map(p.x, p.y, world_yaw, self._origin)
        _, _, qz, qw = quaternion_from_yaw(yaw)
        stamp = msg.header.stamp

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        # map → <ns>/odom이 identity(launch에서 static으로 낸다)라, odom 기준·map 기준 자세가 같은 값이다
        for pub, frame_id in ((self._local_pub, self._odom_frame), (self._filtered_pub, self._map_frame)):
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = frame_id
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist = msg.twist
            pub.publish(odom)

        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp = stamp
        amcl.header.frame_id = self._map_frame
        amcl.pose.pose.position.x = x
        amcl.pose.pose.position.y = y
        amcl.pose.pose.orientation.z = qz
        amcl.pose.pose.orientation.w = qw
        for i in (0, 7, 35):  # x, y, yaw 대각(robot_localization 6x6 공분산 레이아웃)
            amcl.pose.covariance[i] = _COVARIANCE_DIAG
        self._amcl_pub.publish(amcl)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
