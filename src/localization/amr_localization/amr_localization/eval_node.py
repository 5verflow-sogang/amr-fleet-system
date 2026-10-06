"""정답(GT) 대비 위치추정 오차를 기록하는 노드.

    ros2 run amr_localization eval_node --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
        -p map_origin_in_world:="[0.0, 1.5, 0.0]"

구독 (상대 이름, 네임스페이스는 실행 시 부여):
    ground_truth/odom   nav_msgs/Odometry   정답, world 기준 (amr_gazebo)
    odometry/filtered   nav_msgs/Odometry   추정, map 기준 (estimate_type=odometry, 기본)
    또는 amcl_pose      geometry_msgs/PoseWithCovarianceStamped (estimate_type=pose)
서비스:
    ~/save              std_srvs/Trigger    지금까지 모은 기록을 저장. 종료할 때도 자동 저장
저장 형식은 하네스 로그 포맷(tools/harness/README.md). 계산은 amr_localization.eval이 한다.
"""

import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .eval import EvalRecorder, Pose2D, format_summary
from .eval.trajectory import yaw_from_quaternion


def _to_pose2d(stamp, pose) -> Pose2D:
    q = pose.orientation
    return Pose2D(
        stamp.sec + stamp.nanosec * 1e-9,
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(q.x, q.y, q.z, q.w),
    )


class EvalNode(Node):

    def __init__(self):
        super().__init__('localization_eval')
        robot_default = self.get_namespace().strip('/') or 'amr'
        self.declare_parameter('robot', robot_default)
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('estimate_topic', 'odometry/filtered')
        self.declare_parameter('estimate_type', 'odometry')
        self.declare_parameter('map_origin_in_world', [0.0, 1.5, 0.0])
        self.declare_parameter('output_dir', '')
        self.declare_parameter('max_gap', 0.05)
        self.declare_parameter('moving_v', 0.01)
        self.declare_parameter('moving_w', 0.05)
        self.declare_parameter('skip_initial_stationary', True)

        p = self.get_parameter
        robot = p('robot').value
        origin = tuple(float(v) for v in p('map_origin_in_world').value)
        if len(origin) != 3:
            raise ValueError('map_origin_in_world는 [x, y, yaw] 3개여야 한다')
        out = p('output_dir').value or os.path.join(
            '/tmp/amr_eval', f"{robot}_{time.strftime('%Y%m%d-%H%M%S')}")
        self.output_dir = out
        # 궤적(원자료)은 받는 즉시 파일에 이어 쓴다. 강제 종료돼도 남고, eval_offline으로 다시 계산한다
        self.recorder = EvalRecorder(
            robot, origin, p('max_gap').value, p('moving_v').value, p('moving_w').value, stream_dir=out,
            skip_initial_stationary=p('skip_initial_stationary').value)

        # best effort 구독은 reliable·best effort 발행자 모두와 호환된다.
        # (Gazebo 브리지·시뮬레이터 정답 토픽 중 best effort로 발행하는 것이 있다)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, p('gt_topic').value, self._on_gt, qos)
        est_type = p('estimate_type').value
        if est_type == 'odometry':
            self.create_subscription(Odometry, p('estimate_topic').value, self._on_est_odom, qos)
        elif est_type == 'pose':
            self.create_subscription(
                PoseWithCovarianceStamped, p('estimate_topic').value, self._on_est_pose, qos)
        else:
            raise ValueError(f"estimate_type은 'odometry' 또는 'pose' (지금 '{est_type}')")
        self.create_service(Trigger, '~/save', self._on_save)
        self.create_timer(10.0, self._log_counts)
        self.get_logger().info(
            f'기록 시작: robot={robot}, map_origin_in_world={origin}, 저장 위치={out}')

    def _on_gt(self, msg: Odometry) -> None:
        self.recorder.add_gt(_to_pose2d(msg.header.stamp, msg.pose.pose))

    def _on_est_odom(self, msg: Odometry) -> None:
        self.recorder.add_est(_to_pose2d(msg.header.stamp, msg.pose.pose))

    def _on_est_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.recorder.add_est(_to_pose2d(msg.header.stamp, msg.pose.pose))

    def _log_counts(self) -> None:
        gt, est = self.recorder.counts
        self.get_logger().info(f'수신: 정답 {gt}개, 추정 {est}개')

    def save(self) -> str:
        summary = self.recorder.save(self.output_dir)
        line = format_summary(summary)
        self.get_logger().info(f'저장 {self.output_dir}: {line}')
        return line

    def _on_save(self, _request, response):
        try:
            response.message = self.save()
            response.success = True
        except Exception as e:  # noqa: BLE001 저장 실패를 서비스 응답으로 돌려준다
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EvalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C(SIGINT)와 launch·docker stop(SIGTERM) 모두 저장하고 끝낸다
        pass
    finally:
        gt, est = node.recorder.counts
        if gt and est:
            node.save()
        node.recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
