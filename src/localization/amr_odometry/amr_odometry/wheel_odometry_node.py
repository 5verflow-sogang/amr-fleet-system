"""joint_states(바퀴 각도) → 휠 오도메트리 odom (nav_msgs/Odometry).

    ros2 launch amr_odometry odometry.launch.py namespace:=amr_01

토픽은 상대 이름이다. 네임스페이스는 launch(bringup)가 붙인다.
TF(odom → base_footprint)는 기본으로 발행하지 않는다. D4(A안)에서 ekf_local이 발행한다 (docs/api.md).
"""

import math
import signal

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from amr_odometry.encoder_model import EncoderModel, EncoderNoiseConfig, LEFT, RIGHT
from amr_odometry.kinematics import DiffDriveOdometry

# 2D 주행에서 쓰지 않는 성분(z, roll, pitch)의 분산. 소비 측이 무시하도록 크게 둔다
UNUSED_VARIANCE = 1e6


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry')
        p = self.declare_parameter
        self.wheel_radius = p('wheel_radius', 0.1).value
        self.wheel_separation = p('wheel_separation', 0.34).value
        self.joint_names = (p('left_wheel_joint', 'left_wheel_joint').value,
                            p('right_wheel_joint', 'right_wheel_joint').value)
        prefix = p('frame_prefix', '').value
        self.odom_frame = prefix + p('odom_frame', 'odom').value
        self.base_frame = prefix + p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', False).value
        self.twist_variance_floor = p('twist_variance_floor', 1.0e-4).value
        config = EncoderNoiseConfig(
            ticks_per_rev=p('encoder.ticks_per_rev', 4096).value,
            wheel_radius_scale_stddev=p('encoder.wheel_radius_scale_stddev', 0.005).value,
            distance_noise_k=p('encoder.distance_noise_k', 0.01).value,
            seed=p('encoder.seed', 0).value,
            noise_enabled=p('encoder.noise_enabled', True).value,
        )

        self.encoder = EncoderModel(self.wheel_radius, config)
        self.odometry = DiffDriveOdometry(
            self.wheel_separation, config.distance_noise_k if config.noise_enabled else 0.0)
        self.indices = None
        rate = p('publish_rate', 50.0).value
        if not rate > 0.0:
            raise ValueError(f'publish_rate는 양수여야 한다: {rate}')
        self.publish_period = 1.0 / rate
        self.last_publish = None
        self.reset_accumulators()

        self.odom_pub = self.create_publisher(Odometry, p('odom_topic', 'odom').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)

        scale = self.encoder.radius_scale
        self.get_logger().info(
            f'wheel odometry: {self.odom_frame} → {self.base_frame}, '
            f'publish_tf={self.publish_tf}, noise={config.noise_enabled} '
            f'(radius scale L={scale[LEFT]:.4f} R={scale[RIGHT]:.4f}, seed={config.seed})')

    def find_indices(self, names):
        # URDF 조인트 이름에 네임스페이스 접두어(amr_01/)가 붙으므로 끝부분으로 찾는다
        found = []
        for wanted in self.joint_names:
            matches = [i for i, n in enumerate(names) if n == wanted or n.endswith('/' + wanted)]
            if len(matches) != 1:
                return None
            found.append(matches[0])
        return tuple(found)

    def on_joint_states(self, msg: JointState):
        if self.indices is None:
            self.indices = self.find_indices(msg.name)
            if self.indices is None:
                self.get_logger().warn(
                    f'joint_states에 바퀴 조인트 {self.joint_names}가 없다: '
                    f'{list(msg.name)}', throttle_duration_sec=5.0)
                return
        if len(msg.position) <= max(self.indices):
            return

        # Gazebo는 joint_states를 물리 스텝(1 kHz)마다 보낸다. 적분은 매번 하고,
        # 발행은 publish_rate로 묶어 그 구간의 평균 속도를 낸다
        d_left = self.encoder.measure(LEFT, msg.position[self.indices[LEFT]])
        d_right = self.encoder.measure(RIGHT, msg.position[self.indices[RIGHT]])
        ds, dyaw = self.odometry.update(d_left, d_right)
        self.acc_ds += ds
        self.acc_dyaw += dyaw
        self.acc_abs_left += abs(d_left)
        self.acc_abs_right += abs(d_right)

        stamp = Time.from_msg(msg.header.stamp)
        if self.last_publish is None:
            self.last_publish = stamp
            self.reset_accumulators()
            return
        dt = (stamp - self.last_publish).nanoseconds * 1e-9
        if dt + 1e-9 < self.publish_period:
            return
        var_v, var_w = self.odometry.twist_variance(self.acc_abs_left, self.acc_abs_right, dt)
        self.publish(msg.header.stamp, self.acc_ds / dt, self.acc_dyaw / dt, var_v, var_w)
        self.last_publish = stamp
        self.reset_accumulators()

    def reset_accumulators(self):
        self.acc_ds = self.acc_dyaw = 0.0
        self.acc_abs_left = self.acc_abs_right = 0.0

    def publish(self, stamp, v, w, var_v, var_w):
        pose = self.odometry.pose
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # 6x6 행 우선 (x, y, z, roll, pitch, yaw). 위치는 전파한 3x3 (x, y, yaw)을 넣는다
        cov = self.odometry.covariance
        floor = self.twist_variance_floor
        pose_cov = [0.0] * 36
        for i, a in enumerate((0, 1, 5)):
            for j, b in enumerate((0, 1, 5)):
                pose_cov[a * 6 + b] = float(cov[i, j])
        for a in (2, 3, 4):
            pose_cov[a * 6 + a] = UNUSED_VARIANCE
        for a in (0, 1, 5):
            pose_cov[a * 6 + a] = max(pose_cov[a * 6 + a], floor)
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = var_v + floor
        twist_cov[7] = floor          # vy: 차동구동은 옆으로 못 움직인다 (EKF가 vy=0 제약으로 씀)
        for a in (2, 3, 4):
            twist_cov[a * 6 + a] = UNUSED_VARIANCE
        twist_cov[35] = var_w + floor
        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C·launch 종료는 정상 종료로 본다 (exit code 0)
        pass
    finally:
        # Ctrl+C는 터미널과 launch에서 SIGINT가 두 번 온다. 정리 중에 끊기지 않게 이후 신호는 무시한다
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
