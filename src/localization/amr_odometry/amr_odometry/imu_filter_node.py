"""imu/data_raw (Gazebo 원본) → imu/data (바이어스 보정 + 저역통과 + 공분산).

인계 계약(명세서 8-2절)의 /imu/data다. 시작 후 calibration_duration 동안 로봇이 정지해 있어야 한다.
보정이 끝나기 전에는 발행하지 않는다. 자기장 센서가 없어 방향(orientation)은 추정하지 않는다 (REP 145).
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu

from amr_odometry.imu_filter import ImuFilter, ImuFilterConfig


def diag3(variance: float):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class ImuFilterNode(Node):

    def __init__(self):
        super().__init__('imu_filter')
        p = self.declare_parameter
        self.config = ImuFilterConfig(
            gyro_cutoff_hz=p('gyro_cutoff_hz', 20.0).value,
            accel_cutoff_hz=p('accel_cutoff_hz', 5.0).value,
            calibration_duration=p('calibration_duration', 2.0).value,
            stationary_gyro_stddev=p('stationary_gyro_stddev', 0.01).value,
            stationary_accel_stddev=p('stationary_accel_stddev', 0.1).value,
        )
        # 공분산에 쓰는 센서 노이즈 σ. amr_description/config/sensors.yaml의 IMU 값과 같게 둔다
        self.gyro_sigma = p('gyro_noise_stddev', 2.0e-4).value
        self.accel_sigma = p('accel_noise_stddev', 1.7e-2).value
        self.filter = ImuFilter(self.config)
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.reported = False
        self.restarts_logged = 0
        self.get_logger().info(
            f'imu filter: imu/data_raw → imu/data, 바이어스 보정 {self.config.calibration_duration} s '
            f'(로봇을 정지 상태로 두세요)')

    def on_imu(self, msg: Imu):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        w, a = msg.angular_velocity, msg.linear_acceleration
        result = self.filter.update(t, (w.x, w.y, w.z), (a.x, a.y, a.z))
        cal = self.filter.calibrator
        if cal.restarts > self.restarts_logged:
            self.restarts_logged = cal.restarts
            self.get_logger().warn('바이어스 보정 중 움직임 감지 → 다시 잽니다')
        if result is None:
            return
        if not self.reported:
            self.reported = True
            self.get_logger().info(
                f'바이어스 보정 완료 ({cal.samples} 샘플): gyro {cal.gyro_bias.round(6).tolist()} rad/s, '
                f'accel {cal.accel_bias.round(4).tolist()} m/s²')
        gyro, accel = result
        # 보정 평균의 표준오차(σ/√N)를 남은 바이어스 불확실성으로 더한다
        n = max(cal.samples, 1)
        gyro_var = self.gyro_sigma ** 2 * (1.0 + 1.0 / n)
        accel_var = self.accel_sigma ** 2 * (1.0 + 1.0 / n)

        out = Imu()
        out.header = msg.header
        out.orientation.w = 1.0                # 단위 쿼터니언 (값은 쓰지 않음)
        out.orientation_covariance[0] = -1.0   # 방향 추정 없음 (REP 145)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro.tolist()
        out.angular_velocity_covariance = diag3(gyro_var)
        acc = out.linear_acceleration
        acc.x, acc.y, acc.z = accel.tolist()
        out.linear_acceleration_covariance = diag3(accel_var)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C는 터미널과 launch에서 SIGINT가 두 번 온다. 정리 중에 끊기지 않게 이후 신호는 무시한다
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
