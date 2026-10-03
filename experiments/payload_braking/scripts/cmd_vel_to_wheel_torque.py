#!/usr/bin/env python3
"""cmd_vel(Twist) -> 좌우 바퀴 목표 각속도(Float64, rad/s).

T 드라이브 모드(JointController, use_force_commands=true)는 바퀴 "각속도 목표"를
받아 cmd_max/cmd_min으로 묶인 힘(토크)으로 그 목표를 추종한다. 이 노드가 그 목표를
만든다 - DiffDrive가 하는 cmd_vel -> 바퀴 각속도 변환 + (normal 정지일 때만) 가속
제한 램프를 대신한다.

구독: /cmd_vel (geometry_msgs/Twist) - linear.x, angular.z만 쓴다
발행: /model/<model_name>/joint/left_wheel_joint/cmd_vel (std_msgs/Float64, rad/s)
      /model/<model_name>/joint/right_wheel_joint/cmd_vel (std_msgs/Float64, rad/s)
      (실제 Gazebo 토픽으로는 ros_gz_bridge가 중계한다)

ramp_limited 파라미터:
  true  (normal stop_mode): 목표가 바뀌면 선속도 가속도를 ±linear_accel_limit로
         제한해 한 걸음씩 다가간다.
  false (emergency stop_mode): 램프 없이 받은 cmd_vel을 그대로 바퀴 각속도로
         변환해 즉시 내보낸다 - 실제로 얼마나 빨리 서는지는 cmd_max/cmd_min
         토크 한계와 로봇 질량이 결정한다(이 실험의 핵심 측정 대상).

왜 rclpy 타이머(use_sim_time)를 안 쓰나: 이 실험은 Gazebo를 real_time_factor=0
(스로틀 없음)으로 돌려 실제 시간보다 훨씬 빠르게 진행한다. rclpy의 use_sim_time +
/clock 구독으로 타이머에 시뮬레이션 시각을 쓰게 하는 표준 방법을 먼저 시도했지만,
실측에서 램프가 여전히 벽시계 속도로 진행되는 것을 확인했다(1.0 m/s^2로 설계한
감속이 시뮬레이션 10초 넘게 걸림 - RTF만큼 느리게 보임). 원인을 더 파기보다, 이미
GT 위치 미분에 쓰는 것과 같은 방법으로 직접 해결한다: /clock 메시지를 구독해서
그 안의 시각 두 개를 직접 빼 dt를 구하고, 그 dt로 램프를 갱신한다 - rclpy
타이머·시계 내부 동작에 기대지 않는다.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


class CmdVelToWheelTorque(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_wheel_torque')

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('ramp_limited', True)
        self.declare_parameter('linear_accel_limit', 1.0)  # m/s^2, normal stop_mode
        self.declare_parameter('model_name', 'amr_payload')

        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheel_separation = self.get_parameter('wheel_separation').value
        self._ramp_limited = self.get_parameter('ramp_limited').value
        self._accel_limit = self.get_parameter('linear_accel_limit').value
        model_name = self.get_parameter('model_name').value

        # 목표(명령받은 값)와 현재 출력(램프 적용 후 실제로 내보내는 값)을 나눈다.
        self._target_v = 0.0
        self._target_w = 0.0
        self._output_v = 0.0
        self._last_sim_time = None

        self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._clock_sub = self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._left_pub = self.create_publisher(
            Float64, f'/model/{model_name}/joint/left_wheel_joint/cmd_vel', 10)
        self._right_pub = self.create_publisher(
            Float64, f'/model/{model_name}/joint/right_wheel_joint/cmd_vel', 10)

    def _on_cmd_vel(self, msg: Twist):
        self._target_v = msg.linear.x
        self._target_w = msg.angular.z

    def _on_clock(self, msg: Clock):
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self._last_sim_time is None:
            self._last_sim_time = sim_time
            return
        dt = sim_time - self._last_sim_time
        self._last_sim_time = sim_time
        if dt <= 0.0:
            return
        self._step(dt)

    def _step(self, dt):
        if self._ramp_limited:
            max_step = self._accel_limit * dt
            error = self._target_v - self._output_v
            step = max(-max_step, min(max_step, error))
            self._output_v += step
        else:
            self._output_v = self._target_v

        # 각속도(angular.z)는 이 실험(직진만)에서 안 쓰지만, 일반적인 차동구동
        # 변환식은 그대로 둔다 - w가 0이 아니면 각속도에는 램프를 걸지 않는다
        # (이 실험 범위 밖이라 단순하게 통과시킨다).
        v = self._output_v
        w = self._target_w

        left_wheel_v = (v - w * self._wheel_separation / 2.0) / self._wheel_radius
        right_wheel_v = (v + w * self._wheel_separation / 2.0) / self._wheel_radius

        if not math.isfinite(left_wheel_v) or not math.isfinite(right_wheel_v):
            return

        self._left_pub.publish(Float64(data=left_wheel_v))
        self._right_pub.publish(Float64(data=right_wheel_v))


def main():
    rclpy.init()
    node = CmdVelToWheelTorque()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
