"""동적 장애물 회피 KPI(test-plan I17) 기록 노드. 정답(GT)으로 충돌·경로 이탈·복귀 시간을 잰다.

    ros2 run amr_safety avoidance_eval --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
        -p "map_origin_in_world:=[0.0, 1.5, 0.0]" -p output_dir:=/tmp/amr_avoidance/run1

구독 (상대 이름은 실행 시 네임스페이스 밑):
    ground_truth/odom                          nav_msgs/Odometry  로봇 정답, world 기준 (amr_gazebo)
    plan                                       nav_msgs/Path      Nav2 planner_server 전역 경로 (map 기준)
    /dynamic_obstacles/<id>/ground_truth/odom  nav_msgs/Odometry  장애물 정답, world 기준 (amr_gazebo)
        obstacle_ids가 비면 이 이름의 토픽을 1초마다 찾아 붙는다
서비스:
    ~/save   std_srvs/Trigger   지금까지 모은 기록으로 계산해 저장. 종료할 때도 자동 저장

로봇 판단에 쓰지 않는 평가 전용 노드다(GT 사용). 계산은 amr_safety.avoidance_eval이 하고,
출력 파일과 판정 기준은 README "회피 평가 (I17)".
"""

import math
import os
import re
import time
from dataclasses import fields

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .avoidance_eval import AvoidanceParams, AvoidanceRecorder, Pose, format_summary, shape_for_id
from .avoidance_eval.metrics import CIRCLE

OBSTACLE_TOPIC = re.compile(r'^/dynamic_obstacles/([^/]+)/ground_truth/odom$')


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class AvoidanceEvalNode(Node):

    def __init__(self):
        super().__init__('avoidance_eval')
        robot_default = self.get_namespace().strip('/') or 'amr'
        self.declare_parameter('robot', robot_default)
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('plan_topic', 'plan')
        # 비우면 /dynamic_obstacles/*/ground_truth/odom을 찾아 모두 쓴다
        self.declare_parameter('obstacle_ids', [''])
        # id 앞부분(worker·forklift·robot)으로 모양을 못 정하는 장애물의 원 반지름 [m]. 0 이하면 그런 장애물은 무시
        self.declare_parameter('default_obstacle_radius', 0.3)
        self.declare_parameter('map_origin_in_world', [0.0, 1.5, 0.0])
        self.declare_parameter('reference_mode', 'first_plan')
        # reference_mode=fixed일 때 map 좌표 [x0, y0, x1, y1, ...]
        self.declare_parameter('fixed_path', [0.0])
        self.declare_parameter('output_dir', '')
        # 판정 기준값: AvoidanceParams 필드 이름 그대로 (README "회피 평가 (I17)")
        defaults = AvoidanceParams()
        for f in fields(AvoidanceParams):
            self.declare_parameter(f.name, getattr(defaults, f.name))

        p = self.get_parameter
        robot = p('robot').value
        params = AvoidanceParams(**{f.name: p(f.name).value for f in fields(AvoidanceParams)})
        flat = [float(v) for v in p('fixed_path').value]
        fixed = list(zip(flat[0::2], flat[1::2])) if len(flat) >= 4 else None
        self.default_radius = float(p('default_obstacle_radius').value)
        out = p('output_dir').value or os.path.join(
            '/tmp/amr_avoidance', f"{robot}_{time.strftime('%Y%m%d-%H%M%S')}")
        self.output_dir = out
        self.recorder = AvoidanceRecorder(
            robot, [float(v) for v in p('map_origin_in_world').value], {}, params,
            p('reference_mode').value, fixed, stream_dir=out)

        # GT 브리지 중 best effort로 내는 것이 있어 best effort로 받는다 (reliable 발행자와도 호환)
        self.gt_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, p('gt_topic').value, self._on_robot, self.gt_qos)
        self.create_subscription(Path, p('plan_topic').value, self._on_plan,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._obstacle_subs = {}
        ids = [i for i in p('obstacle_ids').value if i]
        for oid in ids:
            self._subscribe_obstacle(oid)
        if not ids:
            self.create_timer(1.0, self._discover)
        self.create_service(Trigger, '~/save', self._on_save)
        self.create_timer(10.0, self._log_counts)
        self.get_logger().info(
            f'기록 시작: robot={robot}, 기준 경로={p("reference_mode").value}, '
            f'map_origin_in_world={self.recorder.origin}, 장애물={ids or "자동 탐색"}, 저장 위치={out}')

    def _subscribe_obstacle(self, oid: str) -> None:
        if oid in self._obstacle_subs:
            return
        try:
            shape = shape_for_id(oid, self.default_radius if self.default_radius > 0 else None)
        except KeyError:
            self.get_logger().warn(f'모양을 모르는 장애물 {oid}는 무시한다 (default_obstacle_radius > 0이면 원으로 본다)')
            self._obstacle_subs[oid] = None
            return
        self.recorder.add_shape(oid, shape)
        desc = f'원 r={shape.radius}' if shape.kind == CIRCLE else f'사각형 {shape.length}×{shape.width}'
        self.get_logger().info(f'장애물 {oid} 구독 ({desc} m)')
        self._obstacle_subs[oid] = self.create_subscription(
            Odometry, f'/dynamic_obstacles/{oid}/ground_truth/odom',
            lambda msg, oid=oid: self._on_obstacle(oid, msg), self.gt_qos)

    def _discover(self) -> None:
        for name, _types in self.get_topic_names_and_types():
            m = OBSTACLE_TOPIC.match(name)
            if m:
                self._subscribe_obstacle(m.group(1))

    def _on_robot(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.recorder.add_robot(Pose(_stamp(msg.header.stamp), pose.position.x, pose.position.y,
                                     _yaw(pose.orientation)))

    def _on_obstacle(self, oid: str, msg: Odometry) -> None:
        pose, tw = msg.pose.pose, msg.twist.twist.linear
        self.recorder.add_obstacle(oid, Pose(_stamp(msg.header.stamp), pose.position.x, pose.position.y,
                                             _yaw(pose.orientation), math.hypot(tw.x, tw.y)))

    def _on_plan(self, msg: Path) -> None:
        stamp = _stamp(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9
        frame = msg.header.frame_id
        if frame and frame != 'map':
            self.get_logger().warn(
                f'전역 경로 좌표계가 map이 아니다 ({frame}). map_origin_in_world로 world에 옮기므로 틀릴 수 있다',
                throttle_duration_sec=30.0)
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if self.recorder.add_plan(stamp, frame, pts):
            self.get_logger().info(f'기준 경로 채택: 점 {len(pts)}개, t={stamp:.2f}')

    def _log_counts(self) -> None:
        r, o, plans = self.recorder.counts
        self.get_logger().info(f'수신: 로봇 GT {r}개, 장애물 GT {o}개, 전역 경로 {plans}개')

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
    node = AvoidanceEvalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C(SIGINT)와 launch·하네스 정리(SIGTERM) 모두 저장하고 끝낸다
        pass
    finally:
        # 받은 것이 없어도 저장한다: 무효 사유(로봇 GT 없음 등)가 남고, 하네스가 wait_files를 헛되이 기다리지 않는다
        try:
            node.save()
        except Exception as e:  # noqa: BLE001 저장 실패가 종료를 막지 않게 한다
            node.get_logger().error(f'저장 실패: {e}')
        node.recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
