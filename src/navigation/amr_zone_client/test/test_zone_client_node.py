"""zone_client_node.py의 위치 토픽 처리와 끊겼을 때 동작(fail-closed) 확인 (#66 하연 리뷰,
perf/zone-client-pose-topic).

launch_testing 없이 노드를 직접 만들어서 확인하는 가벼운 rclpy 단위 테스트다 - 실제
fleet_manager·Gazebo까지는 필요 없다. test_zone_client_launch.py(launch_testing)와는
별도의, 더 가벼운 경로다.
"""

import rclpy
from amr_msgs.msg import SafetyStatus, ZoneGrant
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

from amr_zone_client.zone_client_node import ZoneClientNode


def _odom(x: float, y: float, frame_id: str = 'map') -> Odometry:
    return Odometry(
        header=Header(frame_id=frame_id),
        pose=PoseWithCovariance(pose=Pose(position=Point(x=x, y=y))))


def test_no_robot_pose_sets_speed_cap_to_zero():
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            # 위치 토픽을 한 번도 못 받았으니 _robot_xy()는 반드시 None이다
            node._update()
            assert node._speed_cap == 0.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_robot_pose_timeout_sets_speed_cap_to_zero():
    # 위치를 받은 적은 있지만 robot_pose_timeout_s보다 오래 조용하면 끊김으로 본다
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            node._on_robot_pose(_odom(1.0, 2.0))
            assert node._robot_xy() == (1.0, 2.0)

            node._last_pose_t -= node._robot_pose_timeout_s + 0.1
            assert node._robot_xy() is None

            node._update()
            assert node._speed_cap == 0.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_robot_pose_with_wrong_frame_id_is_discarded():
    # ekf_local(odom 기준)을 잘못 물리는 등 frame_id가 map_frame이 아니면 그 위치는 버리고
    # fail-closed로 본다 (#66 하연 리뷰 후속)
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            node._on_robot_pose(_odom(1.0, 2.0, frame_id='amr_01/odom'))
            assert node._robot_xy() is None

            node._update()
            assert node._speed_cap == 0.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


class _RecordingCore:
    """core.update에 넘어온 robot_xy를 그대로 기록한다 (구역 진입·이탈 판단 자체는 test_core.py에서
    본다 - 여기서는 위치 토픽 → _robot_xy() → core.update 연결만 확인한다)."""

    def __init__(self):
        self.seen_xy = []

    def update(self, now, path_xy, robot_xy, safety_limit, current_speed):
        self.seen_xy.append(robot_xy)
        return [], None, None, []

    def state_label(self, now):
        return 'idle'


def test_robot_pose_topic_feeds_zone_update():
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            core = _RecordingCore()
            node._core = core

            node._on_robot_pose(_odom(1.0, 2.0))
            node._update()
            node._on_robot_pose(_odom(5.5, -3.0))
            node._update()

            assert core.seen_xy == [(1.0, 2.0), (5.5, -3.0)]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_prepare_shutdown_stops_timer_and_guards_callbacks():
    # #146 CI(test_zone_client_launch 간헐적 segfault): destroy_node() 전에 타이머를 지우고
    # 플래그를 켜서, 그 뒤 들어오는 콜백이 clock·TF·publisher를 더 안 쓰고 바로 반환하는지 확인한다.
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            node.prepare_shutdown()
            assert node._shutting_down is True
            assert node._update_timer is None

            # 가드 뒤에는 아무것도 안 하고 바로 반환한다 - 상태가 그대로다(바뀌면 가드가 안 걸린 것)
            node._update()
            assert node._speed_cap is None

            node._on_path(Path())
            assert node._path_xy == []
            node._on_safety_status(SafetyStatus(speed_limit=1.0))
            assert node._safety_speed_limit is None
            # 예외 없이 바로 반환하는지만 본다(clock·publisher를 안 써야 하므로 메시지 내용은 안 중요)
            node._on_cmd_vel(Twist())
            node._on_grant(ZoneGrant())
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


class _FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.linear.x)


class _FakeCore:
    """core.update가 정해 둔 상한만 돌려준다 (구역 상태 기계는 test_core.py에서 따로 본다)."""

    def __init__(self):
        self.cap = None

    def update(self, now, path_xy, robot_xy, safety_limit, current_speed):
        return [], self.cap, None, []

    def state_label(self, now):
        return 'waiting' if self.cap == 0.0 else 'approaching'


def test_stop_published_once_when_speed_cap_drops_to_zero_without_new_cmd_vel():
    # Nav2가 0.5 m/s를 낸 뒤 cmd_vel_nav 발행을 멈춰도, 상한이 0이 되는 순간 0을 한 번 낸다
    # (fleet_deadlock 조사: 상한은 cmd_vel_nav가 들어올 때만 적용돼 마지막 명령이 남았다)
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            pub, core = _FakePublisher(), _FakeCore()
            node._cmd_vel_pub = pub
            node._core = core
            node._robot_xy = lambda: (0.0, 0.0)

            node._on_cmd_vel(Twist(linear=Vector3(x=0.5)))
            assert pub.sent == [0.5]

            core.cap = 0.0
            node._update()
            assert pub.sent == [0.5, 0.0]
            node._update()
            node._update()
            assert pub.sent == [0.5, 0.0]  # 계속 0이면 다시 내지 않는다

            core.cap = 0.3
            node._update()
            assert pub.sent == [0.5, 0.0]  # 0이 아닌 상한으로 바뀌는 건 cmd_vel_nav에 맡긴다
            core.cap = 0.0
            node._update()
            assert pub.sent == [0.5, 0.0, 0.0]  # 다시 0으로 떨어지는 순간은 또 낸다
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

