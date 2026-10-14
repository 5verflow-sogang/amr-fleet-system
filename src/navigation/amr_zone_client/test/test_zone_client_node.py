"""zone_client_node.py의 TF 조회 실패 시 동작(fail-closed) 확인 (#66 하연 리뷰).

launch_testing 없이 노드를 직접 만들어서 확인하는 가벼운 rclpy 단위 테스트다 - 실제
fleet_manager·Gazebo까지는 필요 없다. test_zone_client_launch.py(launch_testing)와는
별도의, 더 가벼운 경로다.
"""

import rclpy
from amr_msgs.msg import SafetyStatus, ZoneGrant
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Path

from amr_zone_client.zone_client_node import ZoneClientNode


def test_tf_lookup_failure_sets_speed_cap_to_zero():
    rclpy.init()
    try:
        node = ZoneClientNode()
        try:
            # 아무 TF도 흘려주지 않았으니 map->base_footprint 조회는 반드시 실패한다
            node._update()
            assert node._speed_cap == 0.0
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
            node._lookup_robot_xy = lambda: (0.0, 0.0)

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

