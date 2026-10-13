"""zone_client_node.py의 TF 조회 실패 시 동작(fail-closed) 확인 (#66 하연 리뷰).

launch_testing 없이 노드를 직접 만들어서 확인하는 가벼운 rclpy 단위 테스트다 - 실제
fleet_manager·Gazebo까지는 필요 없다. test_zone_client_launch.py(launch_testing)와는
별도의, 더 가벼운 경로다.
"""

import rclpy
from amr_msgs.msg import SafetyStatus, ZoneGrant
from geometry_msgs.msg import Twist
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
