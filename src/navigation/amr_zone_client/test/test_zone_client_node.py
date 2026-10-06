"""zone_client_node.py의 TF 조회 실패 시 동작(fail-closed) 확인 (#66 하연 리뷰).

launch_testing 없이 노드를 직접 만들어서 확인하는 가벼운 rclpy 단위 테스트다 - 실제
fleet_manager·Gazebo까지는 필요 없다. test_zone_client_launch.py(launch_testing)와는
별도의, 더 가벼운 경로다.
"""

import rclpy

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
