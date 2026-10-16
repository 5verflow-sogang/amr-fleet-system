#!/usr/bin/env python3
"""로봇 주변 추적 트랙(perception/obstacles)을 거리순으로 보여 준다. dev 컨테이너 안에서 돈다.

DWA가 "유효한 궤적을 못 찾아" 멈췄을 때, 어떤 트랙이 로봇 차체 가까이 있는지(정적/동적, 반경, 속도)를 본다.
DWA는 차체 표면까지 거리 - 반경 <= dynamic_obstacle_safety_margin(0.2 m)이면 그 궤적을 버린다.

    tools/dev/nav_check.sh --obstacles            # 호스트에서 (amr_01)
    tools/dev/nav_check.sh --obstacles amr_02
"""

import math
import sys
import time

ROBOT = sys.argv[1] if len(sys.argv) > 1 else 'amr_01'
HALF_LENGTH, HALF_WIDTH, MARGIN = 0.3, 0.2, 0.2  # nav2_params.yaml DWA와 같다


def surface_distance(rx, ry, ryaw, px, py):
    dx, dy = px - rx, py - ry
    lx = dx * math.cos(ryaw) + dy * math.sin(ryaw)
    ly = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    return math.hypot(max(0.0, abs(lx) - HALF_LENGTH), max(0.0, abs(ly) - HALF_WIDTH))


def main():
    import rclpy
    from amr_msgs.msg import TrackedObstacleArray
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('obstacles_near')
    buffer = Buffer()
    TransformListener(buffer, node)
    got = []
    node.create_subscription(TrackedObstacleArray, f'/{ROBOT}/perception/obstacles', got.append, 10)
    deadline = time.monotonic() + 5.0
    while not got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print(f'/{ROBOT}/perception/obstacles 메시지가 5초 동안 없다 (추적 노드가 꺼졌거나 3단계가 아님)')
        return 1
    msg = got[-1]
    frame = msg.header.frame_id or f'{ROBOT}/odom'
    pose = None
    while pose is None and time.monotonic() < deadline + 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            pose = buffer.lookup_transform(frame, f'{ROBOT}/base_footprint', Time()).transform
        except Exception:  # noqa: BLE001
            pass
    if pose is None:
        print(f'TF {frame} → {ROBOT}/base_footprint 없음')
        return 1
    q = pose.rotation
    ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    rx, ry = pose.translation.x, pose.translation.y

    rows = []
    for o in msg.obstacles:
        px, py = o.pose.pose.position.x, o.pose.pose.position.y
        gap = surface_distance(rx, ry, ryaw, px, py) - o.radius
        speed = math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y)
        rows.append((gap, o.track_id, o.is_dynamic, o.radius, speed))
    rows.sort()
    dynamic = sum(r[2] for r in rows)
    print(f'{ROBOT} 트랙 {len(rows)}개 (동적 {dynamic}개), 좌표계 {frame}')
    print(f'{"track":>6} {"동적":>4} {"반경":>6} {"속력":>7} {"차체까지":>8}  DWA')
    for gap, tid, dyn, radius, speed in rows[:10]:
        blocks = 'X 막음' if dyn and gap <= MARGIN else ('(정적: 무시)' if not dyn and gap <= MARGIN else '')
        print(f'{tid:>6} {"예" if dyn else "-":>4} {radius:>5.2f}m {speed:>5.2f}m/s {gap:>7.2f}m  {blocks}')
    trapped = [r for r in rows if r[2] and r[0] <= MARGIN]
    print()
    if trapped:
        print(f'동적 트랙 {len(trapped)}개가 차체 {MARGIN} m 안 → DWA는 모든 궤적을 버린다 (빠져나오는 궤적 포함)')
    else:
        print(f'차체 {MARGIN} m 안의 동적 트랙 없음 → 다른 원인 (costmap 등)')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
