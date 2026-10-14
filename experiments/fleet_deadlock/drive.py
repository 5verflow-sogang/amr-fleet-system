#!/usr/bin/env python3
"""교착 유도 주행: 두 로봇을 좁은 통로 ② 양 끝에서 동시에 마주 보내고 결과를 deadlock_result.json에 쓴다.

하네스 fleet_deadlock 시나리오의 주행 명령. system.launch(2대, robots.yaml·zones_split.yaml)가 떠 있는 상태에서:
  1. 두 로봇에 NavigateThroughPoses를 동시에 보낸다. 경유점이 통로 ② 두 구역(narrow_02w·narrow_02e)을 차례로
     지나게 해서, 전역 계획이 통로를 피해 북쪽 넓은 길로 돌아가지 못하게 한다.
       amr_01: 서쪽 → narrow_02w → narrow_02e → 동쪽 끝 너머
       amr_02: 동쪽 → narrow_02e → narrow_02w → 서쪽 끝 너머
     각자 가까운 절반을 먼저 쥐고 반대쪽 절반을 기다리면 순환 대기(교착)다.
  2. 구역 허가 답(fleet/zone_grants)을 로봇마다 기록한다 (교착 해소 REVOKED·REJECTED가 왔는지).
  3. 두 목표의 결과(SUCCEEDED/ABORTED/시간 초과), 걸린 시간(시뮬레이션 시간), 끝 위치(GT, world)를 남긴다.

성공 = 두 로봇 모두 SUCCEEDED (교착이 생겼다면 해소된 것). 실패도 파일에 남기고 종료 코드 0 - 하네스 실행을
실패로 만들지 않는다. 결과 파일을 못 쓴 경우만 0이 아니다.

    python3 drive.py --out <실행 폴더>/deadlock_result.json [--timeout 600]
"""
import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_msgs.msg import ZoneGrant
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

# 경유점 (world x, y, yaw). 통로 중심선 y = -1.1, 두 구역 경계 x = -1.9 (zones_split.yaml)
ROUTES = {
    'amr_01': [(-3.2, -1.1, 0.0), (-0.6, -1.1, 0.0), (3.6, -0.6, 0.0)],
    'amr_02': [(-0.6, -1.1, math.pi), (-3.2, -1.1, math.pi), (-7.4, -0.6, math.pi)],
}
GRANT_NAMES = {ZoneGrant.GRANT_GRANTED: 'GRANTED', ZoneGrant.GRANT_QUEUED: 'QUEUED',
               ZoneGrant.GRANT_REJECTED: 'REJECTED', ZoneGrant.GRANT_REVOKED: 'REVOKED'}
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}


def world_to_map(x, y, yaw, origin):
    ox, oy, oyaw = origin
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    dx, dy = x - ox, y - oy
    return c * dx - s * dy, s * dx + c * dy, yaw - oyaw


def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=600.0, help='두 목표 전체 제한 (실제 시간 s)')
    ap.add_argument('--origin', default='[-0.0562, 1.5317, 0.00009]', help='map_origin_in_world (system.launch 기본)')
    args = ap.parse_args()
    origin = [float(v) for v in args.origin.strip('[]').split(',')]

    rclpy.init()
    node = rclpy.create_node('deadlock_drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
    robots = list(ROUTES)
    grants = {r: [] for r in robots}
    gt = {}
    t0 = None

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_grant(robot):
        def cb(msg):
            name = GRANT_NAMES.get(msg.grant, str(msg.grant))
            log = grants[robot]
            # 같은 답이 연장 요청마다 반복되므로 바뀔 때만 남긴다
            if not log or (log[-1]['zone'], log[-1]['grant']) != (msg.zone_id, name):
                log.append({'t': round(sim_now() - (t0 or sim_now()), 2), 'zone': msg.zone_id, 'grant': name,
                            'holder': msg.holder_robot_id, 'reason': msg.reason})
        return cb

    for r in robots:
        node.create_subscription(ZoneGrant, f'/{r}/fleet/zone_grants', on_grant(r), qos_profile_sensor_data)
        node.create_subscription(Odometry, f'/{r}/ground_truth/odom',
                                 lambda m, r=r: gt.__setitem__(r, m.pose.pose), qos_profile_sensor_data)

    clients = {r: ActionClient(node, NavigateThroughPoses, f'/{r}/navigate_through_poses') for r in robots}
    result = {'robots': {}, 'success': False}
    try:
        for r, c in clients.items():
            if not c.wait_for_server(timeout_sec=60.0):
                raise RuntimeError(f'{r} navigate_through_poses 서버 없음')
        while node.get_clock().now().nanoseconds == 0:  # /clock 대기
            rclpy.spin_once(node, timeout_sec=0.1)

        t0 = sim_now()
        handles = {}
        for r in robots:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [pose(node, *world_to_map(x, y, yaw, origin)) for x, y, yaw in ROUTES[r]]
            handles[r] = clients[r].send_goal_async(goal)
        print(f'두 목표 전송 (sim t0={t0:.1f})', flush=True)

        pending, results, done_at = dict(handles), {}, {}
        wall_end = time.monotonic() + args.timeout
        while pending and time.monotonic() < wall_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            for r, fut in list(pending.items()):
                if not fut.done():
                    continue
                h = fut.result()
                if hasattr(h, 'accepted'):  # 목표 핸들 → 결과 future로 바꿔 계속 본다
                    if not h.accepted:
                        results[r] = 'rejected'
                        del pending[r]
                    else:
                        pending[r] = h.get_result_async()
                    continue
                results[r] = STATUS_NAMES.get(h.status, f'status_{h.status}')
                done_at[r] = sim_now() - t0
                print(f'{r}: {results[r]} ({done_at[r]:.1f} s)', flush=True)
                del pending[r]
        for r in pending:
            results[r] = 'timeout'

        for r in robots:
            p = gt.get(r)
            revoked = [g for g in grants[r] if g['grant'] == 'REVOKED']
            rejected = [g for g in grants[r] if g['grant'] == 'REJECTED']
            result['robots'][r] = {
                'result': results.get(r, 'unknown'),
                'time_s': round(done_at[r], 2) if r in done_at else None,
                'end_world': [round(p.position.x, 3), round(p.position.y, 3)] if p else None,
                'revoked': len(revoked),
                'rejected': len(rejected),
                'grants': grants[r],
            }
        result['success'] = all(v['result'] == 'succeeded' for v in result['robots'].values())
        result['deadlock_seen'] = any(v['revoked'] or v['rejected'] for v in result['robots'].values())
        result['time_s'] = max((v['time_s'] or 0.0) for v in result['robots'].values())
    except Exception as e:  # noqa: BLE001 - 실패도 결과로 남긴다
        result['error'] = str(e)
    finally:
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(json.dumps({k: v for k, v in result.items() if k != 'robots'}, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
