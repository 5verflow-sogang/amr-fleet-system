"""배송 중 로봇을 순간 이동(Kidnapped)시켜 BT가 위치 복구와 맞물려 도는지 본다 (4주차 작업 5).

near(같은 통로 1–2.5 m): 복구 노드가 tracking을 벗어나면 BT가 Nav2 목표를 취소하고 멈춤 → 복구되어 tracking이면
같은 목표를 다시 보내 재개 → COMPLETED.
far(다른 통로): 복구 노드가 시도를 다 하고 failed → BT가 바로 FAILED(LOCALIZATION_LOST) → RobotState MODE_ERROR.
--restore면 GT로 initialpose를 줘 위치를 다시 잡고 MODE_IDLE로 돌아오는지 본다.

    python3 run_kidnap.py --mode near --seed 1 --pairs rack_a5:rack_a6

로봇 1대, system.launch.py nav2:=true docking:=true enable_camera:=true에 kidnap_recovery 노드가 떠 있어야 한다.
순간 이동 목표는 amr_localization kidnap_trial과 같은 pick_target(mode, seed)이다.
"""

import argparse
import csv
import math
import os
import subprocess
import time

from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from amr_localization.eval.trajectory import Pose2D
from amr_localization.kidnap.trial import AISLE_AB_Y, pick_target
from amr_msgs.msg import RobotState, Task, TaskStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml

STATES = {0: 'PENDING', 1: 'IN_PROGRESS', 2: 'COMPLETED', 3: 'FAILED'}
MODES = {0: 'IDLE', 1: 'BUSY', 2: 'CHARGING', 3: 'ERROR', 4: 'ESTOP'}
GOAL = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['near', 'far'], default='near')
    ap.add_argument('--seeds', nargs='+', type=int, default=[1])
    ap.add_argument('--pairs', nargs='+', default=['rack_a5:rack_a6'], help='픽업:드롭오프. 시드마다 차례로')
    ap.add_argument('--trigger-m', type=float, default=2.0, help='드롭오프로 이만큼 간 뒤 순간 이동 [m]')
    ap.add_argument('--restore', action='store_true', help='far에서 FAILED 뒤 GT로 initialpose를 줘 되돌린다')
    ap.add_argument('--origin', nargs=3, type=float, default=None,
                    help='map_origin_in_world. 안 주면 TF world → map (system.launch.py 정렬 원점)')
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--world', default='warehouse')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    stations = yaml.safe_load(open(os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'config', 'stations.yaml'), encoding='utf-8'))['stations']
    def to_map(x, y, yaw):
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        return c * dx - s * dy, s * dx + c * dy, yaw - oyaw

    def pose_of(name):
        x, y, yaw = to_map(*stations[name])
        p = Task().pickup_pose
        p.position.x, p.position.y = x, y
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    rclpy.init()
    node = rclpy.create_node('bt_kidnap')
    # 위치를 다시 잡을 때 initialpose가 GT와 몇 cm라도 어긋나면 복구 노드가 tracking으로 못 돌아온다 (10/16 far 2회).
    # 그래서 기본값 (0, 1.5) 대신 실제 정렬 원점을 TF에서 읽는다
    if a.origin is None:
        tf = Buffer()
        TransformListener(tf, node)
        t0 = time.monotonic()
        while not tf.can_transform('world', 'map', rclpy.time.Time()) and time.monotonic() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        o = tf.lookup_transform('world', 'map', rclpy.time.Time()).transform
        a.origin = [o.translation.x, o.translation.y, yaw_of(o.rotation)]
    ox, oy, oyaw = a.origin
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Task, '/fleet/task_requests', qos)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, f'/{a.robot}/initialpose', 10)
    s = {'id': None, 'status': None, 'kidnap': 'tracking', 'mode': None, 'gt': None, 'goals': {}, 't0': None, 'moved': 0.0,
         'log': []}

    def stamp():
        return f'{time.monotonic() - s["t0"]:+7.1f} s' if s['t0'] else '        '

    def note(what):
        line = f'  {time.strftime("%H:%M:%S")} {stamp()} {what}'
        print(line, flush=True)
        s['log'].append((time.monotonic(), what))

    def on_status(m):
        if m.task_id != s['id']:
            return
        prev = s['status']
        s['status'] = m
        if prev is None or prev.state != m.state or prev.phase != m.phase:
            note(f'작업 {STATES.get(m.state, m.state)} {m.phase} (code {m.error.code} {m.error.message})')

    def on_kidnap(m):
        if m.data != s['kidnap']:
            note(f'위치 복구 {s["kidnap"]} → {m.data}')
            s['kidnap'] = m.data

    def on_state(m):
        if m.mode != s['mode']:
            note(f'RobotState {MODES.get(s["mode"], s["mode"])} → {MODES.get(m.mode, m.mode)}')
            s['mode'] = m.mode

    def on_goals(m):
        for g in m.status_list:
            gid = bytes(g.goal_info.goal_id.uuid).hex()[:6]
            if s['goals'].get(gid) != g.status:
                s['goals'][gid] = g.status
                if s['t0'] is not None:
                    note(f'Nav2 목표 {gid} {GOAL.get(g.status, g.status)}')

    node.create_subscription(TaskStatus, '/fleet/task_status', on_status, qos)
    node.create_subscription(String, f'/{a.robot}/kidnap_recovery/state', on_kidnap, 10)
    node.create_subscription(RobotState, '/fleet/robot_states', lambda m: m.robot_id == a.robot and on_state(m), 10)
    node.create_subscription(GoalStatusArray, f'/{a.robot}/navigate_to_pose/_action/status', on_goals, 10)
    def on_gt(m):
        # 드롭오프 이동 중 GT 경로 길이 (TaskStatus는 단계가 바뀔 때만 와서 distance_traveled가 이동 중엔 안 는다)
        p = m.pose.pose
        st = s['status']
        if st is not None and st.state == 1 and st.phase == 'move_to_dropoff' and s['gt'] is not None:
            s['moved'] += math.hypot(p.position.x - s['gt'].position.x, p.position.y - s['gt'].position.y)
        s['gt'] = p

    node.create_subscription(Odometry, f'/{a.robot}/ground_truth/odom', on_gt, 10)
    end = time.monotonic() + 30
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    def spin_until(cond, limit):
        t = time.monotonic()
        while time.monotonic() - t < limit:
            rclpy.spin_once(node, timeout_sec=0.1)
            if cond():
                return True
        return False

    def gt_pose():
        p = s['gt']
        return Pose2D(0.0, p.position.x, p.position.y, yaw_of(p.orientation))

    def teleport(t):
        req = (f'name: "{a.robot}", position: {{x: {t.x:.4f}, y: {t.y:.4f}, z: 0.02}}, '
               f'orientation: {{z: {math.sin(t.yaw / 2):.6f}, w: {math.cos(t.yaw / 2):.6f}}}')
        for _ in range(3):
            out = subprocess.run(['ign', 'service', '-s', f'/world/{a.world}/set_pose', '--reqtype',
                                  'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '10000',
                                  '--req', req], capture_output=True, text=True, timeout=20)
            if 'true' in out.stdout:
                return True
        return False

    def restore():
        g = gt_pose()
        x, y, yaw = to_map(g.x, g.y, g.yaw)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        # 시각 0 = 최신 TF. 이 노드는 실제 시계라 지금 시각을 넣으면 sim 시계보다 미래라 AMCL이 TF 변환을 못 해 버린다
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = 0.05 ** 2
        for _ in range(3):   # 한 번은 가끔 안 닿는다
            init_pub.publish(m)
            spin_until(lambda: False, 0.5)
        note(f'initialpose를 GT로 (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')

    run_tag = time.strftime('%m%d%H%M')
    for n, seed in enumerate(a.seeds, 1):
        pickup, dropoff = a.pairs[(n - 1) % len(a.pairs)].split(':')
        task_id = f'kidnap_{a.mode}_{run_tag}_{n:02d}'
        s.update(id=task_id, status=None, t0=None, log=[], moved=0.0)
        t = Task()
        t.task_id, t.task_type, t.item_type = task_id, Task.TYPE_DELIVERY, Task.ITEM_SMALL
        t.pickup_station_id, t.pickup_pose = pickup, pose_of(pickup)
        t.dropoff_station_id, t.dropoff_pose = dropoff, pose_of(dropoff)
        print(f'#{n} {task_id}: {pickup} → {dropoff}, {a.mode} seed {seed}', flush=True)
        start = time.monotonic()
        pub.publish(t)

        # 드롭오프로 trigger_m 넘게 갔고 A–B 통로 안이면 순간 이동
        def ready():
            m = s['status']
            if not m or m.state != 1 or m.phase != 'move_to_dropoff' or s['gt'] is None:
                return False
            # A–B 통로(랙 면 y 4.4 ~ −1.4) 안. 옮길 자리는 pick_target이 통로 가운데 ±1 m로 고른다
            return s['moved'] >= a.trigger_m and abs(s['gt'].position.y - AISLE_AB_Y) < 2.5

        def ready_or_done():
            m = s['status']
            return ready() or (m is not None and m.state in (2, 3))

        row = {'n': n, 'task_id': task_id, 'mode': a.mode, 'seed': seed, 'pickup': pickup, 'dropoff': dropoff}
        if not spin_until(ready_or_done, a.timeout) or not ready():
            note('순간 이동 조건을 못 만남')
            row.update(result='NO_TRIGGER')
        else:
            g = gt_pose()
            target = pick_target(a.mode, seed, g)
            s['t0'] = time.monotonic()
            ok = teleport(target)
            note(f'순간 이동 {"성공" if ok else "실패"}: world ({g.x:.2f}, {g.y:.2f}, {math.degrees(g.yaw):.0f}°) → '
                 f'({target.x:.2f}, {target.y:.2f}, {math.degrees(target.yaw):.0f}°), '
                 f'{math.hypot(target.x - g.x, target.y - g.y):.2f} m')
            spin_until(lambda: s['status'] is not None and s['status'].state in (2, 3), a.timeout)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
            last = s['status']
            log = s['log']

            def first(pred):
                for tm, w in log:
                    if pred(w):
                        return round(tm - s['t0'], 1)
                return ''

            x, y, yaw = stations[dropoff]
            p = s['gt']
            row.update(
                jump_m=round(math.hypot(target.x - g.x, target.y - g.y), 2),
                result=STATES.get(last.state, '') if last else 'NO_STATUS',
                error_code=last.error.code if last else '',
                error_message=(last.error.message if last else '').replace(',', ';'),
                lost_s=first(lambda w: w.startswith('위치 복구 tracking →')),
                nav_cancel_s=first(lambda w: w.startswith('Nav2') and 'CANCELED' in w),
                tracking_s=first(lambda w: w.endswith('→ tracking')),
                failed_s=first(lambda w: w.endswith('→ failed')),
                resend_s=first(lambda w: w.startswith('Nav2') and 'EXECUTING' in w and 'CANCELED' not in w),
                end_s=round(time.monotonic() - s['t0'], 1),
                end_pos_err_cm=round(math.hypot(p.position.x - x, p.position.y - y) * 100, 2)
                if last and last.state == 2 else '',
            )
            # resend_s는 취소 뒤 처음 EXECUTING인 목표
            cancel_t = row['nav_cancel_s']
            if cancel_t != '':
                later = [round(tm - s['t0'], 1) for tm, w in log
                         if w.startswith('Nav2') and 'EXECUTING' in w and tm - s['t0'] > cancel_t]
                row['resend_s'] = later[0] if later else ''
            if a.restore and last and last.state == 3:
                spin_until(lambda: s['mode'] == RobotState.MODE_ERROR, 10)
                row['mode_after_fail'] = MODES.get(s['mode'], s['mode'])
                restore()
                back = spin_until(lambda: s['mode'] == RobotState.MODE_IDLE, 120)
                row['restored_idle_s'] = round(time.monotonic() - s['t0'], 1) if back else ''
            print(f'  → {row}', flush=True)
        row['wall_s'] = round(time.monotonic() - start, 1)
        if a.csv:
            new = not os.path.exists(a.csv)
            with open(a.csv, 'a', newline='', encoding='utf-8') as f:
                fields = ['n', 'task_id', 'mode', 'seed', 'pickup', 'dropoff', 'jump_m', 'result', 'error_code',
                          'error_message', 'lost_s', 'nav_cancel_s', 'tracking_s', 'failed_s', 'resend_s', 'end_s',
                          'end_pos_err_cm', 'mode_after_fail', 'restored_idle_s', 'wall_s']
                w = csv.DictWriter(f, fieldnames=fields)
                if new:
                    w.writeheader()
                w.writerow({k: row.get(k, '') for k in fields})
        s['t0'] = None
        # 다음 작업 전에 IDLE
        spin_until(lambda: s['mode'] == RobotState.MODE_IDLE, 60)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
