#!/usr/bin/env python3
"""스택을 띄우고, Nav2가 준비되면 목표를 보내고, 결과를 표로 정리한다. dev 컨테이너 안에서 돈다.

호스트에서는 nav_check.sh로 부른다 (컨테이너 찾기·빌드·로그 복사를 거기서 한다).
같은 로봇의 목표는 순서대로, 다른 로봇의 목표는 동시에 보낸다.

    python3 nav_check.py --robots 2 --stage 3 --goal amr_01:6,0 --out /tmp/nav_check/x
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Optional

# 로그에서 셀 것: (이름, 정규식). 로봇은 줄의 [amr_NN.<노드>]에서 뽑는다
LOG_PATTERNS = [
    ('dwa_no_traj', re.compile(r'유효한 궤적을 못 찾아')),
    ('no_progress', re.compile(r'Failed to make progress')),
    ('goal_failed', re.compile(r'Goal failed')),
    ('plan_failed', re.compile(r'Failed to create plan|failed to create plan|No valid path', re.I)),
    ('died', re.compile(r'process has died')),
]
# 이 줄은 화면에도 바로 보여 준다 (DWA 경고는 너무 많아 개수만 센다)
SHOW_LIVE = {'no_progress', 'goal_failed', 'plan_failed', 'died'}
ROBOT_RE = re.compile(r'\[(amr_\d+)[./]')

STAGES = {
    1: {},
    2: {'nav2': 'true', 'zone_client': 'true'},
    3: {'nav2': 'true', 'zone_client': 'true', 'tracking': 'true', 'safety': 'true'},
}

COLOR = {'red': '31', 'green': '32', 'yellow': '33', 'blue': '34', 'dim': '2', 'bold': '1'}
USE_COLOR = True


def paint(text: str, color: str) -> str:
    return f'\033[{COLOR[color]}m{text}\033[0m' if USE_COLOR else text


def say(text: str, color: Optional[str] = None) -> None:
    stamp = time.strftime('%H:%M:%S')
    print(f'{paint(stamp, "dim")} {paint(text, color) if color else text}', flush=True)


@dataclass
class Goal:
    robot: str
    x: float
    y: float
    yaw: float = 0.0
    # 결과
    result: str = '대기'
    elapsed_s: float = 0.0
    remaining_m: Optional[float] = None
    end_pose: Optional[tuple] = None
    recoveries: int = 0
    counts: Counter = field(default_factory=Counter)

    def label(self) -> str:
        return f'{self.robot} → ({self.x:g}, {self.y:g})'


def parse_goal(text: str, default_robot: str = 'amr_01') -> Goal:
    """'amr_01:6,0', 'amr_02:6,0,1.57', '6,0' (로봇 생략 시 amr_01). 좌표는 map 기준."""
    robot, _, coords = text.rpartition(':')
    values = [float(v) for v in coords.split(',')]
    if len(values) not in (2, 3):
        raise ValueError(f'목표는 x,y 또는 x,y,yaw: {text}')
    robot = robot or default_robot
    if not re.fullmatch(r'amr_\d{2}', robot):
        raise ValueError(f'로봇 이름은 amr_NN: {text}')
    return Goal(robot, *values)


def classify(line: str) -> list:
    """로그 한 줄에서 (로봇 또는 None, 이름) 목록."""
    found = []
    for name, pattern in LOG_PATTERNS:
        if pattern.search(line):
            match = ROBOT_RE.search(line)
            found.append((match.group(1) if match else None, name))
    return found


def launch_command(robots: int, stage: int, extra: list) -> list:
    args = {'num_robots': str(robots), **STAGES[stage]}
    for item in extra:
        key, sep, value = item.partition(':=')
        if not sep:
            raise ValueError(f'launch 인자는 이름:=값: {item}')
        args[key] = value
    return ['ros2', 'launch', 'amr_bringup', 'system.launch.py'] + [f'{k}:={v}' for k, v in args.items()]


class LogWatcher(threading.Thread):
    """launch 로그 파일을 따라 읽으며 패턴을 센다."""

    def __init__(self, path: str):
        super().__init__(daemon=True)
        self.path = path
        self.counts = Counter()  # (robot, name)
        self.lock = threading.Lock()
        self.stop = threading.Event()

    def run(self):
        while not os.path.exists(self.path) and not self.stop.is_set():
            time.sleep(0.2)
        with open(self.path, encoding='utf-8', errors='replace') as f:
            while not self.stop.is_set():
                line = f.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                for robot, name in classify(line):
                    with self.lock:
                        self.counts[(robot, name)] += 1
                    if name in SHOW_LIVE:
                        say(f'  로그: {line.strip()[:200]}', 'red' if name == 'died' else 'yellow')

    def snapshot(self) -> Counter:
        with self.lock:
            return Counter(self.counts)


def stop_stack(proc: Optional[subprocess.Popen]) -> None:
    """launch를 SIGINT로 끄고, 안 꺼지면 죽인다. Gazebo 잔여 프로세스도 지운다."""
    if proc and proc.poll() is None:
        say('스택 종료 중 (최대 30초)')
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=30)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    kill_leftovers()


def kill_leftovers() -> None:
    # bridge·clock_throttle은 kill -9로 끝난 실행 뒤 고아로 남는다 (#243)
    for pattern in ('ros2 launch amr_bringup system.launch.py', 'ign gazebo', 'gz sim', 'ruby.*ign',
                    'parameter_bridge', 'clock_throttle'):
        subprocess.run(['pkill', '-9', '-f', pattern], check=False)


def main() -> int:
    global USE_COLOR
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-r', '--robots', type=int, default=1)
    p.add_argument('--stage', type=int, choices=sorted(STAGES), default=2,
                   help='2: Nav2+구역 클라이언트, 3: +추적·안전')
    p.add_argument('-g', '--goal', action='append', default=[], help='amr_01:x,y[,yaw] (map). 여러 번 줄 수 있다')
    p.add_argument('--set', action='append', default=[], help='launch 인자 추가 (예: dynamic_obstacles:=true)')
    p.add_argument('--ready-timeout', type=float, default=300.0, help='Nav2 활성 대기 (실제 시간 s)')
    p.add_argument('--goal-timeout', type=float, default=300.0, help='목표 하나 제한 (실제 시간 s)')
    p.add_argument('--attach', action='store_true', help='이미 떠 있는 스택에 목표만 보낸다')
    p.add_argument('--keep', action='store_true', help='끝나도 스택을 끄지 않는다 (RViz로 볼 때)')
    p.add_argument('--out', default=f'/tmp/nav_check/{time.strftime("%Y%m%d-%H%M%S")}')
    p.add_argument('--no-color', action='store_true')
    opt = p.parse_args()
    USE_COLOR = not opt.no_color

    goals = [parse_goal(g) for g in (opt.goal or ['amr_01:20,-14'])]
    for g in goals:
        if int(g.robot[4:]) > opt.robots:
            p.error(f'{g.robot}에 목표를 줬는데 --robots {opt.robots}')
    os.makedirs(opt.out, exist_ok=True)
    with open(os.path.join(opt.out, 'pid'), 'w') as f:
        f.write(str(os.getpid()))

    import rclpy
    from action_msgs.msg import GoalStatus
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    # Ctrl+C는 아래 핸들러가 받아 스택을 정리하고 끝낸다 (rclpy가 먼저 컨텍스트를 닫지 않게)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('nav_check')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    proc = None
    watcher = None
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())

    def pose_of(robot):
        try:
            t = tf_buffer.lookup_transform('map', f'{robot}/base_footprint', Time()).transform
        except Exception:  # noqa: BLE001 - TF가 아직 없으면 위치 모름
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.translation.x, t.translation.y, yaw)

    try:
        if not opt.attach:
            time.sleep(2.0)  # 발견 대기
            others = [n for n, ns in node.get_node_names_and_namespaces()
                      if n != 'nav_check' and 'rviz' not in n and 'transform_listener' not in n]
            if others:
                say(f'이미 떠 있는 ROS 노드 {len(others)}개 (예: {", ".join(sorted(others)[:5])}). '
                    '이전 실행 잔여물을 지운다. compose fleet 서비스가 떠 있으면 Fleet 서버가 둘이 된다', 'yellow')
            kill_leftovers()
            time.sleep(1.0)
            cmd = launch_command(opt.robots, opt.stage, opt.set)
            log_path = os.path.join(opt.out, 'launch.log')
            say('실행: ' + ' '.join(cmd), 'blue')
            say(f'launch 로그: {log_path}', 'dim')
            env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT='0')
            with open(log_path, 'w') as log:
                proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
            watcher = LogWatcher(log_path)
            watcher.start()

        # Nav2 활성 대기
        robots = [f'amr_{i:02d}' for i in range(1, opt.robots + 1)]
        say(f'Nav2 활성 대기: {", ".join(robots)} (최대 {opt.ready_timeout:.0f}초)')
        clients = {r: node.create_client(GetState, f'/{r}/bt_navigator/get_state') for r in robots}
        start = time.monotonic()
        pending = set(robots)
        last_note = 0.0
        while pending:
            if interrupted.is_set():
                raise KeyboardInterrupt
            if proc and proc.poll() is not None:
                say(f'launch가 먼저 끝났다 (exit {proc.returncode}). 로그를 본다', 'red')
                return 2
            elapsed = time.monotonic() - start
            if elapsed > opt.ready_timeout:
                say(f'Nav2 활성 시간 초과: {", ".join(sorted(pending))}', 'red')
                return 2
            for r in sorted(pending):
                if clients[r].service_is_ready():
                    future = clients[r].call_async(GetState.Request())
                    deadline = time.monotonic() + 3.0
                    while not future.done() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    if future.done() and future.result() and future.result().current_state.id == 3:
                        say(f'  {r} Nav2 활성 ({elapsed:.0f}초)', 'green')
                        pending.discard(r)
            if elapsed - last_note >= 30:
                last_note = elapsed
                if elapsed > 0:
                    say(f'  ... {elapsed:.0f}초, 남은 로봇: {", ".join(sorted(pending))}', 'dim')
            time.sleep(1.0)

        # 목표: 로봇별 큐
        nav = {r: ActionClient(node, NavigateToPose, f'/{r}/navigate_to_pose') for r in robots}
        queues = {r: [g for g in goals if g.robot == r] for r in robots}
        active = {}  # robot → dict(goal, handle_future, result_future, handle, start, feedback)
        before = watcher.snapshot() if watcher else Counter()
        last_report = time.monotonic()

        def send(goal):
            msg = NavigateToPose.Goal()
            msg.pose.header.frame_id = 'map'
            msg.pose.pose.position.x = goal.x
            msg.pose.pose.position.y = goal.y
            msg.pose.pose.orientation.z = math.sin(goal.yaw / 2)
            msg.pose.pose.orientation.w = math.cos(goal.yaw / 2)
            state = {'goal': goal, 'start': time.monotonic(), 'feedback': None, 'handle': None, 'result': None,
                     'counts_before': watcher.snapshot() if watcher else Counter()}

            def on_feedback(fb):
                state['feedback'] = fb.feedback
            if not nav[goal.robot].wait_for_server(timeout_sec=10.0):
                goal.result = '서버 없음'
                return None
            state['handle_future'] = nav[goal.robot].send_goal_async(msg, feedback_callback=on_feedback)
            say(f'목표 전송: {goal.label()}', 'blue')
            return state

        while any(queues.values()) or active:
            if interrupted.is_set():
                raise KeyboardInterrupt
            for r in robots:
                if r not in active and queues[r]:
                    state = send(queues[r].pop(0))
                    if state:
                        active[r] = state
            for r, state in list(active.items()):
                goal = state['goal']
                now = time.monotonic()
                if state['handle'] is None and state['handle_future'].done():
                    state['handle'] = state['handle_future'].result()
                    if not state['handle'].accepted:
                        goal.result = '거부'
                        del active[r]
                        continue
                    state['result'] = state['handle'].get_result_async()
                done = state['result'] is not None and state['result'].done()
                timed_out = now - state['start'] > opt.goal_timeout
                if not (done or timed_out):
                    continue
                if done:
                    status = state['result'].result().status
                    goal.result = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                                   GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, f'status {status}')
                else:
                    goal.result = '시간 초과'
                    if state['handle']:
                        state['handle'].cancel_goal_async()
                goal.elapsed_s = now - state['start']
                goal.end_pose = pose_of(r)
                if goal.end_pose:
                    goal.remaining_m = math.hypot(goal.x - goal.end_pose[0], goal.y - goal.end_pose[1])
                if state['feedback'] is not None:
                    goal.recoveries = state['feedback'].number_of_recoveries
                if watcher:
                    diff = watcher.snapshot() - state['counts_before']
                    goal.counts = Counter({name: n for (robot, name), n in diff.items() if robot in (r, None)})
                say(f'결과: {goal.label()} {goal.result} ({goal.elapsed_s:.0f}초)',
                    'green' if goal.result == 'SUCCEEDED' else 'red')
                del active[r]
            if time.monotonic() - last_report >= 15 and active:
                last_report = time.monotonic()
                for r, state in active.items():
                    fb = state['feedback']
                    pose = pose_of(r)
                    where = f'({pose[0]:.1f}, {pose[1]:.1f})' if pose else '(위치 모름)'
                    rest = f'남은 거리 {fb.distance_remaining:.1f} m, 복구 {fb.number_of_recoveries}회' if fb else '피드백 없음'
                    dwa = (watcher.snapshot() - state['counts_before'])[(r, 'dwa_no_traj')] if watcher else 0
                    say(f'  {r} {time.monotonic() - state["start"]:.0f}초 {where} {rest}, DWA 궤적 없음 {dwa}회', 'dim')
            time.sleep(0.2)

        report = summary(goals, watcher.snapshot() - before if watcher else None, opt)
        print(report, flush=True)
        with open(os.path.join(opt.out, 'summary.txt'), 'w') as f:
            f.write(re.sub(r'\033\[[0-9;]*m', '', report) + '\n')
        return 0 if all(g.result == 'SUCCEEDED' for g in goals) else 1
    except KeyboardInterrupt:
        say('중단 요청', 'yellow')
        return 130
    finally:
        if watcher:
            watcher.stop.set()
        if opt.keep and proc and proc.poll() is None:
            say('--keep: 스택을 그대로 둔다. 끌 때: tools/dev/nav_check.sh --stop', 'yellow')
        elif not opt.attach:
            stop_stack(proc)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def summary(goals, counts, opt) -> str:
    lines = ['', paint('== 결과 ==', 'bold')]
    lines.append(f'{"목표":<26} {"결과":<10} {"시간":>6} {"남은 거리":>9} {"끝 위치 (map)":<22} {"복구":>4} '
                 f'{"DWA 궤적없음":>12} {"진행 실패":>8}')
    for g in goals:
        end = f'({g.end_pose[0]:.2f}, {g.end_pose[1]:.2f}, {math.degrees(g.end_pose[2]):.0f}°)' if g.end_pose else '-'
        rest = f'{g.remaining_m:.2f} m' if g.remaining_m is not None else '-'
        color = 'green' if g.result == 'SUCCEEDED' else 'red'
        lines.append(f'{g.label():<26} {paint(f"{g.result:<10}", color)} {g.elapsed_s:>5.0f}s {rest:>9} {end:<22} '
                     f'{g.recoveries:>4} {g.counts["dwa_no_traj"]:>12} {g.counts["no_progress"]:>8}')
    if counts is not None:
        died = sum(n for (_, name), n in counts.items() if name == 'died')
        lines.append('')
        lines.append(f'프로세스 죽음 {died}건' + ('' if died == 0 else ' ← launch.log에서 "process has died" 검색'))
    lines.append(f'로그: {opt.out}')
    return '\n'.join(lines)


if __name__ == '__main__':
    sys.exit(main())
