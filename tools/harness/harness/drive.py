"""시나리오 주행. runner가 별도 프로세스로 부른다 (rclpy init·shutdown을 실행마다 깨끗하게).

    python3 drive.py '{"mode": "segments", "topic": "/amr_01/cmd_vel", "segments": [[0.5, 0.0, 6.0]]}'
    python3 drive.py '{"mode": "wait", "duration_s": 600, "use_sim_time": false}'
    python3 drive.py '{"mode": "command", "cmd": "python3 measure.py --out /tmp/x.json"}'

segments: [선속도 m/s, 각속도 rad/s, 시간 s]를 차례로 보낸다. 구간 사이에 정지 명령을 보낸다.
          then_wait_s가 있으면 끝나고 아무것도 보내지 않으며 그만큼 더 기다린다 (복구 노드가 cmd_vel을 쓰는 시나리오).
command: 셸 명령 하나를 끝날 때까지 돌린다(예: Nav2 목표를 보내고 결과를 재는 스크립트). 그동안 시계를
돌려 rtf를 잰다. 명령이 0이 아닌 코드로 끝나거나 timeout_s(실제 시간)를 넘기면 주행 실패로 본다.
명령은 자기 프로세스 그룹으로 띄워 끝·시간 초과 때 그룹째 정리한다(proc.py) - runner는 시간 초과 때 이
프로세스를 SIGKILL로 죽여 손자(측정 스크립트)를 못 치우므로, timeout_s는 시나리오 timeout_s보다 짧게 둔다.
마지막 줄에 {"sim_s", "wall_s", "rtf"} JSON을 출력한다. sim_s는 대기 시계(use_sim_time)와 상관없이 /clock으로 잰다.
/clock이 없으면(가짜 로봇만 띄우는 시나리오) sim_s·rtf는 null이다.
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from proc import TIMEOUT, run_group  # 같은 폴더 (runner가 이 파일을 스크립트로 부른다)


def main(argv):
    spec = json.loads(argv[1])
    use_sim_time = bool(spec.get('use_sim_time', True))
    rclpy.init()
    node = Node('harness_drive', parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    # RTF는 /clock으로 잰다. 실제 시간으로 기다리는 시나리오(use_sim_time: false)에서 노드 시계로 재면
    # sim_s = wall_s가 되어 Gazebo가 느려도 늘 1.0이었다 (fleet_stack, 11/2). 주행 구간에서 처음·마지막으로
    # 받은 (sim 시각, 실제 시각)을 둔다
    sim_clock = {}

    def on_clock(m):
        sim_clock.setdefault('first', (m.clock.sec + m.clock.nanosec * 1e-9, time.monotonic()))
        sim_clock['last'] = (m.clock.sec + m.clock.nanosec * 1e-9, time.monotonic())

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)

    # 시뮬레이션 시계(/clock)가 들어올 때까지 기다린다
    t_wait = time.monotonic()
    while now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t_wait > 60:
            print('시계(/clock)가 오지 않음', file=sys.stderr)
            return 2

    def spin_for(duration, publish=None):
        t0 = now()
        while now() - t0 < duration:
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=0.05)

    wall0 = time.monotonic()
    sim_clock.clear()
    mode = spec.get('mode', 'segments')
    if mode == 'segments':
        pub = node.create_publisher(Twist, spec['topic'], 10)
        for v, w, duration in spec['segments']:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            spin_for(float(duration), lambda: pub.publish(cmd))
            for _ in range(5):
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
            print(f'구간 끝: v={v} w={w} {duration}s', file=sys.stderr, flush=True)
        spin_for(float(spec.get('then_wait_s', 0.0)))
    elif mode == 'wait':
        spin_for(float(spec['duration_s']))
    elif mode == 'command':
        # 명령의 출력은 이 프로세스의 stderr(drive.log)로 보낸다. stdout은 마지막 JSON 한 줄만 쓴다
        timeout_s = spec.get('timeout_s')
        code = run_group(spec['cmd'], None if timeout_s is None else float(timeout_s),
                         poll=lambda: rclpy.spin_once(node, timeout_sec=0.1))
        if code == TIMEOUT:
            print(f'명령 시간 초과 ({timeout_s}s), 프로세스 그룹 정리: {spec["cmd"]}', file=sys.stderr)
            return 1
        if code != 0:
            print(f'명령 실패 (종료 코드 {code}): {spec["cmd"]}', file=sys.stderr)
            return 1
    else:
        print(f'알 수 없는 mode: {mode}', file=sys.stderr)
        return 2

    wall_s = time.monotonic() - wall0
    sim_s = sim_clock['last'][0] - sim_clock['first'][0] if 'first' in sim_clock else None
    print(json.dumps({'sim_s': None if sim_s is None else round(sim_s, 3), 'wall_s': round(wall_s, 3),
                      'rtf': rtf(sim_clock)}), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def rtf(sim_clock: dict):
    """처음·마지막 /clock 사이의 sim 시간 ÷ 실제 시간. /clock이 없거나 한 번뿐이면 None."""
    if 'first' not in sim_clock:
        return None
    (s0, w0), (s1, w1) = sim_clock['first'], sim_clock['last']
    return round((s1 - s0) / (w1 - w0), 3) if w1 > w0 else None


if __name__ == '__main__':
    sys.exit(main(sys.argv))
