"""시나리오 주행. runner가 별도 프로세스로 부른다 (rclpy init·shutdown을 실행마다 깨끗하게).

    python3 drive.py '{"mode": "segments", "topic": "/amr_01/cmd_vel", "segments": [[0.5, 0.0, 6.0]]}'
    python3 drive.py '{"mode": "wait", "duration_s": 600, "use_sim_time": false}'

segments: [선속도 m/s, 각속도 rad/s, 시간 s]를 차례로 보낸다. 구간 사이에 정지 명령을 보낸다.
마지막 줄에 {"sim_s", "wall_s", "rtf"} JSON을 출력한다.
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter


def main(argv):
    spec = json.loads(argv[1])
    use_sim_time = bool(spec.get('use_sim_time', True))
    rclpy.init()
    node = Node('harness_drive', parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

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

    wall0, sim0 = time.monotonic(), now()
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
    elif mode == 'wait':
        spin_for(float(spec['duration_s']))
    else:
        print(f'알 수 없는 mode: {mode}', file=sys.stderr)
        return 2

    sim_s, wall_s = now() - sim0, time.monotonic() - wall0
    print(json.dumps({'sim_s': round(sim_s, 3), 'wall_s': round(wall_s, 3),
                      'rtf': round(sim_s / wall_s, 3) if wall_s > 0 else None}), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
