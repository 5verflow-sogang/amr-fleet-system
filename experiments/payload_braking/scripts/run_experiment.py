#!/usr/bin/env python3
"""적재 질량 제동 거리 실험 전체 절차를 자동으로 돌리고 results.csv를 쓴다.

    python3 scripts/run_experiment.py

전제: 이 스크립트를 실행하는 쉘에 ROS2(소스됨)와 ign(Gazebo Fortress CLI)이
있어야 한다. 컨테이너 안에서 실행한다(README "실행 방법" 참고).

절차 (docs 요청 그대로):
  1. 정지 상태에서 1.0 m/s 명령 -> GT 속도 0.95 m/s 도달 시간 = accel_time_s
  2. 1.0 m/s 정속 1초 유지 -> 정지 명령 -> GT 속도 0.01 m/s 미만까지 이동 거리
     = braking_distance_m
  3. 적재 0/2/10/25 kg x 5회 x drive_mode(V,T) x stop_mode(normal,emergency)
  4. 이론값도 같이 기록

세부 설계 결정(README에도 적음):
  - 월드 리셋(world control reset)은 "스폰 이후 만들어진 엔티티"의 자세를 원래대로
    되돌리지 못하는 걸 실측으로 확인했다(원인: 리셋은 SDF에 있던 초기 상태로 되돌리는
    것이라, create 서비스로 나중에 추가한 모델은 그 스냅샷에 없다). 그래서 트라이얼마다
    모델을 지우고 처음 위치에 다시 스폰한다 - 느리지만 확실하다.
  - V 모드는 가속 한계가 DiffDrive 플러그인 설정값이라 normal/emergency마다 다른
    스폰이 필요하다(accel_limited 인자). T 모드는 변환 노드의 ramp_limited 파라미터만
    바꾸면 되므로 같은 스폰을 두 stop_mode에 다 쓴다.
  - GT 속도는 플러그인 Twist 필드를 쓰지 않고 위치를 직접 미분한다(지시사항 그대로) -
    플러그인 속도는 이동평균이라 늦게 반응한다.
"""
import csv
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXP_DIR = os.path.dirname(SCRIPT_DIR)
MODEL_DIR = os.path.join(EXP_DIR, 'model')
WORLD_PATH = os.path.join(EXP_DIR, 'world', 'flat_ground.sdf')
RESULTS_CSV = os.path.join(EXP_DIR, 'results.csv')
XACRO_FILE = os.path.join(MODEL_DIR, 'robot.urdf.xacro')
TMP_URDF = '/tmp/payload_braking_robot.urdf'

WORLD_NAME = 'payload_braking'
MODEL_NAME = 'amr_payload'
SPAWN_X, SPAWN_Y, SPAWN_Z = -20.0, 0.0, 0.05

PAYLOADS_KG = [0.0, 2.0, 10.0, 25.0]
TRIALS_PER_CONDITION = 5

# 로봇 스펙 (model/properties.xacro와 동일한 값. 이론값 계산에만 쓴다)
BASE_MASS_KG = 25.0
WHEEL_MASS_KG = 1.5
NUM_WHEELS = 2
WHEEL_RADIUS_M = 0.1
WHEEL_TORQUE_MAX_NM = 3.0
NORMAL_DECEL_MPS2 = 1.0

CMD_RATE_HZ = 30.0
TARGET_SPEED = 1.0
ACCEL_THRESHOLD = 0.95
STOP_THRESHOLD = 0.01
HOLD_DURATION_S = 1.0
SLIP_RATIO_THRESHOLD = 0.15  # 바퀴 표면속도 vs GT 속도, 이 이상 벌어지면 미끄러짐으로 본다

PHASE_TIMEOUT_S = 15.0  # 가속/제동 한 구간이 이 시간(시뮬레이션 기준) 안에 안 끝나면 이상 상황


def sh(args, timeout=10, **kwargs):
    # ign service는 --timeout(ms)으로 "응답을 기다리는 시간"을 주지만, 실측 중
    # 네트워크 계층이 꼬이면("NodeShared::RecvSrvRequest ... Host unreachable")
    # 그 안에서도 CLI 프로세스 자체가 영영 안 끝나는 경우를 봤다 - subprocess
    # 수준에서도 반드시 타임아웃을 걸어, 멈춰버리는 대신 예외로 떨어져서
    # run_trial의 재시도 루프가 다음 시도로 넘어가게 한다.
    try:
        return subprocess.run(
            args, capture_output=True, text=True, timeout=timeout, **kwargs)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f'명령이 {timeout}초 안에 안 끝남: {args}') from exc


def xacro_generate(payload_kg, drive_mode, accel_limited, out_path):
    args = [
        'xacro', XACRO_FILE,
        f'payload_kg:={payload_kg}',
        f'drive_mode:={drive_mode}',
        f'accel_limited:={"true" if accel_limited else "false"}',
        f'wheel_torque_max:={WHEEL_TORQUE_MAX_NM}',
        f'gt_rate:=100',
        '-o', out_path,
    ]
    result = sh(args)
    if result.returncode != 0:
        raise RuntimeError(f'xacro 실패: {result.stderr}')


def remove_entity(name):
    try:
        sh([
            'ign', 'service', '-s', f'/world/{WORLD_NAME}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', f'name: "{name}" type: MODEL',
        ])
    except RuntimeError as exc:
        # 제거 자체가 실패해도(예: 원래 없었음) 재시도 루프가 이어서 스폰을
        # 시도하게 한다 - 여기서 멈추지 않는다.
        print(f'  (remove_entity 경고: {exc})', flush=True)


def spawn_entity(urdf_path, name, x, y, z):
    result = sh([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-file', urdf_path, '-name', name,
        '-x', str(x), '-y', str(y), '-z', str(z),
    ], timeout=15)
    if result.returncode != 0 or 'OK creation' not in (result.stdout + result.stderr):
        raise RuntimeError(f'스폰 실패: {result.stdout}\n{result.stderr}')


def theory_normal_m():
    # normal = v^2 / (2 * 1.0)
    return TARGET_SPEED ** 2 / (2.0 * NORMAL_DECEL_MPS2)


def theory_emergency_m(payload_kg):
    total_mass = BASE_MASS_KG + NUM_WHEELS * WHEEL_MASS_KG + payload_kg
    decel = (2.0 * WHEEL_TORQUE_MAX_NM) / (WHEEL_RADIUS_M * total_mass)
    return TARGET_SPEED ** 2 / (2.0 * decel)


class Harness(Node):
    """GT odom(위치 미분)·joint_states 구독, cmd_vel 발행을 한데 묶은 보조 노드."""

    def __init__(self):
        super().__init__('payload_braking_harness')
        self._gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self._on_gt, 10)
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._prev_pos = None  # (t_sec, x, y)
        self._cur_pos = None
        self._speed = 0.0
        self._max_slip_ratio = 0.0
        self._wheel_surface_speed = 0.0

    def _on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._cur_pos is not None:
            self._prev_pos = self._cur_pos
        self._cur_pos = (t, x, y)
        if self._prev_pos is not None:
            dt = self._cur_pos[0] - self._prev_pos[0]
            if dt > 1e-6:
                dx = self._cur_pos[1] - self._prev_pos[1]
                dy = self._cur_pos[2] - self._prev_pos[2]
                self._speed = math.hypot(dx, dy) / dt
                # 슬립 비교는 GT 콜백에서만 한다(여기서만 하면 GT·joint_states
                # 두 비동기 스트림을 서로 다른 시점에 번갈아 비교하며 생기는
                # 잡음(실제 슬립이 아닌 샘플링 시차)을 줄일 수 있다). 속도가
                # 충분히 날 때만 비교해 출발·정지 근처의 분모 잡음도 피한다.
                if self._speed > 0.2 and self._wheel_surface_speed > 0.2:
                    denom = max(self._wheel_surface_speed, self._speed)
                    ratio = abs(self._wheel_surface_speed - self._speed) / denom
                    if ratio > self._max_slip_ratio:
                        self._max_slip_ratio = ratio

    def _on_joint(self, msg: JointState):
        if not msg.velocity:
            return
        avg_wheel_vel = sum(abs(v) for v in msg.velocity) / len(msg.velocity)
        self._wheel_surface_speed = avg_wheel_vel * WHEEL_RADIUS_M

    def reset_slip_tracking(self):
        self._max_slip_ratio = 0.0

    def reset_motion_tracking(self):
        """리스폰 직후 호출 - 이전 트라이얼(제거된 엔티티)의 마지막 위치와
        새 엔티티의 첫 위치를 이어서 미분해버리는 걸 막는다."""
        self._prev_pos = None
        self._cur_pos = None
        self._speed = 0.0
        self._wheel_surface_speed = 0.0

    @property
    def slipped(self):
        return self._max_slip_ratio > SLIP_RATIO_THRESHOLD

    @property
    def sim_time(self):
        return self._cur_pos[0] if self._cur_pos else None

    @property
    def position(self):
        return (self._cur_pos[1], self._cur_pos[2]) if self._cur_pos else None

    @property
    def speed(self):
        return self._speed

    def publish_cmd(self, v):
        msg = Twist()
        msg.linear.x = v
        self._cmd_pub.publish(msg)

    def spin_until(self, predicate, timeout_s, publish_v=None, rate_hz=CMD_RATE_HZ):
        """predicate()가 True가 될 때까지 spin하며, publish_v가 주어지면 그 속도를
        rate_hz로 계속 퍼블리시한다. 실제 대기 판단은 전부 시뮬레이션 시각 기준이다."""
        period = 1.0 / rate_hz
        last_pub = 0.0
        start_wall = time.monotonic()
        start_sim = self.sim_time
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            now_wall = time.monotonic()
            if publish_v is not None and (now_wall - last_pub) >= period:
                self.publish_cmd(publish_v)
                last_pub = now_wall
            if predicate():
                return True
            if start_sim is not None and self.sim_time is not None:
                if self.sim_time - start_sim > timeout_s:
                    return False
            if now_wall - start_wall > timeout_s * 5 + 10:
                # 시뮬레이션 시각을 못 받아오는 등 비정상 상황에 대한 안전망(실시간 기준)
                return False


def run_trial(harness, drive_mode, stop_mode, payload_kg, trial_idx):
    # --- 0. 시작 전 상태 확인: 새로 스폰한 로봇의 GT가 스폰 위치 근처로 들어오는지
    # 확인한다(단순히 "메시지가 오는가"만 보면 리스폰 경합 중 남아있던 메시지를
    # 새 트라이얼로 착각할 수 있다). ---
    harness.reset_slip_tracking()

    def _arrived():
        pos = harness.position
        if pos is None:
            return False
        return abs(pos[0] - SPAWN_X) < 1.0 and abs(pos[1] - SPAWN_Y) < 1.0

    if not harness.spin_until(_arrived, timeout_s=8.0):
        raise RuntimeError('스폰 위치 근처에서 GT odom을 못 받음(리스폰 실패 추정)')

    # --- 1. 가속: 1.0 m/s 명령, GT 속도 0.95 도달까지 시간 ---
    t_cmd_start = harness.sim_time
    reached = harness.spin_until(
        lambda: harness.speed >= ACCEL_THRESHOLD,
        timeout_s=PHASE_TIMEOUT_S, publish_v=TARGET_SPEED)
    if not reached:
        raise RuntimeError(f'가속 단계에서 {ACCEL_THRESHOLD} m/s에 도달 못 함')
    accel_time_s = harness.sim_time - t_cmd_start

    # --- 2. 1.0 m/s 정속 1초 유지 ---
    t_hold_start = harness.sim_time
    harness.spin_until(
        lambda: (harness.sim_time - t_hold_start) >= HOLD_DURATION_S,
        timeout_s=PHASE_TIMEOUT_S, publish_v=TARGET_SPEED)

    # --- 3. 정지 명령, 이동 거리 측정 ---
    pos_stop_start = harness.position

    if drive_mode == 'V':
        # emergency: 가속 한계 자체가 스폰 설정(accel_limited=false)이라 여기서는
        # normal/emergency 둘 다 "0 명령을 바로 낸다"로 똑같다 - 실제 감속 프로파일
        # 차이는 플러그인 설정이 만든다.
        stopped = harness.spin_until(
            lambda: harness.speed < STOP_THRESHOLD,
            timeout_s=PHASE_TIMEOUT_S, publish_v=0.0)
    else:
        # T: 변환 노드가 ramp_limited에 따라 다르게 행동한다. 여기서는 그냥 0을
        # 계속 보내기만 하면 된다.
        stopped = harness.spin_until(
            lambda: harness.speed < STOP_THRESHOLD,
            timeout_s=PHASE_TIMEOUT_S, publish_v=0.0)

    if not stopped:
        raise RuntimeError('정지 단계에서 속도가 임계값 밑으로 안 내려감')

    pos_stop_end = harness.position
    braking_distance_m = math.hypot(
        pos_stop_end[0] - pos_stop_start[0], pos_stop_end[1] - pos_stop_start[1])

    if stop_mode == 'normal':
        theory_m = theory_normal_m()
    else:
        theory_m = theory_emergency_m(payload_kg)

    return {
        'drive_mode': drive_mode,
        'stop_mode': stop_mode,
        'payload_kg': payload_kg,
        'trial': trial_idx,
        'accel_time_s': round(accel_time_s, 4),
        'braking_distance_m': round(braking_distance_m, 4),
        'theory_m': round(theory_m, 4),
        'slipped': harness.slipped,
    }


def start_bridge_v():
    return subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ])


def start_bridge_t():
    return subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        f'/model/{MODEL_NAME}/joint/left_wheel_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/model/{MODEL_NAME}/joint/right_wheel_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
        '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ])


def start_clock_bridge():
    # 시뮬레이션은 real_time_factor=0(스로틀 없음)으로 실시간보다 훨씬 빠르게 돈다.
    # cmd_vel_to_wheel_torque.py의 램프 타이머가 벽시계를 쓰면, 실제로는 "1.0 m/s^2"가
    # 아니라 시뮬레이션 시각 기준으로 훨씬 빠른(=RTF배) 감속으로 보인다 - 이 버그를
    # 실측 전에 코드 리뷰로 잡아서, /clock을 브리지하고 변환 노드에 use_sim_time을
    # 켠다(rclpy 타이머가 자동으로 시뮬레이션 시각을 쓰게 된다).
    return subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ])


def start_converter(ramp_limited):
    return subprocess.Popen([
        sys.executable, os.path.join(SCRIPT_DIR, 'cmd_vel_to_wheel_torque.py'),
        '--ros-args', '-p', f'ramp_limited:={"true" if ramp_limited else "false"}',
        '-p', f'linear_accel_limit:={NORMAL_DECEL_MPS2}',
        '-p', f'model_name:={MODEL_NAME}',
    ])


def stop_proc(proc):
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    rclpy.init()
    harness = Harness()

    rows = []
    gz_proc = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '--iterations', '0', WORLD_PATH])
    time.sleep(3)
    clock_bridge = start_clock_bridge()
    time.sleep(1)

    try:
        for drive_mode in ['V', 'T']:
            stop_mode_groups = ['normal', 'emergency'] if drive_mode == 'V' else [None]
            for group in stop_mode_groups:
                bridge = start_bridge_v() if drive_mode == 'V' else start_bridge_t()
                time.sleep(2)

                for payload_kg in PAYLOADS_KG:
                    accel_limited = (group == 'normal') if drive_mode == 'V' else True
                    xacro_generate(payload_kg, drive_mode, accel_limited, TMP_URDF)

                    stop_modes_here = [group] if drive_mode == 'V' else ['normal', 'emergency']
                    for stop_mode in stop_modes_here:
                        converter = None
                        if drive_mode == 'T':
                            converter = start_converter(ramp_limited=(stop_mode == 'normal'))
                            time.sleep(1)

                        for trial in range(1, TRIALS_PER_CONDITION + 1):
                            print(f'[{drive_mode}/{stop_mode}] payload={payload_kg}kg '
                                  f'trial={trial} ...', flush=True)

                            row = None
                            last_error = None
                            # 가끔 리스폰 직후 토픽 연결이 늦게 맺혀 가속 단계가
                            # 실패하는 경우가 있었다(README "문제가 있었으면" 참고) -
                            # settle 시간을 늘려 재시도하면 대개 통과한다.
                            for attempt in range(1, 4):
                                try:
                                    remove_entity(MODEL_NAME)
                                    time.sleep(0.5)
                                    spawn_entity(TMP_URDF, MODEL_NAME, SPAWN_X, SPAWN_Y, SPAWN_Z)
                                    time.sleep(1.0 * attempt)
                                    harness.reset_motion_tracking()
                                    row = run_trial(
                                        harness, drive_mode, stop_mode, payload_kg, trial)
                                    break
                                except RuntimeError as exc:
                                    last_error = exc
                                    print(f'  (시도 {attempt} 실패: {exc}, 재시도)', flush=True)

                            if row is None:
                                print(f'  -> 3회 시도 모두 실패, 이 트라이얼은 건너뜀: '
                                      f'{last_error}', flush=True)
                                row = {
                                    'drive_mode': drive_mode, 'stop_mode': stop_mode,
                                    'payload_kg': payload_kg, 'trial': trial,
                                    'accel_time_s': '', 'braking_distance_m': '',
                                    'theory_m': '', 'slipped': f'FAILED: {last_error}',
                                }
                            else:
                                print(f'  -> accel={row["accel_time_s"]}s '
                                      f'brake={row["braking_distance_m"]}m '
                                      f'theory={row["theory_m"]}m slip={row["slipped"]}',
                                      flush=True)
                            rows.append(row)

                            with open(RESULTS_CSV, 'w', newline='') as f:
                                writer = csv.DictWriter(f, fieldnames=[
                                    'drive_mode', 'stop_mode', 'payload_kg', 'trial',
                                    'accel_time_s', 'braking_distance_m', 'theory_m', 'slipped'])
                                writer.writeheader()
                                writer.writerows(rows)

                        if converter is not None:
                            stop_proc(converter)

                stop_proc(bridge)
                remove_entity(MODEL_NAME)
                time.sleep(0.3)
    finally:
        stop_proc(clock_bridge)
        stop_proc(gz_proc)
        harness.destroy_node()
        rclpy.shutdown()

    print(f'완료: {len(rows)}개 트라이얼, {RESULTS_CSV}에 저장')


if __name__ == '__main__':
    main()
