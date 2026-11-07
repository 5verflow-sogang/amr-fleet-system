#!/usr/bin/env python3
"""식당 시연 장면 스크립트: 장면 파일(tools/harness/tasks/demo_*.yaml)대로 작업을 넣고 핵심 요청 시각을 /demo/t0으로 찍는다.

임시 도구다 (장면 2 녹화용, 연우 장면 스크립트가 들어오기 전까지). 하네스 시나리오는 params scene_script 한 줄로 이 명령을
부르므로, 같은 인자(<장면 파일> --stations --map-origin --num-robots --set --out)를 받는 다른 스크립트로 바로 바꿀 수 있다.

    python3 /repo/tools/harness/demo_scene.py /repo/tools/harness/tasks/demo_s2a.yaml \\
        --stations <stations_restaurant.yaml> --num-robots 5 --set collect_priority=200 --out <run_dir>/scene.json

장면은 단계(steps)의 목록이다. 단계마다 조건(after)이 맞으면 그 단계 작업을 차례로 /fleet/task_requests에 낸다.
- after.assigned: 이 장면이 낸 작업 중 할당받은(작업표 assignee_id가 찬) 작업이 N건 이상
- after.pending: 이 장면이 낸 작업 중 아직 할당 안 된 작업이 N건 이상
- after.delay_s: 앞 단계 마지막 작업을 낸 뒤 N s (시뮬레이션 시각)
- after.timeout_s: 앞 단계 뒤 N s가 지나면 조건이 안 맞아도 낸다 (scene.json에 timeout으로 남는다)
- gap_s: 단계 안 작업 사이 간격 (기본 0, 연달아 넣기)
- repeat: 단계 작업 목록을 N번 되풀이한다 (작업 id 뒤에 _01, _02, ...). 일정한 간격의 배달 흐름 (장면 3)
- t0: true면 그 단계 첫 작업을 낼 때 /demo/t0(std_msgs/Header, stamp = 시뮬레이션 시각, frame_id = 첫 작업 id)을 낸다
단계와 따로 0초를 배터리로 정할 수도 있다 (장면 3, 켬/끔을 같은 배터리 순간에 맞춘다. 단계의 t0과 같이 쓰지 않는다):
- t0_when.battery_below: 로봇 하나라도 /fleet/robot_states 배터리가 이 값(%) 밑이면 그때 /demo/t0 (frame_id = battery)
작업 값 안의 {이름}은 --set 이름=값으로 채운다 (예: priority: "{collect_priority}").
Task.created_time은 시뮬레이션 시각이다 (aging이 노드 시계로 대기 시간을 잰다). 정차 자세는 stations 파일(world)을
map_origin_in_world로 map 좌표로 바꾼다 (작업 생성기와 같다).

단계 진행(Scene)은 rclpy 없이 돈다 - tests/test_demo_scene.py가 가짜 시계·작업표로 시험한다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

TASK_TYPES = {'delivery': 0, 'charge': 1, 'return': 2, 'collect': 3}   # amr_msgs/msg/Task.TYPE_*
STATION_RESERVE_M = 1.0   # 서버 스테이션 예약 반경: 시작 작업이 핵심 요청과 이 안의 자리를 쓰면 핵심 요청이 밀린다


@dataclass(frozen=True)
class SceneTask:
    task_id: str
    type: str
    pickup: str
    dropoff: str
    priority: int


@dataclass
class Step:
    name: str
    tasks: list[SceneTask]
    after: dict = field(default_factory=dict)
    gap_s: float = 0.0
    t0: bool = False


def _fill(value, params):
    return value.format_map(params) if isinstance(value, str) else value


def load_scene(path, params: Optional[dict] = None) -> list[Step]:
    """장면 파일 → 단계 목록. {이름}을 params로 채우고 작업 id 중복·종류·priority를 확인한다."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    params = params or {}
    steps, seen = [], set()
    for s in data['steps']:
        tasks = []
        repeat = int(s.get('repeat', 1))
        if repeat < 1:
            raise ValueError(f'{s["name"]}: repeat은 1 이상: {repeat}')
        for k in range(1, repeat + 1):
            for t in s['tasks']:
                task_id = str(t['id']) if repeat == 1 else f'{t["id"]}_{k:02d}'
                task = SceneTask(task_id=task_id, type=str(t['type']), pickup=str(t['pickup']),
                                 dropoff=str(t['dropoff']), priority=int(_fill(t.get('priority', 100), params)))
                tasks.append(task)
        for task in tasks:
            if task.type not in TASK_TYPES:
                raise ValueError(f'{task.task_id}: type은 {sorted(TASK_TYPES)} 중 하나: {task.type!r}')
            if not 0 <= task.priority <= 255:
                raise ValueError(f'{task.task_id}: priority는 0~255: {task.priority}')
            if task.task_id in seen:
                raise ValueError(f'작업 id 중복: {task.task_id}')
            seen.add(task.task_id)
        steps.append(Step(name=str(s['name']), tasks=tasks, after=dict(s.get('after') or {}),
                          gap_s=float(s.get('gap_s', 0.0)), t0=bool(s.get('t0', False))))
    if sum(s.t0 for s in steps) > 1:
        raise ValueError('t0 단계는 하나만')
    if data.get('t0_when') and any(s.t0 for s in steps):
        raise ValueError('t0_when과 단계의 t0은 같이 쓰지 않는다')
    return steps


def load_t0_when(path) -> Optional[dict]:
    """장면 파일의 t0_when (배터리로 정하는 0초). 없으면 None."""
    rule = yaml.safe_load(Path(path).read_text(encoding='utf-8')).get('t0_when')
    if not rule:
        return None
    if set(rule) != {'battery_below'}:
        raise ValueError(f't0_when은 battery_below만: {sorted(rule)}')
    return {'battery_below': float(rule['battery_below'])}


def stations_near(stations: dict, a: str, b: str, radius: float = STATION_RESERVE_M) -> bool:
    ax, ay = stations[a][:2]
    bx, by = stations[b][:2]
    return math.hypot(ax - bx, ay - by) < radius


class Scene:
    """단계 진행. tick(지금 시각, 작업표 {task_id: assignee}, 배터리 {robot_id: %})이 이번에 할 일
    [('t0', t, frame_id) | ('task', SceneTask)]을 돌려준다. t0_when이 있으면 배터리로 0초를 찍는다 (단계와 따로)."""

    def __init__(self, steps: list[Step], t0_when: Optional[dict] = None):
        self.steps = steps
        self.t0_when = t0_when
        self.index = 0          # 다음 단계
        self.queue: list[SceneTask] = []   # 지금 단계에서 아직 안 낸 작업
        self.next_at = 0.0      # 다음 작업을 낼 수 있는 시각
        self.step_start: Optional[float] = None   # 앞 단계 마지막 작업을 낸 시각 (첫 단계는 첫 tick)
        self.sent: list[str] = []
        self.log: list[dict] = []
        self.t0: Optional[float] = None

    @property
    def done(self) -> bool:
        # 배터리 0초를 기다리는 장면은 0초가 찍힐 때까지 끝내지 않는다
        return self.index >= len(self.steps) and not self.queue and (self.t0_when is None or self.t0 is not None)

    def _ready(self, step: Step, now: float, table: dict) -> Optional[str]:
        """조건이 맞으면 'ok', 시간 초과면 'timeout', 아직이면 None."""
        a = step.after
        mine = [table.get(t) for t in self.sent]
        assigned = sum(1 for x in mine if x)
        pending = sum(1 for x in mine if not x)
        waited = now - self.step_start
        ok = (assigned >= int(a.get('assigned', 0)) and pending >= int(a.get('pending', 0))
              and waited >= float(a.get('delay_s', 0.0)))
        if ok:
            return 'ok'
        if 'timeout_s' in a and waited >= float(a['timeout_s']):
            return 'timeout'
        return None

    def tick(self, now: float, table: dict, batteries: Optional[dict] = None) -> list[tuple]:
        if self.step_start is None:
            self.step_start = now
        out = []
        if self.t0_when and self.t0 is None and batteries \
                and min(batteries.values()) < self.t0_when['battery_below']:
            self.t0 = now
            out.append(('t0', now, 'battery'))
            self.log.append({'t': round(now, 3), 'ev': 't0', 'why': 'battery',
                             'battery': {k: round(v, 2) for k, v in sorted(batteries.items())}})
        while True:
            if self.queue:
                if now < self.next_at:
                    return out
                task = self.queue.pop(0)
                self.sent.append(task.task_id)
                out.append(('task', task))
                self.log.append({'t': round(now, 3), 'ev': 'request', 'task': task.task_id, 'priority': task.priority})
                self.next_at = now + self.steps[self.index - 1].gap_s
                if not self.queue:
                    self.step_start = now
                continue
            if self.index >= len(self.steps):
                return out
            step = self.steps[self.index]
            why = self._ready(step, now, table)
            if why is None:
                return out
            self.log.append({'t': round(now, 3), 'ev': 'step', 'step': step.name, 'why': why})
            self.index += 1
            self.queue = list(step.tasks)
            self.next_at = now
            if step.t0:
                self.t0 = now
                out.append(('t0', now, step.tasks[0].task_id if step.tasks else step.name))
                self.log.append({'t': round(now, 3), 'ev': 't0', 'step': step.name})
            if not self.queue:
                self.step_start = now


# ---------------------------------------------------------------- ROS (하네스에서만)
def _run(args, scene, stations, origin):
    import rclpy
    from amr_msgs.msg import FleetTaskTable, RobotState, Task
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header

    ox, oy, oyaw = origin
    c, s = math.cos(-oyaw), math.sin(-oyaw)

    def pose(name):
        x, y, yaw = stations[name]
        dx, dy = x - ox, y - oy
        p = Pose()
        p.position.x, p.position.y = c * dx - s * dy, s * dx + c * dy
        p.orientation.z, p.orientation.w = math.sin((yaw - oyaw) / 2), math.cos((yaw - oyaw) / 2)
        return p

    rclpy.init()
    node = Node('demo_scene', parameter_overrides=[rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, not args.wall_clock)])
    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Task, '/fleet/task_requests', reliable)
    t0_pub = node.create_publisher(Header, '/demo/t0', latched)
    table, robots = {}, {}

    def on_table(m):
        table.clear()
        table.update({r.task.task_id: r.assignee_id for r in m.records})

    def on_state(m):
        robots[m.robot_id] = float(m.battery_percent)

    node.create_subscription(FleetTaskTable, '/fleet/task_table', on_table, 10)
    node.create_subscription(RobotState, '/fleet/robot_states', on_state, 10)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin_until(cond, what, limit_s):
        import time
        w0 = time.time()
        while not cond():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - w0 > limit_s:
                raise SystemExit(f'준비 안 됨: {what}')

    # 서버(작업 요청 구독)와 로봇 N대 상태, 시뮬레이션 시계가 보일 때까지 (실제 시간 기준 한도)
    spin_until(lambda: pub.get_subscription_count() > 0 and len(robots) >= args.num_robots and now() > 0,
               f'Fleet 서버·로봇 {args.num_robots}대·/clock', args.ready_timeout_s)
    start = now()
    spin_until(lambda: now() - start >= args.settle_s, 'settle', args.ready_timeout_s)

    t_end = now() + args.timeout_s
    while not scene.done and now() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        for action in scene.tick(now(), table, robots):
            if action[0] == 't0':
                h = Header(frame_id=action[2])
                h.stamp = node.get_clock().now().to_msg()
                t0_pub.publish(h)
                continue
            task = action[1]
            m = Task(task_id=task.task_id, task_type=TASK_TYPES[task.type], priority=task.priority,
                     pickup_station_id=task.pickup, dropoff_station_id=task.dropoff)
            m.pickup_pose, m.dropoff_pose = pose(task.pickup), pose(task.dropoff)
            m.created_time = node.get_clock().now().to_msg()
            pub.publish(m)
    status = 'done' if scene.done else 'timeout'
    # 늦게 붙은 구독자(녹화)가 /demo/t0을 받을 수 있게 노드를 잠시 더 둔다
    hold = now() + 2.0
    while now() < hold:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    return status


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('scene')
    ap.add_argument('--stations', required=True)
    ap.add_argument('--map-origin', default='[0.0, 0.0, 0.0]', help='map_origin_in_world [x, y, yaw]')
    ap.add_argument('--num-robots', type=int, default=5)
    ap.add_argument('--set', action='append', default=[], metavar='이름=값', help='장면 파일의 {이름}')
    ap.add_argument('--settle-s', type=float, default=5.0, help='준비된 뒤 첫 작업까지 (시뮬레이션 s)')
    ap.add_argument('--timeout-s', type=float, default=600.0, help='장면 전체 한도 (시뮬레이션 s)')
    ap.add_argument('--ready-timeout-s', type=float, default=900.0, help='준비 대기 한도 (실제 s)')
    ap.add_argument('--out', help='장면 기록 json')
    ap.add_argument('--wall-clock', action='store_true', help='시뮬레이션 시계 대신 실제 시계 (Gazebo 없는 가짜 로봇)')
    a = ap.parse_args(argv)
    params = dict(item.split('=', 1) for item in a.set)
    steps = load_scene(a.scene, params)
    t0_when = load_t0_when(a.scene)
    stations = {k: tuple(float(x) for x in v) for k, v in
                yaml.safe_load(Path(a.stations).read_text(encoding='utf-8'))['stations'].items()}
    unknown = sorted({n for s in steps for t in s.tasks for n in (t.pickup, t.dropoff)} - set(stations))
    if unknown:
        raise SystemExit(f'stations 파일에 없는 자리: {unknown}')
    scene = Scene(steps, t0_when)
    status = 'interrupted'
    try:
        status = _run(a, scene, stations, [float(v) for v in yaml.safe_load(a.map_origin)])
    except KeyboardInterrupt:
        pass   # 하네스가 녹화 시간이 끝나 SIGINT로 멈췄다. 그때까지 낸 것을 남긴다
    except Exception as e:   # rclpy 종료 예외 등도 기록은 남긴다
        status = f'error: {e!r}'
    finally:
        record = {'scene': a.scene, 'params': params, 'status': status, 't0': scene.t0, 'events': scene.log}
        print(json.dumps({k: record[k] for k in ('status', 't0')}), flush=True)
        if a.out:
            Path(a.out).write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding='utf-8')
    return 0 if status == 'done' else 1


if __name__ == '__main__':
    sys.exit(main())
