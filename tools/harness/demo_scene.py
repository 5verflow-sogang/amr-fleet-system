#!/usr/bin/env python3
"""식당 시연 장면 스크립트: 장면 파일(tools/harness/tasks/demo_*.yaml)대로 작업을 넣고 핵심 요청 시각을 /demo/t0으로 찍는다.

임시 도구다 (장면 2 녹화용, 연우 장면 스크립트가 들어오기 전까지). 하네스 시나리오는 params scene_script 한 줄로 이 명령을
부르므로, 같은 인자(<장면 파일> --stations --map-origin --num-robots --set --out)를 받는 다른 스크립트로 바로 바꿀 수 있다.

    python3 /repo/tools/harness/demo_scene.py /repo/tools/harness/tasks/demo_s2a.yaml \\
        --stations <stations_restaurant.yaml> --num-robots 5 --set collect_priority=200 --out <run_dir>/scene.json

장면은 단계(steps)의 목록이다. 단계마다 조건(after)이 맞으면 그 단계 작업을 차례로 /fleet/task_requests에 낸다.
- after.assigned: 이 장면이 낸 작업 중 할당받은(작업표 assignee_id가 찬) 작업이 N건 이상. 서버는 할당한 뒤 IN_PROGRESS·
  COMPLETED가 되어도 assignee_id를 그대로 두므로 그 작업도 센다. 실패 뒤 다시 큐에 들어가면(assignee_id 빔) 안 센다
- after.pending: 이 장면이 낸 작업 중 아직 할당 안 된 작업이 N건 이상
- after.delay_s: 앞 단계 마지막 작업을 낸 뒤 N s (시뮬레이션 시각)
- after.near: {task: 작업 id, within_m: D} - 그 작업을 맡은 로봇(작업표 assignee_id)이 그 작업 하역 자리에서 D m 안에 있다
  (/fleet/robot_states 자세). 0초를 "그 로봇이 곧 비는 때"에 맞춘다 - 실제 속도(녹화에서 약 0.2 m/s)와 상관없이
  0초 직후 그 로봇이 일을 마친다
- after.timeout_s: 앞 단계 뒤 N s가 지나면 조건이 안 맞아도 낸다 (scene.json에 timeout으로 남는다)
- gap_s: 단계 안 작업 사이 간격 (기본 0, 연달아 넣기)
- t0: true면 그 단계 첫 작업을 낼 때 /demo/t0(std_msgs/Header, stamp = 시뮬레이션 시각, frame_id = 첫 작업 id)을 낸다
작업 값 안의 {이름}은 --set 이름=값으로 채운다 (예: priority: "{collect_priority}").
Task.created_time은 시뮬레이션 시각이다 (aging이 노드 시계로 대기 시간을 잰다). 정차 자세는 stations 파일(world)을
map_origin_in_world로 map 좌표로 바꾼다 (작업 생성기와 같다).

서버 스테이션 예약 (Fleet 서버 assignment.station_clearance_m, 1 m): 서버는 맡긴 작업의 픽업·하역 자리를 그 작업이 끝날
때까지 잡아 두고, 그 안을 쓰는 작업은 큐에서 기다리게 한다(큐 안에서 겹치면 하나만 맡긴다). 같은 픽업을 쓰는 시작 작업 둘은
둘째가 첫째가 끝날 때까지 할당되지 않아 assigned 조건이 시간 초과로 끝난다 (11/8 녹화: 1-2 request, 2막 more 단계).
그래서 같이 잡혀 있을 수 있는 작업은 자리가 서로 1 m 넘게 떨어져야 한다 - station_conflicts()가 확인하고, 시작할 때
걸리면 멈춘다. 0초(t0) 단계 작업끼리(2막 회수 5건이 퇴식구 둘을 나눠 씀)는 일부러 줄을 세우므로 뺀다.

scene.json에는 1 s(시뮬레이션)마다 이 장면 작업의 작업표 상태(assigned·pending·done·failed)를 남긴다 (ev: count).
단계를 다 낸 뒤에도 하네스가 멈출 때까지(또는 --timeout-s) 계속 남긴다.

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
STATION_RESERVE_M = 1.0   # 서버 스테이션 예약 반경 (fleet_manager.yaml assignment.station_clearance_m)
STATE_COMPLETED, STATE_FAILED = 2, 3   # amr_msgs/msg/TaskStatus.STATE_* (작업표 TaskRecord.state)


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
        for t in s['tasks']:
            task = SceneTask(task_id=str(t['id']), type=str(t['type']), pickup=str(t['pickup']),
                             dropoff=str(t['dropoff']), priority=int(_fill(t.get('priority', 100), params)))
            if task.type not in TASK_TYPES:
                raise ValueError(f'{task.task_id}: type은 {sorted(TASK_TYPES)} 중 하나: {task.type!r}')
            if not 0 <= task.priority <= 255:
                raise ValueError(f'{task.task_id}: priority는 0~255: {task.priority}')
            if task.task_id in seen:
                raise ValueError(f'작업 id 중복: {task.task_id}')
            seen.add(task.task_id)
            tasks.append(task)
        steps.append(Step(name=str(s['name']), tasks=tasks, after=dict(s.get('after') or {}),
                          gap_s=float(s.get('gap_s', 0.0)), t0=bool(s.get('t0', False))))
    if sum(s.t0 for s in steps) > 1:
        raise ValueError('t0 단계는 하나만')
    ids = {t.task_id for st in steps for t in st.tasks}
    for st in steps:
        near = st.after.get('near')
        if near is not None and near.get('task') not in ids:
            raise ValueError(f'{st.name}: after.near의 작업이 장면에 없다: {near}')
    return steps


def stations_near(stations: dict, a: str, b: str, radius: float = STATION_RESERVE_M) -> bool:
    ax, ay = stations[a][:2]
    bx, by = stations[b][:2]
    return math.hypot(ax - bx, ay - by) <= radius   # 서버 _station_free와 같은 비교


def station_conflicts(steps: list[Step], stations: dict, radius: float = STATION_RESERVE_M) -> list[tuple]:
    """같이 잡혀 있을 수 있는 작업끼리 자리가 radius 안에 겹치는 쌍 [(작업, 작업, 자리, 자리)].

    서버는 맡긴 작업의 픽업·하역을 끝날 때까지 잡아 두므로, 장면의 작업은 서로 자리가 달라야 차례대로 할당된다.
    0초 단계 안의 작업끼리는 뺀다 (일부러 같은 자리로 줄을 세운다).
    """
    tasks = [(i, t) for i, step in enumerate(steps) for t in step.tasks]
    t0_step = next((i for i, step in enumerate(steps) if step.t0), None)
    out = []
    for k, (i, a) in enumerate(tasks):
        for j, b in tasks[k + 1:]:
            if i == j == t0_step:
                continue
            for sa in (a.pickup, a.dropoff):
                for sb in (b.pickup, b.dropoff):
                    if stations_near(stations, sa, sb, radius):
                        out.append((a.task_id, b.task_id, sa, sb))
    return out


def table_counts(sent: list[str], table: dict, states: Optional[dict] = None) -> dict:
    """이 장면이 낸 작업의 작업표 상태. table = {task_id: assignee_id}, states = {task_id: TaskRecord.state}."""
    states = states or {}
    assigned = sum(1 for t in sent if table.get(t))
    return {'sent': len(sent), 'assigned': assigned, 'pending': len(sent) - assigned,
            'done': sum(1 for t in sent if states.get(t) == STATE_COMPLETED),
            'failed': sum(1 for t in sent if states.get(t) == STATE_FAILED)}


class Scene:
    """단계 진행. tick(지금 시각, 작업표 {task_id: assignee}, 로봇 자세 {robot: (x, y)})이 이번에 할 일
    [('t0', t) | ('task', SceneTask)]을 돌려준다. stations는 after.near에 쓴다 (작업 하역 자리 좌표)."""

    def __init__(self, steps: list[Step], stations: Optional[dict] = None):
        self.steps = steps
        self.stations = stations or {}
        self.tasks = {t.task_id: t for s in steps for t in s.tasks}
        self.index = 0          # 다음 단계
        self.queue: list[SceneTask] = []   # 지금 단계에서 아직 안 낸 작업
        self.next_at = 0.0      # 다음 작업을 낼 수 있는 시각
        self.step_start: Optional[float] = None   # 앞 단계 마지막 작업을 낸 시각 (첫 단계는 첫 tick)
        self.sent: list[str] = []
        self.log: list[dict] = []
        self.t0: Optional[float] = None

    @property
    def done(self) -> bool:
        return self.index >= len(self.steps) and not self.queue

    def _near(self, cond: dict, table: dict, robots: dict) -> bool:
        task = self.tasks[cond['task']]
        robot = table.get(task.task_id)
        if not robot or robot not in robots or task.dropoff not in self.stations:
            return False
        return math.dist(robots[robot], self.stations[task.dropoff][:2]) <= float(cond['within_m'])

    def _ready(self, step: Step, now: float, table: dict, robots: dict) -> Optional[str]:
        """조건이 맞으면 'ok', 시간 초과면 'timeout', 아직이면 None."""
        a = step.after
        mine = [table.get(t) for t in self.sent]
        assigned = sum(1 for x in mine if x)
        pending = sum(1 for x in mine if not x)
        waited = now - self.step_start
        ok = (assigned >= int(a.get('assigned', 0)) and pending >= int(a.get('pending', 0))
              and waited >= float(a.get('delay_s', 0.0))
              and ('near' not in a or self._near(a['near'], table, robots)))
        if ok:
            return 'ok'
        if 'timeout_s' in a and waited >= float(a['timeout_s']):
            return 'timeout'
        return None

    def record(self, now: float, table: dict, states: Optional[dict] = None) -> None:
        """작업표 상태를 기록에 남긴다 (scene.json ev: count)."""
        self.log.append({'t': round(now, 3), 'ev': 'count', **table_counts(self.sent, table, states)})

    def tick(self, now: float, table: dict, robots: Optional[dict] = None) -> list[tuple]:
        robots = robots or {}
        if self.step_start is None:
            self.step_start = now
        out = []
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
            why = self._ready(step, now, table, robots)
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
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Task, '/fleet/task_requests', reliable)
    t0_pub = node.create_publisher(Header, '/demo/t0', latched)
    table, states, poses = {}, {}, {}

    def on_table(m):
        table.clear()
        table.update({r.task.task_id: r.assignee_id for r in m.records})
        states.clear()
        states.update({r.task.task_id: r.state for r in m.records})

    node.create_subscription(FleetTaskTable, '/fleet/task_table', on_table, 10)
    node.create_subscription(RobotState, '/fleet/robot_states',
                             lambda m: poses.__setitem__(m.robot_id, (m.pose.position.x, m.pose.position.y)), 10)

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
    spin_until(lambda: pub.get_subscription_count() > 0 and len(poses) >= args.num_robots and now() > 0,
               f'Fleet 서버·로봇 {args.num_robots}대·/clock', args.ready_timeout_s)
    start = now()
    spin_until(lambda: now() - start >= args.settle_s, 'settle', args.ready_timeout_s)

    # 단계를 다 낸 뒤에도 timeout_s까지(보통은 하네스가 녹화를 끝내며 SIGINT로 멈출 때까지) 1 s마다 작업표 상태를 남긴다 -
    # 0초 뒤 할당이 멈춘 구간을 scene.json으로 본다
    t_end = now() + args.timeout_s
    last_count = -1e9
    while now() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if now() - last_count >= 1.0:
            last_count = now()
            scene.record(last_count, table, states)
        for action in scene.tick(now(), table, poses):
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
    ap.add_argument('--timeout-s', type=float, default=600.0,
                    help='장면 전체 한도 (시뮬레이션 s). 단계를 다 낸 뒤에도 이때까지 작업표 상태를 기록한다')
    ap.add_argument('--ready-timeout-s', type=float, default=900.0, help='준비 대기 한도 (실제 s)')
    ap.add_argument('--clearance', type=float, default=STATION_RESERVE_M,
                    help='서버 스테이션 예약 반경 (assignment.station_clearance_m). 장면 작업끼리 이 안에 겹치면 시작하지 않는다')
    ap.add_argument('--out', help='장면 기록 json')
    a = ap.parse_args(argv)
    params = dict(item.split('=', 1) for item in a.set)
    steps = load_scene(a.scene, params)
    stations = {k: tuple(float(x) for x in v) for k, v in
                yaml.safe_load(Path(a.stations).read_text(encoding='utf-8'))['stations'].items()}
    unknown = sorted({n for s in steps for t in s.tasks for n in (t.pickup, t.dropoff)} - set(stations))
    if unknown:
        raise SystemExit(f'stations 파일에 없는 자리: {unknown}')
    conflicts = station_conflicts(steps, stations, a.clearance)
    if conflicts:
        raise SystemExit(f'같이 잡힐 수 있는 작업의 자리가 {a.clearance} m 안에 겹친다 (서버 스테이션 예약으로 할당이 '
                         f'멈춘다): {conflicts}')
    # after.near의 하역 자리는 map 좌표로 (robot_states 자세가 map 기준)
    origin = [float(v) for v in yaml.safe_load(a.map_origin)]
    c, s = math.cos(-origin[2]), math.sin(-origin[2])
    map_stations = {k: (c * (x - origin[0]) - s * (y - origin[1]), s * (x - origin[0]) + c * (y - origin[1]))
                    for k, (x, y, *_) in stations.items()}
    scene = Scene(steps, map_stations)
    status = 'interrupted'
    try:
        status = _run(a, scene, stations, [float(v) for v in yaml.safe_load(a.map_origin)])
    except KeyboardInterrupt:
        # 하네스가 녹화 시간이 끝나 SIGINT로 멈췄다. 그때까지 낸 것을 남긴다 (단계를 다 냈으면 done)
        status = 'done' if scene.done else 'interrupted'
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
