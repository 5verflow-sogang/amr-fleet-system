"""rosbag 기록 → Fleet 지표 CSV (fleet_latency.csv, response_time.csv 등). rclpy를 import하지 않는다.

형식과 규칙은 tools/harness/README.md "지표 CSV"·"연구 1 지연"을 따른다.
- 시각은 모두 bag에 저장된 수신 시각(실제 시간, s)이다. 메시지의 header.stamp(시뮬레이션 시간)는
  짝짓기와 latency_sim_ms에만 쓴다.
- 원본(/fleet/<topic>)과 중계본(/<robot>/fleet/<topic>)은 직렬화 바이트로 짝짓는다. 장애 주입기가 바이트를
  그대로 넘기므로 정확하고, request_id처럼 로봇마다 겹치는 번호에 헷갈리지 않는다. key 컬럼은 읽기용이다.
- 관측 마감: 실행 끝 무렵에 보낸 메시지는 아직 오는 중일 수 있어 손실로 세지 않는다.
- 로봇 장애: fault_events.csv의 robot_kill:<로봇>과 bag의 할당·상태로 회수 시간(recovery.csv)을 잰다.
  모든 시각이 실제 시간(bag 수신 시각, fault_events의 UNIX 초)이라 그대로 뺀다.
- 관측 시작: rosbag은 토픽을 찾는 대로 구독하므로 녹화 초반에는 원본만 녹화되고 중계본은 아직 구독 전일 수 있다.
  주기 토픽(heartbeat, task_table)의 모든 링크(토픽 × 로봇)에서 중계본이 한 번씩 녹화된 시각 − 주입 지연보다
  먼저 보낸 메시지는 판정하지 않는다.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence

# 장애 주입기 중계 대상 토픽 → key (tools/harness/README.md fleet_latency.csv)
RELAYED_KEYS: dict[str, Callable[[Any], str]] = {
    'task_assignments': lambda m: str(m.assignment_id),
    'heartbeat': lambda m: f'{m.epoch}:{m.seq}',
    'zone_grants': lambda m: str(m.request_id),
    'task_table': lambda m: f'{m.epoch}:{m.seq}',
}
RELAY_TOPIC = re.compile(r'^/(?P<robot>[^/]+)/fleet/(?P<topic>' + '|'.join(RELAYED_KEYS) + r')$')
ODOM_TOPIC = re.compile(r'^/(?P<robot>[^/]+)/odom$')
# 서버가 일정한 주기로 내는 토픽. 관측 시작(녹화기가 중계본을 구독한 시각)은 이 토픽으로만 잰다.
# 할당 명령처럼 드문 토픽은 첫 메시지가 늦게 나와, 그 시각이 구독 시각이 아니라 첫 메시지 시각이 된다
PERIODIC_TOPICS = ('heartbeat', 'task_table')

ASSIGNMENTS_TOPIC = '/fleet/task_assignments'
ACKS_TOPIC = '/fleet/task_assignment_acks'
REQUESTS_TOPIC = '/fleet/task_requests'
STATUS_TOPIC = '/fleet/task_status'
STATES_TOPIC = '/fleet/robot_states'
# TaskStatus.STATE_*
STATE_COMPLETED = 2
STATE_FAILED = 3
STATE_NAMES = {STATE_COMPLETED: 'COMPLETED', STATE_FAILED: 'FAILED'}
ACTION_ASSIGN = 0  # TaskAssignment.ACTION_ASSIGN

# 응답 판정 임계값 (tools/harness/README.md response_time.csv, 초안)
LINEAR_THRESHOLD = 0.01   # m/s
ANGULAR_THRESHOLD = 0.05  # rad/s

FLEET_LATENCY_HEADER = ['topic', 'robot', 'key', 'send_time', 'recv_time', 'latency_ms', 'delivered']
TASK_OUTCOMES_HEADER = ['task_id', 'request_time', 'first_assign_time', 'robots', 'final_state', 'end_time']
RECOVERY_HEADER = ['robot', 'kill_time', 'task_id', 'reassign_time', 'reassigned_robot', 'recovery_s', 'completed']
ROBOT_STATES_HEADER = ['time', 'robot', 'x', 'y', 'mode', 'task_id']
RESPONSE_TIME_HEADER = ['cmd_time', 'response_time', 'latency_ms', 'robot', 'task_id', 'latency_sim_ms',
                        'assignment_id', 'ack_time', 'ack_rtt_ms', 'resends']


@dataclass(frozen=True)
class Record:
    """bag의 메시지 1건."""

    topic: str
    # bag 수신 시각 (실제 시간, s)
    t: float
    # 직렬화 바이트 (원본·중계본 짝짓기용)
    data: bytes
    # 역직렬화한 메시지 (필드 읽기용)
    msg: Any


@dataclass(frozen=True)
class Cutoff:
    end: float
    delay_ms: float
    grace_s: float
    resend_grace_s: float
    # 관측 시작. 이 시각 전에 보낸 메시지는 지연·손실을 판정하지 않는다 (녹화 초반 구독 지연). None이면 처음부터
    start: Optional[float] = None

    @property
    def latency(self) -> float:
        """이 시각 뒤에 보낸 메시지는 지연·손실을 판정하지 않는다."""
        return self.end - self.delay_ms / 1000.0 - self.grace_s

    @property
    def ack(self) -> float:
        """이 시각 뒤에 보낸 명령은 Ack·응답을 판정하지 않는다 (재전송 시간까지 기다려야 하므로)."""
        return self.latency - self.resend_grace_s

    def as_meta(self) -> dict:
        return {'end': round(self.end, 6), 'delay_ms': self.delay_ms, 'grace_s': self.grace_s,
                'resend_grace_s': self.resend_grace_s,
                'cutoff': round(self.latency, 6), 'ack_cutoff': round(self.ack, 6),
                'start': None if self.start is None else round(self.start, 6)}


def robot_ids(count: int) -> list[str]:
    """amr_01부터 count대의 이름 (가짜 로봇·장애 주입기·system.launch와 같은 규칙)."""
    if count < 1:
        raise ValueError(f'로봇 수는 1 이상: {count}')
    return [f'amr_{i:02d}' for i in range(1, count + 1)]


def observation_start(records: Sequence[Record], delay_ms: float) -> Optional[float]:
    """주기 토픽의 모든 링크(토픽 × 로봇)에서 중계본이 처음 녹화된 시각 중 가장 늦은 것 − 주입 지연.

    그 뒤에 보낸 원본은 중계본이 녹화될 수 있었던 메시지다 (녹화기가 그 전에 구독을 마쳤다).
    링크마다 따로 자르지 않고 실행 하나에 한 시각을 쓴다. 링크마다 자르면 링크마다 초반 손실이 빠져 손실률이 낮게 나온다.
    """
    first: dict[tuple[str, str], float] = {}
    for r in records:
        m = RELAY_TOPIC.match(r.topic)
        if m and m['topic'] in PERIODIC_TOPICS:
            first.setdefault((m['topic'], m['robot']), r.t)
    return max(first.values()) - delay_ms / 1000.0 if first else None


def _digest(data: bytes) -> bytes:
    return hashlib.blake2b(data, digest_size=16).digest()


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _fmt(value: Optional[float], digits: int = 6) -> str:
    return '' if value is None else f'{value:.{digits}f}'


def fleet_latency(records: Sequence[Record], cutoff: Cutoff, robots: Optional[Iterable[str]] = None) -> list[list]:
    """fleet_latency.csv 행. 원본 메시지 1건 × 로봇 1대마다 1행.

    로봇 목록을 주지 않으면 bag에 중계본 토픽이 있는 로봇 전부다. 같은 바이트가 여러 번 오면(재전송) 첫 도착만 쓴다.
    """
    originals: dict[str, dict[bytes, tuple[float, str]]] = defaultdict(dict)
    relayed: dict[tuple[str, str, bytes], float] = {}
    seen_robots = set()
    for r in records:
        if r.topic.startswith('/fleet/') and r.topic[len('/fleet/'):] in RELAYED_KEYS:
            topic = r.topic[len('/fleet/'):]
            originals[topic].setdefault(_digest(r.data), (r.t, RELAYED_KEYS[topic](r.msg)))
            continue
        m = RELAY_TOPIC.match(r.topic)
        if m:
            seen_robots.add(m['robot'])
            relayed.setdefault((m['topic'], m['robot'], _digest(r.data)), r.t)

    rows = []
    for topic in RELAYED_KEYS:
        for digest, (send, key) in sorted(originals[topic].items(), key=lambda kv: kv[1][0]):
            for robot in sorted(robots if robots is not None else seen_robots):
                recv = relayed.get((topic, robot, digest))
                latency = None if recv is None else (recv - send) * 1000.0
                if send > cutoff.latency:
                    delivered = ''  # 아직 오는 중일 수 있어 판정하지 않는다
                elif cutoff.start is not None and send < cutoff.start:
                    delivered = ''  # 녹화기가 중계본을 아직 구독하기 전일 수 있다
                else:
                    delivered = '0' if recv is None else '1'
                rows.append([topic, robot, key, _fmt(send), _fmt(recv), _fmt(latency, 3), delivered])
    return rows


def _moving(odom) -> bool:
    twist = odom.twist.twist
    return (math.hypot(twist.linear.x, twist.linear.y) >= LINEAR_THRESHOLD
            or abs(twist.angular.z) >= ANGULAR_THRESHOLD)


def response_time(records: Sequence[Record], cutoff: Cutoff) -> list[list]:
    """response_time.csv 행. 할당(ASSIGN) 명령 1건마다 1행. 관측 마감 뒤 명령은 행을 만들지 않는다.

    - 명령은 (robot_id, assignment_id, header.stamp)로 구분한다. 서버가 재시작하면 번호가 1부터 다시 시작하지만
      stamp가 달라 섞이지 않는다. Ack는 assignment_stamp로 같은 명령을 되돌려 준다.
    - response_time은 /<robot>/odom이 bag에 있을 때만 채운다 (가짜 로봇은 odom이 없다).
    """
    commands: dict[tuple[str, int, int], dict] = {}
    acks: dict[tuple[str, int, int], float] = {}
    odom: dict[str, list[Record]] = defaultdict(list)
    for r in records:
        if r.topic == ASSIGNMENTS_TOPIC:
            if r.msg.action != ACTION_ASSIGN:
                continue
            ident = (r.msg.robot_id, r.msg.assignment_id, _stamp_ns(r.msg.header.stamp))
            cmd = commands.setdefault(ident, {'t': r.t, 'count': 0, 'msg': r.msg})
            cmd['count'] += 1
        elif r.topic == ACKS_TOPIC:
            ident = (r.msg.robot_id, r.msg.assignment_id, _stamp_ns(r.msg.assignment_stamp))
            acks.setdefault(ident, r.t)
        else:
            m = ODOM_TOPIC.match(r.topic)
            if m:
                odom[m['robot']].append(r)
    for series in odom.values():
        series.sort(key=lambda r: r.t)
    odom_times = {robot: [r.t for r in series] for robot, series in odom.items()}

    rows = []
    for (robot, assignment_id, stamp_ns), cmd in sorted(commands.items(), key=lambda kv: kv[1]['t']):
        cmd_time = cmd['t']
        if cmd_time > cutoff.ack:
            continue
        response = latency = latency_sim = None
        series = odom.get(robot, [])
        start = bisect.bisect_left(odom_times.get(robot, []), cmd_time)
        for r in series[start:]:
            if _moving(r.msg):
                response = r.t
                latency = (response - cmd_time) * 1000.0
                latency_sim = (_stamp_ns(r.msg.header.stamp) - stamp_ns) / 1e6
                break
        ack = acks.get((robot, assignment_id, stamp_ns))
        ack_rtt = None if ack is None else (ack - cmd_time) * 1000.0
        rows.append([
            _fmt(cmd_time), _fmt(response), _fmt(latency, 3), robot, cmd['msg'].task.task_id,
            _fmt(latency_sim, 3), assignment_id, _fmt(ack), _fmt(ack_rtt, 3), cmd['count'] - 1,
        ])
    return rows


def _task_events(records: Sequence[Record]):
    """작업별 요청 시각, 할당 [(시각, 로봇)], 끝 보고 [(시각, 로봇, 상태)]. 재전송은 첫 발송만 센다."""
    requests: dict[str, float] = {}
    assigns: dict[str, list[tuple[float, str]]] = defaultdict(list)
    seen_cmds = set()
    finishes: dict[str, list[tuple[float, str, int]]] = defaultdict(list)
    for r in records:
        if r.topic == REQUESTS_TOPIC:
            requests.setdefault(r.msg.task_id, r.t)
        elif r.topic == ASSIGNMENTS_TOPIC and r.msg.action == ACTION_ASSIGN:
            ident = (r.msg.robot_id, r.msg.assignment_id, _stamp_ns(r.msg.header.stamp))
            if ident not in seen_cmds:
                seen_cmds.add(ident)
                assigns[r.msg.task.task_id].append((r.t, r.msg.robot_id))
        elif r.topic == STATUS_TOPIC and r.msg.state in STATE_NAMES:
            finishes[r.msg.task_id].append((r.t, r.msg.robot_id, r.msg.state))
    for series in (*assigns.values(), *finishes.values()):
        series.sort()
    return requests, assigns, finishes


def task_outcomes(records: Sequence[Record]) -> list[list]:
    """task_outcomes.csv 행. 작업 1건마다 1행.

    final_state: 마지막 끝 보고(COMPLETED·FAILED)가 그 작업의 마지막 할당 뒤에 왔으면 그 상태, 아니면 빈칸
    (실행이 끝날 때 아직 하는 중이거나, FAILED 뒤 다시 할당됐거나, 맡은 로봇이 죽었다).
    """
    requests, assigns, finishes = _task_events(records)
    rows = []
    for task_id in sorted(set(requests) | set(assigns), key=lambda t: (requests.get(t, math.inf), t)):
        series = assigns.get(task_id, [])
        last_assign = series[-1] if series else None
        final = end = None
        for t, robot, state in finishes.get(task_id, []):
            if last_assign is not None and t >= last_assign[0] and robot == last_assign[1]:
                final, end = STATE_NAMES[state], t
                break
        rows.append([task_id, _fmt(requests.get(task_id)), _fmt(series[0][0] if series else None),
                     ' '.join(robot for _, robot in series), final or '', _fmt(end)])
    return rows


def read_kill_events(path: Path) -> list[tuple[float, str]]:
    """fault_events.csv의 로봇 강제종료 [(시각, 로봇)]. 파일이 없으면 빈 목록."""
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return [(float(row['time']), row['event'].split(':', 1)[1])
                for row in csv.DictReader(f) if row['event'].startswith('robot_kill:')]


def recovery(records: Sequence[Record], kills: Sequence[tuple[float, str]]) -> list[list]:
    """recovery.csv 행. 로봇 강제종료 1건 × 그 로봇이 하던 작업 1건마다 1행.

    하던 작업: 죽기 전 마지막 할당이 그 로봇이고, 죽기 전에 그 로봇의 끝 보고가 없다.
    recovery_s = 다른 로봇에 처음 다시 할당된 시각 − 죽은 시각 (서버 회수 대기 + 할당). 다시 할당되지 않았으면 빈칸.
    completed = 죽은 뒤 누구든 COMPLETED를 보고했으면 1. 하던 작업이 없었으면(놀던 로봇) task_id가 빈 행 1개.
    """
    _, assigns, finishes = _task_events(records)
    rows = []
    for kill_time, robot in kills:
        busy = []
        for task_id, series in assigns.items():
            before = [(t, r) for t, r in series if t <= kill_time]
            if not before or before[-1][1] != robot:
                continue
            assigned_at = before[-1][0]
            if any(assigned_at <= t <= kill_time and r == robot for t, r, _ in finishes.get(task_id, [])):
                continue
            busy.append(task_id)
        if not busy:
            rows.append([robot, _fmt(kill_time), '', '', '', '', ''])
        for task_id in sorted(busy):
            again = next(((t, r) for t, r in assigns[task_id] if t > kill_time and r != robot), None)
            done = any(t > kill_time and state == STATE_COMPLETED for t, _, state in finishes.get(task_id, []))
            rows.append([robot, _fmt(kill_time), task_id, _fmt(again[0] if again else None),
                         again[1] if again else '', _fmt(again[0] - kill_time if again else None, 3),
                         '1' if done else '0'])
    return rows


def robot_states(records: Sequence[Record]) -> list[list]:
    """robot_states.csv 행. RobotState 1건마다 1행 (그림용: fleet_plot의 궤적·작업 타임라인)."""
    rows = []
    for r in sorted((r for r in records if r.topic == STATES_TOPIC), key=lambda r: r.t):
        p = r.msg.pose.position
        rows.append([_fmt(r.t), r.msg.robot_id, _fmt(p.x, 3), _fmt(p.y, 3), r.msg.mode, r.msg.current_task_id])
    return rows


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def update_meta(path: Path, cutoff: Cutoff) -> None:
    """meta.json의 observation_cutoff를 채운다. 파일이 없으면 이 키만 있는 파일을 만든다."""
    meta = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    meta['observation_cutoff'] = cutoff.as_meta()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def convert(records: Sequence[Record], out_dir: Path, delay_ms: float, grace_s: float = 1.0,
            resend_grace_s: float = 5.0, robots: Optional[Iterable[str]] = None,
            meta_path: Optional[Path] = None, fault_events: Optional[Path] = None) -> Cutoff:
    """기록 → out_dir/fleet_latency.csv, response_time.csv, task_outcomes.csv, robot_states.csv,
    meta.json observation_cutoff.

    fault_events(기본 out_dir/fault_events.csv)에 로봇 강제종료가 있으면 recovery.csv도 쓴다.
    """
    if not records:
        raise ValueError('기록이 비어 있다')
    cutoff = Cutoff(max(r.t for r in records), delay_ms, grace_s, resend_grace_s,
                    start=observation_start(records, delay_ms))
    write_csv(out_dir / 'fleet_latency.csv', FLEET_LATENCY_HEADER, fleet_latency(records, cutoff, robots))
    write_csv(out_dir / 'response_time.csv', RESPONSE_TIME_HEADER, response_time(records, cutoff))
    write_csv(out_dir / 'task_outcomes.csv', TASK_OUTCOMES_HEADER, task_outcomes(records))
    write_csv(out_dir / 'robot_states.csv', ROBOT_STATES_HEADER, robot_states(records))
    kills = read_kill_events(fault_events or out_dir / 'fault_events.csv')
    if kills:
        write_csv(out_dir / 'recovery.csv', RECOVERY_HEADER, recovery(records, kills))
    update_meta(meta_path or out_dir / 'meta.json', cutoff)
    return cutoff
