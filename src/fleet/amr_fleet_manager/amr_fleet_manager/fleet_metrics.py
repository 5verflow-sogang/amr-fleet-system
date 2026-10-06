"""rosbag 기록 → Fleet 지표 CSV (fleet_latency.csv, response_time.csv). rclpy를 import하지 않는다.

형식과 규칙은 tools/harness/README.md "지표 CSV"·"연구 1 지연"을 따른다.
- 시각은 모두 bag에 저장된 수신 시각(실제 시간, s)이다. 메시지의 header.stamp(시뮬레이션 시간)는
  짝짓기와 latency_sim_ms에만 쓴다.
- 원본(/fleet/<topic>)과 중계본(/<robot>/fleet/<topic>)은 직렬화 바이트로 짝짓는다. 장애 주입기가 바이트를
  그대로 넘기므로 정확하고, request_id처럼 로봇마다 겹치는 번호에 헷갈리지 않는다. key 컬럼은 읽기용이다.
- 관측 마감: 실행 끝 무렵에 보낸 메시지는 아직 오는 중일 수 있어 손실로 세지 않는다.
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

ASSIGNMENTS_TOPIC = '/fleet/task_assignments'
ACKS_TOPIC = '/fleet/task_assignment_acks'
ACTION_ASSIGN = 0  # TaskAssignment.ACTION_ASSIGN

# 응답 판정 임계값 (tools/harness/README.md response_time.csv, 초안)
LINEAR_THRESHOLD = 0.01   # m/s
ANGULAR_THRESHOLD = 0.05  # rad/s

FLEET_LATENCY_HEADER = ['topic', 'robot', 'key', 'send_time', 'recv_time', 'latency_ms', 'delivered']
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
                'cutoff': round(self.latency, 6), 'ack_cutoff': round(self.ack, 6)}


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
            meta_path: Optional[Path] = None) -> Cutoff:
    """기록 → out_dir/fleet_latency.csv, response_time.csv, meta.json observation_cutoff."""
    if not records:
        raise ValueError('기록이 비어 있다')
    cutoff = Cutoff(max(r.t for r in records), delay_ms, grace_s, resend_grace_s)
    write_csv(out_dir / 'fleet_latency.csv', FLEET_LATENCY_HEADER, fleet_latency(records, cutoff, robots))
    write_csv(out_dir / 'response_time.csv', RESPONSE_TIME_HEADER, response_time(records, cutoff))
    update_meta(meta_path or out_dir / 'meta.json', cutoff)
    return cutoff
