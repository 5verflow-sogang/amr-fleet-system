"""작업 생성기의 일정 만들기. rclpy를 import하지 않는다.

시나리오(config/scenarios/*.yaml)와 seed로 작업 일정을 만들거나, JSON Task 파일을 읽는다.
같은 시나리오·seed면 항상 같은 일정이 나온다 (하네스 반복 실험 재현성).
"""

from __future__ import annotations

import fnmatch
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import yaml

Pose2D = tuple[float, float, float]

# amr_msgs/msg/Task.TYPE_*, ITEM_* 와 같은 값
TASK_TYPES = {'delivery': 0, 'charge': 1, 'return': 2}
ITEMS = {'none': 0, 'small': 1, 'medium': 2, 'large': 3}


@dataclass(frozen=True)
class TaskSpec:
    """작업 1건. JSON Task 파일의 한 항목과 같은 모양이다."""

    # 시작 후 이 시각에 발행한다 (노드 시계, s)
    at_s: float
    task_id: str
    # delivery / charge / return
    type: str
    # 물품. delivery가 아니면 none
    item: str
    # 스테이션 이름. delivery가 아니면 빈 문자열
    pickup: str
    dropoff: str
    priority: int = 0
    # 요청 후 마감까지 (s). 0이면 마감 없음
    deadline_s: float = 0.0

    def __post_init__(self):
        if self.type not in TASK_TYPES:
            raise ValueError(f'{self.task_id}: 알 수 없는 type {self.type!r} (가능: {sorted(TASK_TYPES)})')
        if self.item not in ITEMS:
            raise ValueError(f'{self.task_id}: 알 수 없는 item {self.item!r} (가능: {sorted(ITEMS)})')
        if (self.type == 'delivery') != bool(self.pickup):
            raise ValueError(f'{self.task_id}: delivery만 pickup이 있다')
        if not 0 <= self.priority <= 255:
            raise ValueError(f'{self.task_id}: priority는 0~255: {self.priority}')


def load_stations(path: str) -> dict[str, Pose2D]:
    """config/stations.yaml → {이름: (x, y, yaw)} (world 좌표)."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    return {str(k): (float(v[0]), float(v[1]), float(v[2])) for k, v in (data.get('stations') or {}).items()}


def world_to_map(pose: Pose2D, map_origin_in_world: Sequence[float]) -> Pose2D:
    """world 좌표 자세를 map 좌표로. map_origin_in_world는 world에서 본 map 원점 [x, y, yaw]."""
    ox, oy, oyaw = map_origin_in_world
    dx, dy = pose[0] - ox, pose[1] - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    yaw = math.atan2(math.sin(pose[2] - oyaw), math.cos(pose[2] - oyaw))
    return (c * dx - s * dy, s * dx + c * dy, yaw)


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """z축 회전 yaw의 쿼터니언 (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def expand(patterns: Sequence[str], stations: Sequence[str]) -> list[str]:
    """* 패턴을 스테이션 이름으로 펼친다. 이름 순서를 유지하고 중복은 뺀다."""
    names = [s for s in stations if any(fnmatch.fnmatchcase(s, p) for p in patterns)]
    if not names:
        raise ValueError(f'맞는 스테이션이 없다: {list(patterns)}')
    return names


def generate(scenario: dict, stations: dict[str, Pose2D], seed: int, id_prefix: str = 'task') -> list[TaskSpec]:
    """시나리오와 seed로 작업 일정을 만든다."""
    rng = random.Random(seed)
    names = sorted(stations)
    count = int(scenario['count'])
    interval = float(scenario['interval_s'])
    arrival = scenario.get('arrival', 'fixed')
    if arrival not in ('fixed', 'poisson'):
        raise ValueError(f'arrival은 fixed 또는 poisson: {arrival}')
    if interval <= 0:
        raise ValueError(f'interval_s는 0보다 커야 한다: {interval}')

    flows = []
    for flow in scenario['flows']:
        kind = flow.get('type', 'delivery')
        pickups = expand(flow['pickup'], names) if kind == 'delivery' else ['']
        flows.append((kind, pickups, expand(flow['dropoff'], names), float(flow.get('weight', 1.0))))
    item_names = list(scenario.get('items', {'small': 1.0}))
    item_weights = [float(scenario['items'][k]) for k in item_names] if 'items' in scenario else [1.0]
    low, high = scenario.get('priority', [0, 0])
    deadline_s = float(scenario.get('deadline_s', 0.0))

    tasks = []
    t = float(scenario.get('start_delay_s', 0.0))
    for i in range(count):
        if i > 0:
            t += interval if arrival == 'fixed' else rng.expovariate(1.0 / interval)
        kind, pickups, dropoffs, _ = rng.choices(flows, weights=[f[3] for f in flows])[0]
        pickup = rng.choice(pickups)
        # 픽업과 같은 곳으로는 보내지 않는다
        dropoff = rng.choice([d for d in dropoffs if d != pickup] or dropoffs)
        item = rng.choices(item_names, weights=item_weights)[0] if kind == 'delivery' else 'none'
        tasks.append(TaskSpec(
            at_s=round(t, 6), task_id=f'{id_prefix}_s{seed}_{i + 1:04d}', type=kind, item=item,
            pickup=pickup, dropoff=dropoff, priority=rng.randint(int(low), int(high)), deadline_s=deadline_s))
    return tasks


def load_tasks_json(path: str) -> list[TaskSpec]:
    """JSON Task 파일 {"tasks": [{at_s, task_id, type, item, pickup, dropoff, priority, deadline_s}, ...]}."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    tasks = [TaskSpec(**entry) for entry in data['tasks']]
    return sorted(tasks, key=lambda t: t.at_s)


def dump_tasks_json(tasks: Sequence[TaskSpec], path: str, meta: dict | None = None) -> None:
    """일정을 JSON Task 파일로 남긴다. 같은 파일을 tasks_file로 넣으면 그대로 다시 돌릴 수 있다."""
    file = Path(path)
    file.parent.mkdir(parents=True, exist_ok=True)
    data = {'meta': meta or {}, 'tasks': [asdict(t) for t in tasks]}
    file.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def count_ready(subscribers: Iterable, server_node: str) -> int:
    """/fleet/task_requests 구독자 정보(node_name 속성) 중 일정을 시작해도 되는 수.

    server_node가 비면 구독자 전부, 아니면 그 이름의 노드만 센다. 분산 모드는 로봇마다 경매 노드
    (task_auction, 네임스페이스만 다름)가 구독하므로 이름으로 세서 로봇 수와 비교한다 (task_generator server_count).
    """
    if not server_node:
        return sum(1 for _ in subscribers)
    return sum(1 for info in subscribers if info.node_name == server_node)
