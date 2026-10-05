"""서버 재시작 번호(epoch), assignments.csv·fault_events.csv 기록, 구역 목록 읽기. rclpy를 import하지 않는다."""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Iterable

import yaml
from amr_fleet_core.traffic import ZoneSpec

from .dispatcher import AssignmentRow

ASSIGNMENTS_HEADER = ['timestamp', 'call_id', 'algorithm', 'cost_fn', 'task_id', 'robot_id', 'cost']
FAULT_EVENTS_HEADER = ['time', 'event']


def next_epoch(path: str) -> int:
    """파일에 남긴 epoch에 1을 더해 저장하고 돌려준다. 경로가 비면 항상 1.

    로봇은 FleetHeartbeat.epoch가 바뀌면 assignment_id 기준을 초기화한다. 하네스가 서버를 강제종료한 뒤
    다시 띄울 때 epoch가 같으면 로봇이 새 서버의 작은 번호를 무시하므로, 실험에서는 경로를 반드시 준다.
    """
    if not path:
        return 1
    file = Path(path)
    try:
        previous = int(file.read_text().strip())
    except (FileNotFoundError, ValueError):
        previous = 0
    epoch = previous + 1
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(f'{epoch}\n')
    return epoch


class AssignmentLog:
    """할당 호출 결과를 assignments.csv로 남긴다 (tools/harness/README.md). 호출마다 flush한다."""

    def __init__(self, path: str):
        file = Path(path)
        file.parent.mkdir(parents=True, exist_ok=True)
        self._file = file.open('w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(ASSIGNMENTS_HEADER)
        self._file.flush()

    def write(self, timestamp: float, rows: Iterable[AssignmentRow]) -> None:
        for row in rows:
            self._writer.writerow([
                f'{timestamp:.6f}', row.call_id, row.algorithm, row.cost_fn, row.task_id,
                row.robot_id, '' if row.cost is None else f'{row.cost:.6f}',
            ])
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def load_zones(path: str) -> list[ZoneSpec]:
    """config/zones.yaml 형식의 구역 목록. 경로가 비면 빈 목록 (모든 구역 요청을 REJECTED)."""
    if not path:
        return []
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    return [ZoneSpec(str(z['id']), int(z.get('capacity', 1))) for z in data.get('zones') or []]


def append_fault_event(path: str, event: str, now: float | None = None) -> float:
    """서버 강제종료·재시작 시각을 fault_events.csv에 한 줄 덧붙인다. 경로가 비면 기록하지 않는다.

    시각은 실제 시간(UNIX 초)이다. rosbag2가 기록하는 수신 시각과 같은 시계라 heartbeat 끊김과 맞춰 볼 수 있다.
    """
    now = time.time() if now is None else now
    if path:
        file = Path(path)
        file.parent.mkdir(parents=True, exist_ok=True)
        new = not file.exists() or file.stat().st_size == 0
        with file.open('a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if new:
                writer.writerow(FAULT_EVENTS_HEADER)
            writer.writerow([f'{now:.6f}', event])
    return now
