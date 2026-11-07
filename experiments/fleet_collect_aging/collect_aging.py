"""식당 시연 2 리허설 지표 (회수 호출 + aging). rclpy 없이 돈다. analyze.py가 부르고 test_collect_aging.py가 시험한다.

실행 하나 = 하네스 결과 폴더(task_outcomes.csv, tasks.json, meta.json). 작업 1건마다
- 종류: tasks.json의 type이 collect면 회수 호출, 아니면 일반 작업
- 할당 대기: first_assign_time − request_time (s). 관측 마감(meta.json observation_cutoff.cutoff)까지 할당이 없으면 미할당
- 처리량: 마감까지 COMPLETED 수 / (마감 − 첫 요청) × 60 (건/분)
시각은 fleet_metrics가 쓴 값 그대로다. 가짜 로봇은 odom이 없어 실제 시간이다(#285: odom이 없으면 실제 시간 그대로).
"""

from __future__ import annotations

import csv
import json
import statistics as st
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


@dataclass(frozen=True)
class TaskRow:
    task_id: str
    kind: str          # collect / normal
    priority: int
    request: float     # 요청 시각 (s)
    assign: Optional[float]  # 첫 할당 시각. 마감까지 없으면 None
    completed: bool    # 마감까지 COMPLETED

    @property
    def wait(self) -> Optional[float]:
        return None if self.assign is None else self.assign - self.request


def _num(v: str) -> Optional[float]:
    return float(v) if v not in ('', None) else None


def task_rows(outcomes: Sequence[dict], schedule: Sequence[dict], cutoff: float) -> list[TaskRow]:
    """task_outcomes.csv 행 + tasks.json 일정 → TaskRow. 마감 뒤에 들어온 요청은 뺀다."""
    spec = {t['task_id']: t for t in schedule}
    rows = []
    for o in outcomes:
        request = _num(o['request_time'])
        if request is None or request > cutoff or o['task_id'] not in spec:
            continue
        assign = _num(o['first_assign_time'])
        end = _num(o['end_time'])
        s = spec[o['task_id']]
        rows.append(TaskRow(
            task_id=o['task_id'], kind='collect' if s['type'] == 'collect' else 'normal', priority=int(s['priority']),
            request=request, assign=assign if assign is not None and assign <= cutoff else None,
            completed=o['final_state'] == 'COMPLETED' and end is not None and end <= cutoff))
    return rows


def summarize(rows: Sequence[TaskRow], cutoff: float) -> dict:
    """실행 하나의 지표. 대기 평균·최대는 할당받은 작업만으로 낸다(미할당은 따로 센다)."""
    out = {}
    for kind in ('collect', 'normal'):
        mine = [r for r in rows if r.kind == kind]
        waits = [r.wait for r in mine if r.wait is not None]
        out[f'{kind}_n'] = len(mine)
        out[f'{kind}_wait_mean'] = st.mean(waits) if waits else float('nan')
        out[f'{kind}_wait_max'] = max(waits) if waits else float('nan')
        out[f'{kind}_unassigned'] = sum(r.assign is None for r in mine)
    out['unassigned'] = out['collect_unassigned'] + out['normal_unassigned']
    start = min((r.request for r in rows), default=cutoff)
    done = sum(r.completed for r in rows)
    out['completed'] = done
    out['throughput_per_min'] = done / (cutoff - start) * 60.0 if cutoff > start else float('nan')
    return out


def starvation(rows: Sequence[TaskRow], window: tuple[float, float], cutoff: float) -> float:
    """기아 시간: 일반 작업이 기다리는데 일반 작업이 하나도 할당되지 않은 가장 긴 구간 (s). window와 겹치는 구간만 본다.

    일반 작업 할당 시각 a_i 사이(마지막은 관측 마감까지)마다, 그 사이에 일반 작업이 처음 기다리기 시작한 시각
    s = max(a_i, a_i 뒤에도 남은 일반 작업의 가장 이른 요청)부터 다음 할당 a_{i+1}까지를 잰다. 구간이 window 안에서 시작하지
    않아도 겹치면 넣고, 끝을 window에서 자르지 않는다 - 몰림이 끝난 뒤 밀린 회수 호출을 처리하는 동안도 기아다.
    """
    normal = [r for r in rows if r.kind == 'normal']
    assigns = sorted(r.assign for r in normal if r.assign is not None)
    w0, w1 = window
    longest = 0.0
    previous = float('-inf')
    for end in assigns + [cutoff]:
        waiting = [r.request for r in normal if (r.assign is None or r.assign > previous) and r.request < end]
        if waiting:
            start = max(previous, min(waiting))
            if start < w1 and end > w0:
                longest = max(longest, end - start)
        previous = end
    return longest


def schedule_offset(outcomes: Sequence[dict], schedule: Sequence[dict]) -> float:
    """작업 생성기 일정 시각(at_s) → 요청 시각으로 옮기는 값 (중앙값). 일정 시작 시각이다."""
    at = {t['task_id']: float(t['at_s']) for t in schedule}
    diffs = sorted(float(o['request_time']) - at[o['task_id']] for o in outcomes
                   if o['task_id'] in at and o['request_time'])
    return diffs[len(diffs) // 2]


def run_condition(params: dict, restaurant_rate: float) -> tuple[str, float]:
    """meta.json params → (부하, aging_rate). 첫 측정 실행은 load가 없어 overload다.

    aging_rate를 직접 주지 않았으면 aging off는 0.0, on은 식당 설정 파일 값이다.
    """
    load = params.get('load') or 'overload'
    rate = params.get('aging_rate')
    if rate not in (None, ''):
        return load, float(rate)
    return load, restaurant_rate if str(params.get('aging')) == 'on' else 0.0


def load_run(run: Path) -> tuple[list[TaskRow], float, int]:
    """하네스 결과 폴더 → (TaskRow 목록, 관측 마감, seed)."""
    meta = json.loads((run / 'meta.json').read_text(encoding='utf-8'))
    cutoff = float(meta['observation_cutoff']['cutoff'])
    with (run / 'task_outcomes.csv').open(encoding='utf-8') as f:
        outcomes = list(csv.DictReader(f))
    schedule = json.loads((run / 'tasks.json').read_text(encoding='utf-8'))['tasks']
    return task_rows(outcomes, schedule, cutoff), cutoff, int(meta['seed'])


def load_run_info(run: Path) -> tuple[dict, float]:
    """(meta.json params, 일정 시작 시각). 몰림 구간(일정 시각)을 요청 시각으로 옮길 때 쓴다."""
    meta = json.loads((run / 'meta.json').read_text(encoding='utf-8'))
    with (run / 'task_outcomes.csv').open(encoding='utf-8') as f:
        outcomes = list(csv.DictReader(f))
    schedule = json.loads((run / 'tasks.json').read_text(encoding='utf-8'))['tasks']
    return meta.get('params', {}), schedule_offset(outcomes, schedule)
