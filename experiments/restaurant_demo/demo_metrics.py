"""식당 시연 녹화 지표. rclpy 없이 돈다 (bag 읽기는 analyze.py). test_demo_metrics.py가 가짜 데이터로 시험한다.

시각은 모두 메시지 안의 시각(header.stamp, Task.created_time)이다. 녹화는 use_sim_time이라 시뮬레이션 시각(s)이다.

1-2 (붐비는 통로 돌아가기, 로봇 1대)
- 요청 → 배식대 자리(기본 buffet_2) 도착: 그 자리를 pickup 또는 dropoff로 가진 작업의 요청 시각(Task.created_time)부터
  그 자리 작업 단계(pickup이면 loading, dropoff면 unloading)를 처음 보고한 TaskStatus 시각까지.
  단계 보고가 없으면 COMPLETED 시각을 쓴다(dropoff일 때만 같은 뜻이다).
- 멈춘 횟수: 같은 구간에서 그 로봇의 /fleet/robot_states 자세 차분 속도가 stop_speed 밑으로 stop_min_s 이상 머문 구간 수.
  출발 전(처음 stop_speed를 넘기 전)과 도착 직전 정차는 따로 낸다. RobotState.velocity는 amr_bt가 채우지 않아 자세로 잰다.
- 도착하지 못한 실행: 도착 시간은 비우고, 요청부터 녹화 끝까지의 멈춘 횟수·시간을 낸다. 끝까지 멈춰 있던 구간도 멈춤으로 센다.
- 정지 비율: 멈춘 시간 합 / (출발 ~ 도착 또는 녹화 끝).

2 (회수 호출 + aging) - #306 experiments/fleet_collect_aging/collect_aging.py의 정의를 /fleet/task_table로 옮겼다
- 작업 1건: 요청 = Task.created_time(0이면 작업표에 처음 나온 시각), 할당 = assignee_id가 처음 찬 작업표 시각,
  완료 = 작업표 state가 COMPLETED(2)가 된 시각. 종류는 task_type이 TYPE_COLLECT(3)면 회수 호출.
- 1막 회수 대기: 회수 호출마다 할당 − 요청 (평균, 최대, 끝까지 미할당 수)
- 2막 기아 시간: 일반 작업이 기다리는데 일반 작업이 하나도 할당되지 않은 가장 긴 구간 (collect_aging.starvation과 같은 식)
"""

from __future__ import annotations

import math
import statistics as st
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

TYPE_COLLECT = 3
STATE_COMPLETED = 2
STATE_FAILED = 3


# ---------------------------------------------------------------- 공통 입력 형태 (bag에서 analyze.py가 만든다)
@dataclass(frozen=True)
class TableTask:
    """작업표 한 행을 시간 순으로 쌓은 것 (작업 1건)."""
    task_id: str
    task_type: int
    priority: int
    pickup: str
    dropoff: str
    created: float               # Task.created_time (s). 0이면 안 채운 것
    first_seen: float            # 작업표에 처음 나온 시각
    assign: Optional[float]      # assignee_id가 처음 찬 시각
    assignee: str
    end: Optional[float]         # COMPLETED/FAILED가 된 시각
    final_state: int

    @property
    def request(self) -> float:
        return self.created if self.created > 0 else self.first_seen

    @property
    def wait(self) -> Optional[float]:
        return None if self.assign is None else self.assign - self.request

    @property
    def kind(self) -> str:
        return 'collect' if self.task_type == TYPE_COLLECT else 'normal'


def fold_table(snapshots: Iterable[tuple[float, Sequence[dict]]]) -> dict[str, TableTask]:
    """작업표 스냅숏 [(시각, [행])] → 작업 1건마다 TableTask.

    행 = {'task_id', 'task_type', 'priority', 'pickup', 'dropoff', 'created', 'assignee', 'state'}.
    """
    out: dict[str, dict] = {}
    for t, rows in sorted(snapshots, key=lambda s: s[0]):
        for r in rows:
            d = out.setdefault(r['task_id'], {
                'task_id': r['task_id'], 'task_type': int(r['task_type']), 'priority': int(r['priority']),
                'pickup': r['pickup'], 'dropoff': r['dropoff'], 'created': float(r['created']), 'first_seen': t,
                'assign': None, 'assignee': '', 'end': None, 'final_state': int(r['state'])})
            if r['assignee'] and d['assign'] is None:
                d['assign'], d['assignee'] = t, r['assignee']
            d['final_state'] = int(r['state'])
            if d['end'] is None and int(r['state']) in (STATE_COMPLETED, STATE_FAILED):
                d['end'] = t
    return {k: TableTask(**v) for k, v in out.items()}


# ---------------------------------------------------------------- 1-2
def arrival_time(task: TableTask, statuses: Sequence[dict], station: str) -> Optional[float]:
    """task가 station에 닿은 시각. statuses = [{'t', 'task_id', 'state', 'phase'}] (시간 순 아니어도 된다)."""
    if station not in (task.pickup, task.dropoff):
        return None
    work = 'loading' if task.pickup == station else 'unloading'
    mine = sorted((s for s in statuses if s['task_id'] == task.task_id), key=lambda s: s['t'])
    for s in mine:
        if s['phase'] == work:
            return s['t']
    if task.dropoff == station:
        for s in mine:
            if int(s['state']) == STATE_COMPLETED:
                return s['t']
    return None


def speeds(states: Sequence[tuple[float, float, float]]) -> list[tuple[float, float]]:
    """[(t, x, y)] → [(구간 끝 t, 속도 m/s)] (자세 차분)."""
    pts = sorted(states)
    out = []
    for (t0, x0, y0), (t1, x1, y1) in zip(pts, pts[1:]):
        if t1 > t0:
            out.append((t1, math.hypot(x1 - x0, y1 - y0) / (t1 - t0)))
    return out


def stops(states: Sequence[tuple[float, float, float]], t0: float, t1: float,
          stop_speed: float = 0.05, stop_min_s: float = 1.0, arrived: bool = True) -> dict:
    """[t0, t1] 안의 정지 구간. 출발 전 대기는 멈춘 횟수에서 뺀다.

    arrived면 끝까지 이어진 정지는 도착 직전 정차라 빼고(end_stop_s), 아니면(도착 못 함, t1 = 녹화 끝) 멈춤으로 센다.
    반환: {'stops': 주행 중 멈춘 횟수, 'stop_s': 그 시간 합, 'stop_ratio': stop_s / (t1 - 출발),
           'start_wait_s': 출발 전 대기, 'end_stop_s', 'intervals': [(시작, 길이)]}
    """
    sp = [(t, v) for t, v in speeds(states) if t0 < t <= t1]
    depart = next((t for t, v in sp if v >= stop_speed), None)
    if depart is None:
        return {'stops': 0, 'stop_s': 0.0, 'stop_ratio': None, 'start_wait_s': round(t1 - t0, 2),
                'end_stop_s': 0.0, 'intervals': []}
    # 표본 하나(t)는 (앞 표본, t] 구간의 속도다. 정지 구간 = 느린 표본이 이어진 구간의 [첫 표본 앞 시각, 마지막 표본 시각]
    times = [t0] + [t for t, _ in sp]
    intervals, start = [], None
    for i, (t, v) in enumerate(sp):
        if t <= depart:
            continue
        if v < stop_speed:
            if start is None:
                start = times[i]
        elif start is not None:
            intervals.append((start, times[i] - start))
            start = None
    tail = start   # 끝까지 느린 구간
    end_stop = 0.0
    if tail is not None:
        if arrived:
            end_stop = t1 - tail   # 도착 직전 정차 (멈춘 횟수에 안 넣는다)
        else:
            intervals.append((tail, t1 - tail))
    real = [(s, d) for s, d in intervals if d >= stop_min_s]
    stop_s = sum(d for _, d in real)
    return {'stops': len(real), 'stop_s': round(stop_s, 2),
            'stop_ratio': round(stop_s / (t1 - depart), 3) if t1 > depart else None,
            'start_wait_s': round(depart - t0, 2), 'end_stop_s': round(end_stop, 2),
            'intervals': [(round(s, 2), round(d, 2)) for s, d in real]}


def scene12(tasks: dict[str, TableTask], statuses: Sequence[dict], robot_states: dict[str, list],
            station: str = 'buffet_2', end: Optional[float] = None, **stop_kw) -> list[dict]:
    """station에 가는 작업마다 요청 → 도착 시간과 멈춘 횟수. robot_states = {robot_id: [(t, x, y)]}.

    도착하지 못한 작업은 end(녹화 끝)까지의 멈춤을 낸다 (end가 없으면 그 로봇의 마지막 자세 시각).
    """
    out = []
    for task in sorted(tasks.values(), key=lambda k: k.request):
        if station not in (task.pickup, task.dropoff):
            continue
        arrive = arrival_time(task, statuses, station)
        row = {'task_id': task.task_id, 'robot': task.assignee, 'request_s': round(task.request, 2),
               'arrive_s': None if arrive is None else round(arrive, 2),
               'request_to_arrival_s': None if arrive is None else round(arrive - task.request, 2)}
        track = robot_states.get(task.assignee)
        if track:
            if arrive is not None:
                row.update(stops(track, task.request, arrive, **stop_kw))
            else:
                last = end if end is not None else max(t for t, _, _ in track)
                if last > task.request:
                    row.update(stops(track, task.request, last, arrived=False, **stop_kw))
        out.append(row)
    return out


# ---------------------------------------------------------------- 2
def collect_wait(tasks: dict[str, TableTask], cutoff: Optional[float] = None) -> dict:
    """1막 회수 대기: 회수 호출 할당 − 요청. cutoff 뒤 할당은 미할당으로 본다."""
    col = [k for k in tasks.values() if k.kind == 'collect' and (cutoff is None or k.request <= cutoff)]
    waits = [k.wait for k in col if k.assign is not None and (cutoff is None or k.assign <= cutoff)]
    return {'collect_n': len(col), 'collect_unassigned': len(col) - len(waits),
            'collect_wait_mean_s': round(st.mean(waits), 2) if waits else None,
            'collect_wait_max_s': round(max(waits), 2) if waits else None,
            'collect_waits_s': [round(w, 2) for w in waits]}


def starvation(tasks: dict[str, TableTask], cutoff: float,
               window: Optional[tuple[float, float]] = None) -> float:
    """2막 기아 시간 (s): 일반 작업이 기다리는데 일반 작업 할당이 없던 가장 긴 구간. window와 겹치는 구간만 (없으면 전체).

    #306 collect_aging.starvation과 같은 식: 일반 작업 할당 시각 a_i 사이(마지막은 cutoff까지)마다
    max(a_i, a_i 뒤에도 남은 일반 작업의 가장 이른 요청)부터 다음 할당까지를 잰다.
    """
    normal = [k for k in tasks.values() if k.kind == 'normal' and k.request <= cutoff]
    assigns = sorted(k.assign for k in normal if k.assign is not None and k.assign <= cutoff)
    w0, w1 = window if window else (float('-inf'), float('inf'))
    longest, previous = 0.0, float('-inf')
    for end in assigns + [cutoff]:
        waiting = [k.request for k in normal
                   if (k.assign is None or k.assign > previous) and k.request < end]
        if waiting:
            start = max(previous, min(waiting))
            if start < w1 and end > w0:
                longest = max(longest, end - start)
        previous = end
    return round(longest, 2)


def scene2(tasks: dict[str, TableTask], cutoff: float, window: Optional[tuple[float, float]] = None) -> dict:
    out = collect_wait(tasks, cutoff)
    normal = [k for k in tasks.values() if k.kind == 'normal']
    out.update({'normal_n': len(normal), 'normal_unassigned': sum(k.assign is None for k in normal),
                'completed': sum(k.final_state == STATE_COMPLETED for k in tasks.values()),
                'failed': sum(k.final_state == STATE_FAILED for k in tasks.values()),
                'starvation_s': starvation(tasks, cutoff, window),
                'assign_order': [k.task_id for k in sorted((k for k in tasks.values() if k.assign is not None),
                                                           key=lambda k: k.assign)]})
    return out
