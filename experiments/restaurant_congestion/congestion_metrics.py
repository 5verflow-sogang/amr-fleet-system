"""식당 시연 1-1 켬/끔 지표 (혼잡도 반영 할당). rclpy 없이 돈다. analyze.py가 부르고 test_congestion_metrics.py가 시험한다.

실행 하나 = 하네스 결과 폴더(fleet_metrics가 쓴 task_outcomes.csv, robot_states.csv, tasks.json, meta.json).
시각 열은 bag 수신 시각(실제 시간)이고, 길이는 meta.json fleet_clock.rtf를 곱해 시뮬레이션 시간으로 바꾼다(#285와 같은 기준).
완료 시간은 task_outcomes.csv에 request_sim·end_sim(#285 뒤)이 있으면 그 차이, 없으면 task_time_s × rtf다.

- 할당 대기: first_assign_time − request_time
- 완료 시간: 요청 → 완료
- 정지 시간: 작업 중(RobotState MODE_BUSY)인데 속도가 stop_speed 아래인 시간, 로봇 전체 합
- 혼잡 구역 체류: 작업 중 로봇 위치가 crowd_zones 안인 시간, 로봇 전체 합
- 다른 로봇 비율: 할당 순간 놀고 있던 로봇이 2대 이상인 작업 중, 픽업까지 직선거리가 가장 가까운 로봇이 아닌 로봇을 보낸 비율.
  1-1 장면 그 자체다. 끈 쪽(euclidean)은 0이어야 한다(동률 제외)
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import statistics as st
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

MODE_IDLE = 0
MODE_BUSY = 1
# 이보다 느리면 멈춘 것으로 본다 [m/s]. 서빙 로봇 순항(약 0.5 m/s)보다 충분히 낮게
STOP_SPEED = 0.05
# 할당 순간 로봇 상태는 이 시간 [s, 실제] 안의 마지막 RobotState로 본다 (RobotState 1~5 Hz)
STATE_FRESH_S = 3.0

Area = tuple[float, float, float, float]


@dataclass(frozen=True)
class State:
    t: float
    x: float
    y: float
    mode: int


def _num(v) -> Optional[float]:
    return float(v) if v not in ('', None) else None


def _read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _inside(area: Area, x: float, y: float) -> bool:
    return area[0] <= x <= area[1] and area[2] <= y <= area[3]


def load_states(rows: Sequence[dict]) -> dict[str, list[State]]:
    out: dict[str, list[State]] = defaultdict(list)
    for r in rows:
        out[r['robot']].append(State(float(r['time']), float(r['x']), float(r['y']), int(r['mode'])))
    for s in out.values():
        s.sort(key=lambda v: v.t)
    return dict(out)


def busy_times(states: dict[str, list[State]], crowd: Sequence[Area], rtf: float,
               stop_speed: float = STOP_SPEED) -> tuple[float, float]:
    """(정지 시간, 혼잡 구역 체류 시간) [s, 시뮬레이션]. 작업 중(BUSY)인 구간만 센다."""
    stopped = in_crowd = 0.0
    for series in states.values():
        for a, b in zip(series, series[1:]):
            dt = b.t - a.t
            if dt <= 0 or a.mode != MODE_BUSY:
                continue
            sim_dt = dt * rtf
            if math.hypot(b.x - a.x, b.y - a.y) / sim_dt < stop_speed:
                stopped += sim_dt
            if any(_inside(z, a.x, a.y) for z in crowd):
                in_crowd += sim_dt
    return stopped, in_crowd


def _state_at(series: Sequence[State], t: float) -> Optional[State]:
    i = bisect.bisect_right([s.t for s in series], t) - 1
    if i < 0 or t - series[i].t > STATE_FRESH_S:
        return None
    return series[i]


def other_robot_choices(outcomes: Sequence[dict], schedule: dict[str, dict], states: dict[str, list[State]],
                        stations: dict[str, tuple[float, float]]) -> tuple[int, int]:
    """(놀고 있던 로봇이 2대 이상이던 할당 수, 그중 가장 가까운 로봇이 아닌 로봇을 보낸 수)."""
    eligible = other = 0
    for o in outcomes:
        t = _num(o['first_assign_time'])
        chosen = (o.get('robots') or '').split(' ')[0]   # 할당받은 로봇들 (공백 구분, 처음 할당이 앞)
        spec = schedule.get(o['task_id'])
        if t is None or not chosen or spec is None:
            continue
        start = stations.get(spec.get('pickup') or spec.get('dropoff'))
        if start is None:
            continue
        # 할당 직전 상태: 고른 로봇은 아직 IDLE이어야 하고, 할당 메시지가 닿기 전 마지막 상태를 본다
        idle = {}
        for robot, series in states.items():
            s = _state_at(series, t)
            if s is not None and s.mode == MODE_IDLE:
                idle[robot] = math.dist((s.x, s.y), start)
        if len(idle) < 2 or chosen not in idle:
            continue
        eligible += 1
        nearest = min(idle.values())
        if idle[chosen] > nearest + 1e-6:
            other += 1
    return eligible, other


def run_metrics(run_dir: Path, stations: dict[str, tuple[float, float]], crowd: Sequence[Area]) -> dict:
    """실행 폴더 하나의 지표."""
    meta = json.loads((run_dir / 'meta.json').read_text(encoding='utf-8')) if (run_dir / 'meta.json').exists() else {}
    rtf = ((meta.get('fleet_clock') or {}).get('rtf')) or 1.0
    outcomes = _read_csv(run_dir / 'task_outcomes.csv')
    tasks_file = run_dir / 'tasks.json'
    schedule = {t['task_id']: t for t in json.loads(tasks_file.read_text(encoding='utf-8'))['tasks']} \
        if tasks_file.exists() else {}
    states = load_states(_read_csv(run_dir / 'robot_states.csv'))

    waits, times = [], []
    for o in outcomes:
        req, asg = _num(o['request_time']), _num(o['first_assign_time'])
        if req is not None and asg is not None:
            waits.append((asg - req) * rtf)
        if o['final_state'] != 'COMPLETED':
            continue
        if _num(o.get('request_sim')) is not None and _num(o.get('end_sim')) is not None:
            times.append(float(o['end_sim']) - float(o['request_sim']))
        elif _num(o['task_time_s']) is not None:
            times.append(float(o['task_time_s']) * rtf)
    stopped, in_crowd = busy_times(states, crowd, rtf)
    eligible, other = other_robot_choices(outcomes, schedule, states, stations)

    def mean(v):
        return st.mean(v) if v else float('nan')

    def p95(v):
        return sorted(v)[max(0, math.ceil(0.95 * len(v)) - 1)] if v else float('nan')

    return {
        'tasks': len(outcomes), 'completed': len(times), 'rtf': rtf,
        'wait_mean': mean(waits), 'wait_max': max(waits) if waits else float('nan'),
        'task_time_mean': mean(times), 'task_time_p95': p95(times),
        'stopped_s': stopped, 'in_crowd_s': in_crowd,
        'choice_eligible': eligible, 'choice_other': other,
        'choice_other_rate': other / eligible if eligible else float('nan'),
    }
