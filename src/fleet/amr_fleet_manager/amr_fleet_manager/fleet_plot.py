"""하네스 실행 폴더의 Fleet 결과 → 작업 타임라인 그림 (plots/timeline.png). rclpy를 import하지 않는다.

    ros2 run amr_fleet_manager fleet_plot <실행 폴더>
    # ROS 없이 (matplotlib만 있으면 된다). GCP 결과를 받아 맥에서 다시 그릴 때
    PYTHONPATH=src/fleet/amr_fleet_manager python3 -m amr_fleet_manager.fleet_plot <실행 폴더>

읽는 파일. 없는 파일은 그 부분만 빼고 그린다.
- robot_states.csv (fleet_metrics): 로봇별 작업 구간, 상태가 끊긴 구간
- fault_events.csv (launch): 로봇·서버 강제종료 시각
- recovery.csv (fleet_metrics): 죽은 로봇의 작업이 다른 로봇에 다시 할당된 화살표

시각은 모두 실제 시간(UNIX 초)이라 그대로 맞춘다. 그림 글자는 글꼴 문제를 피하려고 영어로 쓴다.
"""

from __future__ import annotations

import argparse
import csv
import sys
import traceback
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


# 가짜 로봇 state_rate_hz 2 Hz. 이보다 오래 RobotState가 없으면 그 로봇이 죽었거나 끊긴 것으로 그린다
SILENT_GAP_S = 3.0
# RobotState.MODE_*
MODE_ERROR = 3
MODE_ESTOP = 4


@dataclass(frozen=True)
class State:
    t: float
    robot: str
    x: float
    y: float
    mode: int
    task_id: str


@dataclass(frozen=True)
class Segment:
    """로봇 1대의 시간 구간. kind: task(작업 중), fault(ERROR·E-Stop), silent(상태 끊김)."""

    robot: str
    start: float
    end: float
    kind: str
    task_id: str = ''


def read_states(path: Path) -> list[State]:
    if not path.exists():
        return []
    states = []
    with path.open(newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            try:
                states.append(State(float(r['time']), r['robot'], float(r['x']), float(r['y']), int(r['mode']),
                                    r['task_id'] or ''))
            except (TypeError, ValueError):
                # 잘린 줄(쓰는 도중 강제종료 등)은 건너뛴다. 나머지로 그린다
                continue
    return sorted(states, key=lambda s: s.t)


def read_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def by_robot(states: Sequence[State]) -> dict[str, list[State]]:
    series: dict[str, list[State]] = defaultdict(list)
    for s in states:
        series[s.robot].append(s)
    return dict(sorted(series.items()))


def segments(states: Sequence[State], gap_s: float = SILENT_GAP_S) -> list[Segment]:
    """로봇별로 같은 값이 이어지는 구간. 상태가 gap_s보다 오래 끊기면 silent 구간을 넣고 앞 구간을 끊는다.
    실행 끝(모든 로봇 중 마지막 상태)까지 돌아오지 않은 로봇도 silent로 끝난다 (죽고 재시작하지 않은 로봇).

    task 구간은 current_task_id가 같은 동안, fault 구간은 mode가 ERROR·E-Stop인 동안이다.
    구간 끝은 다음 값으로 바뀐 첫 상태의 시각이다 (상태 주기만큼의 오차).
    """
    result = []
    t_end = max((s.t for s in states), default=0.0)
    for robot, series in by_robot(states).items():
        open_: dict[str, tuple[float, str]] = {}

        def close(kind: str, t: float) -> None:
            if kind in open_:
                start, task_id = open_.pop(kind)
                result.append(Segment(robot, start, t, kind, task_id))

        prev: Optional[State] = None
        for s in series:
            if prev is not None and s.t - prev.t > gap_s:
                for kind in list(open_):
                    close(kind, prev.t)
                result.append(Segment(robot, prev.t, s.t, 'silent'))
            if open_.get('task', (0.0, s.task_id))[1] != s.task_id:
                close('task', s.t)
            if s.task_id and 'task' not in open_:
                open_['task'] = (s.t, s.task_id)
            if s.mode in (MODE_ERROR, MODE_ESTOP):
                open_.setdefault('fault', (s.t, ''))
            else:
                close('fault', s.t)
            prev = s
        for kind in list(open_):
            close(kind, series[-1].t)
        if t_end - series[-1].t > gap_s:
            result.append(Segment(robot, series[-1].t, t_end, 'silent'))
    return sorted(result, key=lambda g: (g.robot, g.start, g.kind))


def read_events(path: Path) -> list[tuple[float, str, str]]:
    """fault_events.csv → [(시각, 사건, 로봇)]. 서버 사건(kill·restart)은 로봇이 빈 문자열."""
    events = []
    for row in read_rows(path):
        kind, _, robot = row['event'].partition(':')
        events.append((float(row['time']), kind, robot))
    return events


def plot_timeline(ax, segs: Sequence[Segment], events, recovery_rows, robots: Sequence[str], t0: float) -> None:
    import matplotlib
    palette = matplotlib.colormaps['tab20']
    rows = {robot: i for i, robot in enumerate(robots)}
    task_colors: dict[str, int] = {}
    for g in segs:
        if g.robot not in rows:
            continue
        y = rows[g.robot]
        x, w = g.start - t0, g.end - g.start
        if g.kind == 'task':
            color = palette(task_colors.setdefault(g.task_id, len(task_colors)) % 20)
            ax.broken_barh([(x, w)], (y - 0.35, 0.7), facecolors=color, edgecolors='white', linewidth=0.5)
            if w > 12:
                ax.text(x + w / 2, y, g.task_id, ha='center', va='center', fontsize=6, clip_on=True)
        elif g.kind == 'fault':
            ax.broken_barh([(x, w)], (y - 0.45, 0.15), facecolors='tab:red')
        else:
            ax.broken_barh([(x, w)], (y - 0.35, 0.7), facecolors='none', edgecolors='0.5', hatch='///',
                           linewidth=0.5)
    for t, kind, robot in events:
        x = t - t0
        if robot in rows:
            y = rows[robot]
            ax.vlines(x, y - 0.5, y + 0.5, colors='tab:red' if kind.endswith('kill') else 'tab:green', linewidth=2)
        elif not robot:
            ax.axvline(x, color='black' if kind == 'kill' else 'tab:green', linestyle='--', linewidth=1)
            # y축을 뒤집어 그리므로 -0.65가 맨 위다. 강제종료·재시작이 가까워도 겹치지 않게 선 양쪽에 쓴다
            ax.text(x, -0.65, f'server {kind} ' if kind == 'kill' else f' server {kind}', fontsize=7, va='top',
                    ha='right' if kind == 'kill' else 'left')
    for r in recovery_rows:
        if not r.get('reassign_time') or r['robot'] not in rows or r['reassigned_robot'] not in rows:
            continue
        start = (float(r['kill_time']) - t0, rows[r['robot']])
        end = (float(r['reassign_time']) - t0, rows[r['reassigned_robot']])
        ax.annotate('', xy=end, xytext=start, arrowprops={'arrowstyle': '->', 'color': 'tab:red', 'linewidth': 1.2})
        ax.text(end[0], end[1] + 0.4, f"{r['task_id']} +{float(r['recovery_s']):.1f}s", fontsize=7,
                color='tab:red')
    ax.set_yticks(range(len(robots)), robots)
    ax.set_ylim(-0.7, len(robots) - 0.3)
    ax.invert_yaxis()
    ax.set_xlabel('time since first robot state (s)')
    ax.set_title('Task timeline  (bar = task, hatched = no RobotState, red line = kill, red arrow = reassign)',
                 fontsize=9)
    ax.grid(axis='x', alpha=0.3)


def render(run_dir: Path, out_dir: Path) -> list[Path]:
    """그림을 out_dir에 쓰고 쓴 파일 목록을 돌려준다. robot_states.csv가 없거나 비면 아무것도 쓰지 않는다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    states = read_states(run_dir / 'robot_states.csv')
    if not states:
        return []
    robots = list(by_robot(states))
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(14, 1.0 + 0.6 * len(robots)))
    plot_timeline(ax, segments(states), read_events(run_dir / 'fault_events.csv'),
                  read_rows(run_dir / 'recovery.csv'), robots, states[0].t)
    fig.tight_layout()
    path = out_dir / 'timeline.png'
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return [path]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Fleet 실행 폴더 → plots/timeline.png')
    parser.add_argument('run_dir', help='하네스 실행 폴더 (robot_states.csv가 있는 곳)')
    parser.add_argument('--out', help='그림 폴더 (기본: <실행 폴더>/plots)')
    args = parser.parse_args(argv)

    run_dir = Path(args.run_dir)
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        # 그림은 결과가 아니라 보기용이다. 이미지에 matplotlib이 없어도 실행을 실패로 만들지 않는다
        print('경고: matplotlib이 없어 그림을 건너뛴다 (python3-matplotlib 설치)', file=sys.stderr)
        return 0
    try:
        written = render(run_dir, Path(args.out) if args.out else run_dir / 'plots')
    except Exception:  # noqa: BLE001 - 그림 실패가 실행을 실패로 만들지 않게 한다 (plot_run.py와 같은 정책)
        print('fleet_plot 실패 (실행 결과에는 영향 없음):', file=sys.stderr)
        traceback.print_exc()
        return 0
    if not written:
        print(f'경고: {run_dir}/robot_states.csv가 없거나 비어 그림을 건너뛴다', file=sys.stderr)
    for path in written:
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
