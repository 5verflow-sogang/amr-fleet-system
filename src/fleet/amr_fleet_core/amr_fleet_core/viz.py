"""알고리즘 상태를 그림 한 장으로 (디버깅·리뷰용). matplotlib Axes를 받아 그리기만 한다.

라이브러리 본체는 matplotlib에 의존하지 않는다. 이 모듈을 import해도 matplotlib을 부르지 않고,
그림 함수의 인자로 받은 ax만 쓴다. 테스트에서는 FLEET_VIZ=<폴더>일 때 test/conftest.py의 viz fixture가 부른다.

    from amr_fleet_core.viz import draw_assignment
    fig, ax = plt.subplots()
    draw_assignment(ax, robots, tasks, ALGORITHMS['hungarian'](tasks, robots))

그림 글자는 한글 글꼴 문제를 피하려고 영어로 쓴다.
"""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Optional, Sequence

from .assignment import AssignmentResult, CostFn, RobotInfo, TaskInfo, euclidean_cost
from .deadlock import find_deadlocks
from .traffic import GRANT_REVOKED, Decision, ZoneManager

ROBOT_COLOR = '#1f77b4'
TASK_COLOR = '#2ca02c'
MUTED = '#9a9a95'
ALERT = '#d62728'
WARN = '#ff7f0e'
# 할당 화살표의 휨 정도 (matplotlib arc3 rad)
ARC = 0.3


def draw_assignment(ax, robots: Sequence[RobotInfo], tasks: Sequence[TaskInfo], result: AssignmentResult,
                    title: str = '', cost: CostFn = euclidean_cost) -> None:
    """로봇(원)과 작업 시작 지점(사각형), 할당(로봇 → 작업 곡선 화살표)과 비용. 픽업이 있으면 픽업 → 목적지를 흐린 점선으로.

    할당 못 한 작업은 회색 X, 후보에서 빠진 로봇(로봇 목록에 있지만 아무 작업도 안 받음)은 그대로 둔다.
    """
    by_robot = {r.robot_id: r for r in robots}
    by_task = {t.task_id: t for t in tasks}
    for t in tasks:
        if t.pickup_xy is not None:
            ax.annotate('', xy=t.dropoff_xy, xytext=t.pickup_xy,
                        arrowprops={'arrowstyle': '->', 'color': MUTED, 'linestyle': ':', 'linewidth': 1})
    for task_id, robot_id in result.assignments.items():
        r, t = by_robot[robot_id], by_task[task_id]
        # 곡선 화살표: 로봇·작업이 한 줄에 놓여도 할당 선이 겹치지 않는다
        ax.annotate('', xy=t.start_xy, xytext=r.xy, zorder=2,
                    arrowprops={'arrowstyle': '-|>', 'color': ROBOT_COLOR, 'linewidth': 2, 'shrinkA': 7,
                                'shrinkB': 7, 'connectionstyle': f'arc3,rad={ARC}'})
        # 곡선(2차 베지어)의 가운데 = 직선 가운데 + ARC/2 × (dy, -dx)
        dx, dy = t.start_xy[0] - r.xy[0], t.start_xy[1] - r.xy[1]
        mx, my = r.xy[0] + dx / 2 + ARC / 2 * dy, r.xy[1] + dy / 2 - ARC / 2 * dx
        ax.text(mx, my, f'{cost(r, t):.1f}', fontsize=8, ha='center', va='center', color=ROBOT_COLOR,
                bbox={'boxstyle': 'round,pad=0.15', 'fc': 'white', 'ec': 'none', 'alpha': 0.85})
    for r in robots:
        ax.scatter(*r.xy, s=160, color=ROBOT_COLOR, zorder=3, edgecolors='white')
        extra = f'\nload={r.load}' if r.load else ''
        ax.annotate(f'{r.robot_id}{extra}', r.xy, xytext=(0, -14), textcoords='offset points', ha='center',
                    va='top', fontsize=8)
    unassigned = set(result.unassigned)
    for t in tasks:
        missed = t.task_id in unassigned
        ax.scatter(*t.start_xy, s=110, marker='X' if missed else 's', color=MUTED if missed else TASK_COLOR,
                   zorder=3, edgecolors='white')
        ax.annotate(f'{t.task_id} p={t.priority}', t.start_xy, xytext=(0, 9), textcoords='offset points',
                    ha='center', va='bottom', fontsize=8, color=MUTED if missed else 'black')
    ax.set_title(f'{title}  total cost={result.total_cost:.2f}'.strip()
                 + (f'  unassigned={result.unassigned}' if result.unassigned else ''), fontsize=9)
    # 범위를 직접 잡는다. 점 위아래에 이름표를 쓰므로 여백을 1.5 m 이상 둔다
    points = [r.xy for r in robots] + [p for t in tasks for p in (t.start_xy, t.dropoff_xy)]
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    pad = max(1.5, 0.12 * max(max(xs) - min(xs), max(ys) - min(ys)))
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=0.3)


def draw_zones(ax, manager: ZoneManager, decisions: Iterable[Decision] = (), title: str = '') -> None:
    """구역마다 상자 하나. 안에 점유 로봇, 아래에 대기열(허가 순서대로). 회수(REVOKED)된 점유는 주황 빗금.

    회수 여부는 ZoneManager가 밖으로 내지 않으므로, 그때까지 받은 Decision에서 GRANT_REVOKED를 찾아 표시한다.
    """
    revoked = {(d.robot_id, d.zone_id) for d in decisions if d.grant == GRANT_REVOKED}
    zone_ids = manager.zone_ids
    for i, zone_id in enumerate(zone_ids):
        x = i * 2.2
        ax.add_patch(_rect((x, 0), 2.0, 1.2, fc='#fff3e6', ec=WARN))
        ax.text(x + 1.0, 1.3, zone_id, ha='center', va='bottom', fontsize=9, fontweight='bold')
        for j, robot_id in enumerate(manager.holders(zone_id)):
            hit = (robot_id, zone_id) in revoked
            ax.add_patch(_rect((x + 0.15, 0.75 - j * 0.45), 1.7, 0.35, fc='#ffd9b3' if hit else '#d6ecff',
                               ec=WARN if hit else ROBOT_COLOR, hatch='///' if hit else None))
            ax.text(x + 1.0, 0.925 - j * 0.45, robot_id + (' REVOKED' if hit else ''), ha='center', va='center',
                    fontsize=8, color=ALERT if hit else 'black')
        for j, robot_id in enumerate(manager.queue(zone_id)):
            ax.text(x + 1.0, -0.3 - j * 0.35, f'{j + 1}. {robot_id}', ha='center', va='center', fontsize=8,
                    color=MUTED)
    if any(manager.queue(z) for z in zone_ids):
        ax.text(-0.1, -0.3, 'queue', ha='right', va='center', fontsize=8, color=MUTED, style='italic')
    ax.set_xlim(-0.8, max(len(zone_ids), 1) * 2.2)
    deepest = max((len(manager.queue(z)) for z in zone_ids), default=0)
    ax.set_ylim(-0.6 - deepest * 0.35, 1.8)
    ax.set_title(title or 'Zones (box = holder, below = queue in grant order)', fontsize=9)
    ax.axis('off')


def draw_wait_for(ax, edges: Mapping[str, Mapping[str, str] | Iterable[str]], victim: Optional[str] = None,
                  title: str = '') -> None:
    """wait-for 그래프. 로봇을 원 위에 놓고 A → B(A가 B를 기다림) 화살표. 교착 순환은 빨강, 물러날 로봇은 빨간 테두리.

    edges는 ZoneManager.wait_for() 모양({A: {B: 구역}}, 화살표에 구역 이름)이나 find_deadlocks 입력({A: [B]}) 둘 다 받는다.
    """
    pairs: dict[tuple[str, str], str] = {}
    for a, targets in edges.items():
        for b in targets:
            pairs[(a, b)] = targets[b] if isinstance(targets, Mapping) else ''
    nodes = sorted({n for pair in pairs for n in pair} | set(edges))
    cycles = find_deadlocks({a: list(t) for a, t in edges.items()})
    in_cycle = {(g[i], g[j]) for g in cycles for i in range(len(g)) for j in range(len(g))}
    pos = {n: (math.cos(math.pi / 2 - 2 * math.pi * i / max(len(nodes), 1)),
               math.sin(math.pi / 2 - 2 * math.pi * i / max(len(nodes), 1))) for i, n in enumerate(nodes)}
    for (a, b), zone in pairs.items():
        hot = (a, b) in in_cycle
        both = (b, a) in pairs
        ax.annotate('', xy=pos[b], xytext=pos[a],
                    arrowprops={'arrowstyle': '-|>', 'color': ALERT if hot else MUTED, 'linewidth': 2 if hot else 1.2,
                                'shrinkA': 16, 'shrinkB': 16,
                                'connectionstyle': 'arc3,rad=0.2' if both else 'arc3,rad=0'})
        if zone:
            mx, my = (pos[a][0] + pos[b][0]) / 2, (pos[a][1] + pos[b][1]) / 2
            # 마주 보는 두 화살표는 곡선이라 글자도 바깥으로 비킨다
            if both:
                dx, dy = pos[b][0] - pos[a][0], pos[b][1] - pos[a][1]
                mx, my = mx + dy * 0.12, my - dx * 0.12
            ax.text(mx, my, zone, fontsize=7, ha='center', va='center', color=ALERT if hot else MUTED,
                    bbox={'boxstyle': 'round,pad=0.15', 'fc': 'white', 'ec': 'none', 'alpha': 0.85})
    for n, (x, y) in pos.items():
        ax.scatter(x, y, s=700, color='#d6ecff', edgecolors=ALERT if n == victim else ROBOT_COLOR,
                   linewidths=3 if n == victim else 1.2, zorder=3)
        ax.text(x, y, n, ha='center', va='center', fontsize=7, zorder=4)
    label = f'deadlocks={cycles}' if cycles else 'no deadlock'
    ax.set_title(f'{title or "Wait-for graph (A -> B: A waits for B)"}\n{label}'
                 + (f'  victim={victim}' if victim else ''), fontsize=9)
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.axis('off')


def _rect(xy, w, h, **kw):
    from matplotlib.patches import Rectangle  # 그림을 그릴 때만 matplotlib을 부른다
    return Rectangle(xy, w, h, linewidth=1.2, **{k: v for k, v in kw.items() if v is not None})
