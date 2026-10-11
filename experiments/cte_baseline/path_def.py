"""고정 경로 정의 (ROS 비의존). path_publisher.py와 analyze.py가 같이 쓴다.

odom 프레임(로봇 출발 자세 = 원점, +x 방향) 기준:
  직선 3 m (0,0)→(3,0) → 왼쪽 반원 R = 1 m (중심 (3,1)) → 직선 3 m (3,2)→(0,2)
점 간격 0.05 m (Nav2 경로와 비슷하게).
월드 v0에서 출발 자세가 (0, 1.5)면 경로는 y 1.5~3.5로, A열 랙(y 4.4부터) 앞 통로 안에 있다.
"""

import math

STRAIGHT_1 = 3.0
RADIUS = 1.0
STRAIGHT_2 = 3.0
SPACING = 0.05


def build_path():
    """[(x, y, segment)] 목록. segment는 그 점에서 시작하는 선분의 구간 이름 (straight·curve)."""
    pts = []
    n1 = int(round(STRAIGHT_1 / SPACING))
    for i in range(n1):
        pts.append((i * SPACING, 0.0, 'straight'))
    n_arc = int(math.ceil(math.pi * RADIUS / SPACING))
    for i in range(n_arc):
        a = math.pi * i / n_arc
        pts.append((STRAIGHT_1 + RADIUS * math.sin(a), RADIUS - RADIUS * math.cos(a), 'curve'))
    n2 = int(round(STRAIGHT_2 / SPACING))
    for i in range(n2 + 1):
        pts.append((STRAIGHT_1 - i * SPACING, 2.0 * RADIUS, 'straight'))
    return pts


def leg_of(segment, y):
    """구간 이름과 y로 경로 순서상 구간을 가른다: straight_1(가는 직선, y = 0), curve, straight_2(돌아오는 직선, y = 2R)."""
    if segment == 'curve':
        return 'curve'
    return 'straight_1' if y < RADIUS else 'straight_2'


def project(path, x, y):
    """(x, y)에서 가장 가까운 경로 위 점, 부호 있는 CTE(왼쪽 +), 그 선분의 구간 이름."""
    best = None
    for i in range(len(path) - 1):
        ax, ay, seg = path[i]
        bx, by, _ = path[i + 1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 <= 0.0:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        px, py = ax + t * dx, ay + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if best is None or d2 < best[0]:
            cross = dx * (y - py) - dy * (x - px)
            best = (d2, px, py, math.copysign(math.sqrt(d2), cross), seg)
    _, px, py, cte, seg = best
    return px, py, cte, seg
