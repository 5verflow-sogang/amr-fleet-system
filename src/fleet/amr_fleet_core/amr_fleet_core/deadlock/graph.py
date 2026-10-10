"""wait-for 그래프에서 교착(순환 대기)을 찾고, 풀 때 물러날 로봇을 고른다.

간선 A → B는 "A가 기다리는 구역을 B가 점유 중"이라는 뜻이다. 순환이 있으면 그 안의 로봇은
서로가 비켜 주기를 기다리며 영영 움직이지 못한다.
순환을 하나씩 나열하는 대신 강한 연결 요소(SCC)를 찾는다. 크기가 2 이상인 SCC의 모든 로봇은
어떤 순환 위에 있으므로, 그중 한 대를 물러나게 하면 적어도 순환 하나가 풀린다.
"""

from __future__ import annotations

from collections import deque
from typing import Callable, Hashable, Iterable, Mapping


def find_deadlocks(edges: Mapping[str, Iterable[str]]) -> list[list[str]]:
    """순환 대기 중인 로봇 묶음(크기 2 이상인 SCC)을 돌려준다. 묶음 안과 묶음끼리 모두 robot_id 순."""
    nodes = sorted(set(edges) | {v for vs in edges.values() for v in vs})
    succ = {n: sorted(set(edges.get(n, ()))) for n in nodes}

    # Tarjan. 로봇 수가 적어 재귀 깊이는 문제 되지 않지만, 반복으로 써서 한도를 신경 쓰지 않게 한다
    index: dict[str, int] = {}
    low: dict[str, int] = {}
    on_stack: set[str] = set()
    stack: list[str] = []
    groups: list[list[str]] = []
    counter = 0

    for root in nodes:
        if root in index:
            continue
        work = [(root, 0)]
        while work:
            node, i = work.pop()
            if i == 0:
                index[node] = low[node] = counter
                counter += 1
                stack.append(node)
                on_stack.add(node)
            if i < len(succ[node]):
                work.append((node, i + 1))
                nxt = succ[node][i]
                if nxt not in index:
                    work.append((nxt, 0))
                elif nxt in on_stack:
                    low[node] = min(low[node], index[nxt])
                continue
            # 자식을 다 봤다. 부모의 low를 갱신하고, 루트면 SCC를 꺼낸다
            if work:
                parent = work[-1][0]
                low[parent] = min(low[parent], low[node])
            if low[node] == index[node]:
                group = []
                while True:
                    member = stack.pop()
                    on_stack.discard(member)
                    group.append(member)
                    if member == node:
                        break
                if len(group) > 1:
                    groups.append(sorted(group))
    return sorted(groups)


def choose_victim(robots: Iterable[str], priority: Callable[[str], Hashable]) -> str:
    """교착을 풀려고 물러날 로봇. 작업 우선순위가 가장 낮은 로봇, 같으면 robot_id가 가장 뒤인 로봇.

    구역 허가 순서(우선순위 높은 순 → robot_id 앞 순)와 반대로 골라, 허가에서 밀리는 로봇이 물러나게 한다.
    """
    robots = list(robots)
    if not robots:
        raise ValueError('로봇이 없다')
    lowest = min(priority(r) for r in robots)
    return max(r for r in robots if priority(r) == lowest)


def would_deadlock(edges: Mapping[str, Iterable[str]], waiter: str, holders: Iterable[str]) -> bool:
    """waiter가 holders를 기다리게 되면(간선 waiter → holder 추가) 순환이 생기는지.

    holder 중 하나라도 지금 그래프에서 waiter에 닿으면 순환이 닫힌다 (해소 2: 회피).
    """
    seen = set()
    todo = deque(h for h in holders if h != waiter)
    while todo:
        node = todo.popleft()
        if node == waiter:
            return True
        if node in seen:
            continue
        seen.add(node)
        todo.extend(edges.get(node, ()))
    return False
