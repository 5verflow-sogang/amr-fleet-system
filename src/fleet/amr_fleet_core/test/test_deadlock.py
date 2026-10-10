"""교착 탐지(wait-for 그래프 순환)와 물러날 로봇 고르기."""

import pytest

from amr_fleet_core.deadlock import choose_victim, find_deadlocks


def test_no_cycle():
    assert find_deadlocks({}) == []
    # 줄 서서 기다리는 건 교착이 아니다
    assert find_deadlocks({'a': ['b'], 'b': ['c']}) == []


def test_two_robot_cycle():
    assert find_deadlocks({'a': ['b'], 'b': ['a']}) == [['a', 'b']]


def test_three_robot_cycle_with_tail():
    # d는 순환 밖에서 a를 기다릴 뿐이다
    edges = {'a': ['b'], 'b': ['c'], 'c': ['a'], 'd': ['a']}
    assert find_deadlocks(edges) == [['a', 'b', 'c']]


def test_separate_cycles():
    edges = {'a': ['b'], 'b': ['a'], 'x': ['y'], 'y': ['z'], 'z': ['x']}
    assert find_deadlocks(edges) == [['a', 'b'], ['x', 'y', 'z']]


def test_large_cycle_iterative():
    n = 2000
    edges = {f'r{i:04d}': [f'r{(i + 1) % n:04d}'] for i in range(n)}
    [group] = find_deadlocks(edges)
    assert len(group) == n


def test_victim_lowest_priority_then_last_id():
    priority = {'amr_01': 5, 'amr_02': 1, 'amr_03': 1}.get
    assert choose_victim(['amr_01', 'amr_02', 'amr_03'], priority) == 'amr_03'
    assert choose_victim(['amr_01', 'amr_02'], priority) == 'amr_02'
    with pytest.raises(ValueError):
        choose_victim([], priority)
