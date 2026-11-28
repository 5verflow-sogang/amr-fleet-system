"""guest_routes 단위 테스트: 작은 격자 지도에서 경로·자리 옮기기·비킬 방향."""

import math

import pytest

from amr_gazebo.guest_routes import GridMap, _read_pgm, build_obstacles


def _grid(rows):
    # rows: 위쪽(y 큰 쪽)부터 문자열, '#' 점유. 칸 0.1 m, 원점 (0, 0)
    occ = [[ch == '#' for ch in row] for row in reversed(rows)]
    return GridMap(occ, 0.1, (0.0, 0.0))


ROOM = [
    '####################',
    '#..................#',
    '#..................#',
    '#..................#',
    '#........#.........#',
    '#........#.........#',
    '#........#.........#',
    '#........#.........#',
    '#........#.........#',
    '#..................#',
    '#..................#',
    '#..................#',
    '####################',
]


def test_plan_goes_around_wall_and_keeps_clearance():
    g = _grid(ROOM)
    a, b = g.snap((0.45, 0.65), 0.2), g.snap((1.55, 0.65), 0.2)
    path = g.plan(a, b, 0.2)
    assert path[-1] == b
    pts = [a] + path
    assert all(g.segment_clear(p, q, 0.2) for p, q in zip(pts, pts[1:]))
    assert sum(math.dist(p, q) for p, q in zip(pts, pts[1:])) > math.dist(a, b) + 0.1   # 벽을 돌아간다


def test_snap_moves_point_off_obstacle():
    g = _grid(ROOM)
    p = g.snap((0.95, 0.65), 0.2)
    assert g.clear_at(p) >= 0.2 and math.dist(p, (0.95, 0.65)) < 0.5


def test_no_path_raises():
    g = _grid(['#######', '#..#..#', '#######'])
    with pytest.raises(ValueError):
        g.plan((0.15, 0.15), (0.45, 0.15), 0.05)


def test_evade_dirs_skip_walls():
    g = _grid(ROOM)
    dirs = g.evade_dirs((0.25, 0.25), step_m=0.3, clearance=0.05)
    assert [1, 0] in dirs and [0, 1] in dirs and [-1, 0] not in dirs and [0, -1] not in dirs


def test_build_rejects_broken_loop_and_unknown_place():
    g = _grid(ROOM)
    spec = {'places': {'a': [0.5, 0.5], 'b': [1.5, 0.5]},
            'guests': [{'id': 'x', 'start': 'a', 'steps': [{'go': 'b', 'wait_s': [1, 2]}], 'loop_from': 0}]}
    with pytest.raises(ValueError):
        build_obstacles(spec, g)                       # 마지막 b와 start a가 달라 되풀이가 끊긴다
    spec['guests'][0]['steps'].append({'go': 'c', 'wait_s': [1, 2]})
    with pytest.raises(ValueError):
        build_obstacles(spec, g)


def test_build_makes_a_looping_routine():
    g = _grid(['#' * 40] + ['#' + '.' * 38 + '#'] * 28 + ['#' * 40])      # 4 x 3 m 빈 방
    spec = {'places': {'a': [0.75, 0.75], 'b': [3.05, 2.05]},
            'defaults': {'speed_range_mps': [0.5, 0.8]},
            'guests': [{'id': 'x', 'start': 'a', 'loop_from': 1,
                        'steps': [{'go': 'a', 'wait_s': [1, 2]}, {'go': 'b', 'wait_s': [1, 2]},
                                  {'go': 'a', 'wait_s': [3, 4]}]}]}
    (ob,) = build_obstacles(spec, g)
    assert ob['pattern'] == 'routine' and ob['loop_from'] == 1 and ob['speed_range_mps'] == [0.5, 0.8]
    assert [st['name'] for st in ob['steps']] == ['a', 'b', 'a']
    assert ob['steps'][0]['path'] == [[0.75, 0.75]] and ob['steps'][1]['path'][-1] == [3.05, 2.05]


def test_read_pgm_with_comment():
    raw = b'P5\n# c\n2 1\n255\n' + bytes([0, 254])
    assert _read_pgm(raw) == (2, 1, 255, bytes([0, 254]))
