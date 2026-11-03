"""전역 planner 고르기 (nav2.launch.py planner 인자). ROS 없이 돈다.

기본은 nav2_params.yaml의 GridBased(NavFn) 그대로다. 'astar'면 같은 파일의 planner_server.AStar 블록을 GridBased 이름에
얹는다 - bt_navigator 기본 트리와 측정 스크립트가 planner_id "GridBased"를 쓰므로 이름은 바꾸지 않는다
(DWA를 FollowPath 이름으로 등록하는 것과 같은 방식). docs/algorithms/navigation.md 3절.
"""

PLANNERS = ('navfn', 'astar')


def planner_overrides(nav2_params: dict, planner: str) -> dict:
    """planner_server에 덧붙일 파라미터 (점으로 이은 이름 → 값). navfn이면 빈 dict."""
    if planner not in PLANNERS:
        raise ValueError(f'planner는 {", ".join(PLANNERS)} 중 하나: {planner!r}')
    if planner == 'navfn':
        return {}
    block = nav2_params['planner_server']['ros__parameters']['AStar']
    return {f'GridBased.{key}': value for key, value in block.items()}
