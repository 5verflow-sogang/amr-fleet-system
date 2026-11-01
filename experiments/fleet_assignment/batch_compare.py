"""할당 함수만 비교 (ROS 없음): 작업 T건 × 빈 로봇 R대를 무작위로 만들어 세 알고리즘의 총 이동 비용을 잰다.
사용: uv run --with scipy python batch_compare.py <저장소 루트>"""
import random
import statistics as st
import sys

sys.path.insert(0, f'{sys.argv[1]}/src/fleet/amr_fleet_core')
from amr_fleet_core.assignment import ALGORITHMS, RobotInfo, TaskInfo  # noqa: E402
from amr_fleet_core.assignment.types import MODE_IDLE, TYPE_DELIVERY  # noqa: E402

W, H, N = 60.0, 40.0, 2000  # 물류센터 60×40 m (원 명세), 크기마다 2000회
rng = random.Random(0)
pt = lambda: (rng.uniform(0, W), rng.uniform(0, H))  # noqa: E731

print('로봇×작업 | 알고리즘 | 평균 총비용(m) | Hungarian 대비 초과 | 최적이 아닌 비율')
for R, T in ((2, 2), (3, 3), (5, 5), (10, 10), (10, 30)):
    cost = {a: [] for a in ALGORITHMS}
    for _ in range(N):
        robots = [RobotInfo(f'amr_{i:02d}', pt(), 100.0, MODE_IDLE, load=rng.randint(0, 5)) for i in range(R)]
        tasks = [TaskInfo(f't{j:03d}', TYPE_DELIVERY, pt(), 0, float(j), pickup_xy=pt()) for j in range(T)]
        for a, f in ALGORITHMS.items():
            res = f(tasks, robots)
            assert len(res.assignments) == min(R, T)  # 세 알고리즘 모두 같은 수를 짝짓는다
            cost[a].append(res.total_cost)
    opt = cost['hungarian']
    for a in ALGORITHMS:
        assert all(c >= o - 1e-9 for c, o in zip(cost[a], opt))  # Hungarian이 항상 최소여야 한다
        excess = st.mean((c - o) / o * 100 for c, o in zip(cost[a], opt))
        worse = sum(c > o + 1e-9 for c, o in zip(cost[a], opt)) / N * 100
        print(f'{R:2d}×{T:<3d} | {a:12s} | {st.mean(cost[a]):7.1f} | +{excess:5.1f}% | {worse:5.1f}%')
