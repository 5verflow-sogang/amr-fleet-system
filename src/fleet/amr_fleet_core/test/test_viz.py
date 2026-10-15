"""그림 함수가 깨지지 않았는지 (CI). 평소 테스트는 FLEET_VIZ가 없으면 그리지 않으므로 여기서 한 번씩 그린다."""

import pytest

from amr_fleet_core.assignment import ALGORITHMS, RobotInfo, TaskInfo
from amr_fleet_core.traffic import ZoneManager, ZoneSpec
from amr_fleet_core.viz import draw_assignment, draw_wait_for, draw_zones

matplotlib = pytest.importorskip('matplotlib')
matplotlib.use('Agg')
plt = pytest.importorskip('matplotlib.pyplot')


@pytest.fixture
def ax():
    fig, ax = plt.subplots()
    yield ax
    fig.canvas.draw()  # 화살표·글자 배치까지 실제로 그려 본다
    plt.close(fig)


def test_draw_assignment(ax):
    robots = [RobotInfo('amr_01', (0.0, 0.0), 100.0, 0, load=1), RobotInfo('amr_02', (3.0, 0.0), 100.0, 0)]
    tasks = [TaskInfo('t1', 0, (1.0, 1.0), 0, 0.0, pickup_xy=(1.0, 0.0)), TaskInfo('t2', 1, (5.0, 0.0), 0, 0.0),
             TaskInfo('t3', 0, (9.0, 9.0), 0, 1.0, pickup_xy=(8.0, 8.0))]
    draw_assignment(ax, robots, tasks, ALGORITHMS['hungarian'](tasks, robots), 'hungarian')
    assert 'unassigned' in ax.get_title()


def test_draw_zones_and_wait_for(ax):
    zm = ZoneManager([ZoneSpec('z1'), ZoneSpec('z2', capacity=2)], lease_s=2.0, release_timeout_s=10.0)
    zm.request('amr_01', 'z1', 1, 0.0)
    zm.request('amr_02', 'z2', 1, 0.0)
    zm.request('amr_01', 'z2', 2, 1.0)
    zm.request('amr_03', 'z1', 1, 1.0)
    draw_zones(ax, zm)
    fig, other = plt.subplots()
    draw_wait_for(other, zm.wait_for(), victim='amr_03')
    draw_wait_for(other, {'a': ['b'], 'b': ['a']})
    plt.close(fig)
