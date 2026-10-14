"""테스트 공용 fixture.

viz: FLEET_VIZ=<폴더>로 돌리면 알고리즘 상태를 PNG로 남긴다 (<폴더>/<테스트 파일>/<이름>.png). 평소에는 아무것도
하지 않고 matplotlib도 부르지 않는다. 그림 함수는 amr_fleet_core.viz.

    FLEET_VIZ=/tmp/fleet_viz python3 -m pytest src/fleet/amr_fleet_core/test
"""

import os
from pathlib import Path

import pytest


@pytest.fixture
def viz(request):
    out = os.environ.get('FLEET_VIZ')

    def snapshot(name, *draws, title=''):
        """draws: ax 하나를 받는 함수들. 왼쪽부터 한 칸씩 그려 한 장으로 저장한다."""
        if not out:
            return None
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, len(draws), figsize=(5.5 * len(draws), 4.5), squeeze=False)
        for ax, draw in zip(axes[0], draws):
            draw(ax)
        fig.suptitle(title or request.node.name, fontsize=10)
        fig.tight_layout()
        path = Path(out) / Path(request.module.__file__).stem / f'{name}.png'
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path

    return snapshot
