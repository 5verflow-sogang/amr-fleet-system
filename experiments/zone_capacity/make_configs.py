#!/usr/bin/env python3
"""구역 정원 측정용 설정 만들기: 통로마다 왕복 웨이포인트, 로봇 시작 자세, 손님 N명 파일.

    python3 experiments/zone_capacity/make_configs.py

만드는 파일 (이 폴더 아래, 커밋한다):
  waypoints/<통로>.yaml   nav_kpi.py 형식. 남쪽 끝 → 북쪽 끝 → 남쪽 끝 ... (쌍 하나 = 통과 한 번)
  robots/<통로>.yaml      system.launch robots_file. amr_01이 첫 웨이포인트에서 북쪽을 본다
  guests/<통로>_n<N>.yaml 동적 장애물 파일. 손님 N명이 통로 띠 안에서 무작위로 걷는다 (pattern random)

통로 가운데 줄과 폭은 정답 지도(restaurant_gt, 0.05 m)에서 장애물까지 여유가 가장 큰 x로 잡았다 (11/28):
  west_aisle  x -10.5   폭 약 1.6 m (셀프 코너 ~ 1열 의자)
  aisle_3     x -1.175  폭 1.37 m (3·4열 의자 등받이 사이, 기둥 없음)
  aisle_5     x 4.72    폭 1.0 m (4인석 ~ 6인석, 로봇 한 대만 지나간다)
"""

from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent

# 통로: 웨이포인트 x, 남쪽 끝 y, 북쪽 끝 y, 손님이 걷는 띠 [x0, x1, y0, y1]
AISLES = {
    'west_aisle': {'x': -10.5, 'y0': -4.6, 'y1': 2.5, 'band': [-11.1, -9.95, -4.2, 2.4]},
    'aisle_3': {'x': -1.175, 'y0': -4.6, 'y1': 2.5, 'band': [-1.45, -0.9, -4.2, 2.4]},
    'aisle_5': {'x': 4.72, 'y0': -2.6, 'y1': 2.5, 'band': [4.55, 4.9, -2.2, 2.4]},
}
GUEST_COUNTS = [0, 1, 2, 4]
PASSES = 10           # 쌍 수 = 통과 횟수 (왕복 5번)
ROBOT = 'amr_01'
NORTH = 1.5707963


def waypoints(a):
    pts = [[a['x'], a['y0'] if i % 2 == 0 else a['y1']] for i in range(PASSES + 1)]
    return {'note': '구역 정원 측정 (experiments/zone_capacity). 첫 점은 스폰 자세, 쌍 i = 통로 끝에서 끝까지 한 번',
            'waypoints': pts}


def guest(i, n, band):
    x0, x1, y0, y1 = band
    # 스폰은 띠를 따라 고르게, 로봇 출발점(남쪽 끝)에서 1.5 m 이상 떨어진 위쪽 구간에 둔다
    sy0 = y0 + 1.5
    y = sy0 + (y1 - sy0) * (i + 0.5) / n
    x = x0 + (x1 - x0) * (0.3 if i % 2 == 0 else 0.7)
    return {
        'id': f'guest_{i + 1}',
        'type': 'customer',
        'pattern': 'random',
        'yield_to_robots': [ROBOT],
        'yield_robot_size': [0.565, 0.537],
        'yield_patience_s': 3.0,
        'yield_progress_m': 0.1,
        'speed_range_mps': [0.4, 0.8],
        'spawn': {'x': round(x, 3), 'y': round(y, 3), 'yaw': NORTH},
        'waypoints': [{'x': x0, 'y': y0}, {'x': x1, 'y': y0}, {'x': x1, 'y': y1}, {'x': x0, 'y': y1}],
        'note': '통로 띠 안 무작위 (4점은 활동 범위)',
    }


def write(path: Path, data, head: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(head + yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def main():
    head = '# make_configs.py가 만든다. 손으로 고치지 말고 스크립트를 고쳐 다시 만든다\n'
    for name, a in AISLES.items():
        write(HERE / 'waypoints' / f'{name}.yaml', waypoints(a), head)
        write(HERE / 'robots' / f'{name}.yaml', {'robots': {ROBOT: [a['x'], a['y0'], NORTH]}}, head)
        for n in GUEST_COUNTS:
            if n == 0:
                continue   # 손님 0명은 dynamic_obstacles:=false로 띄운다 (빈 파일을 받지 않는다)
            write(HERE / 'guests' / f'{name}_n{n}.yaml',
                  {'obstacles': [guest(i, n, a['band']) for i in range(n)]}, head)


if __name__ == '__main__':
    main()
