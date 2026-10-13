"""도킹 스테이션과 ArUco 마커판 (config/dock_stations.yaml).

스테이션마다 마커판 하나에 ArUco 마커 두 장(DICT_4X4_100, 판 k → 마커 2k 왼쪽·2k+1 오른쪽)을 붙인다.
두 마커 중심을 이은 선이 판 면의 방향이라, 마커 한 장의 자세(PnP 회전)보다 방향을 훨씬 정확히 잰다
(0.32 m 간격에서 중심 깊이 차 2 mm ≈ 0.4°). 도킹 서버는 이 선과 판 중심으로 도킹 자세를 정한다.

판은 2D LiDAR 스캔 높이(지면 0.38 m ± 여유)를 피해 바닥 근처에 둔다. 걸리면 지도 v1에 없는 물체가
스캔에 찍혀 AMCL이 어긋난다 (amr_gazebo cargo.py와 같은 규칙). 판에는 충돌이 없다(시각만).

Gazebo에는 텍스처 대신 검은 칸을 얇은 상자로 쌓아 마커를 그린다(텍스처는 상자 면마다 방향이 달라
마커가 뒤집혀 보일 수 있다). 같은 줄의 이어진 검은 칸은 상자 하나로 합친다.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple

import yaml

ARUCO_DICT = 'DICT_4X4_100'
MARKER_BITS = 4                 # 4x4 비트 + 테두리 1칸씩 = 6x6 칸
BOARD_MARGIN = 0.03             # m. 마커 둘레 흰 여백 (인식에 필요)


@dataclass(frozen=True)
class BoardSpec:
    marker_size: float          # m. 검은 테두리 바깥 한 변
    spacing: float              # m. 두 마커 중심 사이 (판 가로)
    center_z: float             # m. 마커 중심 높이 (바닥 기준)


# 판 크기 (dock_stations.yaml board). 도킹 자세의 옆 오차 ≈ standoff × 방향 오차라 멀리서 보는 도크(1.5 m)는
# 방향을 더 정확히 재야 한다. 방향 정확도는 마커 간격·크기에 비례해서 도크는 큰 판을 쓴다
BOARDS = {
    'small': BoardSpec(marker_size=0.14, spacing=0.32, center_z=0.22),   # 랙·충전소 (0.7~0.8 m)
    'large': BoardSpec(marker_size=0.20, spacing=0.80, center_z=0.19),   # 도크 플랫폼 (1.5 m)
}
MARKER_SIZE = BOARDS['small'].marker_size
MARKER_SPACING = BOARDS['small'].spacing
BOARD_CENTER_Z = BOARDS['small'].center_z
BOARD_THICKNESS = 0.004         # m
LIDAR_HEIGHT = 0.38             # m. base_link 0.18 + lidar_link 0.2 (amr_description)
LIDAR_MARGIN = 0.05

WHITE = (0.95, 0.95, 0.93)
BLACK = (0.02, 0.02, 0.02)


@dataclass(frozen=True)
class DockStation:
    name: str
    index: int                  # 마커판 번호 (yaml 순서)
    x: float                    # 도킹 자세 (world) [m]
    y: float
    yaw: float                  # [rad]
    standoff: float             # 도킹 자세 → 마커판 앞면 [m]
    board: BoardSpec = BOARDS['small']

    @property
    def marker_ids(self) -> Tuple[int, int]:
        """(왼쪽, 오른쪽) 마커 id. 로봇이 판을 바라볼 때 기준."""
        return 2 * self.index, 2 * self.index + 1

    @property
    def board_center(self) -> Tuple[float, float, float]:
        """마커판 앞면 중심 (world)."""
        return (self.x + self.standoff * math.cos(self.yaw),
                self.y + self.standoff * math.sin(self.yaw), self.board.center_z)

    @property
    def board_yaw(self) -> float:
        """마커판 앞면이 바라보는 방향 (world). 로봇을 향한다."""
        return math.atan2(math.sin(self.yaw + math.pi), math.cos(self.yaw + math.pi))


def load_stations(path: str) -> Dict[str, DockStation]:
    with open(path, encoding='utf-8') as f:
        specs = yaml.safe_load(f)['stations']
    stations = {}
    for i, (name, s) in enumerate(specs.items()):
        x, y, yaw = (float(v) for v in s['pose'])
        board = s.get('board', 'small')
        if board not in BOARDS:
            raise ValueError(f'{name}: board는 {list(BOARDS)} 중 하나: {board}')
        stations[name] = DockStation(name, i, x, y, yaw, float(s['standoff']), BOARDS[board])
    if 2 * len(stations) > 100:
        raise ValueError(f'{ARUCO_DICT}는 마커 100개까지라 판은 50개까지: {len(stations)}')
    return stations


def board_height_range(board: BoardSpec) -> Tuple[float, float]:
    """마커판(여백 포함) 바닥·위 높이 [m]."""
    half = board.marker_size / 2 + BOARD_MARGIN
    return board.center_z - half, board.center_z + half


def clears_lidar(board: BoardSpec) -> bool:
    low, high = board_height_range(board)
    return high < LIDAR_HEIGHT - LIDAR_MARGIN or low > LIDAR_HEIGHT + LIDAR_MARGIN


def black_runs(bits: Sequence[Sequence[int]]) -> List[Tuple[int, int, int]]:
    """칸 격자(1 = 검정)에서 줄마다 이어진 검은 칸 구간 (row, col_start, length)."""
    runs = []
    for r, row in enumerate(bits):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c < len(row) and row[c]:
                    c += 1
                runs.append((r, start, c - start))
            else:
                c += 1
    return runs


def marker_cells(marker_id: int) -> List[List[int]]:
    """마커 한 장의 6x6 칸 (1 = 검정, 줄 0이 위, 칸 0이 왼쪽). OpenCV 사전에서 만든다."""
    import cv2  # Gazebo 배치·인식에서만 쓴다 (테스트 일부는 cv2 없이 돈다)
    cells = MARKER_BITS + 2
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, cells)
    else:   # OpenCV 4.6 이하
        img = cv2.aruco.drawMarker(dictionary, marker_id, cells)
    return [[1 if img[r, c] < 128 else 0 for c in range(cells)] for r in range(cells)]


def _material(rgb) -> str:
    c = ' '.join(f'{v:g}' for v in rgb)
    return (f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse>'
            f'<specular>0 0 0 1</specular></material>')


def _box(name, pose, size, rgb) -> str:
    p = ' '.join(f'{v:.5f}' for v in pose)
    s = ' '.join(f'{v:.5f}' for v in size)
    return (f'<visual name="{name}"><pose>{p}</pose><cast_shadows>false</cast_shadows>'
            f'<geometry><box><size>{s}</size></box></geometry>{_material(rgb)}</visual>')


def board_link(station: DockStation, cells_of=marker_cells) -> str:
    """스테이션 마커판 하나 (link). 링크 원점 = 판 앞면 중심, +x = 로봇 쪽."""
    bx, by, bz = station.board_center
    b = station.board
    width = b.spacing + b.marker_size + 2 * BOARD_MARGIN
    height = b.marker_size + 2 * BOARD_MARGIN
    t = BOARD_THICKNESS
    parts = [f'<link name="board_{station.name}">'
             f'<pose>{bx:.5f} {by:.5f} {bz:.5f} 0 0 {station.board_yaw:.8f}</pose>',
             _box('plate', (-t / 2, 0, 0, 0, 0, 0), (t, width, height), WHITE)]
    cell = b.marker_size / (MARKER_BITS + 2)
    for side, marker_id in zip(('left', 'right'), station.marker_ids):
        cy = (b.spacing / 2) * (1 if side == 'right' else -1)
        for r, c0, n in black_runs(cells_of(marker_id)):
            # 칸 (r, c) 중심: y = 왼쪽 끝 + (c + 0.5)·cell (로봇이 볼 때 오른쪽이 +y), z = 위 끝 − (r + 0.5)·cell
            y = cy - b.marker_size / 2 + (c0 + n / 2) * cell
            z = b.marker_size / 2 - (r + 0.5) * cell
            parts.append(_box(f'm{marker_id}_r{r}_c{c0}', (0.0005, y, z, 0, 0, 0),
                              (0.001, n * cell, cell), BLACK))
    parts.append('</link>')
    return ''.join(parts)


def markers_sdf(stations: Sequence[DockStation], name: str = 'aruco_markers', cells_of=marker_cells) -> str:
    """마커판 전부를 static 모델 하나로 묶은 SDF (ros_gz_sim create -string)."""
    links = ''.join(board_link(s, cells_of) for s in stations)
    return f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>{links}</model></sdf>'
