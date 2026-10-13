"""합성 카메라 영상으로 마커 인식 → 도킹 오차까지 확인한다.

Gazebo에 그리는 것과 같은 칸 배치(markers.board_link와 같은 식)로 판을 그려서, 좌우·뒤집힘 규칙까지 같이 본다.
"""
import math

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from amr_docking.detector import MarkerDetector, to_gray  # noqa: E402
from amr_docking.geometry import dock_error  # noqa: E402
from amr_docking.markers import BOARD_MARGIN, BOARDS, MARKER_BITS, black_runs, marker_cells  # noqa: E402

W, H, HFOV = 640, 480, math.radians(87.0)
FX = (W / 2) / math.tan(HFOV / 2)
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
CAMERA_IN_BASE = np.array([0.18, 0.0, 0.43])     # base_footprint 기준 (amr_description)
# base(x 앞, y 왼쪽, z 위) → 광학(x 오른쪽, y 아래, z 앞)
BASE_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)


def board_polygons(ids, standoff, along, lateral, heading, board):
    """로봇 base 기준 3D 다각형 (흰 판, 검은 칸들)."""
    rx, ry = -along, lateral          # 목표 좌표계(x 판 쪽, y 왼쪽)에서 로봇 위치
    c, s = math.cos(-heading), math.sin(-heading)

    def to_base(by, bz):
        # 판 앞면 위 점 (판 좌표 y: 로봇이 볼 때 오른쪽 +) → 목표 좌표 → base
        tx, ty = standoff, -by
        dx, dy = tx - rx, ty - ry
        return (c * dx - s * dy, s * dx + c * dy, board.center_z + bz)

    def rect(y0, y1, z0, z1):
        return [to_base(y0, z1), to_base(y1, z1), to_base(y1, z0), to_base(y0, z0)]

    size, spacing = board.marker_size, board.spacing
    half_w = (spacing + size) / 2 + BOARD_MARGIN
    half_h = size / 2 + BOARD_MARGIN
    white = [rect(-half_w, half_w, -half_h, half_h)]
    black = []
    cell = size / (MARKER_BITS + 2)
    for side, marker_id in zip(('left', 'right'), ids):
        cy = (spacing / 2) * (1 if side == 'right' else -1)
        for r, c0, n in black_runs(marker_cells(marker_id)):
            y0 = cy - size / 2 + c0 * cell
            z1 = size / 2 - r * cell
            black.append(rect(y0, y0 + n * cell, z1 - cell, z1))
    return white, black


def render(ids, standoff, along, lateral, heading, board=BOARDS['small'], scale=4):
    """scale배로 그려 줄인다 (fillPoly는 경계 픽셀까지 칠해 그대로 그리면 마커가 반 픽셀 커 보인다)."""
    ks = K.copy()
    ks[:2] *= scale
    ks[0, 2], ks[1, 2] = (K[0, 2] + 0.5) * scale - 0.5, (K[1, 2] + 0.5) * scale - 0.5
    img = np.full((H * scale, W * scale), 90, np.uint8)
    white, black = board_polygons(ids, standoff, along, lateral, heading, board)
    for polys, value in ((white, 240), (black, 10)):
        for poly in polys:
            cam = (BASE_TO_OPTICAL @ (np.array(poly) - CAMERA_IN_BASE).T).T
            px, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), ks, np.zeros(5))
            cv2.fillPoly(img, [np.round(px.reshape(-1, 2) * 16).astype(np.int32)], value, shift=4)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def xy(c):
    v = BASE_TO_OPTICAL.T @ np.array(c) + CAMERA_IN_BASE
    return v[0], v[1]


def measure(img, ids, standoff, board=BOARDS['small']):
    """도킹 서버와 같은 방법(판 전체 PnP)."""
    pair = MarkerDetector().board_centers(img, K, ids, board.marker_size, board.spacing)
    assert pair is not None, f'인식 {sorted(MarkerDetector().detect(img))}'
    return dock_error(xy(pair[0]), xy(pair[1]), standoff)


def test_marker_centers_close_to_board_pnp():
    # 마커마다 푼 중심과 판 PnP 중심이 정면에서는 거의 같다
    spec = BOARDS['small']
    img = render((6, 7), 0.795, 0.0, 0.0, 0.0, spec)
    each = MarkerDetector().centers(img, K, wanted=(6, 7), marker_size=spec.marker_size)
    pair = MarkerDetector().board_centers(img, K, (6, 7), spec.marker_size, spec.spacing)
    for mid, c in zip((6, 7), pair):
        assert np.linalg.norm(np.array(each[mid]) - np.array(c)) < 0.003


# (standoff, 판): 충전소, 랙, 도크 (dock_stations.yaml)
@pytest.mark.parametrize('standoff,board', [(0.695, 'small'), (0.795, 'small'), (1.495, 'large')])
# Nav2 도착 오차 범위. 판에 너무 가까운 채 많이 돌아 있으면 한쪽 마커가 시야(87°) 밖이라 제어기가 돌며 찾는다
@pytest.mark.parametrize('pose', [(0.0, 0.0, 0.0), (0.01, 0.01, 0.5), (0.15, 0.05, 8.0), (0.1, -0.06, -6.0),
                                  (0.3, 0.1, 10.0)])
def test_measured_error_matches_truth(standoff, board, pose):
    along, lateral, heading_deg = pose
    ids = (6, 7)
    spec = BOARDS[board]
    e = measure(render(ids, standoff, along, lateral, math.radians(heading_deg), spec), ids, standoff, spec)
    # 도킹 직전(목표 근처)이 KPI에 걸린다: 거기서는 2 cm·1도보다 충분히 작아야 한다
    near = abs(along) < 0.05 and abs(lateral) < 0.05
    tol_pos, tol_deg = (0.008, 0.3) if near else (0.02, 0.6)
    assert e.along == pytest.approx(along, abs=tol_pos)
    assert e.lateral == pytest.approx(lateral, abs=tol_pos)
    assert math.degrees(e.heading) == pytest.approx(heading_deg, abs=tol_deg)


def test_other_station_ids_ignored():
    img = render((6, 7), 0.795, 0.0, 0.0, 0.0)
    assert MarkerDetector().centers(img, K, wanted=(8, 9)) == {}
    assert MarkerDetector().board_centers(img, K, (8, 9), 0.14, 0.32) is None


def test_to_gray_rgb8():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[0, 0] = (255, 255, 255)
    gray = to_gray(rgb.tobytes(), 3, 2, 9, 'rgb8')
    assert gray.shape == (2, 3) and gray[0, 0] == 255 and gray[1, 2] == 0
