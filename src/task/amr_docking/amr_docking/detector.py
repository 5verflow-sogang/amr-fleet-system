"""카메라 영상에서 ArUco 마커 중심 위치 찾기 (OpenCV, ROS 비의존).

도킹에는 board_centers를 쓴다: 판 위 마커 두 장의 꼭짓점 8개로 판 하나의 PnP(SQPNP)를 풀고, 그 자세에서
두 마커 중심 자리를 카메라 광학 좌표계(z 앞, x 오른쪽, y 아래)로 낸다. 마커마다 따로 푼 중심(centers)보다
비스듬히 볼 때 방향이 정확하다 (Gazebo, 랙 앞 6° 돌아 있을 때 5.7° vs 4.4°). 방향·위치는 그 두 점으로
정한다 (geometry.dock_error).

OpenCV 4.7 이후(ArucoDetector)와 이전(detectMarkers 함수) API를 둘 다 받는다.
"""

from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

from .markers import ARUCO_DICT, MARKER_SIZE

Vec3 = Tuple[float, float, float]


class MarkerDetector:

    def __init__(self, marker_size: float = MARKER_SIZE, dictionary: str = ARUCO_DICT):
        self.marker_size = marker_size
        aruco = cv2.aruco
        self._dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, 'DetectorParameters_create'):
            params = aruco.DetectorParameters_create()
        else:
            params = aruco.DetectorParameters()
        # 꼭짓점을 픽셀 아래로 다듬는다. 도킹 위치 정확도가 여기서 나온다
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        self._params = params
        self._detector = aruco.ArucoDetector(self._dict, params) if hasattr(aruco, 'ArucoDetector') else None

    def detect(self, gray: np.ndarray) -> Dict[int, np.ndarray]:
        """마커 id → 꼭짓점 (4, 2) 픽셀."""
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}

    def centers(self, gray: np.ndarray, camera_matrix: np.ndarray, dist: Optional[np.ndarray] = None,
                wanted: Optional[Sequence[int]] = None, marker_size: Optional[float] = None) -> Dict[int, Vec3]:
        """마커 id → 중심 (카메라 광학 좌표계, m). wanted를 주면 그 id만. 판마다 크기가 다르면 marker_size."""
        dist = np.zeros(5) if dist is None else dist
        h = (marker_size or self.marker_size) / 2
        # IPPE_SQUARE가 요구하는 순서: 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래 (마커 좌표 x 오른쪽, y 위)
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        out = {}
        for marker_id, corners in self.detect(gray).items():
            if wanted is not None and marker_id not in wanted:
                continue
            ok, _, tvec = cv2.solvePnP(obj, corners.astype(np.float64), camera_matrix, dist,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                out[marker_id] = tuple(float(v) for v in tvec.flatten())
        return out


    def board_centers(self, gray: np.ndarray, camera_matrix: np.ndarray, ids: Tuple[int, int],
                      marker_size: float, spacing: float,
                      dist: Optional[np.ndarray] = None) -> Optional[Tuple[Vec3, Vec3]]:
        """판(왼쪽 ids[0], 오른쪽 ids[1], 중심 간격 spacing) 하나의 PnP → (왼쪽, 오른쪽) 마커 중심.
        두 장이 다 안 보이면 None."""
        corners = self.detect(gray)
        if ids[0] not in corners or ids[1] not in corners:
            return None
        h, half = marker_size / 2, spacing / 2
        obj = []
        for cx in (-half, half):   # 판 좌표: x 로봇이 볼 때 오른쪽, y 위, z 판 밖
            obj += [[cx - h, h, 0], [cx + h, h, 0], [cx + h, -h, 0], [cx - h, -h, 0]]
        img = np.vstack([corners[ids[0]], corners[ids[1]]]).astype(np.float64)
        dist = np.zeros(5) if dist is None else dist
        ok, rvec, tvec = cv2.solvePnP(np.array(obj, dtype=np.float64), img, camera_matrix, dist,
                                      flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rot, _ = cv2.Rodrigues(rvec)
        t = tvec.flatten()
        left = rot @ np.array([-half, 0.0, 0.0]) + t
        right = rot @ np.array([half, 0.0, 0.0]) + t
        return tuple(float(v) for v in left), tuple(float(v) for v in right)


def to_gray(data: bytes, width: int, height: int, step: int, encoding: str) -> np.ndarray:
    """sensor_msgs/Image 데이터 → 흑백 영상 (cv_bridge 없이)."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'지원하지 않는 영상 형식: {encoding}')
    img = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :width * channels]
    if channels == 1:
        return img.copy()
    img = img.reshape(height, width, channels)
    code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY,
            'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}[encoding]
    return cv2.cvtColor(img, code)
