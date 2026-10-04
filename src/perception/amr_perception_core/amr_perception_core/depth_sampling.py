"""bbox 안의 깊이 값으로 객체의 대표 깊이를 구한다 (ROS 비의존).

bbox 가장자리에는 배경이 섞이므로 중심 쪽으로 줄인 ROI만 본다.
대표 깊이는 기본적으로 ROI에서 가장 가까운 덩어리(전경)의 중앙값을 쓴다.
기둥·표지판처럼 가는 물체는 ROI 안에서도 배경이 더 많을 수 있어, 전체 중앙값은 배경 깊이를 고른다.

bbox 좌표는 YOLO와 같은 픽셀 경계 관례다: 픽셀 i는 [i, i+1) 구간을 덮고 중심은 i + 0.5다.
CameraInfo K(camera_model)는 픽셀 i의 중심을 i로 보므로, bbox 중심을 역투영할 때는
bbox_center_to_pixel로 바꿔서 넘긴다.

센서 데이터 부족(유효 깊이 없음 등)은 예외 대신 DepthSampleResult.status로 알리고,
잘못된 인자(bbox, 파라미터, 영상 차원)는 ValueError를 낸다.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Sequence, Tuple

import numpy as np

BBox = Tuple[float, float, float, float]

METHOD_NEAREST_CLUSTER = 'nearest_cluster'
METHOD_MEDIAN = 'median'


@dataclass(frozen=True)
class DepthSamplingConfig:
    """깊이 추출 파라미터. 실제 값은 노드 패키지가 YAML에서 읽어 채운다."""

    # bbox 대비 ROI 크기 비율. 중심은 유지하고 폭·높이만 줄인다.
    roi_scale: float = 0.5
    # 원시 깊이 값 → m 배율. 32FC1(m)이면 1.0, 16UC1(mm)이면 0.001.
    depth_scale: float = 1.0
    # 유효 깊이 범위 [m]. 명세 depth 카메라 최대 거리는 10m.
    min_depth: float = 0.1
    max_depth: float = 10.0
    # ROI 픽셀 중 유효 깊이 비율이 이보다 낮으면 실패로 본다.
    min_valid_ratio: float = 0.1
    # 대표 깊이 계산 방식.
    #   nearest_cluster: 하위 near_percentile 백분위를 전경 기준으로 잡고,
    #       기준 + foreground_band 이내 픽셀(전경)의 중앙값. 가는 물체에도 물체 깊이를 고른다.
    #       백분위를 쓰는 이유: 최솟값은 깊이 잡음으로 튄 픽셀 하나에 결과가 좌우된다.
    #   median: 유효 깊이 전체의 중앙값. 물체가 ROI의 절반 이상일 때만 물체 깊이가 나온다.
    method: str = METHOD_NEAREST_CLUSTER
    # 전경 기준 백분위 [0, 100]. 이보다 적은 비율의 가까운 튀는 픽셀은 기준에 영향을 주지 않는다.
    near_percentile: float = 10.0
    # 전경 두께 [m]. 기준에서 이만큼 더 먼 픽셀까지 같은 물체로 본다.
    foreground_band: float = 0.3

    def __post_init__(self):
        if not 0.0 < self.roi_scale <= 1.0:
            raise ValueError(f'roi_scale은 (0, 1] 범위여야 한다: {self.roi_scale}')
        if not (math.isfinite(self.depth_scale) and self.depth_scale > 0.0):
            raise ValueError(f'depth_scale은 양수여야 한다: {self.depth_scale}')
        if not (math.isfinite(self.min_depth) and math.isfinite(self.max_depth)):
            raise ValueError(f'깊이 범위는 유한해야 한다: [{self.min_depth}, {self.max_depth}]')
        if not 0.0 <= self.min_depth < self.max_depth:
            raise ValueError(
                f'0 <= min_depth < max_depth여야 한다: [{self.min_depth}, {self.max_depth}]')
        if not 0.0 <= self.min_valid_ratio <= 1.0:
            raise ValueError(f'min_valid_ratio는 [0, 1] 범위여야 한다: {self.min_valid_ratio}')
        if self.method not in (METHOD_NEAREST_CLUSTER, METHOD_MEDIAN):
            raise ValueError(
                f'method는 {METHOD_NEAREST_CLUSTER!r} 또는 {METHOD_MEDIAN!r}이어야 한다: '
                f'{self.method!r}')
        if not 0.0 <= self.near_percentile <= 100.0:
            raise ValueError(f'near_percentile은 [0, 100] 범위여야 한다: {self.near_percentile}')
        if not (math.isfinite(self.foreground_band) and self.foreground_band > 0.0):
            raise ValueError(f'foreground_band는 양수여야 한다: {self.foreground_band}')


class DepthSampleStatus(Enum):
    OK = 'ok'
    EMPTY_ROI = 'empty_roi'              # ROI가 영상 밖에 있음
    NO_VALID_DEPTH = 'no_valid_depth'    # ROI에 유효 깊이가 하나도 없음
    LOW_VALID_RATIO = 'low_valid_ratio'  # 유효 깊이 비율이 min_valid_ratio 미만


@dataclass(frozen=True)
class DepthSampleResult:
    status: DepthSampleStatus
    depth: Optional[float]  # 대표 깊이 [m]. status가 OK일 때만 값이 있다.
    valid_ratio: float      # num_valid / num_total (ROI가 비었으면 0)
    num_valid: int
    num_total: int          # clip 후 ROI 픽셀 수
    num_foreground: int = 0  # 대표 깊이 계산에 쓴 픽셀 수 (median 방식이면 num_valid와 같다)

    @property
    def ok(self) -> bool:
        return self.status is DepthSampleStatus.OK


def _validate_bbox(bbox: Sequence[float]) -> BBox:
    if len(bbox) != 4:
        raise ValueError(f'bbox는 (x1, y1, x2, y2) 4개 값이어야 한다: {bbox}')
    x1, y1, x2, y2 = (float(c) for c in bbox)
    if not all(math.isfinite(c) for c in (x1, y1, x2, y2)):
        raise ValueError(f'bbox 좌표는 유한해야 한다: {bbox}')
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f'bbox는 x1 < x2, y1 < y2여야 한다: {bbox}')
    return x1, y1, x2, y2


def bbox_center_to_pixel(bbox: Sequence[float]) -> Tuple[float, float]:
    """bbox(픽셀 경계 관례)의 중심을 CameraInfo K 관례(픽셀 i의 중심 = i) 좌표 (u, v)로 바꾼다.

    PinholeCamera.backproject에는 이 값을 넘긴다. 바꾸지 않으면 u, v가 0.5 px씩 밀린다.
    """
    x1, y1, x2, y2 = _validate_bbox(bbox)
    return (x1 + x2) / 2.0 - 0.5, (y1 + y2) / 2.0 - 0.5


def shrink_bbox(bbox: Sequence[float], roi_scale: float) -> BBox:
    """bbox (x1, y1, x2, y2)를 중심은 그대로 두고 폭·높이를 roi_scale배로 줄인다."""
    if not 0.0 < roi_scale <= 1.0:
        raise ValueError(f'roi_scale은 (0, 1] 범위여야 한다: {roi_scale}')
    x1, y1, x2, y2 = _validate_bbox(bbox)
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    half_w = (x2 - x1) * roi_scale / 2.0
    half_h = (y2 - y1) * roi_scale / 2.0
    return cx - half_w, cy - half_h, cx + half_w, cy + half_h


def _nearest_cluster(valid: np.ndarray, config: DepthSamplingConfig) -> np.ndarray:
    """하위 백분위 기준 + foreground_band 이내의 깊이(전경)만 남긴다. 항상 1개 이상이다."""
    near = np.percentile(valid, config.near_percentile)
    return valid[valid <= near + config.foreground_band]


def sample_bbox_depth(
    depth_image: np.ndarray,
    bbox: Sequence[float],
    config: DepthSamplingConfig = DepthSamplingConfig(),
    image_size: Optional[Tuple[int, int]] = None,
) -> DepthSampleResult:
    """bbox 중심 ROI에서 객체의 대표 깊이 [m]를 구한다.

    depth_image: HxW 깊이 영상. 원시 값 * config.depth_scale = m.
    bbox: 픽셀 좌표 (x1, y1, x2, y2), 픽셀 경계 관례. ROI에 걸친 픽셀을 모두 포함하고
        영상 밖 부분은 잘라낸다.
    image_size: bbox를 검출한 영상의 (width, height). 주면 깊이 영상 크기와 같은지 확인한다.
        RGB와 깊이 영상은 픽셀이 1:1로 정렬되어 있어야 하므로, 크기가 다르면 ValueError.
    NaN·inf·0 이하 값과 [min_depth, max_depth] 밖의 값은 무효로 제외한다.
    """
    depth_image = np.asarray(depth_image)
    if depth_image.ndim != 2 or depth_image.size == 0:
        raise ValueError(f'깊이 영상은 비어 있지 않은 2차원이어야 한다: shape={depth_image.shape}')
    height, width = depth_image.shape
    if image_size is not None and tuple(image_size) != (width, height):
        raise ValueError(
            f'검출 영상 크기 {tuple(image_size)}와 깊이 영상 크기 {(width, height)}가 다르다 '
            '(width, height). RGB와 깊이 영상이 정렬되어 있어야 한다')

    x1, y1, x2, y2 = shrink_bbox(bbox, config.roi_scale)
    col0 = max(math.floor(x1), 0)
    row0 = max(math.floor(y1), 0)
    col1 = min(math.ceil(x2), width)
    row1 = min(math.ceil(y2), height)
    if col1 <= col0 or row1 <= row0:
        return DepthSampleResult(DepthSampleStatus.EMPTY_ROI, None, 0.0, 0, 0)

    roi = depth_image[row0:row1, col0:col1].astype(np.float64) * config.depth_scale
    valid_mask = (
        np.isfinite(roi)
        & (roi > 0.0)
        & (roi >= config.min_depth)
        & (roi <= config.max_depth)
    )
    num_total = int(roi.size)
    num_valid = int(np.count_nonzero(valid_mask))
    valid_ratio = num_valid / num_total

    if num_valid == 0:
        return DepthSampleResult(DepthSampleStatus.NO_VALID_DEPTH, None, 0.0, 0, num_total)
    if valid_ratio < config.min_valid_ratio:
        return DepthSampleResult(
            DepthSampleStatus.LOW_VALID_RATIO, None, valid_ratio, num_valid, num_total)

    valid = roi[valid_mask]
    if config.method == METHOD_NEAREST_CLUSTER:
        selected = _nearest_cluster(valid, config)
    else:
        selected = valid
    return DepthSampleResult(
        DepthSampleStatus.OK,
        float(np.median(selected)),
        valid_ratio,
        num_valid,
        num_total,
        int(selected.size),
    )
