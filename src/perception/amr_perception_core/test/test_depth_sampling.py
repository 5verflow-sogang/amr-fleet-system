import numpy as np
import pytest

from amr_perception_core.camera_model import PinholeCamera
from amr_perception_core.depth_sampling import (
    bbox_center_to_pixel,
    DepthSampleStatus,
    DepthSamplingConfig,
    sample_bbox_depth,
    shrink_bbox,
)

MEDIAN = DepthSamplingConfig(method='median')

# 객체 bbox (200..300, 100..200). 기본 roi_scale=0.5면 ROI는 (225..275, 125..175) = 50x50
BBOX = (200.0, 100.0, 300.0, 200.0)
ROI = (slice(125, 175), slice(225, 275))
ROI_PIXELS = 50 * 50


def make_scene(object_depth=2.0, background_depth=8.0, dtype=np.float32):
    """배경 위에 BBOX 영역만 object_depth인 640x480 깊이 영상."""
    depth = np.full((480, 640), background_depth, dtype=dtype)
    depth[100:200, 200:300] = object_depth
    return depth


# --- ROI shrink ---

def test_shrink_bbox_keeps_center_and_scales_size():
    assert shrink_bbox(BBOX, 0.5) == (225.0, 125.0, 275.0, 175.0)


def test_shrink_bbox_scale_one_is_identity():
    assert shrink_bbox(BBOX, 1.0) == BBOX


@pytest.mark.parametrize('roi_scale', [0.0, -0.5, 1.5, np.nan])
def test_shrink_bbox_rejects_invalid_scale(roi_scale):
    with pytest.raises(ValueError):
        shrink_bbox(BBOX, roi_scale)


def test_roi_shrink_ignores_background_inside_loose_bbox():
    # 객체보다 넓은 bbox라 배경이 섞여 있어도, 중심 ROI만 보므로 객체 깊이가 나온다
    depth = make_scene()
    loose_bbox = (170.0, 70.0, 330.0, 230.0)
    assert sample_bbox_depth(depth, loose_bbox).depth == pytest.approx(2.0)
    full = sample_bbox_depth(
        depth, loose_bbox, DepthSamplingConfig(roi_scale=1.0, method='median'))
    assert full.depth == pytest.approx(8.0)  # bbox 전체를 중앙값으로 보면 배경이 과반


# --- 정상 동작 ---

def test_median_of_uniform_roi():
    result = sample_bbox_depth(make_scene(), BBOX)
    assert result.ok
    assert result.status is DepthSampleStatus.OK
    assert result.depth == pytest.approx(2.0)
    assert result.valid_ratio == pytest.approx(1.0)
    assert result.num_valid == result.num_total == ROI_PIXELS


def test_median_rejects_minority_outliers():
    depth = make_scene()
    depth[ROI][:10, :] = 9.5  # ROI의 20%만 튀는 값
    assert sample_bbox_depth(depth, BBOX).depth == pytest.approx(2.0)


def test_median_of_known_distribution():
    depth = make_scene()
    depth[ROI] = np.linspace(1.0, 3.0, ROI_PIXELS).reshape(50, 50)
    assert sample_bbox_depth(depth, BBOX, MEDIAN).depth == pytest.approx(2.0)


# --- 가장 가까운 덩어리(전경) ---

def make_thin_object(object_ratio, object_depth=2.0, background_depth=8.0):
    """ROI 왼쪽 열들만 물체(object_ratio 비율)이고 나머지는 배경인 깊이 영상."""
    depth = make_scene(object_depth=background_depth)
    depth[ROI][:, :int(50 * object_ratio)] = object_depth
    return depth


def test_thin_object_uses_object_depth_not_background():
    # 기둥·표지판처럼 ROI의 20%만 물체여도 물체 깊이를 고른다
    depth = make_thin_object(0.2)
    result = sample_bbox_depth(depth, BBOX)
    assert result.depth == pytest.approx(2.0)
    assert result.num_foreground == ROI_PIXELS // 5


def test_median_method_picks_background_for_thin_object():
    # 같은 상황에서 중앙값 방식은 배경을 고른다 (리뷰에서 지적된 동작)
    assert sample_bbox_depth(make_thin_object(0.2), BBOX, MEDIAN).depth == pytest.approx(8.0)


def test_nearest_cluster_ignores_few_near_spikes():
    # 전경 기준 백분위(10%)보다 적은 가까운 튀는 픽셀은 결과를 바꾸지 못한다
    depth = make_scene(object_depth=3.0)
    depth[ROI][:1, :] = 0.5  # ROI의 2%
    result = sample_bbox_depth(depth, BBOX)
    assert result.depth == pytest.approx(3.0)


def test_nearest_cluster_with_depth_noise():
    # 시뮬레이터 깊이 잡음(σ=0.02 m)이 있어도 물체 깊이 근처를 고른다
    rng = np.random.default_rng(0)
    depth = make_thin_object(0.3)
    depth[ROI][:, :15] += rng.normal(0.0, 0.02, size=(50, 15)).astype(np.float32)
    assert sample_bbox_depth(depth, BBOX).depth == pytest.approx(2.0, abs=0.01)


def test_foreground_band_keeps_object_thickness():
    # 물체 앞면~뒷면 깊이 차(0.2 m)는 band(0.3 m) 안이라 모두 전경, 배경은 제외
    depth = make_scene(object_depth=8.0)
    depth[ROI][:, :25] = np.linspace(2.0, 2.2, 25)
    result = sample_bbox_depth(depth, BBOX)
    assert result.num_foreground == ROI_PIXELS // 2
    assert 2.0 <= result.depth <= 2.2


def test_uniform_object_same_as_median():
    for config in (DepthSamplingConfig(), MEDIAN):
        result = sample_bbox_depth(make_scene(), BBOX, config)
        assert result.depth == pytest.approx(2.0)
        assert result.num_foreground == ROI_PIXELS


# --- bbox 중심 → K 픽셀 관례 ---

def test_bbox_center_to_pixel_shifts_half_pixel():
    # 픽셀 (320, 240) 하나만 덮는 bbox의 중심은 K 관례로 (320, 240)
    assert bbox_center_to_pixel((320.0, 240.0, 321.0, 241.0)) == (320.0, 240.0)
    assert bbox_center_to_pixel(BBOX) == (249.5, 149.5)


def test_bbox_center_backprojects_onto_optical_axis():
    camera = PinholeCamera(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    u, v = bbox_center_to_pixel((310.0, 230.0, 331.0, 251.0))
    np.testing.assert_allclose(camera.backproject(u, v, 2.0), [0.0, 0.0, 2.0], atol=1e-12)


def test_bbox_center_to_pixel_rejects_invalid_bbox():
    with pytest.raises(ValueError):
        bbox_center_to_pixel((10.0, 10.0, 5.0, 20.0))


# --- 검출 영상 크기 확인 ---

def test_accepts_matching_image_size():
    assert sample_bbox_depth(make_scene(), BBOX, image_size=(640, 480)).ok


@pytest.mark.parametrize('image_size', [(480, 640), (1280, 720), (640, 479)])
def test_rejects_mismatched_image_size(image_size):
    with pytest.raises(ValueError):
        sample_bbox_depth(make_scene(), BBOX, image_size=image_size)


# --- 무효 값 제거 ---

@pytest.mark.parametrize('invalid', [np.nan, np.inf, -np.inf, 0.0, -1.0])
def test_removes_invalid_values(invalid):
    depth = make_scene()
    depth[ROI][:25, :] = invalid  # ROI 절반을 무효로
    depth[ROI][25:, :] = 3.0
    result = sample_bbox_depth(depth, BBOX)
    assert result.depth == pytest.approx(3.0)
    assert result.num_valid == ROI_PIXELS // 2
    assert result.valid_ratio == pytest.approx(0.5)


def test_zero_is_invalid_even_when_min_depth_is_zero():
    depth = make_scene()
    depth[ROI][:25, :] = 0.0
    result = sample_bbox_depth(depth, BBOX, DepthSamplingConfig(min_depth=0.0))
    assert result.depth == pytest.approx(2.0)
    assert result.valid_ratio == pytest.approx(0.5)


def test_filters_by_min_and_max_depth():
    depth = make_scene()
    depth[ROI][:10, :] = 0.05   # min_depth 0.1 미만
    depth[ROI][10:20, :] = 12.0  # max_depth 10 초과
    depth[ROI][20:, :] = 4.0
    result = sample_bbox_depth(depth, BBOX)
    assert result.depth == pytest.approx(4.0)
    assert result.valid_ratio == pytest.approx(0.6)


def test_depth_range_bounds_are_inclusive():
    depth = make_scene()
    depth[ROI][:25, :] = 0.5
    depth[ROI][25:, :] = 1.5
    result = sample_bbox_depth(depth, BBOX, DepthSamplingConfig(min_depth=0.5, max_depth=1.5))
    assert result.valid_ratio == pytest.approx(1.0)


# --- 단위 변환 ---

def test_converts_millimeter_depth_to_meters():
    depth_mm = make_scene(object_depth=2345, background_depth=8000, dtype=np.uint16)
    result = sample_bbox_depth(depth_mm, BBOX, DepthSamplingConfig(depth_scale=0.001))
    assert result.depth == pytest.approx(2.345)


def test_depth_range_is_applied_after_scaling():
    # 2000mm는 스케일 전에는 max_depth(10) 밖이지만 m로 바꾸면 유효하다
    depth_mm = make_scene(object_depth=2000, dtype=np.uint16)
    assert sample_bbox_depth(depth_mm, BBOX, DepthSamplingConfig(depth_scale=0.001)).ok


# --- 영상 경계 ---

def test_clips_bbox_partially_outside_image():
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    # ROI (625..675, 465..515) → clip 후 (625..640, 465..480) = 15x15
    result = sample_bbox_depth(depth, (600.0, 440.0, 700.0, 540.0))
    assert result.depth == pytest.approx(3.0)
    assert result.num_total == 15 * 15


def test_clips_negative_coordinates():
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    # ROI (-20..20, -20..20) → clip 후 (0..20, 0..20) = 20x20
    result = sample_bbox_depth(depth, (-40.0, -40.0, 40.0, 40.0))
    assert result.ok
    assert result.num_total == 20 * 20


def test_bbox_fully_outside_image_is_empty_roi():
    result = sample_bbox_depth(make_scene(), (700.0, 500.0, 800.0, 600.0))
    assert result.status is DepthSampleStatus.EMPTY_ROI
    assert not result.ok
    assert result.depth is None
    assert result.num_total == 0


def test_subpixel_roi_uses_covering_pixel():
    result = sample_bbox_depth(make_scene(), (250.2, 150.2, 250.8, 150.8))
    assert result.depth == pytest.approx(2.0)
    assert result.num_total == 1


# --- 실패 상태 ---

def test_no_valid_depth():
    depth = make_scene()
    depth[ROI] = np.nan
    result = sample_bbox_depth(depth, BBOX)
    assert result.status is DepthSampleStatus.NO_VALID_DEPTH
    assert not result.ok
    assert result.depth is None
    assert result.num_valid == 0
    assert result.num_total == ROI_PIXELS


def test_valid_ratio_below_threshold():
    depth = make_scene()
    depth[ROI] = np.nan
    depth[ROI][:2, :] = 2.0  # 4%
    result = sample_bbox_depth(depth, BBOX, DepthSamplingConfig(min_valid_ratio=0.1))
    assert result.status is DepthSampleStatus.LOW_VALID_RATIO
    assert result.depth is None
    assert result.valid_ratio == pytest.approx(0.04)
    assert result.num_valid == 100


def test_valid_ratio_equal_to_threshold_is_ok():
    depth = make_scene()
    depth[ROI] = np.nan
    depth[ROI][:5, :] = 2.0  # 정확히 10%
    assert sample_bbox_depth(depth, BBOX, DepthSamplingConfig(min_valid_ratio=0.1)).ok


# --- 인자 검증 ---

@pytest.mark.parametrize('bbox', [
    (300.0, 100.0, 200.0, 200.0),   # x2 < x1
    (200.0, 100.0, 200.0, 200.0),   # 폭 0
    (200.0, 200.0, 300.0, 100.0),   # y2 < y1
    (200.0, 100.0, 300.0),          # 원소 3개
    (np.nan, 100.0, 300.0, 200.0),  # NaN
    (200.0, 100.0, np.inf, 200.0),  # inf
])
def test_rejects_invalid_bbox(bbox):
    with pytest.raises(ValueError):
        sample_bbox_depth(make_scene(), bbox)


@pytest.mark.parametrize('image', [np.zeros(10), np.zeros((48, 64, 3)), np.zeros((0, 640))])
def test_rejects_invalid_image_shape(image):
    with pytest.raises(ValueError):
        sample_bbox_depth(image, BBOX)


@pytest.mark.parametrize('kwargs', [
    {'roi_scale': 0.0},
    {'roi_scale': 1.1},
    {'depth_scale': 0.0},
    {'depth_scale': -0.001},
    {'depth_scale': np.nan},
    {'min_depth': -0.1},
    {'min_depth': 5.0, 'max_depth': 5.0},
    {'min_depth': 6.0, 'max_depth': 5.0},
    {'max_depth': np.inf},
    {'min_valid_ratio': -0.1},
    {'min_valid_ratio': 1.1},
    {'method': 'mean'},
    {'near_percentile': -1.0},
    {'near_percentile': 100.5},
    {'foreground_band': 0.0},
    {'foreground_band': np.nan},
])
def test_rejects_invalid_config(kwargs):
    with pytest.raises(ValueError):
        DepthSamplingConfig(**kwargs)
