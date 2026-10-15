"""동적 장애물 회피 KPI 평가 (test-plan I17). ROS에 의존하지 않아 시뮬레이터 없이 테스트한다."""

from .metrics import (
    BOX,
    CIRCLE,
    SHAPES_BY_TYPE,
    AvoidanceParams,
    Episode,
    Pose,
    RunResult,
    Shape,
    aggregate,
    aggregate_markdown,
    evaluate_run,
    footprint_clearance,
    format_summary,
    polyline_distance,
    shape_for_id,
    transform_path,
)
from .recorder import CSV_COLUMNS, AvoidanceRecorder, load_recorder

__all__ = [
    'BOX',
    'CIRCLE',
    'CSV_COLUMNS',
    'SHAPES_BY_TYPE',
    'AvoidanceParams',
    'AvoidanceRecorder',
    'Episode',
    'Pose',
    'RunResult',
    'Shape',
    'aggregate',
    'aggregate_markdown',
    'evaluate_run',
    'footprint_clearance',
    'format_summary',
    'load_recorder',
    'polyline_distance',
    'shape_for_id',
    'transform_path',
]
