"""회피 평가 원자료 기록·저장·다시 읽기. ROS에 의존하지 않는다.

하네스 로그 포맷 원칙(tools/harness/README.md "원자료를 남기고 지표는 나중에 계산한다")대로
받은 그대로의 원자료를 받는 즉시 파일에 이어 쓰고(강제 종료돼도 남는다), 지표는 저장할 때 계산한다.
같은 폴더를 avoidance_eval_offline으로 다시 계산할 수 있다.

출력 폴더:
    avoidance_config.json   로봇 이름, map_origin_in_world, 기준 경로 방식, 장애물 모양, 판정 기준값
    robot_gt.csv            t, x, y, yaw                      (world, 로봇 GT)
    obstacles_gt.csv        t, obstacle, x, y, yaw, speed     (world, 장애물 GT)
    plans.csv               plan, stamp, frame, x, y          (받은 전역 경로 전부, 받은 좌표계 그대로)
    reference_path.csv      x, y                              (world, 평가에 쓴 기준 경로)
    avoidance.csv           로봇 GT 시점마다 지표 (CSV_COLUMNS)
    avoidance_summary.json  실행 요약 + KPI 판정 (metrics.evaluate_run)
"""

from __future__ import annotations

import csv
import json
import os
from dataclasses import asdict, fields
from typing import Optional, Sequence

from .metrics import (
    AvoidanceParams,
    Pose,
    RunResult,
    Shape,
    evaluate_run,
    transform_path,
)

CSV_COLUMNS = ('timestamp', 'robot_x', 'robot_y', 'robot_yaw', 'deviation', 'clearance',
               'nearest_obstacle', 'obstacle_speed', 'episode', 'collision', 'robot')

REFERENCE_MODES = ('first_plan', 'fixed')


def _fmt(v) -> str:
    if v is None:
        return ''
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return f'{v:.6f}'
    return str(v)


def params_from_dict(d: dict) -> AvoidanceParams:
    names = {f.name for f in fields(AvoidanceParams)}
    return AvoidanceParams(**{k: v for k, v in d.items() if k in names})


class AvoidanceRecorder:
    """원자료를 모으고 save()에서 지표를 계산한다.

    reference_mode:
        first_plan  처음 받은 (점 2개 이상) 전역 경로를 기준으로 쓴다. 이후 재계획 경로는 plans.csv에만 남긴다.
                    Nav2는 주기적으로 재계획해 회피 중 경로가 장애물을 돌아가게 바뀌므로, 회피 전 경로를 고정해야
                    "원래 경로" 대비 이탈을 잴 수 있다.
        fixed       fixed_path(map 좌표 꺾은선)를 기준으로 쓴다. reference_t는 첫 로봇 샘플.
    """

    def __init__(self, robot: str, map_origin_in_world: Sequence[float], shapes: dict[str, Shape],
                 params: AvoidanceParams = AvoidanceParams(), reference_mode: str = 'first_plan',
                 fixed_path: Optional[Sequence[tuple[float, float]]] = None, stream_dir: Optional[str] = None):
        if reference_mode not in REFERENCE_MODES:
            raise ValueError(f'reference_mode는 {REFERENCE_MODES} 중 하나: {reference_mode}')
        if reference_mode == 'fixed' and (not fixed_path or len(fixed_path) < 2):
            raise ValueError('reference_mode=fixed면 fixed_path에 점이 2개 이상 있어야 한다')
        if len(map_origin_in_world) != 3:
            raise ValueError('map_origin_in_world는 [x, y, yaw] 3개여야 한다')
        self.robot = robot
        self.origin = [float(v) for v in map_origin_in_world]
        self.shapes = dict(shapes)
        self.params = params
        self.reference_mode = reference_mode
        self.fixed_path = [tuple(map(float, p)) for p in fixed_path] if fixed_path else None
        self.robot_gt: list[Pose] = []
        self.obstacles: dict[str, list[Pose]] = {}
        self.plans: list[tuple[float, str, list[tuple[float, float]]]] = []
        self._files = {}
        self._writers = {}
        self.stream_dir = stream_dir
        if stream_dir:
            os.makedirs(stream_dir, exist_ok=True)
            self._write_config(stream_dir)
            for name, header in (('robot_gt', ('t', 'x', 'y', 'yaw')),
                                 ('obstacles_gt', ('t', 'obstacle', 'x', 'y', 'yaw', 'speed')),
                                 ('plans', ('plan', 'stamp', 'frame', 'x', 'y'))):
                f = open(os.path.join(stream_dir, f'{name}.csv'), 'w', newline='', encoding='utf-8')
                w = csv.writer(f)
                w.writerow(header)
                self._files[name], self._writers[name] = f, w

    # --- 입력 ---
    def add_robot(self, pose: Pose) -> None:
        self.robot_gt.append(pose)
        if 'robot_gt' in self._writers:
            self._writers['robot_gt'].writerow([_fmt(pose.t), _fmt(pose.x), _fmt(pose.y), _fmt(pose.yaw)])

    def add_obstacle(self, obstacle_id: str, pose: Pose) -> None:
        if obstacle_id not in self.shapes:
            raise KeyError(f'모양을 모르는 장애물: {obstacle_id}')
        self.obstacles.setdefault(obstacle_id, []).append(pose)
        if 'obstacles_gt' in self._writers:
            self._writers['obstacles_gt'].writerow(
                [_fmt(pose.t), obstacle_id, _fmt(pose.x), _fmt(pose.y), _fmt(pose.yaw), _fmt(pose.speed)])

    def add_plan(self, stamp: float, frame: str, points: Sequence[tuple[float, float]]) -> bool:
        """전역 경로 하나. 기준 경로로 채택됐으면 True."""
        pts = [(float(x), float(y)) for x, y in points]
        if len(pts) < 2:
            return False
        idx = len(self.plans)
        self.plans.append((float(stamp), frame, pts))
        if 'plans' in self._writers:
            for x, y in pts:
                self._writers['plans'].writerow([idx, _fmt(float(stamp)), frame, _fmt(x), _fmt(y)])
            self._files['plans'].flush()
        return self.reference_mode == 'first_plan' and idx == 0

    def add_shape(self, obstacle_id: str, shape: Shape) -> None:
        self.shapes[obstacle_id] = shape
        if self.stream_dir:
            self._write_config(self.stream_dir)

    @property
    def counts(self) -> tuple[int, int, int]:
        return len(self.robot_gt), sum(len(v) for v in self.obstacles.values()), len(self.plans)

    # --- 기준 경로 ---
    def reference(self) -> tuple[list[tuple[float, float]], Optional[float]]:
        """(world 좌표 기준 경로, 기준 시각). 아직 없으면 ([], None)."""
        if self.reference_mode == 'fixed':
            t0 = min((p.t for p in self.robot_gt), default=None)
            return transform_path(self.fixed_path, self.origin), t0
        if not self.plans:
            return [], None
        stamp, _frame, pts = self.plans[0]
        return transform_path(pts, self.origin), stamp

    # --- 저장 ---
    def config(self) -> dict:
        return {'robot': self.robot, 'map_origin_in_world': self.origin, 'reference_mode': self.reference_mode,
                'fixed_path': [list(p) for p in self.fixed_path] if self.fixed_path else None,
                'shapes': {k: asdict(v) for k, v in sorted(self.shapes.items())}, 'params': asdict(self.params)}

    def _write_config(self, out_dir: str) -> None:
        with open(os.path.join(out_dir, 'avoidance_config.json'), 'w', encoding='utf-8') as f:
            json.dump(self.config(), f, ensure_ascii=False, indent=2)

    def evaluate(self) -> RunResult:
        ref, ref_t = self.reference()
        return evaluate_run(self.robot_gt, self.obstacles, self.shapes, ref, self.params, ref_t)

    def save(self, out_dir: str) -> dict:
        for f in self._files.values():
            f.flush()
        os.makedirs(out_dir, exist_ok=True)
        self._write_config(out_dir)
        result = self.evaluate()
        ref, _ = self.reference()
        with open(os.path.join(out_dir, 'reference_path.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(('x', 'y'))
            w.writerows([_fmt(x), _fmt(y)] for x, y in ref)
        with open(os.path.join(out_dir, 'avoidance.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(CSV_COLUMNS)
            for s in result.samples:
                w.writerow([_fmt(s.t), _fmt(s.x), _fmt(s.y), _fmt(s.yaw), _fmt(s.deviation), _fmt(s.clearance),
                            s.nearest_obstacle, _fmt(s.obstacle_speed), _fmt(s.episode), _fmt(s.collision),
                            self.robot])
        summary = {'robot': self.robot, 'reference_mode': self.reference_mode,
                   'map_origin_in_world': self.origin, 'plans_received': len(self.plans), **result.summary}
        with open(os.path.join(out_dir, 'avoidance_summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        return summary

    def close(self) -> None:
        for f in self._files.values():
            f.close()
        self._files.clear()
        self._writers.clear()


def _opt_float(v: str) -> Optional[float]:
    return float(v) if v not in ('', None) else None


def load_recorder(run_dir: str, params: Optional[AvoidanceParams] = None) -> AvoidanceRecorder:
    """기록 폴더(원자료)를 다시 읽는다. params를 주면 기록 때 기준값 대신 쓴다 (기준값 바꿔 재계산)."""
    with open(os.path.join(run_dir, 'avoidance_config.json'), encoding='utf-8') as f:
        cfg = json.load(f)
    shapes = {k: Shape(**v) for k, v in cfg['shapes'].items()}
    rec = AvoidanceRecorder(cfg['robot'], cfg['map_origin_in_world'], shapes,
                            params or params_from_dict(cfg.get('params', {})), cfg['reference_mode'],
                            cfg.get('fixed_path'))
    with open(os.path.join(run_dir, 'robot_gt.csv'), newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            rec.add_robot(Pose(float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])))
    with open(os.path.join(run_dir, 'obstacles_gt.csv'), newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            if r['obstacle'] not in rec.shapes:
                continue
            rec.add_obstacle(r['obstacle'], Pose(float(r['t']), float(r['x']), float(r['y']), float(r['yaw']),
                                                 _opt_float(r['speed'])))
    plans: dict[int, tuple[float, str, list]] = {}
    path = os.path.join(run_dir, 'plans.csv')
    if os.path.exists(path):
        with open(path, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                i = int(r['plan'])
                plans.setdefault(i, (float(r['stamp']), r['frame'], []))[2].append((float(r['x']), float(r['y'])))
    for i in sorted(plans):
        rec.add_plan(*plans[i])
    return rec
