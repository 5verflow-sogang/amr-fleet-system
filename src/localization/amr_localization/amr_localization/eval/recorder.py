"""정답·추정 자세를 모아 하네스 형식으로 저장한다. ROS에 의존하지 않는다 (노드는 eval_node.py).

저장 파일 (tools/harness/README.md 실행 결과 배치):
    <out_dir>/trajectories/<robot>_gt.tum    정답 궤적 (map 기준으로 변환)
    <out_dir>/trajectories/<robot>_est.tum   추정 궤적 (map 기준)
    <out_dir>/localization.csv               시점별 오차
    <out_dir>/localization_summary.json      구간별 RMSE·Max와 KPI 판정

stream_dir를 주면 궤적(원자료)을 받는 즉시 TUM 파일에 이어 쓴다. 실행이 강제 종료돼도
원자료는 남고, 오차는 나중에 from_tum()(또는 eval_offline 명령)으로 다시 계산할 수 있다.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict
from typing import Optional

from .metrics import STATIONARY, compute_errors, kpi_check, summarize_by_segment, write_localization_csv
from .trajectory import (
    Pose2D,
    associate_interpolated,
    read_tum,
    transform_to_map,
    tum_line,
    write_tum,
)

# 스트리밍할 때 이만큼 줄을 쓸 때마다 디스크로 내보낸다 (100 Hz 정답이면 약 1초)
FLUSH_EVERY = 100


def trajectory_paths(out_dir: str, robot: str) -> tuple[str, str]:
    """(정답 TUM 경로, 추정 TUM 경로)."""
    traj_dir = os.path.join(out_dir, 'trajectories')
    return os.path.join(traj_dir, f'{robot}_gt.tum'), os.path.join(traj_dir, f'{robot}_est.tum')


class EvalRecorder:
    """정답(world 기준)과 추정(map 기준) 자세를 모은다.

    map_origin_in_world: world에서 본 map 원점 (x, y, yaw). 지도 인계 때 받은 매핑 시작 자세.
    stream_dir: 주면 궤적을 받는 즉시 <stream_dir>/trajectories에 이어 쓴다.
    skip_initial_stationary: 로봇이 처음 움직이기 전 샘플을 평가에서 뺀다. 초기 위치를 정답으로 주므로
        이 구간은 모든 추정기가 오차 0이라 정지 RMSE를 실제보다 낮춘다(10/6 실측: 대기 60초가 정지 RMSE를
        5.5 → 2.4 cm로 낮춤). 궤적 원자료(TUM)에는 그대로 남는다.
    """

    def __init__(
        self,
        robot: str,
        map_origin_in_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
        max_gap: float = 0.05,
        moving_v: float = 0.01,
        moving_w: float = 0.05,
        stream_dir: Optional[str] = None,
        skip_initial_stationary: bool = True,
    ):
        self.robot = robot
        self.map_origin_in_world = map_origin_in_world
        self.max_gap = max_gap
        self.moving_v = moving_v
        self.moving_w = moving_w
        self.skip_initial_stationary = skip_initial_stationary
        self.skipped_initial = 0
        # 정답은 받을 때 바로 map 기준으로 바꿔 둔다 (저장·평가 때 다시 변환하지 않게)
        self._gt_map: list[Pose2D] = []
        self._est: list[Pose2D] = []
        self.stream_dir = stream_dir
        self._streams = None
        self._unflushed = 0
        if stream_dir:
            gt_path, est_path = trajectory_paths(stream_dir, robot)
            os.makedirs(os.path.dirname(gt_path), exist_ok=True)
            self._streams = (open(gt_path, 'w', encoding='utf-8'), open(est_path, 'w', encoding='utf-8'))

    @classmethod
    def from_tum(cls, out_dir: str, robot: str, **kwargs) -> 'EvalRecorder':
        """저장된(또는 스트리밍된) TUM 궤적으로 다시 만든다. 정답 TUM은 이미 map 기준이다."""
        rec = cls(robot, (0.0, 0.0, 0.0), **kwargs)
        # 원래 변환값은 TUM에 남지 않는다. 요약에 null로 남겨 이미 map 기준임을 표시한다
        rec.map_origin_in_world = None
        gt_path, est_path = trajectory_paths(out_dir, robot)
        rec._gt_map = read_tum(gt_path)
        rec._est = read_tum(est_path)
        return rec

    def _write(self, index: int, pose: Pose2D) -> None:
        if self._streams is None:
            return
        self._streams[index].write(tum_line(pose))
        self._unflushed += 1
        if self._unflushed >= FLUSH_EVERY:
            self.flush()

    def add_gt(self, pose: Pose2D) -> None:
        [p] = transform_to_map([pose], self.map_origin_in_world)
        self._gt_map.append(p)
        self._write(0, p)

    def add_est(self, pose: Pose2D) -> None:
        self._est.append(pose)
        self._write(1, pose)

    def flush(self) -> None:
        if self._streams is not None:
            for f in self._streams:
                f.flush()
        self._unflushed = 0

    def close(self) -> None:
        if self._streams is not None:
            for f in self._streams:
                f.close()
            self._streams = None

    @property
    def counts(self) -> tuple[int, int]:
        """(정답 수, 추정 수)."""
        return len(self._gt_map), len(self._est)

    def evaluate(self) -> tuple[list, dict, dict]:
        """(시점별 오차, 구간별 요약, KPI 판정)."""
        gt = sorted(self._gt_map, key=lambda p: p.t)
        est = sorted(self._est, key=lambda p: p.t)
        samples = compute_errors(associate_interpolated(gt, est, self.max_gap), gt, self.moving_v, self.moving_w)
        if self.skip_initial_stationary:
            first = next((i for i, s in enumerate(samples) if s.segment != STATIONARY), len(samples))
            self.skipped_initial = first
            samples = samples[first:]
        summaries = summarize_by_segment(samples)
        return samples, summaries, kpi_check(summaries)

    def save(self, out_dir: str) -> dict:
        """파일로 저장하고 요약을 돌려준다. 같은 곳에 스트리밍 중이면 궤적은 다시 쓰지 않는다."""
        if self._streams is not None and os.path.abspath(out_dir) == os.path.abspath(self.stream_dir):
            self.flush()
        else:
            gt_path, est_path = trajectory_paths(out_dir, self.robot)
            os.makedirs(os.path.dirname(gt_path), exist_ok=True)
            write_tum(gt_path, sorted(self._gt_map, key=lambda p: p.t))
            write_tum(est_path, sorted(self._est, key=lambda p: p.t))

        samples, summaries, kpi = self.evaluate()
        write_localization_csv(os.path.join(out_dir, 'localization.csv'), samples, robot=self.robot)
        summary = {
            'robot': self.robot,
            'map_origin_in_world': None if self.map_origin_in_world is None else list(self.map_origin_in_world),
            'counts': {'gt': len(self._gt_map), 'est': len(self._est), 'paired': len(samples),
                       'skipped_initial_stationary': self.skipped_initial},
            'segments': {seg: (None if s is None else asdict(s)) for seg, s in summaries.items()},
            'kpi_pass': kpi,
        }
        with open(os.path.join(out_dir, 'localization_summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        return summary


def format_summary(summary: dict) -> str:
    """로그 한 줄 요약. 예: 'stationary n=120 rmse=0.012 max=0.031 PASS | ...'."""
    parts = []
    for seg, s in summary['segments'].items():
        ok: Optional[bool] = summary['kpi_pass'][seg]
        if s is None:
            parts.append(f'{seg} n=0')
            continue
        verdict = 'PASS' if ok else 'FAIL'
        parts.append(f"{seg} n={s['n']} rmse={s['rmse']:.3f} max={s['max']:.3f} {verdict}")
    return ' | '.join(parts)
