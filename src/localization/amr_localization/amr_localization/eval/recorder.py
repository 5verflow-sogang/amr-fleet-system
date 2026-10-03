"""정답·추정 자세를 모아 하네스 형식으로 저장한다. ROS에 의존하지 않는다 (노드는 eval_node.py).

저장 파일 (tools/harness/README.md 실행 결과 배치):
    <out_dir>/trajectories/<robot>_gt.tum    정답 궤적 (map 기준으로 변환)
    <out_dir>/trajectories/<robot>_est.tum   추정 궤적 (map 기준)
    <out_dir>/localization.csv               시점별 오차
    <out_dir>/localization_summary.json      구간별 RMSE·Max와 KPI 판정
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict
from typing import Optional

from .metrics import compute_errors, kpi_check, summarize_by_segment, write_localization_csv
from .trajectory import Pose2D, associate_interpolated, transform_to_map, write_tum


class EvalRecorder:
    """정답(world 기준)과 추정(map 기준) 자세를 모은다.

    map_origin_in_world: world에서 본 map 원점 (x, y, yaw). 지도 인계 때 받은 매핑 시작 자세.
    """

    def __init__(
        self,
        robot: str,
        map_origin_in_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
        max_gap: float = 0.05,
        moving_v: float = 0.01,
        moving_w: float = 0.05,
    ):
        self.robot = robot
        self.map_origin_in_world = map_origin_in_world
        self.max_gap = max_gap
        self.moving_v = moving_v
        self.moving_w = moving_w
        self._gt_world: list[Pose2D] = []
        self._est: list[Pose2D] = []

    def add_gt(self, pose: Pose2D) -> None:
        self._gt_world.append(pose)

    def add_est(self, pose: Pose2D) -> None:
        self._est.append(pose)

    @property
    def counts(self) -> tuple[int, int]:
        """(정답 수, 추정 수)."""
        return len(self._gt_world), len(self._est)

    def evaluate(self) -> tuple[list, dict, dict]:
        """(시점별 오차, 구간별 요약, KPI 판정)."""
        gt = sorted(transform_to_map(self._gt_world, self.map_origin_in_world), key=lambda p: p.t)
        est = sorted(self._est, key=lambda p: p.t)
        samples = compute_errors(associate_interpolated(gt, est, self.max_gap), gt, self.moving_v, self.moving_w)
        summaries = summarize_by_segment(samples)
        return samples, summaries, kpi_check(summaries)

    def save(self, out_dir: str) -> dict:
        """파일로 저장하고 요약을 돌려준다."""
        traj_dir = os.path.join(out_dir, 'trajectories')
        os.makedirs(traj_dir, exist_ok=True)
        gt_map = sorted(transform_to_map(self._gt_world, self.map_origin_in_world), key=lambda p: p.t)
        write_tum(os.path.join(traj_dir, f'{self.robot}_gt.tum'), gt_map)
        write_tum(os.path.join(traj_dir, f'{self.robot}_est.tum'), sorted(self._est, key=lambda p: p.t))

        samples, summaries, kpi = self.evaluate()
        write_localization_csv(os.path.join(out_dir, 'localization.csv'), samples, robot=self.robot)
        summary = {
            'robot': self.robot,
            'map_origin_in_world': list(self.map_origin_in_world),
            'counts': {'gt': len(self._gt_world), 'est': len(self._est), 'paired': len(samples)},
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
