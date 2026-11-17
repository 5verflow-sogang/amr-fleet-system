#!/usr/bin/env python3
"""YOLO 인식 FPS KPI(원 명세 6장 "CPU 10 FPS")를 실행 로그에서 뽑는다. ROS 없이 돈다 (표준 라이브러리만).

    python3 tools/harness/detection_fps.py <실행 폴더>        # → <실행 폴더>/detection/fps.json

detection_node(amr_detection)는 stats_period(5 s)마다 이런 줄을 남긴다. launch 출력이라 logs/<프로세스>.log에 있다.
    [detection_node-12] [INFO] [1730512345.123456789] [amr_01.detection]: 11.8 FPS, 추론 62 ms, 촬영→발행 95 ms,
    객체 3개, 깊이 없어 뺀 검출 1개, TF 실패 0프레임   (실제로는 한 줄)
FPS는 그 구간에 발행한 프레임 수 ÷ 실제 시간(time.monotonic)이고, 대괄호 안 시각은 로그 시각(실제 시간, UNIX 초)이다.
구간은 앞 통계 줄 ~ 이 줄이다 (노드가 줄을 남기며 통계를 비운다).

측정 구간: yolo_fps 시나리오의 주행 명령이 detection/window_start·window_end에 실제 시각을 적는다. 구간이 이 안에
다 드는 줄만 쓴다 (데움·시작에 걸친 구간은 빠진다). 표시 파일이 없으면 노드마다 첫 skip개 줄을 데움으로 빼고 나머지를 쓴다.

결과 fps.json: fps_mean(프레임 수 합 ÷ 시간 합, KPI), fps_min·fps_max(구간), infer_ms_mean·latency_ms_mean(프레임 가중),
frames(구간 FPS × 구간 길이의 합, 추정), windows, duration_s, objects_per_frame, no_depth, tf_fail, 구간 목록.
쓸 구간이 하나도 없으면 1로 끝난다 (하네스가 후처리 실패로 남긴다).
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Iterable, Optional

ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
# detection_node._log_stats와 같은 형식. 앞의 [시각]은 로그 형식(RCUTILS_CONSOLE_OUTPUT_FORMAT)에 따라 없을 수 있다
STATS_RE = re.compile(
    r'(?:\[(?P<t>\d+(?:\.\d+)?)\] )?\[(?P<logger>[^\]\s]+)\]: '
    r'(?P<fps>\d+(?:\.\d+)?) FPS, 추론 (?P<infer>-?\d+) ms, 촬영→발행 (?P<latency>-?\d+) ms, '
    r'객체 (?P<objects>\d+)개, 깊이 없어 뺀 검출 (?P<no_depth>\d+)개, TF 실패 (?P<tf_fail>\d+)프레임')


def parse_lines(lines: Iterable[str]) -> list[dict]:
    """통계 줄 → [{t, logger, fps, infer_ms, latency_ms, objects, no_depth, tf_fail}] (나온 순서). 다른 줄은 건너뛴다."""
    out = []
    for line in lines:
        m = STATS_RE.search(ANSI_RE.sub('', line))
        if m:
            out.append({'t': float(m['t']) if m['t'] else None, 'logger': m['logger'], 'fps': float(m['fps']),
                        'infer_ms': float(m['infer']), 'latency_ms': float(m['latency']),
                        'objects': int(m['objects']), 'no_depth': int(m['no_depth']), 'tf_fail': int(m['tf_fail'])})
    return out


def select_windows(stats: list[dict], start: Optional[float] = None, end: Optional[float] = None,
                   skip: int = 1, period: float = 5.0) -> list[dict]:
    """측정할 구간만 골라 구간 길이 dt [s]를 붙인다. 노드(logger)마다 따로 본다.

    start·end가 있으면 구간(앞 줄 시각 ~ 이 줄 시각)이 그 안에 다 드는 줄만 쓴다. 시각이 없는 줄과 노드의 첫 줄은
    구간 시작을 몰라 뺀다. 없으면 노드마다 첫 skip개 줄을 데움으로 뺀다. dt는 앞 줄과의 시각 차, 모르면 period.
    """
    prev: dict[str, Optional[float]] = {}
    count: dict[str, int] = {}
    out = []
    for s in stats:
        logger = s['logger']
        t_prev = prev.get(logger)
        prev[logger] = s['t']
        count[logger] = count.get(logger, 0) + 1
        known = s['t'] is not None and t_prev is not None
        if start is not None or end is not None:
            if not known or (start is not None and t_prev < start) or (end is not None and s['t'] > end):
                continue
        elif count[logger] <= skip:
            continue
        dt = s['t'] - t_prev if known and s['t'] > t_prev else period
        out.append({**s, 'dt': dt})
    return out


def summarize(windows: list[dict]) -> dict:
    """구간들 → FPS·추론·지연 요약. 평균 FPS는 구간 평균이 아니라 프레임 수 합 ÷ 시간 합이다."""
    if not windows:
        return {'windows': 0, 'frames': 0, 'duration_s': 0.0, 'fps_mean': None, 'fps_min': None, 'fps_max': None,
                'infer_ms_mean': None, 'latency_ms_mean': None, 'objects_per_frame': None, 'no_depth': 0,
                'tf_fail': 0, 'robots': []}
    frames = [w['fps'] * w['dt'] for w in windows]
    total_frames = sum(frames)
    total_s = sum(w['dt'] for w in windows)

    def per_frame(key):
        # 로그 값은 구간의 프레임 평균이라 프레임 수로 가중한다
        return round(sum(w[key] * f for w, f in zip(windows, frames)) / total_frames, 1) if total_frames > 0 else None

    return {
        'windows': len(windows), 'frames': round(total_frames), 'duration_s': round(total_s, 3),
        'fps_mean': total_frames / total_s, 'fps_min': min(w['fps'] for w in windows),
        'fps_max': max(w['fps'] for w in windows),
        'infer_ms_mean': per_frame('infer_ms'), 'latency_ms_mean': per_frame('latency_ms'),
        'objects_per_frame': sum(w['objects'] for w in windows) / total_frames if total_frames > 0 else None,
        'no_depth': sum(w['no_depth'] for w in windows), 'tf_fail': sum(w['tf_fail'] for w in windows),
        # 로거 이름 amr_01.detection → 로봇 amr_01
        'robots': sorted({w['logger'].rsplit('.', 1)[0] for w in windows}),
    }


def _read_time(path: Path) -> Optional[float]:
    try:
        return float(path.read_text(encoding='utf-8').strip())
    except (OSError, ValueError):
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='detection_node 통계 줄 → FPS KPI (detection/fps.json)')
    ap.add_argument('run_dir', type=Path, help='하네스 실행 폴더 (logs/*.log를 읽는다)')
    ap.add_argument('--out', type=Path, help='결과 json (기본 <실행 폴더>/detection/fps.json)')
    ap.add_argument('--skip', type=int, default=1, help='측정 구간 표시가 없을 때 노드마다 데움으로 뺄 첫 통계 줄 수')
    ap.add_argument('--period', type=float, default=5.0,
                    help='시각이 없는 줄의 구간 길이 [s] (detection.yaml stats_period)')
    args = ap.parse_args(argv)

    det_dir = args.run_dir / 'detection'
    start, end = _read_time(det_dir / 'window_start'), _read_time(det_dir / 'window_end')
    lines: list[str] = []
    for log in sorted((args.run_dir / 'logs').glob('*.log')):
        lines += log.read_text(encoding='utf-8', errors='replace').splitlines()
    stats = parse_lines(lines)
    windows = select_windows(stats, start, end, args.skip, args.period)
    result = {**summarize(windows), 'stats_lines': len(stats), 'window_start': start, 'window_end': end,
              'window_list': [{k: w[k] for k in ('t', 'dt', 'logger', 'fps', 'infer_ms', 'latency_ms', 'objects',
                                                 'no_depth', 'tf_fail')} for w in windows]}
    out = args.out or det_dir / 'fps.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    if not windows:
        print(f'측정 구간 안의 통계 줄이 없다 (통계 줄 {len(stats)}개, 구간 {start} ~ {end}): {out}', file=sys.stderr)
        return 1
    print(f"FPS 평균 {result['fps_mean']:.1f} (최소 {result['fps_min']:.1f}), 추론 {result['infer_ms_mean']} ms, "
          f"촬영→발행 {result['latency_ms_mean']} ms, 구간 {result['windows']}개 {result['duration_s']:.0f} s: {out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
