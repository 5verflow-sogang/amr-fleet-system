"""YOLO FPS 로그 파서 (detection_fps.py, yolo_fps 시나리오). 가짜 launch 로그로 본다. ROS 없이 돈다."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import detection_fps  # noqa: E402

T0 = 1730512300.0


def stats_line(t, fps, infer=80, latency=120, objects=0, no_depth=0, tf_fail=0, robot='amr_01'):
    # launch 출력 그대로: [프로세스] [수준] [로그 시각] [로거]: 메시지 (detection_node._log_stats 형식)
    return (f'[detection_node-14] [INFO] [{T0 + t:.9f}] [{robot}.detection]: {fps:.1f} FPS, 추론 {infer} ms, '
            f'촬영→발행 {latency} ms, 객체 {objects}개, 깊이 없어 뺀 검출 {no_depth}개, TF 실패 {tf_fail}프레임')


LOG = [
    f'[detection_node-14] [INFO] [{T0:.9f}] [amr_01.detection]: '
    'detection: camera → perception/objects (map), 입력 640 px',
    f'[detection_node-14] [WARN] [{T0 + 1:.9f}] [amr_01.detection]: TF 없음, 프레임 건너뜀: "map" passed to lookupTransform',
    stats_line(10, 2.0, infer=300),             # 노드 첫 줄: 구간 시작을 모른다
    stats_line(15, 8.0),                         # 10~15: 측정 시작(18) 전
    stats_line(20, 12.0),                        # 15~20: 시작에 걸침
    stats_line(25, 10.0, 80, 120, 100, 2, 0),    # 20~25 ┐
    '\x1b[0m' + stats_line(31, 12.0, 70, 100, 144, 0, 1) + '\x1b[0m',   # 25~31 │ 측정 구간 18~40 (색 코드가 붙어도 읽는다)
    stats_line(36, 9.0, 90, 130, 45, 3, 0),      # 31~36 ┘
    stats_line(42, 11.0),                        # 36~42: 끝(40)을 넘음
    '[system-1] [INFO] [launch]: process has finished cleanly',
]


def test_parse_lines_reads_only_stats_lines():
    stats = detection_fps.parse_lines(LOG)
    assert len(stats) == 7
    s = stats[3]
    assert s == {'t': pytest.approx(T0 + 25), 'logger': 'amr_01.detection', 'fps': 10.0, 'infer_ms': 80.0,
                 'latency_ms': 120.0, 'objects': 100, 'no_depth': 2, 'tf_fail': 0}
    # 로그 형식에 시각이 없으면 t는 None
    plain = detection_fps.parse_lines(['[INFO] [amr_01.detection]: 9.5 FPS, 추론 95 ms, 촬영→발행 140 ms, '
                                       '객체 2개, 깊이 없어 뺀 검출 0개, TF 실패 0프레임'])
    assert plain[0]['t'] is None and plain[0]['fps'] == 9.5


def test_window_markers_keep_only_windows_fully_inside():
    windows = detection_fps.select_windows(detection_fps.parse_lines(LOG), start=T0 + 18, end=T0 + 40)
    assert [w['fps'] for w in windows] == [10.0, 12.0, 9.0]
    assert [round(w['dt'], 6) for w in windows] == [5.0, 6.0, 5.0]
    s = detection_fps.summarize(windows)
    # 프레임 50 + 72 + 45 = 167, 시간 16 s. 구간 평균(10.33)이 아니라 프레임 수 합 ÷ 시간 합
    assert s['frames'] == 167 and s['duration_s'] == pytest.approx(16.0)
    assert s['fps_mean'] == pytest.approx(167 / 16)
    assert (s['fps_min'], s['fps_max'], s['windows']) == (9.0, 12.0, 3)
    # 추론·지연은 프레임 가중 평균
    assert s['infer_ms_mean'] == pytest.approx((80 * 50 + 70 * 72 + 90 * 45) / 167, abs=0.05)
    assert s['latency_ms_mean'] == pytest.approx((120 * 50 + 100 * 72 + 130 * 45) / 167, abs=0.05)
    assert s['objects_per_frame'] == pytest.approx(289 / 167)
    assert (s['no_depth'], s['tf_fail'], s['robots']) == (5, 1, ['amr_01'])


def test_without_markers_skips_first_lines_per_node():
    other = [stats_line(12, 20.0, robot='amr_02'), stats_line(17, 20.0, robot='amr_02')]
    stats = detection_fps.parse_lines(LOG + other)
    windows = detection_fps.select_windows(stats, skip=2)
    # amr_01은 첫 두 줄(10, 15 s)을 빼고 5개, amr_02는 2줄뿐이라 모두 빠진다
    assert [w['fps'] for w in windows] == [12.0, 10.0, 12.0, 9.0, 11.0]
    assert windows[0]['dt'] == pytest.approx(5.0)
    # 시각이 없는 줄은 구간 길이를 stats_period로 본다
    plain = detection_fps.parse_lines(['[amr_01.detection]: 10.0 FPS, 추론 90 ms, 촬영→발행 100 ms, 객체 1개, '
                                       '깊이 없어 뺀 검출 0개, TF 실패 0프레임'] * 3)
    assert [w['dt'] for w in detection_fps.select_windows(plain, skip=1, period=5.0)] == [5.0, 5.0]
    # 시각이 없으면 측정 구간 표시가 있어도 고를 수 없다
    assert detection_fps.select_windows(plain, start=0.0, end=1e12) == []


def test_main_writes_json_and_fails_without_windows(tmp_path):
    run = tmp_path / 'run'
    (run / 'logs').mkdir(parents=True)
    (run / 'detection').mkdir()
    (run / 'logs' / 'system.log').write_text('\n'.join(LOG) + '\n', encoding='utf-8')
    (run / 'logs' / 'drive.log').write_text('구간 끝\n', encoding='utf-8')
    (run / 'detection' / 'window_start').write_text(f'{T0 + 18:.9f}\n')
    (run / 'detection' / 'window_end').write_text(f'{T0 + 40:.9f}\n')
    assert detection_fps.main([str(run)]) == 0
    out = json.loads((run / 'detection' / 'fps.json').read_text(encoding='utf-8'))
    assert out['windows'] == 3 and out['stats_lines'] == 7 and out['fps_mean'] == pytest.approx(167 / 16)
    assert len(out['window_list']) == 3 and out['window_start'] == pytest.approx(T0 + 18)
    # 측정 구간 안에 통계 줄이 없으면 1 (하네스 후처리 실패). json은 남긴다
    (run / 'detection' / 'window_start').write_text(f'{T0 + 100:.9f}\n')
    (run / 'detection' / 'window_end').write_text(f'{T0 + 200:.9f}\n')
    assert detection_fps.main([str(run)]) == 1
    out = json.loads((run / 'detection' / 'fps.json').read_text(encoding='utf-8'))
    assert out['windows'] == 0 and out['fps_mean'] is None
