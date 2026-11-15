#!/usr/bin/env python3
"""식당 시연 녹화 rosbag → 지표 표. 컨테이너에서 돈다 (rosbag2_py, amr_msgs).

    source /ros2_ws/install/setup.bash
    python3 experiments/restaurant_demo/analyze.py runs/ --out experiments/restaurant_demo/results.md

입력: 하네스·버킷 경로 그대로 runs/demo_<장면>-<on|off>/<실행 id>/ (#319 demo_s11.yaml 머리말).
- 장면: demo_s11 → 1-1, demo_s12 → 1-2, demo_s2…(장면 2, 1막·2막 이름 미정) → 2. 숫자 이름(demo_1-2-on)도 받는다
- 실행 id(하네스 run.py: <시나리오>-<라벨>_<YYYYMMDD-HHMMSS>_<nn>)마다 한 줄. 표에는 장면 폴더마다 가장 최근 실행만
  쓰고(--all이면 전부), .json에는 전부 남긴다. 장면 폴더 바로 아래에 bag이 있으면 그 폴더를 실행 하나로 본다
- bag: 측정용 rosbag/ 폴더를 먼저 본다. 카메라 bag(camera_bag/)은 mp4로 바꾸지 못해 남아 있어도 고르지 않는다
지표 (정의는 demo_metrics.py):
- 1-2: /fleet/task_table(요청 시각·자리), /fleet/task_status(도착), /fleet/robot_states(멈춘 횟수)
- 2: /fleet/task_table 만 쓴다. 1막 회수 대기, 2막 기아 시간
- 1-1: 아직 지표가 없어 작업 수만
--act2 START END (요청 첫 시각 기준 s)를 주면 기아 시간을 그 구간과 겹치는 것만 본다. 안 주면 녹화 전체.
"""

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo_metrics as dm  # noqa: E402

RUN_DIR = re.compile(r'^demo_(?P<scene>s?[0-9][0-9a-z_-]*?)-(?P<mode>on|off)$')
RUN_STAMP = re.compile(r'_(\d{8}-\d{6})_(\d+)$')
MEASURE_BAG = 'rosbag'
CAMERA_BAG = 'camera_bag'
TOPICS = ('/fleet/task_table', '/fleet/task_status', '/fleet/robot_states')


def scene_name(raw):
    """폴더의 장면 이름 → 지표 장면. s11 → 1-1, s12 → 1-2, s2… → 2. 숫자 이름(1-2, 2-1)은 그대로."""
    if not raw.startswith('s'):
        return raw
    digits = raw[1:]
    if digits.startswith('2'):
        return '2'
    if len(digits) == 2 and digits.isdigit():
        return f'{digits[0]}-{digits[1]}'
    return digits


def parse_run_name(name):
    """'demo_s12-on' → ('1-2', 'on'), 'demo_1-2-on' → ('1-2', 'on'). 규칙에 안 맞으면 None."""
    m = RUN_DIR.match(name)
    return (scene_name(m['scene']), m['mode']) if m else None


def is_bag(path):
    return (path / 'metadata.yaml').is_file()


def find_bag(run):
    """실행 폴더 → 측정 bag. rosbag/ 먼저, 다음은 폴더 자체, 그다음 아래 bag 중 카메라 bag이 아닌 것."""
    if is_bag(run / MEASURE_BAG):
        return run / MEASURE_BAG
    if is_bag(run):
        return run
    hits = sorted(p.parent for p in run.rglob('metadata.yaml') if CAMERA_BAG not in p.relative_to(run).parts)
    return hits[0] if hits else None


def run_order(run):
    """오래된 것부터 정렬하는 열쇠: 이름의 시각·번호(하네스 실행 id), 없으면 폴더 수정 시각."""
    m = RUN_STAMP.search(run.name)
    return (1, m[1], int(m[2]), 0.0) if m else (0, '', 0, run.stat().st_mtime)


def run_dirs(scene_dir):
    """장면 폴더 → 실행 폴더들 (오래된 것부터). 장면 폴더 바로 아래에 bag이 있으면 그 폴더 하나."""
    if is_bag(scene_dir) or is_bag(scene_dir / MEASURE_BAG):
        return [scene_dir]
    subs = [p for p in scene_dir.iterdir() if p.is_dir() and p.name != CAMERA_BAG]
    return sorted(subs, key=run_order) or [scene_dir]


def _sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag):
    """bag → (작업표 스냅숏, 작업 상태, 로봇 자세, 마지막 시각). 시각은 메시지 시각, 0이면 녹화 시각."""
    import rosbag2_py
    from amr_msgs.msg import FleetTaskTable, RobotState, TaskStatus
    from rclpy.serialization import deserialize_message
    types = {'/fleet/task_table': FleetTaskTable, '/fleet/task_status': TaskStatus,
             '/fleet/robot_states': RobotState}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    table, statuses, poses, last = [], [], {}, 0.0
    while reader.has_next():
        topic, data, recv = reader.read_next()
        m = deserialize_message(data, types[topic])
        t = _sec(m.header.stamp) or recv * 1e-9
        last = max(last, t)
        if topic == '/fleet/task_table':
            table.append((t, [{'task_id': r.task.task_id, 'task_type': r.task.task_type, 'priority': r.task.priority,
                               'pickup': r.task.pickup_station_id, 'dropoff': r.task.dropoff_station_id,
                               'created': _sec(r.task.created_time), 'assignee': r.assignee_id, 'state': r.state}
                              for r in m.records]))
        elif topic == '/fleet/task_status':
            statuses.append({'t': t, 'task_id': m.task_id, 'state': m.state, 'phase': m.phase})
        else:
            poses.setdefault(m.robot_id, []).append((t, m.pose.position.x, m.pose.position.y))
    return table, statuses, poses, last


def analyze_run(scene_dir, run, station, act2, stop_kw):
    scene, mode = parse_run_name(scene_dir.name)
    name = scene_dir.name if run == scene_dir else f'{scene_dir.name}/{run.name}'
    base = {'run': name, 'scene_dir': scene_dir.name, 'run_id': run.name, 'scene': scene, 'mode': mode}
    bag = find_bag(run)
    if bag is None:
        return {**base, 'error': 'rosbag 없음'}
    table, statuses, poses, last = read_bag(bag)
    tasks = dm.fold_table(table)
    out = {**base, 'bag': str(bag), 'tasks': len(tasks)}
    if scene == '1-2':
        out['scene12'] = dm.scene12(tasks, statuses, poses, station=station, end=last, **stop_kw)
    elif scene.startswith('2'):
        t0 = min((k.request for k in tasks.values()), default=0.0)
        window = (t0 + act2[0], t0 + act2[1]) if act2 else None
        out['scene2'] = dm.scene2(tasks, last, window)
    return out


def latest(results):
    """장면 폴더마다 가장 최근 실행 (results는 장면 폴더 안에서 오래된 것부터)."""
    last = {}
    for r in results:
        last[r['scene_dir']] = r
    return [r for r in results if last[r['scene_dir']] is r]


def table_md(results):
    lines = []
    s12 = [(r, row) for r in results for row in r.get('scene12', [])]
    if s12:
        lines += ['| 실행 | 작업 | 요청 → 도착 (s) | 멈춘 횟수 (합 s) | 정지 비율 | 출발 전 대기 (s) |',
                  '|---|---|---|---|---|---|']
        for r, row in s12:
            arrive = row['request_to_arrival_s']
            ratio = row.get('stop_ratio')
            lines.append(f"| {r['run']} | {row['task_id']} | {'-' if arrive is None else arrive} | "
                         f"{row.get('stops', '-')} ({row.get('stop_s', '-')}) | "
                         f"{'-' if ratio is None else f'{ratio:.0%}'} | {row.get('start_wait_s', '-')} |")
        lines.append('')
    s2 = [r for r in results if 'scene2' in r]
    if s2:
        lines += ['| 실행 | 1막 회수 대기 평균 / 최대 (s) | 회수 미할당 | 2막 기아 시간 (s) | 완료 / 실패 |',
                  '|---|---|---|---|---|']
        for r in s2:
            s = r['scene2']
            lines.append(f"| {r['run']} | {s['collect_wait_mean_s']} / {s['collect_wait_max_s']} | "
                         f"{s['collect_unassigned']}/{s['collect_n']} | {s['starvation_s']} | "
                         f"{s['completed']} / {s['failed']} |")
    for r in results:
        if 'error' in r:
            lines.append(f"- {r['run']}: {r['error']}")
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', help='runs/ (demo_<장면>-<on|off>/<실행 id>/ 폴더들이 있는 곳)')
    ap.add_argument('--out', help='표(.md). 같은 이름 .json에 실행 id마다 원값')
    ap.add_argument('--station', default='buffet_2', help='1-2 도착 자리')
    ap.add_argument('--act2', nargs=2, type=float, metavar=('START', 'END'), help='2막 구간 (첫 요청 기준 s)')
    ap.add_argument('--stop-speed', type=float, default=0.05)
    ap.add_argument('--stop-min-s', type=float, default=1.0)
    ap.add_argument('--all', action='store_true', help='표에 실행 id 전부 (기본은 장면 폴더마다 가장 최근 실행)')
    a = ap.parse_args(argv)
    scenes = sorted(p for p in Path(a.runs).iterdir() if p.is_dir() and parse_run_name(p.name))
    if not scenes:
        raise SystemExit(f'{a.runs}에 demo_<장면>-<on|off>/ 폴더가 없다 (예: demo_s12-on/<실행 id>/)')
    stop_kw = {'stop_speed': a.stop_speed, 'stop_min_s': a.stop_min_s}
    results = [analyze_run(s, r, a.station, a.act2, stop_kw) for s in scenes for r in run_dirs(s)]
    md = table_md(results if a.all else latest(results))
    print(md)
    if a.out:
        out = Path(a.out)
        out.write_text(md, encoding='utf-8')
        out.with_suffix('.json').write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
