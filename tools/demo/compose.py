#!/usr/bin/env python3
"""식당 시연 끔·켬 비교 영상 만들기 (강조 표시 포함). 컷시트(tools/demo/cuts/*.yaml)대로 자르고 배속을 맞춰 붙인다.

    # 1) 실행마다 bag → timeline.json (컨테이너: rosbag2_py, amr_msgs)
    source /ros2_ws/install/setup.bash
    python3 tools/demo/compose.py extract runs/demo_s11-off/<날짜_실행>/<실행 id>
    python3 tools/demo/compose.py extract runs/demo_s11-on/<날짜_실행>/<실행 id>
    # 2) 컷 시각·문구 확인 (ffmpeg 없이)
    python3 tools/demo/compose.py plan  --off <끔 실행> --on <켬 실행> --cuts tools/demo/cuts/s11.yaml
    # 3) 컷마다 가운데 한 장 PNG로 미리 보기, 4) 완성본 (ffmpeg)
    python3 tools/demo/compose.py render --off <끔 실행> --on <켬 실행> --cuts tools/demo/cuts/s11.yaml --stills out/
    python3 tools/demo/compose.py render --off <끔 실행> --on <켬 실행> --cuts tools/demo/cuts/s11.yaml --out s11.mp4

실행 폴더는 하네스·버킷 그대로다: video/<카메라>.mp4 + video/video.json, rosbag/ (측정 토픽). extract가 그 안에
timeline.json을 쓴다. 영상 프레임 k의 시각 = video.json start_ns / 1e9 + k / fps (bag_to_mp4.py, 시뮬레이션 시각).
두 실행은 각자의 /demo/t0(핵심 요청)을 0초로 맞춰 같은 상대 시각을 자른다 (가이드 "두 조건을 한 화면에 비교하는 컷").
완성본: 1600×900, 컷시트 fps(20), H.264 CRF 17, yuv420p, 무음.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import overlay as ov  # noqa: E402

FFMPEG = os.environ.get('FFMPEG', 'ffmpeg')   # PATH에 없으면 FFMPEG=<경로>
TOPICS = ('/fleet/task_table', '/fleet/task_status', '/fleet/robot_states', '/fleet/congestion', '/demo/t0')


# ---------------------------------------------------------------- bag → timeline.json
def _sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag: Path):
    """측정 bag → build_timeline 원자료. 시각은 메시지 시각(시뮬레이션), 0이면 녹화 시각."""
    import rosbag2_py
    from amr_msgs.msg import FleetTaskTable, RobotState, TaskStatus, ZoneCongestionArray
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Header
    types = {'/fleet/task_table': FleetTaskTable, '/fleet/task_status': TaskStatus,
             '/fleet/robot_states': RobotState, '/fleet/congestion': ZoneCongestionArray, '/demo/t0': Header}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    have = {t.name for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in have]))
    table, statuses, poses, congestion, t0 = [], [], {}, [], []
    while reader.has_next():
        topic, data, recv = reader.read_next()
        m = deserialize_message(data, types[topic])
        stamp = m.stamp if topic == '/demo/t0' else m.header.stamp
        t = _sec(stamp) or recv * 1e-9
        if topic == '/fleet/task_table':
            table.append((t, [{'task_id': r.task.task_id, 'task_type': r.task.task_type, 'priority': r.task.priority,
                               'pickup': r.task.pickup_station_id, 'dropoff': r.task.dropoff_station_id,
                               'created': _sec(r.task.created_time), 'assignee': r.assignee_id, 'state': r.state}
                              for r in m.records]))
        elif topic == '/fleet/task_status':
            statuses.append({'t': t, 'task_id': m.task_id, 'state': m.state, 'phase': m.phase})
        elif topic == '/fleet/robot_states':
            poses.setdefault(m.robot_id, []).append((t, m.pose.position.x, m.pose.position.y))
        elif topic == '/fleet/congestion':
            congestion.append((t, [(z.zone_id, [float(a) for a in z.area], float(z.value)) for z in m.zones]))
        else:
            t0.append((t, m.frame_id))
    return table, statuses, poses, congestion, t0


def cmd_extract(args) -> int:
    run = Path(args.run)
    bag = run / 'rosbag'
    if not (bag / 'metadata.yaml').is_file():
        raise SystemExit(f'{bag}에 측정 bag이 없다')
    videos = json.loads((run / 'video' / 'video.json').read_text(encoding='utf-8'))
    table, statuses, poses, congestion, t0 = read_bag(bag)
    tl = ov.build_timeline(table, statuses, poses, congestion, t0, videos, key_task=args.key_task)
    out = Path(args.out) if args.out else run / 'timeline.json'
    out.write_text(json.dumps(tl, ensure_ascii=False), encoding='utf-8')
    m = tl['metrics']
    print(f"{out}: 핵심 작업 {tl['key_task']} ({tl['pickup']} → {tl['dropoff']}) → {tl['assignee'] or '미배정'}, "
          f"배정 +{m.get('assign_s')} s, 도착 +{m.get('arrive_s')} s, 멈춤 {m.get('stops', '-')}회")
    return 0


# ---------------------------------------------------------------- 컷 계획
def load_runs(args) -> dict[str, ov.Run]:
    runs = {}
    for side in ov.SIDES:
        path = Path(getattr(args, side))
        tl = path / 'timeline.json' if path.is_dir() else path
        if not tl.is_file():
            raise SystemExit(f'{tl}이 없다: 먼저 compose.py extract {path}')
        runs[side] = ov.Run.from_timeline(side, json.loads(tl.read_text(encoding='utf-8')))
        runs[side].dir = path if path.is_dir() else path.parent
    return runs


def plan(sheet: ov.CutSheet, runs: dict[str, ov.Run]):
    """컷마다 (컷, 요청 기준 시작 s, 원본 길이 s) 와 경고. 사건이 녹화에 없는 컷은 빼고 경고."""
    ev = ov.events(runs)
    rows, warnings = [], []
    for cut in sheet.cuts:
        if cut.result:
            rows.append((cut, None, 0.0))
            continue
        try:
            start = ov.resolve(cut.start, ev)
        except ValueError as e:
            warnings.append(f'컷 "{cut.title}" 뺌: {e}')
            continue
        w = ov.check_cut(cut, ev)
        if w:
            warnings.append(w)
        src = cut.length * cut.speed
        if start + src > ev['end']:
            warnings.append(f'"{cut.title}": 녹화 끝({ev["end"]:.1f} s)을 넘는다 ({start:.1f} + {src:.1f} s)')
        for side, r in runs.items():
            v = r.videos.get(cut.camera)
            if v is None:
                warnings.append(f'"{cut.title}": {side} 실행에 {cut.camera} 영상이 없다')
        rows.append((cut, start, src))
    return rows, warnings, ev


def cmd_plan(args) -> int:
    sheet = ov.CutSheet.load(args.cuts)
    runs = load_runs(args)
    rows, warnings, ev = plan(sheet, runs)
    values = ov.text_values(runs, ov.load_stations(), sheet.candidates)
    print('사건 (요청 기준 s): ' + ', '.join(f'{k} {v:.1f}' for k, v in ev.items() if v is not None))
    t = 0.0
    for cut, start, src in rows:
        if cut.result:
            print(f'{t:5.1f}–{t + cut.total:5.1f}  결과 장표  {ov.fill(sheet.result.get("title", ""), values)}')
        else:
            print(f'{t:5.1f}–{t + cut.total:5.1f}  {cut.camera:8s} 요청 {start:+6.1f} s부터 {src:4.1f} s ({cut.speed:g}×'
                  f'{f", 멈춤 {cut.hold:g} s" if cut.hold else ""})  {ov.fill(cut.title, values)}')
        t += cut.total
    for w in warnings:
        print('경고:', w)
    return 0


# ---------------------------------------------------------------- 영상 읽기·쓰기 (ffmpeg)
class FrameReader:
    """mp4 한 구간을 차례로 읽는다. get(시뮬레이션 시각)은 시각이 늘어나는 순서로만 부른다."""

    def __init__(self, path: Path, video: dict, sim_start: float, sim_end: float, size):
        self.fps = float(video['fps'])
        self.t_video = video['start_ns'] / 1e9
        self.frames = int(video['frames'])
        self.size = size
        k0 = min(max(int((sim_start - self.t_video) * self.fps) - 1, 0), max(self.frames - 1, 0))
        self.k = k0 - 1
        dur = max(sim_end - sim_start, 0.0) + 2.0 / self.fps + 0.5
        self.proc = subprocess.Popen(
            [FFMPEG, '-loglevel', 'error', '-ss', f'{k0 / self.fps:.4f}', '-i', str(path), '-t', f'{dur:.3f}',
             '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.cur = None

    def _read(self) -> bool:
        n = self.size[0] * self.size[1] * 3
        buf = self.proc.stdout.read(n)
        if len(buf) < n:
            return False
        self.cur = ov.Image.frombytes('RGB', self.size, buf)
        self.k += 1
        return True

    def get(self, t: float):
        want = min(max(round((t - self.t_video) * self.fps), 0), self.frames - 1)
        while self.k < want or self.cur is None:
            if not self._read():
                break
        return self.cur

    def close(self):
        self.proc.kill()          # 구간보다 넉넉히 읽게 했으니 남은 것은 버린다
        self.proc.stdout.close()
        self.proc.wait()


class Writer:
    def __init__(self, path: str, fps: int, crf: int):
        self.proc = subprocess.Popen(
            [FFMPEG, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '%dx%d' % ov.CANVAS,
             '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', str(crf),
             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', path], stdin=subprocess.PIPE)

    def write(self, img):
        self.proc.stdin.write(img.tobytes())

    def close(self):
        self.proc.stdin.close()
        if self.proc.wait() != 0:
            raise RuntimeError('ffmpeg 인코딩 실패')


def cmd_render(args) -> int:
    sheet = ov.CutSheet.load(args.cuts)
    runs = load_runs(args)
    rows, warnings, _ = plan(sheet, runs)
    for w in warnings:
        print('경고:', w)
    cameras = ov.load_cameras()
    stations = ov.load_stations()
    fonts = ov.Fonts(args.font)
    painter = ov.Painter(sheet, cameras, stations, fonts)
    values = ov.text_values(runs, stations, sheet.candidates)
    fps = sheet.fps
    total = sum(c.total for c, _, _ in rows)
    writer = Writer(args.out, fps, sheet.crf) if args.out else None
    stills = Path(args.stills) if args.stills else None
    if stills:
        stills.mkdir(parents=True, exist_ok=True)
    if not writer and not stills:
        raise SystemExit('--out(완성본)이나 --stills(컷마다 한 장) 중 하나는 준다')
    done = 0.0
    for i, (cut, start, src) in enumerate(rows, 1):
        n = round(cut.total * fps)
        if cut.result:
            img = ov.result_frame(sheet, runs, fonts, i, len(rows), values)
            if stills:
                img.save(stills / f'{i:02d}_result.png')
            for _ in range(n if writer else 0):
                writer.write(img)
            done += cut.total
            continue
        cam = cameras[cut.camera]
        readers = {}
        for side, r in runs.items():
            v = r.videos[cut.camera]
            readers[side] = FrameReader(r.dir / 'video' / f'{cut.camera}.mp4', v, r.t0 + start,
                                        r.t0 + start + src, (cam.width, cam.height))
        picks = range(n) if writer else [min(round(cut.length * fps / 2), n - 1)]
        for j in picks:
            tau = min(j / fps, cut.length)
            rel = start + tau * cut.speed
            panels = {}
            for side, r in runs.items():
                frame = readers[side].get(r.t0 + rel)
                panels[side] = painter.video(frame, cut.camera, r, r.t0 + rel, cut.layers)
            img = painter.frame(cut, i, len(rows), (done + j / fps) / total, panels, values)
            if writer:
                writer.write(img)
            if stills and j == min(round(cut.length * fps / 2), n - 1):
                img.save(stills / f'{i:02d}_{cut.camera}.png')
        for rd in readers.values():
            rd.close()
        done += cut.total
        print(f'[{i}/{len(rows)}] {ov.fill(cut.title, values)}')
    if writer:
        writer.close()
        print(f'{args.out}: {total:.1f} s, {ov.CANVAS[0]}×{ov.CANVAS[1]} {fps} fps')
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='식당 시연 끔·켬 비교 영상 (강조 표시)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('extract', help='실행 폴더의 rosbag → timeline.json (ROS 필요)')
    e.add_argument('run', help='실행 폴더 (video/, rosbag/가 있는 곳)')
    e.add_argument('--key-task', default='', help='핵심 작업 id (기본: /demo/t0의 frame_id)')
    e.add_argument('--out', default='', help='기본 <실행 폴더>/timeline.json')
    for name, fn in (('plan', cmd_plan), ('render', cmd_render)):
        p = sub.add_parser(name)
        p.add_argument('--off', required=True, help='끔 실행 폴더 (또는 timeline.json)')
        p.add_argument('--on', required=True, help='켬 실행 폴더 (또는 timeline.json)')
        p.add_argument('--cuts', default=str(HERE / 'cuts' / 's11.yaml'))
        if name == 'render':
            p.add_argument('--out', default='', help='완성본 mp4')
            p.add_argument('--stills', default='', help='컷마다 가운데 한 장을 PNG로 쓸 폴더')
            p.add_argument('--font', default=None, help='한글 글꼴 (기본: wqy-zenhei → 맑은 고딕 순)')
        p.set_defaults(fn=fn)
    e.set_defaults(fn=cmd_extract)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    raise SystemExit(main())
