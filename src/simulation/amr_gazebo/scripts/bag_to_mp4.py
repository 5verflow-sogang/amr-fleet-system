#!/usr/bin/env python3
"""rosbag의 카메라 이미지(sensor_msgs/Image) → 토픽마다 mp4 한 개 (시뮬레이션 시각 기준 fps).

    ros2 run amr_gazebo bag_to_mp4.py <bag 폴더> --out <폴더>            # /demo_cam/<이름>/image → <이름>.mp4
    ros2 run amr_gazebo bag_to_mp4.py <bag 폴더> --out <폴더> --fps 15 --crf 23

영상마다 첫 프레임의 시뮬레이션 시각을 <out>/video.json에 남긴다. 켬/끔 영상을 /demo/t0(핵심 요청 시각)에
맞출 때 (t0 - start_ns)/1e9 초를 앞에서 자른다. 이미지를 메모리에 모으지 않고 ffmpeg로 바로 흘려 보내
카메라 3대 8 GB 녹화도 그대로 바꾼다.
"""

import argparse
import json
import os
import subprocess

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from amr_gazebo.demo_video import Resampler

PIX_FMT = {'rgb8': 'rgb24', 'bgr8': 'bgr24', 'mono8': 'gray'}


def video_name(topic: str) -> str:
    """/demo_cam/top/image → top, 그 밖에는 토픽을 _로 이은 이름."""
    parts = [p for p in topic.split('/') if p]
    if len(parts) == 3 and parts[0] == 'demo_cam':
        return parts[1]
    return '_'.join(parts)


class Encoder:
    def __init__(self, path: str, msg: Image, fps: float, crf: int):
        if msg.encoding not in PIX_FMT:
            raise ValueError(f'지원하지 않는 이미지 형식: {msg.encoding}')
        self.size = (msg.width, msg.height)
        self.proc = subprocess.Popen([
            'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', PIX_FMT[msg.encoding],
            '-s', f'{msg.width}x{msg.height}', '-r', f'{fps:g}', '-i', '-',
            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', str(crf), '-pix_fmt', 'yuv420p', path,
        ], stdin=subprocess.PIPE)

    def write(self, frame: bytes, times: int) -> None:
        for _ in range(times):
            self.proc.stdin.write(frame)

    def close(self) -> None:
        self.proc.stdin.close()
        if self.proc.wait() != 0:
            raise RuntimeError(f'ffmpeg 실패 (종료 코드 {self.proc.returncode})')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='rosbag 카메라 이미지 → mp4 (시뮬레이션 시각 기준)')
    ap.add_argument('bag', help='rosbag 폴더')
    ap.add_argument('--out', required=True, help='mp4를 쓸 폴더')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--crf', type=int, default=23, help='x264 화질 (낮을수록 좋고 크다)')
    ap.add_argument('--storage', default='sqlite3')
    args = ap.parse_args(argv)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    topics = [t.name for t in reader.get_all_topics_and_types() if t.type == 'sensor_msgs/msg/Image']
    if not topics:
        raise SystemExit(f'{args.bag}에 sensor_msgs/Image 토픽이 없다')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    os.makedirs(args.out, exist_ok=True)

    # 토픽마다: 인코더, 시각 재표본기, 직전 프레임, 첫 시각
    state = {t: {'enc': None, 'res': Resampler(args.fps), 'prev': None, 'start_ns': None, 'dropped': 0}
             for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, Image)
        s = state[topic]
        if msg.step != msg.width * {'mono8': 1}.get(msg.encoding, 3):
            raise ValueError(f'{topic}: 줄 끝 여백이 있는 이미지는 지원하지 않는다 (step {msg.step})')
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if s['enc'] is None:
            s['enc'] = Encoder(os.path.join(args.out, f'{video_name(topic)}.mp4'), msg, args.fps, args.crf)
            s['start_ns'] = stamp
        n = s['res'].push(stamp)
        if n < 0:
            s['dropped'] += 1
            continue
        if s['prev'] is not None:
            s['enc'].write(s['prev'], n)
        s['prev'] = bytes(msg.data)

    summary = {}
    for topic, s in state.items():
        if s['enc'] is None:
            continue
        s['enc'].write(s['prev'], s['res'].finish())
        s['enc'].close()
        summary[video_name(topic)] = {'topic': topic, 'start_ns': s['start_ns'], 'fps': args.fps,
                                      'frames': s['res'].frames, 'dropped_out_of_order': s['dropped']}
    with open(os.path.join(args.out, 'video.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    for name, v in summary.items():
        print(f"{name}.mp4: {v['frames']}프레임 ({v['frames'] / args.fps:.1f} s), 시작 {v['start_ns'] / 1e9:.3f} s")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
