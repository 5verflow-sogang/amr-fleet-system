"""Gazebo에서 YOLO 학습 데이터(사진 + 라벨)를 모으는 노드. 라벨은 라벨 카메라가 낸 화면 속 상자다.

    # 시뮬을 먼저 띄운다 (라벨 카메라·화물·동적 장애물 켬)
    ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false label_camera:=true cargo:=true dynamic_obstacles:=true
    ros2 run amr_detection collect_dataset --out /data --count 2500 --seed 1

로봇을 통로 곳곳으로 순간이동시키며 찍는다. 70%는 물체 하나(표지판·화물·작업자)를 1.5~8 m에서 바라보는
자리, 나머지는 빈 공간 아무 데나다. 결과: <out>/<시각>/{images,labels}/<split>/, data.yaml, frames.jsonl.
학습·검증은 장소(5 m 칸)로 나눈다. 같은 장소의 비슷한 사진이 양쪽에 들어가면 검증 성능이 부풀려진다.
"""

import argparse
import datetime
import json
import math
from pathlib import Path
import random
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt
import yaml

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from amr_gazebo.cargo import load_boxes

from amr_detection.dataset import CLASSES, split_of, yolo_lines
WORLD = 'warehouse'
# 지도 원점의 월드 좌표 (지도 v1, localization.md map_origin_in_world). 순간이동 자리 고르기에만 쓴다
MAP_ORIGIN_IN_WORLD = (-0.0571, 1.5167)
WORKERS = ['worker_1', 'worker_2']


class FreeSpace:
    """지도 v1에서 장애물로부터 clearance [m] 넘게 떨어진 자리 (월드 좌표로 조회·추출)."""

    def __init__(self, clearance):
        maps = Path(get_package_share_directory('amr_mapping')) / 'maps'
        meta = yaml.safe_load((maps / 'warehouse_v1.yaml').read_text())
        img = cv2.imread(str(maps / meta['image']), cv2.IMREAD_GRAYSCALE)
        self.res = meta['resolution']
        self.ox = meta['origin'][0] + MAP_ORIGIN_IN_WORLD[0]
        self.oy = meta['origin'][1] + MAP_ORIGIN_IN_WORLD[1]
        # 흰 칸만 빈 곳 (알 수 없음·점유 제외)
        self.ok = distance_transform_edt(img > 250) * self.res > clearance
        self.cells = np.argwhere(self.ok)

    def contains(self, x, y):
        col = int((x - self.ox) / self.res)
        row = self.ok.shape[0] - 1 - int((y - self.oy) / self.res)
        return 0 <= row < self.ok.shape[0] and 0 <= col < self.ok.shape[1] and bool(self.ok[row, col])

    def sample(self, rng):
        row, col = self.cells[rng.randrange(len(self.cells))]
        return (self.ox + (col + 0.5) * self.res,
                self.oy + (self.ok.shape[0] - row - 0.5) * self.res)


def static_targets():
    """표지판(월드 SDF)과 화물(cargo_boxes.yaml)의 (이름, x, y, 앞면 방향 또는 None)."""
    share = Path(get_package_share_directory('amr_gazebo'))
    targets = []
    for inc in ET.parse(share / 'worlds' / 'warehouse.sdf').getroot().iter('include'):
        if inc.find('uri').text.startswith('model://sign_'):
            x, y, _, _, _, yaw = (float(v) for v in inc.find('pose').text.split())
            targets.append((inc.find('name').text, x, y, yaw))
    for b in load_boxes(str(share / 'config' / 'cargo_boxes.yaml')):
        targets.append((b.id, b.x, b.y, None))
    return targets


class Collector(Node):
    def __init__(self, ns):
        super().__init__('yolo_collector', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ns = ns
        self.images, self.labels = {}, {}
        self.pose = None
        self.workers = {}
        self.create_subscription(Image, f'/{ns}/camera/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, f'/{ns}/camera/labels', self.on_labels, 10)
        self.create_subscription(Odometry, f'/{ns}/ground_truth/odom', self.on_pose, 10)
        for w in WORKERS:
            self.create_subscription(Odometry, f'/dynamic_obstacles/{w}/ground_truth/odom',
                                     lambda m, w=w: self.workers.__setitem__(w, m), 10)

    @staticmethod
    def _key(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_image(self, msg):
        self.images[self._key(msg.header.stamp)] = msg
        while len(self.images) > 60:
            self.images.pop(min(self.images))

    def on_labels(self, msg):
        self.labels[self._key(msg.header.stamp)] = msg
        while len(self.labels) > 20:
            self.labels.pop(min(self.labels))

    def on_pose(self, msg):
        self.pose = msg

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def teleport(self, x, y, yaw):
        req = (f'name: "{self.ns}", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.01}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
        r = subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/set_pose',
                            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '2000', '--req', req], capture_output=True, text=True)
        return 'true' in r.stdout

    def wait_frame(self, after_ns, timeout=5.0):
        """after_ns 뒤에 찍힌 라벨과 같은 시각(±5 ms)의 RGB 영상 한 쌍."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            for k in sorted(self.labels):
                if k <= after_ns:
                    continue
                near = [i for i in self.images if abs(i - k) <= 5_000_000]
                if near:
                    return self.images[min(near, key=lambda i: abs(i - k))], self.labels[k]
        return None


def boxes_of(dets: Detection2DArray):
    """라벨 카메라 메시지 → (Gazebo 라벨, 중심 x, 중심 y, 너비, 높이) [px] 목록."""
    return [(int(float(d.results[0].hypothesis.class_id)), d.bbox.center.position.x,
             d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y)
            for d in dets.detections if d.results]


def pick_pose(rng, free, targets, workers, target_ratio):
    if rng.random() < target_ratio:
        kind = rng.random()
        if kind < 0.25 and workers:                      # 작업자 (움직이므로 지금 위치)
            m = rng.choice(list(workers.values())).pose.pose.position
            tx, ty, face = m.x, m.y, None
        else:
            _, tx, ty, face = rng.choice(targets)
        for _ in range(50):
            d = rng.uniform(1.5, 8.0)
            a = face + rng.uniform(-1.2, 1.2) if face is not None else rng.uniform(-math.pi, math.pi)
            x, y = tx + d * math.cos(a), ty + d * math.sin(a)
            if free.contains(x, y):
                return x, y, math.atan2(ty - y, tx - x) + rng.uniform(-0.5, 0.5)
    x, y = free.sample(rng)
    return x, y, rng.uniform(-math.pi, math.pi)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--count', type=int, default=2500, help='저장할 사진 수')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--ns', default='amr_01')
    p.add_argument('--out', type=Path, required=True, help='결과 폴더 (그 아래 <시각>/에 저장)')
    # 작은 상자를 빼면 사진에는 물체가 있는데 라벨이 없어 "작은 건 무시"를 배운다. 4 px 미만만 뺀다
    # (10 px 기준이면 첫 20장에서 상자 31개 중 5개가 빠졌다)
    p.add_argument('--min-px', type=float, default=4.0, help='한 변이 이보다 작은 상자는 라벨에서 뺀다')
    p.add_argument('--empty-keep', type=float, default=0.1, help='물체 없는 사진을 남길 확률')
    p.add_argument('--target-ratio', type=float, default=0.7)
    p.add_argument('--val-ratio', type=float, default=0.15)
    p.add_argument('--clearance', type=float, default=0.6, help='순간이동 자리의 장애물 여유 [m]')
    p.add_argument('--settle', type=float, default=0.3, help='순간이동 뒤 기다리는 sim 시간 [s]')
    args = p.parse_args(argv)

    rng = random.Random(args.seed)
    free = FreeSpace(args.clearance)
    targets = static_targets()
    run = args.out / datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val', 'preview'):
        (run / sub).mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Collector(args.ns)
    saved = tries = 0
    counts = [0] * len(CLASSES)
    with open(run / 'frames.jsonl', 'w') as log:
        while saved < args.count:
            tries += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            x, y, yaw = pick_pose(rng, free, targets, node.workers, args.target_ratio)
            if not node.teleport(x, y, yaw):
                continue
            frame = node.wait_frame(node.now_ns() + int(args.settle * 1e9))
            if frame is None or node.pose is None:
                continue
            gp = node.pose.pose.pose.position
            if math.hypot(gp.x - x, gp.y - y) > 0.1:      # 순간이동 뒤 밀려남 (충돌)
                continue
            img, dets = frame
            lines, dropped = yolo_lines(boxes_of(dets), img.width, img.height, args.min_px)
            if not lines and rng.random() > args.empty_keep:
                continue
            split = split_of(x, y, args.val_ratio)
            name = f'{saved:05d}'
            bgr = cv2.cvtColor(np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3),
                               cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(run / 'images' / split / f'{name}.jpg'), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            (run / 'labels' / split / f'{name}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
            if saved < 30:                                 # 눈으로 확인할 미리보기 (상자 그림)
                for ln in lines:
                    c, cx, cy, w, h = (float(v) for v in ln.split())
                    p1 = (int((cx - w / 2) * img.width), int((cy - h / 2) * img.height))
                    p2 = (int((cx + w / 2) * img.width), int((cy + h / 2) * img.height))
                    cv2.rectangle(bgr, p1, p2, [(0, 0, 255), (0, 200, 255), (255, 80, 0)][int(c)], 2)
                    cv2.putText(bgr, CLASSES[int(c)], (p1[0], max(p1[1] - 4, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imwrite(str(run / 'preview' / f'{name}.jpg'), bgr)
            for ln in lines:
                counts[int(ln.split()[0])] += 1
            log.write(json.dumps({'name': name, 'split': split, 'x': round(x, 3), 'y': round(y, 3),
                                  'yaw': round(yaw, 4), 'stamp_ns': node._key(img.header.stamp),
                                  'boxes': len(lines), 'dropped_small': dropped}) + '\n')
            saved += 1
            if saved % 100 == 0:
                print(f'{saved}/{args.count} (시도 {tries}) 상자 수 {dict(zip(CLASSES, counts))}', flush=True)

    (run / 'data.yaml').write_text(yaml.safe_dump({
        'path': str(run), 'train': 'images/train', 'val': 'images/val',
        'names': dict(enumerate(CLASSES))}, allow_unicode=True))
    (run / 'meta.json').write_text(json.dumps({
        'args': {k: str(v) for k, v in vars(args).items()}, 'saved': saved, 'tries': tries,
        'boxes': dict(zip(CLASSES, counts))}, ensure_ascii=False, indent=2))
    print(f'저장: {run} 사진 {saved}장, 상자 {dict(zip(CLASSES, counts))}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
