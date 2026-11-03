#!/usr/bin/env python3
"""재생 벤치: 녹화된 스캔·TF로 추적기를 ROS·Gazebo 없이 다시 돌리고 평가한다 (추적기 수정 비교용).

    uv run --no-project --with rosbags --with numpy --with pyyaml python -I experiments/tracking_kpi/replay_bench.py \\
        <실행 폴더…> --out <폴더> [--set 이름=값 …] [--check-node]

- 실행 폴더: fetch_runs.py --with-rosbag으로 받은 실행 하나(`…_00`). rosbag/와 meta.json이 있어야 한다
- 추적 노드(amr_tracking tracking_node)가 스캔마다 하는 일을 코어 함수로 그대로 한다: 스캔 시각 TF(odom ← LiDAR,
  녹화 /tf 보간) → 군집화(detect_obstacles) → odom으로 옮김 → 빈 공간 증거(FreeSpaceGrid) → 추적(MultiTargetTracker)
- 설정은 저장소의 amr_tracking/config/tracking.yaml을 읽고, --set으로 바꾼다 (예: --set accel_noise_std=2.0)
- 평가는 tracking_eval 노드와 같다: 트랙을 로봇 GT와 odom TF로 world로 옮겨 평가 코어(TrackingEvaluator)에 넣는다.
  평가 시간 범위는 실행 폴더에 GCP 평가 결과(tracking/frames.csv)가 있으면 그 범위를 그대로 쓴다(GCP 평가 노드는 녹화보다
  먼저 떠서 녹화 첫 프레임과 시작이 다르다). 없으면 첫 벤치 프레임부터 실행의 duration_s(0이면 끝까지)
- 출력은 GCP 실행 폴더와 같은 모양(<out>/<장면-라벨>/<묶음>/<실행>/{meta.json, tracking/…})이라
  tools/harness/summarize.py, make_tables.py, analyze.py를 그대로 쓴다. meta.json 라벨에는 +bench_<이름>이 붙고
  git_sha는 벤치 시점 코드다(bench_meta)
- --check-node (설정을 바꾸지 않았을 때만 뜻이 있다):
  - 실행 폴더에 GCP 평가 결과(tracking/summary.json·frames.csv)가 있으면 추적기가 정하는 값(위치·속도 RMSE와 짝 수,
    예측·정지 가정 오차는 짝마다, 동적 판정 지연·접근 판정)이 같은지 본다. 다르면 다른 값의 이름을 보이고 실패 코드로
    끝난다 (구간 시작에 따라 달라지는 값 GT_START_KEYS와 창 끝 뒤를 목표로 하는 예측 짝 PRED_COLS는 뺀다)
  - 벤치 트랙이 녹화 속 노드 출력과 같은 프레임 수를 참고로 보인다. 하네스는 추적 노드를 녹화보다 약 2 s 먼저
    띄워, 녹화 전부터 있던 정지 물체 트랙 때문에 몇 프레임은 다를 수 있다(장애물은 그 뒤에 띄운다)
- 폐루프는 못 본다: 로봇이 트랙에 반응하는 실험(I17)은 시뮬레이션으로 잰다
- 노드에서만 하는 일은 벤치에 없다(메시지로 낼 때의 반경 규칙, 카메라 연관, 지도 대조, TTC). 추적기 수정은
  코어(amr_perception_core)에 넣어야 벤치로 비교된다. tracking.yaml에 벤치가 모르는 값이 생기면 경고한다
- GCP 평가와 비교: 추적기가 정하는 값(위치·속도 RMSE와 짝 수, 짝마다의 예측 오차, 처음 본 뒤 판정 지연·판정 거리, D1)은
  같다. 예측 RMSE·짝 수는 창 끝 뒤를 목표로 하는 짝만큼 다를 수 있다(PRED_COLS). 구간 시작과 거기서 잰 값
  (판정 지연 from_start, 시작 거리)은 녹화에 새로 뜬 장애물의 GT가 늦게부터 들어 있어 GCP와 다를 수 있다. 프레임당 값
  (가짜 동적·트랙 수)은 하네스가 추적 노드를 녹화보다 먼저 띄워 녹화 전 프레임만큼 분모가 달라 조금 다르다.
  수정 전후는 벤치끼리 비교한다 (같은 녹화라 이 차이가 똑같다)
"""
import argparse
import bisect
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, get_typestore, Stores

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/perception/amr_perception_core'))
from amr_perception_core.clustering import ClusteringConfig, detect_obstacles, transform_points_2d  # noqa: E402
from amr_perception_core.free_space import FreeSpaceGrid  # noqa: E402
from amr_perception_core.kalman import KalmanConfig  # noqa: E402
from amr_perception_core.tracker import MultiTargetTracker, TrackerConfig  # noqa: E402
from amr_perception_core.tracking_eval import (  # noqa: E402
    EvalConfig, TimeSeries, TrackingEvaluator, TrackObs, world_from_odom)

TRACKING_YAML = REPO / 'src/perception/amr_tracking/config/tracking.yaml'
MSG_DIR = REPO / 'src/interfaces/amr_msgs/msg'
NS = 'amr_01'
ODOM, BASE = f'{NS}/odom', f'{NS}/base_footprint'
ROBOT_GT_MAX_GAP = 0.05   # tracking_eval_node와 같다
# 평가 창 끝 뒤 GT를 이만큼만 쓴다. GCP 평가 노드는 기록 시간이 끝나면 바로 저장해 그 뒤 GT가 없다
# (안 자르면 창 끝 1 s의 +1 s 예측 짝이 GCP보다 많아진다)
GT_AFTER_WINDOW_S = 0.1
# 구간 시작(장애물이 움직이며 범위에 들어온 시각)에 따라 달라지는 값. 녹화에는 새로 뜬 장애물의 GT가 GCP 평가 노드보다
# 늦게부터 들어 있어(본 측정 C10 0.8 s, 개발용 C 0.15 s. 새 토픽을 찾는 시간으로 보임) 구간 시작이 GCP보다 늦을 수 있다.
# 그래서 --check-node 대조에서 뺀다
# (by_speed는 구간 평균 속력으로 칸을 나눈다). GT만으로 정해져 같은 녹화끼리는 똑같으므로 수정 전후 비교에는 영향이 없다
GT_START_KEYS = ('t_start', 'start_dist', 'latency_from_start', 'from_start', 'by_speed')
# 예측·정지 가정 오차 열과 앞을 보는 시간 [s]. 목표 시각(t + 앞)이 평가 창 끝보다 뒤인 짝은 GCP 평가 노드가 멈추기 전에
# 받은 GT에 달려 있어(실행마다 몇십 ms 다르다. 벤치는 GT_AFTER_WINDOW_S로 어림) --check-node에서 한쪽에만 있어도 된다
# (개발용 E 실행 00: 창 끝 98.7 s, GCP에만 목표 98.8 s 짝 2개)
PRED_COLS = {'pred_err_1s': 1.0, 'hold_err_1s': 1.0, 'pred_err_2s': 2.0, 'hold_err_2s': 2.0}
# 벤치가 쓰는 tracking.yaml 이름 (노드의 설정 읽기와 같아야 한다. --check-node로 확인)
KALMAN_KEYS = ('accel_noise_std', 'measurement_noise_std', 'initial_velocity_std')
TRACKER_KEYS = ('gate_mahalanobis_sq', 'confirm_hits', 'max_coast_time', 'dynamic_speed_threshold',
                'dynamic_confirm_frames', 'dynamic_evidence_frames', 'radius_smoothing')
CLUSTER_KEYS = ('breakpoint_lambda_deg', 'range_noise_std', 'max_gap_beams', 'min_points', 'max_obstacle_extent',
                'leg_max_extent', 'leg_merge_distance')
FREE_KEYS = {'free_space_resolution': 'resolution', 'free_space_size': 'size', 'free_space_max_range': 'max_range',
             'free_space_endpoint_margin': 'endpoint_margin', 'free_space_memory': 'free_memory',
             'free_space_hit_quiet': 'hit_quiet', 'free_space_hit_clearance': 'hit_clearance'}
OTHER_KEYS = ('use_free_space', 'free_evidence_min_points', 'obstacle_radius')
# 노드에서만 쓰는 값 (좌표계·TTC·발행·지도 대조·카메라 연관). 지도 대조가 켜져 있으면 스캔 점이 달라져 벤치가 멈춘다.
# 카메라 연관은 기본으로 켜져 있지만 카메라를 끈 녹화에는 검출이 없어 영향이 없다 (검출이 녹화돼 있으면 경고)
NODE_ONLY_KEYS = ('measurement_source', 'fixed_frame', 'tf_timeout', 'base_frame', 'robot_length', 'robot_width',
                  'ttc_horizon', 'odom_timeout', 'publish_markers', 'stats_period', 'use_static_map', 'map_topic',
                  'map_frame', 'map_inflation', 'map_occupied_threshold', 'map_align_max_shift', 'map_align_step',
                  'use_camera_association', 'detections_topic', 'assoc_camera_recent_s', 'assoc_evidence_tau',
                  'assoc_gate_mahalanobis_sq', 'assoc_max_distance', 'assoc_min_confidence', 'assoc_min_evidence',
                  'assoc_release_evidence', 'min_radius_classes', 'min_radius_values')


def load_params(overrides):
    params = yaml.safe_load(TRACKING_YAML.read_text())['/**/tracking']['ros__parameters']
    known = set(KALMAN_KEYS + TRACKER_KEYS + CLUSTER_KEYS + OTHER_KEYS) | set(FREE_KEYS)
    for item in overrides:
        key, _, value = item.partition('=')
        if key not in known:
            raise SystemExit(f'--set {key}: 벤치가 쓰지 않는 이름이다. 쓰는 이름: {sorted(known)}')
        params[key] = yaml.safe_load(value)
    unknown = sorted(set(params) - known - set(NODE_ONLY_KEYS))
    if unknown:
        print(f'경고: 벤치가 모르는 tracking.yaml 값 {unknown} - 노드에 새 설정을 더했으면 벤치에도 더한다', file=sys.stderr)
    if params.get('use_static_map'):
        raise SystemExit('use_static_map이 켜져 있다. 벤치는 지도 대조(R2)를 하지 않는다')
    return params


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    # 추적 노드와 같은 식 (rclpy Time.nanoseconds * 1e-9). sec + nanosec * 1e-9와는 끝자리가 달라
    # 못 본 시간 2.0 s 같은 경계 판정이 노드와 엇갈렸다 (개발용 D 실행 01·02, B 실행 01)
    return (header.stamp.sec * 1_000_000_000 + header.stamp.nanosec) * 1e-9


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class Recording:
    """녹화 하나: 스캔, odom ← 차체 TF, 고정 TF, 로봇·장애물 GT, 노드 출력."""

    def __init__(self, bag: Path):
        store = get_typestore(Stores.ROS2_HUMBLE)
        types = {}
        for name in ('TrackedObstacle', 'TrackedObstacleArray'):
            types.update(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(), f'amr_msgs/msg/{name}'))
        store.register(types)
        self.scans, self.odom, self.static, self.robot, self.obstacles, self.node = [], [], {}, [], {}, []
        with AnyReader([bag], default_typestore=store) as reader:
            self.has_detections = any(c.topic == f'/{NS}/perception/objects' for c in reader.connections)
            for conn, _, raw in reader.messages():
                topic = conn.topic
                if topic not in (f'/{NS}/scan', '/tf', '/tf_static', f'/{NS}/ground_truth/odom',
                                 f'/{NS}/perception/obstacles') and not topic.startswith('/dynamic_obstacles/'):
                    continue
                m = reader.deserialize(raw, conn.msgtype)
                if topic == f'/{NS}/scan':
                    self.scans.append((stamp_of(m.header), m.header.frame_id, m.angle_min, m.angle_increment,
                                       np.asarray(m.ranges)))
                elif topic in ('/tf', '/tf_static'):
                    for tf in m.transforms:
                        tr = tf.transform
                        pose = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                        if topic == '/tf_static':
                            self.static[tf.child_frame_id] = (tf.header.frame_id, pose)
                        elif tf.header.frame_id == ODOM and tf.child_frame_id == BASE:
                            self.odom.append((stamp_of(tf.header), *pose))
                elif topic == f'/{NS}/ground_truth/odom':
                    p, v = m.pose.pose.position, m.twist.twist.linear
                    self.robot.append((stamp_of(m.header), (p.x, p.y, yaw_of(m.pose.pose.orientation), v.x, v.y)))
                elif topic == f'/{NS}/perception/obstacles':
                    self.node.append((stamp_of(m.header), m.obstacles))
                elif topic.endswith('/ground_truth/odom'):
                    gid = topic.split('/')[2]
                    a = yaw_of(m.pose.pose.orientation)
                    bx, by = m.twist.twist.linear.x, m.twist.twist.linear.y   # 몸체 기준 → world
                    self.obstacles.setdefault(gid, []).append(
                        (stamp_of(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
                         math.cos(a) * bx - math.sin(a) * by, math.sin(a) * bx + math.cos(a) * by))
        self.odom.sort()
        self.odom_t = [o[0] for o in self.odom]

    def odom_base(self, t):
        """odom ← 차체 (tf2처럼 앞뒤 샘플 선형 보간). 범위 밖이면 None (노드는 TF 없음으로 그 스캔을 건너뛴다)."""
        i = bisect.bisect_left(self.odom_t, t)
        if i < len(self.odom_t) and abs(self.odom_t[i] - t) < 1e-9:
            return self.odom[i][1:]
        if i == 0 or i >= len(self.odom_t):
            return None
        t0, x0, y0, a0 = self.odom[i - 1]
        t1, x1, y1, a1 = self.odom[i]
        w = (t - t0) / (t1 - t0)
        return (x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * ((a1 - a0 + math.pi) % (2 * math.pi) - math.pi))

    def base_to(self, frame):
        """차체 → frame (고정 TF 사슬)."""
        chain = []
        while frame != BASE:
            parent, pose = self.static[frame]
            chain.append(pose)
            frame = parent
        out = (0.0, 0.0, 0.0)
        for pose in reversed(chain):
            out = compose(out, pose)
        return out


def replay(rec: Recording, p: dict):
    """추적 노드 on_scan과 같은 순서로 돌려 [(시각, [TrackObs…])]를 돌려준다 (odom 기준).

    추적기는 Track을 그 자리에서 고쳐 쓰므로, 프레임마다 그 순간 값을 TrackObs로 떠 둔다 (노드의 메시지와 같다).
    """
    tracker = MultiTargetTracker(TrackerConfig(
        kalman=KalmanConfig(**{k: p[k] for k in KALMAN_KEYS}), **{k: p[k] for k in TRACKER_KEYS}))
    clustering = ClusteringConfig(**{k: p[k] for k in CLUSTER_KEYS})
    free = FreeSpaceGrid(**{v: p[k] for k, v in FREE_KEYS.items()}) if p['use_free_space'] else None
    if not rec.scans:
        raise SystemExit('녹화에 스캔(/amr_01/scan)이 없다')
    lidar = rec.base_to(rec.scans[0][1])
    frames = []
    for t, _, a0, da, ranges in rec.scans:
        ob = rec.odom_base(t)
        if ob is None:
            continue
        sx, sy, syaw = compose(ob, lidar)
        clusters = detect_obstacles(ranges, a0, da, clustering)
        points = transform_points_2d(np.array([c.centroid for c in clusters]).reshape(-1, 2), syaw, (sx, sy))
        evidence = None
        if free is not None:
            evidence = [int(free.evidence(transform_points_2d(c.points, syaw, (sx, sy)), t).sum())
                        >= p['free_evidence_min_points'] for c in clusters]
            angles = syaw + a0 + da * np.arange(len(ranges))
            free.integrate((sx, sy), np.c_[np.cos(angles), np.sin(angles)], np.asarray(ranges, dtype=float), t)
        tracks = tracker.step(t, points, [c.radius for c in clusters], evidence)
        frames.append((t, [track_obs(tr, p['obstacle_radius']) for tr in tracks]))
    return frames


def track_obs(track, default_radius):
    kf = track.kf
    return TrackObs(int(track.track_id), *map(float, kf.position), *map(float, kf.velocity), bool(track.is_dynamic),
                    np.asarray(kf.position_covariance), np.asarray(kf.velocity_covariance),
                    float(track.radius if track.radius is not None else default_radius))


class _Evaluator(TrackingEvaluator):
    """'기록 시작 전부터'(censored) 구간을 첫 평가 프레임 대신 평가 시작(벤치 추적기가 돌고 평가 범위 안인 첫 시각)
    기준으로 가르는 평가 코어.

    첫 스캔 때 로봇 GT가 아직 없으면(첫 샘플이 조금 늦다) 그 프레임은 평가에서 빠지지만 추적기는 이미 돌았다.
    그 사이 나타난 장애물은 추적기가 처음부터 본 구간인데 첫 평가 프레임 기준이면 censored가 되어 D1이 빈다
    (개발용 C 실행 02: 첫 스캔 3.6 s, 로봇 GT 3.62 s, 작업자 3.7 s). 평가 시작 전부터 진행 중이던 구간은
    그대로 censored라, 녹화보다 먼저 움직이던 장애물이 벤치에서 중간부터 잡혀 판정 지연이 짧게 섞이지 않는다.
    코어의 _episodes(history, first_t)를 덮어쓴다. 코어에서 이름·인자가 바뀌면 여기도 고친다
    (안 고치면 예전 기준으로 돌아가 개발용 C 실행 02의 --check-node가 '다름'으로 잡는다).
    """

    def __init__(self, config, gids, eval_start):
        super().__init__(config, gids)
        self.eval_start = eval_start

    def _episodes(self, history, first_t):
        if self.eval_start is not None and first_t is not None:
            first_t = min(first_t, self.eval_start)
        return super()._episodes(history, first_t)


def evaluate(rec: Recording, frames, window):
    """tracking_eval 노드와 같은 평가. window = (시작, 끝) 시각, 끝이 None이면 녹화 끝까지."""
    ev = _Evaluator(EvalConfig(), sorted(rec.obstacles), max(window[0], frames[0][0]) if frames else None)
    gt_end = math.inf if window[1] is None else window[1] + GT_AFTER_WINDOW_S
    for gid, samples in rec.obstacles.items():
        for s in samples:
            if s[0] <= gt_end:
                ev.add_gt(gid, *s)
    robot = TimeSeries(angle_columns=(2,))
    for t, sample in rec.robot:
        robot.add(t, sample)
    counts = {'msgs': len(frames), 'frames': 0, 'no_robot_gt': 0, 'tf_fail': 0, 'frame_world': 0}
    for t, tracks in frames:
        if t < window[0] - 1e-6 or (window[1] is not None and t > window[1] + 1e-6):
            continue
        rb, ob = robot.at(t, ROBOT_GT_MAX_GAP), rec.odom_base(t)
        if rb is None:
            counts['no_robot_gt'] += 1
            continue
        world_odom = world_from_odom(tuple(rb[:3]), ob)
        c, s = math.cos(rb[2]), math.sin(rb[2])
        ev.add_frame(t, [tr.transformed(world_odom) for tr in tracks], robot_xy=(rb[0], rb[1]), robot_yaw=rb[2],
                     robot_velocity=(c * rb[3] - s * rb[4], s * rb[3] + c * rb[4]))
        counts['frames'] += 1
    rows, episodes, summary = ev.evaluate()
    summary['input'] = dict(counts, obstacle_ids=sorted(rec.obstacles),
                            robot_gt_samples=len(rec.robot),
                            obstacle_gt_samples={g: len(v) for g, v in rec.obstacles.items()})
    return rows, episodes, summary


def check_node(rec: Recording, frames, warmup_s=5.0):
    """벤치 트랙 ↔ 녹화 속 노드 출력. 녹화 시작 warmup_s 뒤부터 같은 시각끼리.

    트랙 번호는 비교하지 않는다: 노드는 녹화 전부터 돌아 정지 물체 트랙을 먼저 만들어 번호가 앞서 있다.
    같은 프레임 = 트랙 수가 같고, 노드 트랙마다 1 mm 안에 동적 판정이 같은 벤치 트랙이 있다.
    """
    mine = {round(t, 6): tracks for t, tracks in frames}
    t0 = rec.node[0][0] if rec.node else 0.0
    same = total = 0
    worst = 0.0
    for t, obstacles in rec.node:
        if t - t0 < warmup_s or round(t, 6) not in mine:
            continue
        total += 1
        bench = mine[round(t, 6)]
        if len(bench) != len(obstacles):
            continue
        d = 0.0
        for o in obstacles:
            px, py = o.pose.pose.position.x, o.pose.pose.position.y
            near = [math.hypot(b.x - px, b.y - py) for b in bench if b.is_dynamic == o.is_dynamic]
            d = max(d, min(near, default=math.inf))
        worst = max(worst, d) if math.isfinite(d) else worst
        same += d < 1e-3
    return same, total, worst


def eval_window(run: Path, frames, meta):
    """GCP 평가 결과가 있으면 그 시간 범위, 없으면 첫 벤치 프레임 + duration_s."""
    gcp = run / 'tracking/frames.csv'
    if gcp.is_file():
        with open(gcp) as f:
            ts = [float(r['t']) for r in csv.DictReader(f)]
        if ts:
            return min(ts), max(ts)
    start = frames[0][0] if frames else 0.0
    duration = float(meta.get('params', {}).get('duration_s', 0.0))
    return start, (start + duration if duration > 0 else None)


def _same(a, b) -> bool:
    """summary.json 값 비교 (실수는 1e-9 안, NaN끼리 같다)."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, float) and isinstance(b, float):
        return abs(a - b) <= 1e-9 or (math.isnan(a) and math.isnan(b))
    return a == b


def _match_rows(path: Path):
    """frames.csv의 짝 행 {(시각, GT id): 행} (값은 write_csv가 쓴 문자열 그대로)."""
    with open(path, newline='', encoding='utf-8') as f:
        return {(r['t'], r['gt_id']): r for r in csv.DictReader(f) if r['kind'] == 'match'}


def check_gcp(run: Path, summary, bench_frames: Path, window_end: float):
    """GCP 평가와 다른 값의 이름 목록 (같으면 빈 목록). GCP 결과가 없으면 None.

    위치·속도 오차(overall), 동적 판정 지연(latency), 접근 판정(approach)은 요약끼리 본다. 구간 시작에 따라
    달라지는 값(GT_START_KEYS)만 뺀다. 예측·정지 가정 오차는 짝마다(frames.csv의 시각·GT) 본다: 둘 다 있으면
    같아야 하고, 한쪽에만 있어도 되는 것은 목표 시각이 평가 창 끝보다 뒤인 짝뿐이다(PRED_COLS).
    """
    path, frames = run / 'tracking/summary.json', run / 'tracking/frames.csv'
    if not (path.is_file() and frames.is_file()):
        return None
    gcp = json.loads(path.read_text())
    diffs = [f'overall.{k}' for k in ('pos', 'vel') if not _same(summary['overall'][k], gcp['overall'][k])]
    for block in ('latency', 'approach'):
        a, b = summary.get(block) or {}, gcp.get(block) or {}
        diffs += [f'{block}.{k}' for k in sorted(set(a) | set(b))
                  if k not in GT_START_KEYS and not _same(a.get(k), b.get(k))]
    g_rows, b_rows = _match_rows(frames), _match_rows(bench_frames)
    if g_rows.keys() != b_rows.keys():
        return diffs + ['frames.match']
    for col, ahead in PRED_COLS.items():
        if any(g.get(col, '') != b_rows[key].get(col, '') and float(key[0]) + ahead <= window_end + 1e-6
               for key, g in g_rows.items()):
            diffs.append(f'frames.{col}')
    return diffs


def write_csv(path: Path, rows):
    """tracking_eval 노드의 write_csv와 같은 형식."""
    keys = []
    for row in rows:
        keys += [k for k in row if k not in keys]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=keys or ['empty'])
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ('' if row.get(k) is None else
                                 (f'{row[k]:.4f}' if isinstance(row[k], float) else row[k])) for k in keys})


def git_state():
    """벤치를 돌린 코드: (짧은 커밋, 커밋 안 한 변경이 있는지). 하네스 runner.git_sha와 같은 식."""
    def git(*args):
        return subprocess.run(['git', '-C', str(REPO), *args], capture_output=True, text=True,
                              timeout=5).stdout.strip()
    try:
        sha = git('rev-parse', '--short', 'HEAD') or 'unknown'
        return sha, bool(git('status', '--porcelain', '--untracked-files=no'))
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown', None


def bench_meta(meta, name, sets, sha, dirty):
    """벤치 결과의 meta.json. 녹화 meta를 바탕으로 두 가지를 바꾼다.

    - 라벨에 +bench_<이름>: 요약은 시나리오 + 라벨로 묶어서(summary.group_key) 그대로 두면 다른 벤치끼리, 녹화와도 섞인다
    - git_sha는 트랙을 만든 벤치 시점 코드 (커밋 안 한 변경이 있으면 -dirty). 녹화 때 값은 bench.recording에 남긴다
    """
    out = dict(meta)
    out['label'] = f"{meta.get('label') or ''}+bench_{name}"
    out['git_sha'] = sha + ('-dirty' if dirty else '')
    out['bench'] = {'name': name, 'set': sets, 'tracking_yaml': str(TRACKING_YAML.relative_to(REPO)),
                    'git_sha': sha, 'dirty': dirty,
                    'recording': {k: meta.get(k) for k in ('run_id', 'label', 'git_sha')}}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='재생 벤치 (녹화 → 추적기 → 평가)')
    ap.add_argument('runs', nargs='+', type=Path, help='실행 폴더 (rosbag/, meta.json)')
    ap.add_argument('--out', type=Path, required=True, help='결과 폴더 (GCP 실행 폴더와 같은 모양)')
    ap.add_argument('--name', help='벤치 이름. 결과 라벨에 +bench_<이름>으로 붙는다 (기본: --out 폴더 이름)')
    ap.add_argument('--set', action='append', default=[], help='tracking.yaml 값 바꾸기 (이름=값)')
    ap.add_argument('--check-node', action='store_true', help='벤치 트랙이 녹화 속 노드 출력과 같은지')
    a = ap.parse_args(argv)
    p = load_params(a.set)
    name = a.name or a.out.resolve().name
    sha, dirty = git_state()
    bad = 0
    for run in a.runs:
        meta = json.loads((run / 'meta.json').read_text())
        rec = Recording(run / 'rosbag')
        if rec.has_detections and p.get('use_camera_association'):
            print(f'경고: {run.name}에 카메라 검출이 녹화돼 있다. 벤치는 카메라 연관(클래스·최소 반경)을 하지 않는다',
                  file=sys.stderr)
        frames = replay(rec, p)
        window = eval_window(run, frames, meta)
        rows, episodes, summary = evaluate(rec, frames, window)
        dest = a.out / Path(*run.resolve().parts[-3:])
        (dest / 'tracking').mkdir(parents=True, exist_ok=True)
        out_meta = bench_meta(meta, name, a.set, sha, dirty)
        (dest / 'meta.json').write_text(json.dumps(out_meta, ensure_ascii=False, indent=2))
        write_csv(dest / 'tracking/frames.csv', rows)
        write_csv(dest / 'tracking/episodes.csv', episodes)
        (dest / 'tracking/summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        line = f"{run.name}: 스캔 {len(rec.scans)}, 벤치 프레임 {len(frames)}, 평가 프레임 {summary['input']['frames']}"
        if a.check_node:
            same, total, worst = check_node(rec, frames)
            line += f' | 노드와 같은 프레임 {same}/{total}'
            diffs = check_gcp(run, summary, dest / 'tracking/frames.csv', window[1])
            line += (' | GCP 평가 없음' if diffs is None else ' | GCP 평가와 같음' if not diffs
                     else f" | GCP 평가와 다름: {', '.join(diffs)}")
            bad += bool(diffs)
        print(line)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
