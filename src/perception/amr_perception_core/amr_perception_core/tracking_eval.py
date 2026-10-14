"""추적 결과를 시뮬레이터 정답(GT)과 비교해 정확도를 잰다 (ROS 비의존).

원 명세 점검 항목 "칼만 필터 기반 객체 추적이 구현되고, 예측 정확도가 적절한가?"와
perception.md 1-5 "동적 판정 지연·정확도 (0.3~1.5 m/s)"를 숫자로 답하기 위한 계산이다.
노드(amr_tracking tracking_eval_node)가 트랙·GT를 모두 같은 좌표계(world)로 옮겨 넣고,
기록이 끝나면 evaluate()로 한 번에 계산한다 (+1 s, +2 s 예측 오차는 미래 GT가 필요해서).

프레임마다: 트랙(확정 트랙 전부)과 GT 물체를 가장 가까운 쌍부터 1:1로 짝짓는다 (게이트 gate [m]).
짝지어진 쌍에서 위치·속도·방향 오차, 등속 예측 오차, NEES를 계산하고, GT 물체마다 짝지어진 트랙 ID가
바뀐 횟수(ID 전환)와 동적 판정 지연을 센다. 짝이 없는 동적 트랙은 가짜 동적 트랙이다.

좌표 변환 도우미(se2_*)는 world → odom 변환을 로봇 GT 자세와 odom TF로 구하는 데 쓴다:
T_world_odom = T_world_base · T_odom_base⁻¹.
"""

from bisect import bisect_left, insort
from dataclasses import asdict, dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

Pose2 = Tuple[float, float, float]   # (x, y, yaw)


# ---------------------------------------------------------------------------- 좌표 변환
def wrap_angle(a: float) -> float:
    """[-π, π)로 감싼다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def se2_compose(a: Pose2, b: Pose2) -> Pose2:
    """a · b (b를 a 좌표계에서 본 자세 → a의 부모 좌표계)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap_angle(ath + bth))


def se2_inverse(a: Pose2) -> Pose2:
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (-(c * x + s * y), s * x - c * y, wrap_angle(-th))


def world_from_odom(world_base: Pose2, odom_base: Pose2) -> Pose2:
    """같은 시각의 로봇 GT 자세(world → base)와 odom TF(odom → base)로 world → odom을 구한다."""
    return se2_compose(world_base, se2_inverse(odom_base))


# ---------------------------------------------------------------------------- 입력 자료
@dataclass
class TrackObs:
    """한 프레임의 트랙 1개. 위치·속도는 평가 좌표계(world) 기준."""
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    is_dynamic: bool
    # 위치·속도 공분산 2x2 (KF P). 없으면 NEES를 계산하지 않는다
    pos_cov: Optional[np.ndarray] = None
    vel_cov: Optional[np.ndarray] = None

    def transformed(self, pose: Pose2) -> 'TrackObs':
        """pose(부모 ← 트랙 좌표계)로 옮긴다. 위치는 회전 + 평행이동, 속도는 회전, 공분산은 R C Rᵀ."""
        tx, ty, th = pose
        c, s = math.cos(th), math.sin(th)
        r = np.array([[c, -s], [s, c]])
        x, y = r @ np.array([self.x, self.y]) + np.array([tx, ty])
        vx, vy = r @ np.array([self.vx, self.vy])
        pc = None if self.pos_cov is None else r @ np.asarray(self.pos_cov, float) @ r.T
        vc = None if self.vel_cov is None else r @ np.asarray(self.vel_cov, float) @ r.T
        return TrackObs(self.track_id, float(x), float(y), float(vx), float(vy),
                        self.is_dynamic, pc, vc)


class TimeSeries:
    """시각별 값 벡터. at(t)는 앞뒤 두 샘플을 선형 보간한다 (angle_columns는 감싼 각도로 보간).

    두 샘플 간격이 max_gap보다 크거나 범위 밖이면 None (외삽하지 않는다).
    """

    def __init__(self, angle_columns: Sequence[int] = ()):
        self._t: List[float] = []
        self._v: Dict[float, np.ndarray] = {}
        self._angles = tuple(angle_columns)

    def __len__(self) -> int:
        return len(self._t)

    @property
    def last_time(self) -> Optional[float]:
        return self._t[-1] if self._t else None

    def add(self, t: float, values: Sequence[float]) -> None:
        t = float(t)
        v = np.asarray(values, dtype=float)
        if not (math.isfinite(t) and np.all(np.isfinite(v))):
            return
        if t not in self._v:
            if self._t and t < self._t[-1]:
                insort(self._t, t)
            else:
                self._t.append(t)
        self._v[t] = v

    def at(self, t: float, max_gap: float) -> Optional[np.ndarray]:
        if not self._t:
            return None
        i = bisect_left(self._t, t)
        if i < len(self._t) and self._t[i] == t:
            return self._v[t].copy()
        if i == 0 or i == len(self._t):
            return None
        t0, t1 = self._t[i - 1], self._t[i]
        if t1 - t0 > max_gap:
            return None
        a, b = self._v[t0], self._v[t1]
        w = (t - t0) / (t1 - t0)
        d = b - a
        for k in self._angles:
            d[k] = wrap_angle(d[k])
        out = a + w * d
        for k in self._angles:
            out[k] = wrap_angle(out[k])
        return out


# ---------------------------------------------------------------------------- 짝짓기
def associate_nearest(tracks_xy: np.ndarray, gt_xy: np.ndarray,
                      gate: float) -> List[Tuple[int, int, float]]:
    """가장 가까운 (트랙, GT) 쌍부터 1:1로 짝짓는다. 거리 > gate인 쌍은 짝짓지 않는다.

    돌려주는 값: (트랙 인덱스, GT 인덱스, 거리) 목록.
    """
    tracks_xy = np.asarray(tracks_xy, dtype=float).reshape(-1, 2)
    gt_xy = np.asarray(gt_xy, dtype=float).reshape(-1, 2)
    if len(tracks_xy) == 0 or len(gt_xy) == 0:
        return []
    dist = np.linalg.norm(tracks_xy[:, None, :] - gt_xy[None, :, :], axis=2)
    pairs = sorted((float(dist[i, j]), i, j) for i in range(dist.shape[0])
                   for j in range(dist.shape[1]) if dist[i, j] <= gate)
    used_t, used_g, out = set(), set(), []
    for d, i, j in pairs:
        if i in used_t or j in used_g:
            continue
        used_t.add(i)
        used_g.add(j)
        out.append((i, j, d))
    return out


# ---------------------------------------------------------------------------- 평가
@dataclass(frozen=True)
class EvalConfig:
    # 트랙 ↔ GT 짝짓기 거리 상한 [m]. LiDAR 군집 중심은 센서 쪽 면으로 치우친다 (지게차 0.45 m 이상)
    gate: float = 1.0
    # GT 속력이 이 값 [m/s] 이상이면 "움직이는 중" (방향 오차·예측 오차·지연 구간)
    moving_speed: float = 0.1
    # 로봇에서 이 거리 [m] 안의 GT만 "센서 범위 안"으로 본다 (LiDAR 12 m, R4 free_space_max_range)
    sensor_range: float = 12.0
    # 등속 예측 오차를 잴 미래 시각 [s]
    horizons: Tuple[float, ...] = (1.0, 2.0)
    # GT 보간 시 두 샘플 사이 최대 간격 [s]
    max_gt_gap: float = 0.2
    # 방향 오차는 트랙 속력도 이 값 [m/s] 이상일 때만 (속력이 0에 가까우면 방향이 무의미)
    heading_min_speed: float = 0.2
    # 지연 구간(움직이며 범위 안)이 이 시간 [s] 이하로 끊기면 한 구간으로 잇는다 (왕복 끝 감속 등)
    episode_merge_gap: float = 1.0
    # 2자유도 χ² 95% (NEES 일관성 비율)
    nees_chi2_95: float = 5.991

    def __post_init__(self):
        for name in ('gate', 'sensor_range', 'max_gt_gap', 'heading_min_speed', 'nees_chi2_95'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')
        if not (math.isfinite(self.moving_speed) and self.moving_speed >= 0.0):
            raise ValueError(f'moving_speed는 0 이상이어야 한다: {self.moving_speed}')
        if not (math.isfinite(self.episode_merge_gap) and self.episode_merge_gap >= 0.0):
            raise ValueError(f'episode_merge_gap은 0 이상이어야 한다: {self.episode_merge_gap}')
        if any(not (math.isfinite(h) and h > 0.0) for h in self.horizons):
            raise ValueError(f'horizons는 양수여야 한다: {self.horizons}')


@dataclass
class _Frame:
    t: float
    tracks: List[TrackObs]
    robot_xy: Optional[Tuple[float, float]]


def _stats(values: Iterable[float]) -> Dict[str, Optional[float]]:
    """개수, RMSE(제곱평균제곱근), 평균, 중앙, 95%, 최대."""
    v = np.asarray([x for x in values if x is not None and math.isfinite(x)], dtype=float)
    if v.size == 0:
        return {'n': 0, 'rmse': None, 'mean': None, 'median': None, 'p95': None, 'max': None}
    return {'n': int(v.size), 'rmse': float(np.sqrt(np.mean(v ** 2))), 'mean': float(np.mean(v)),
            'median': float(np.median(v)), 'p95': float(np.percentile(v, 95)),
            'max': float(np.max(v))}


def _nees(err: np.ndarray, cov: Optional[np.ndarray]) -> Optional[float]:
    if cov is None:
        return None
    cov = np.asarray(cov, dtype=float)
    if cov.shape != (2, 2) or not np.all(np.isfinite(cov)):
        return None
    try:
        if np.linalg.eigvalsh((cov + cov.T) / 2.0).min() <= 0.0:
            return None
        return float(err @ np.linalg.solve(cov, err))
    except np.linalg.LinAlgError:
        return None


def _speed_bin(speed: float) -> str:
    if speed < 0.6:
        return '<0.6'
    if speed < 1.0:
        return '0.6-1.0'
    return '>=1.0'


class TrackingEvaluator:
    """GT와 트랙 프레임을 모아 두었다가 evaluate()로 지표를 계산한다.

    add_gt(물체 id, t, x, y, vx, vy): GT (평가 좌표계, 속도도 같은 좌표계 축)
    add_frame(t, tracks, robot_xy): 한 추적 주기의 트랙 (같은 좌표계). robot_xy를 주면 센서 범위를 본다
    """

    def __init__(self, config: EvalConfig = EvalConfig(), gt_ids: Sequence[str] = ()):
        self.config = config
        self._gt: Dict[str, TimeSeries] = {gid: TimeSeries() for gid in gt_ids}
        self._frames: List[_Frame] = []

    @property
    def gt_ids(self) -> List[str]:
        return list(self._gt)

    @property
    def num_frames(self) -> int:
        return len(self._frames)

    def gt_count(self, gid: str) -> int:
        return len(self._gt.get(gid, ()))

    def add_gt(self, gid: str, t: float, x: float, y: float, vx: float, vy: float) -> None:
        self._gt.setdefault(gid, TimeSeries()).add(t, (x, y, vx, vy))

    def add_frame(self, t: float, tracks: Sequence[TrackObs],
                  robot_xy: Optional[Sequence[float]] = None) -> None:
        if not math.isfinite(t):
            return
        robot = None if robot_xy is None else (float(robot_xy[0]), float(robot_xy[1]))
        self._frames.append(_Frame(float(t), list(tracks), robot))

    # ------------------------------------------------------------------ 계산
    def evaluate(self) -> Tuple[List[dict], List[dict], dict]:
        """(프레임별 행, 동적 판정 지연 구간, 요약) 을 돌려준다."""
        cfg = self.config
        frames = sorted(self._frames, key=lambda f: f.t)
        gids = list(self._gt)
        rows: List[dict] = []
        # GT 물체별 시간순 기록: (t, 움직임, 범위 안, 짝 트랙 id 또는 None, 짝 트랙 동적, GT 속력, 로봇 거리)
        history: Dict[str, List[tuple]] = {gid: [] for gid in gids}
        per_frame_false, per_frame_dup, per_frame_tracks, per_frame_dyn = [], [], [], []
        pos_err_all: Dict[str, List[float]] = {gid: [] for gid in gids}
        acc: Dict[str, Dict[str, list]] = {gid: {} for gid in gids}

        def push(gid, key, value):
            acc[gid].setdefault(key, []).append(value)

        for frame in frames:
            gt_now = []
            for gid in gids:
                v = self._gt[gid].at(frame.t, cfg.max_gt_gap)
                if v is not None:
                    gt_now.append((gid, v))
            txy = np.array([[tr.x, tr.y] for tr in frame.tracks]).reshape(-1, 2)
            gxy = np.array([v[:2] for _, v in gt_now]).reshape(-1, 2)
            matches = associate_nearest(txy, gxy, cfg.gate)
            matched_t = {i for i, _, _ in matches}
            matched_g = {j: (i, d) for i, j, d in matches}

            n_false = n_dup = 0
            for i, tr in enumerate(frame.tracks):
                if i in matched_t:
                    continue
                near = bool(len(gxy)) and float(
                    np.min(np.linalg.norm(gxy - txy[i], axis=1))) <= cfg.gate
                if tr.is_dynamic:
                    if near:
                        n_dup += 1
                    else:
                        n_false += 1
                rows.append({'t': frame.t, 'kind': 'track_only', 'gt_id': '',
                             'track_id': tr.track_id, 'is_dynamic': int(tr.is_dynamic),
                             'near_gt': int(near)})
            per_frame_false.append(n_false)
            per_frame_dup.append(n_dup)
            per_frame_tracks.append(len(frame.tracks))
            per_frame_dyn.append(sum(tr.is_dynamic for tr in frame.tracks))

            for j, (gid, g) in enumerate(gt_now):
                gpos, gvel = g[:2], g[2:]
                gspeed = float(np.hypot(*gvel))
                moving = gspeed >= cfg.moving_speed
                rdist = (None if frame.robot_xy is None
                         else float(np.hypot(gpos[0] - frame.robot_xy[0],
                                             gpos[1] - frame.robot_xy[1])))
                in_range = rdist is None or rdist <= cfg.sensor_range
                row = {'t': frame.t, 'gt_id': gid, 'gt_x': gpos[0], 'gt_y': gpos[1],
                       'gt_vx': gvel[0], 'gt_vy': gvel[1], 'gt_speed': gspeed,
                       'moving': int(moving), 'in_range': int(in_range), 'robot_dist': rdist}
                if j not in matched_g:
                    history[gid].append((frame.t, moving, in_range, None, False, gspeed, rdist))
                    if in_range:
                        row['kind'] = 'miss'
                        rows.append(row)
                    continue
                i, d = matched_g[j]
                tr = frame.tracks[i]
                history[gid].append((frame.t, moving, in_range, tr.track_id, tr.is_dynamic,
                                     gspeed, rdist))
                perr = np.array([tr.x, tr.y]) - gpos
                verr = np.array([tr.vx, tr.vy]) - gvel
                tspeed = float(math.hypot(tr.vx, tr.vy))
                row.update({'kind': 'match', 'track_id': tr.track_id,
                            'is_dynamic': int(tr.is_dynamic),
                            'tr_x': tr.x, 'tr_y': tr.y, 'tr_vx': tr.vx, 'tr_vy': tr.vy,
                            'pos_err': d, 'vel_err': float(np.linalg.norm(verr)),
                            'speed_err': tspeed - gspeed})
                push(gid, 'pos', d)
                pos_err_all[gid].append(d)
                push(gid, 'vel', row['vel_err'])
                push(gid, 'speed', row['speed_err'])
                if frame.robot_xy is not None and rdist and rdist > 1e-6:
                    # 센서 → 물체 방향 성분: 음수면 트랙이 센서 쪽으로 치우침 (군집 중심 치우침)
                    u = (gpos - np.array(frame.robot_xy)) / rdist
                    row['radial_err'] = float(perr @ u)
                    push(gid, 'radial', row['radial_err'])
                if moving and tspeed >= cfg.heading_min_speed:
                    herr = math.degrees(abs(wrap_angle(math.atan2(tr.vy, tr.vx)
                                                       - math.atan2(gvel[1], gvel[0]))))
                    row['heading_err_deg'] = herr
                    push(gid, 'heading', herr)
                nees_p = _nees(perr, tr.pos_cov)
                nees_v = _nees(verr, tr.vel_cov)
                if nees_p is not None:
                    row['nees_pos'] = nees_p
                    push(gid, 'nees_pos', nees_p)
                if nees_v is not None:
                    row['nees_vel'] = nees_v
                    push(gid, 'nees_vel', nees_v)
                if moving:
                    for h in cfg.horizons:
                        fut = self._gt[gid].at(frame.t + h, cfg.max_gt_gap)
                        if fut is None:
                            continue
                        pred = np.array([tr.x + tr.vx * h, tr.y + tr.vy * h])
                        e_cv = float(np.linalg.norm(pred - fut[:2]))
                        e_hold = float(np.linalg.norm(np.array([tr.x, tr.y]) - fut[:2]))
                        row[f'pred_err_{h:g}s'] = e_cv
                        row[f'hold_err_{h:g}s'] = e_hold
                        push(gid, f'pred_{h:g}', e_cv)
                        push(gid, f'hold_{h:g}', e_hold)
                rows.append(row)

        episodes = self._episodes(history)
        summary = self._summarize(frames, gids, acc, history, episodes,
                                  per_frame_false, per_frame_dup, per_frame_tracks, per_frame_dyn)
        return rows, episodes, summary

    def _episodes(self, history: Dict[str, List[tuple]]) -> List[dict]:
        """GT 물체가 '움직이며 센서 범위 안'인 구간마다 동적 판정 지연을 잰다.

        t_start: 구간 시작 (움직이기 시작했거나 범위에 들어온 시각)
        t_seen: 구간 안에서 처음 트랙과 짝지어진 시각 (가려짐으로 늦어질 수 있다)
        t_dynamic: 짝지어진 트랙이 처음 동적으로 판정된 시각
        censored: 기록 첫 프레임부터 이미 구간 안이었다 (시작 시각을 모름) → 지연 통계에서 뺀다
        carried: 처음 짝지어진 트랙이 구간 전부터 이 물체를 추적해 이미 동적이었다 → 지연 통계에서 뺀다
        """
        cfg = self.config
        out = []
        first_t = min((h[0][0] for h in history.values() if h), default=None)
        for gid, hist in history.items():
            segments: List[List[tuple]] = []
            last_active_t = None
            for rec in hist:
                t, moving, in_range = rec[0], rec[1], rec[2]
                if not (moving and in_range):
                    continue
                if last_active_t is None or t - last_active_t > cfg.episode_merge_gap:
                    segments.append([])
                segments[-1].append(rec)
                last_active_t = t
            for seg in segments:
                t_start = seg[0][0]
                seen = next((r for r in seg if r[3] is not None), None)
                dyn = next((r for r in seg if r[3] is not None and r[4]), None)
                earlier_ids = {r[3] for r in hist if r[0] < t_start and r[3] is not None}
                carried = bool(seen is not None and seen[4] and seen[3] in earlier_ids)
                ep = {
                    'gt_id': gid, 't_start': t_start, 't_end': seg[-1][0],
                    'duration': seg[-1][0] - t_start,
                    'mean_speed': float(np.mean([r[5] for r in seg])),
                    'start_dist': seg[0][6],
                    't_seen': None if seen is None else seen[0],
                    't_dynamic': None if dyn is None else dyn[0],
                    'dynamic_dist': None if dyn is None else dyn[6],
                    'latency_from_start': None if dyn is None else dyn[0] - t_start,
                    'latency_from_seen': (None if dyn is None or seen is None
                                          else dyn[0] - seen[0]),
                    'censored': int(first_t is not None and t_start <= first_t),
                    'carried': int(carried),
                }
                ep['speed_bin'] = _speed_bin(ep['mean_speed'])
                out.append(ep)
        return out

    def _summarize(self, frames, gids, acc, history, episodes,
                   per_frame_false, per_frame_dup, per_frame_tracks, per_frame_dyn) -> dict:
        cfg = self.config
        keys = ['pos', 'vel', 'speed', 'radial', 'heading', 'nees_pos', 'nees_vel']
        keys += [f'{p}_{h:g}' for h in cfg.horizons for p in ('pred', 'hold')]

        def block(values: Dict[str, list]) -> dict:
            b = {k: _stats(values.get(k, [])) for k in keys}
            # radial·speed는 평균이 편향이다 (센서 쪽 치우침, 속력 과소·과대)
            for k in ('nees_pos', 'nees_vel'):
                vals = values.get(k, [])
                b[k]['within_chi2_95'] = (None if not vals else
                                          float(np.mean(np.asarray(vals) <= cfg.nees_chi2_95)))
            return b

        per_object = {}
        merged: Dict[str, list] = {}
        for gid in gids:
            hist = history[gid]
            in_range = [r for r in hist if r[2]]
            ids = [r[3] for r in hist if r[3] is not None]
            switches = sum(1 for a, b in zip(ids, ids[1:]) if a != b)
            obj = block(acc[gid])
            obj.update({
                'frames_in_range': len(in_range),
                'frames_matched': sum(1 for r in hist if r[3] is not None),
                'match_rate_in_range': (
                    None if not in_range
                    else sum(1 for r in in_range if r[3] is not None) / len(in_range)),
                'id_switches': switches,
                'distinct_track_ids': len(set(ids)),
            })
            per_object[gid] = obj
            for k, v in acc[gid].items():
                merged.setdefault(k, []).extend(v)
        overall = block(merged)
        total_in_range = sum(o['frames_in_range'] for o in per_object.values())
        overall.update({
            'frames_in_range': total_in_range,
            'frames_matched': sum(o['frames_matched'] for o in per_object.values()),
            'id_switches': sum(o['id_switches'] for o in per_object.values()),
        })
        overall['match_rate_in_range'] = (None if not total_in_range else
                                          overall['frames_matched'] / total_in_range)

        valid = [e for e in episodes if not e['censored'] and not e['carried']]
        seen = [e for e in valid if e['t_seen'] is not None]
        lat = {
            'episodes': len(episodes),
            'censored_or_carried': len(episodes) - len(valid),
            'seen': len(seen),
            'unseen': len(valid) - len(seen),
            'classified': sum(1 for e in seen if e['t_dynamic'] is not None),
            'not_classified': sum(1 for e in seen if e['t_dynamic'] is None),
            'from_seen': _stats(e['latency_from_seen'] for e in seen),
            'from_start': _stats(e['latency_from_start'] for e in seen),
            'dynamic_dist': _stats(e['dynamic_dist'] for e in seen),
            'by_speed': {},
        }
        for b in ('<0.6', '0.6-1.0', '>=1.0'):
            es = [e for e in seen if e['speed_bin'] == b]
            lat['by_speed'][b] = {
                'seen': len(es),
                'classified': sum(1 for e in es if e['t_dynamic'] is not None),
                'from_seen': _stats(e['latency_from_seen'] for e in es),
            }

        n = max(len(frames), 1)
        duration = frames[-1].t - frames[0].t if frames else 0.0
        return {
            'config': {k: (list(v) if isinstance(v, tuple) else v)
                       for k, v in asdict(cfg).items()},
            'frames': len(frames),
            'duration_s': duration,
            'tracks_per_frame': sum(per_frame_tracks) / n,
            'dynamic_tracks_per_frame': sum(per_frame_dyn) / n,
            'false_dynamic_per_frame': sum(per_frame_false) / n,
            'duplicate_dynamic_per_frame': sum(per_frame_dup) / n,
            'frames_with_false_dynamic': (sum(1 for k in per_frame_false if k) / n),
            'overall': overall,
            'per_object': per_object,
            'latency': lat,
        }


# ---------------------------------------------------------------------------- 출력
def _f(value, fmt='.2f', scale=1.0) -> str:
    return '-' if value is None else format(value * scale, fmt)


def format_summary_ko(summary: dict) -> str:
    """요약을 사람이 읽는 한국어 표로 만든다 (터미널 출력용)."""
    cfg = summary['config']
    o = summary['overall']
    lines = [
        f'추적 정확도 (GT 대비): 프레임 {summary["frames"]}개, {summary["duration_s"]:.1f} s, '
        f'게이트 {cfg["gate"]} m, 센서 범위 {cfg["sensor_range"]} m',
        '',
        '| 물체 | 범위 안 짝 비율 | 위치 RMSE [m] | 센서 방향 편향 [m] | 속도 RMSE [m/s] | 방향 오차 평균 [°] | '
        + ' | '.join(f'+{h:g} s 예측 RMSE [m] (정지 가정)' for h in cfg['horizons'])
        + ' | NEES 위치 평균 (χ²95% 안) | ID 전환 |',
        '|' + ' --- |' * (8 + len(cfg['horizons'])),
    ]

    def line(name, b, switches):
        preds = ' | '.join(
            f'{_f(b[f"pred_{h:g}"]["rmse"])} ({_f(b[f"hold_{h:g}"]["rmse"])})'
            for h in cfg['horizons'])
        return (f'| {name} | {_f(b.get("match_rate_in_range"), ".0f", 100)}% '
                f'({b["frames_matched"]}/{b["frames_in_range"]}) | '
                f'{_f(b["pos"]["rmse"], ".3f")} | '
                f'{_f(b["radial"]["mean"], ".3f")} | {_f(b["vel"]["rmse"], ".3f")} | '
                f'{_f(b["heading"]["mean"], ".1f")} | {preds} | '
                f'{_f(b["nees_pos"]["mean"], ".1f")} '
                f'({_f(b["nees_pos"]["within_chi2_95"], ".0f", 100)}%) | '
                f'{switches} |')

    for gid, b in summary['per_object'].items():
        lines.append(line(gid, b, b['id_switches']))
    lines.append(line('**전체**', o, o['id_switches']))
    lat = summary['latency']
    lines += [
        '',
        f'동적 판정 지연: 구간 {lat["episodes"]}개 (기록 시작 전부터·이미 동적 {lat["censored_or_carried"]}개 제외), '
        f'관측 {lat["seen"]}개 중 동적 판정 {lat["classified"]}개, 미판정 {lat["not_classified"]}개, '
        f'안 보임 {lat["unseen"]}개',
        f'- 처음 짝지어진 뒤: 평균 {_f(lat["from_seen"]["mean"])} s, '
        f'중앙 {_f(lat["from_seen"]["median"])} s, '
        f'최대 {_f(lat["from_seen"]["max"])} s',
        f'- 움직이며 범위에 들어온 뒤: 평균 {_f(lat["from_start"]["mean"])} s, '
        f'중앙 {_f(lat["from_start"]["median"])} s (가려짐 포함)',
        f'- 판정 시 로봇 거리: 중앙 {_f(lat["dynamic_dist"]["median"], ".1f")} m',
    ]
    for b, v in lat['by_speed'].items():
        lines.append(f'- 속력 {b} m/s: 판정 {v["classified"]}/{v["seen"]}, '
                     f'지연 평균 {_f(v["from_seen"]["mean"])} s, 최대 {_f(v["from_seen"]["max"])} s')
    lines += [
        '',
        f'프레임당 트랙 {summary["tracks_per_frame"]:.2f}개 '
        f'(동적 {summary["dynamic_tracks_per_frame"]:.2f}), '
        f'가짜 동적 {summary["false_dynamic_per_frame"]:.2f}개 '
        f'(가짜가 있는 프레임 {100 * summary["frames_with_false_dynamic"]:.0f}%), '
        f'GT 옆 중복 동적 {summary["duplicate_dynamic_per_frame"]:.2f}개',
    ]
    return '\n'.join(lines)


__all__ = [
    'EvalConfig', 'TimeSeries', 'TrackObs', 'TrackingEvaluator', 'associate_nearest',
    'format_summary_ko', 'se2_compose', 'se2_inverse', 'world_from_odom', 'wrap_angle',
]
