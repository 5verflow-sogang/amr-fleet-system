"""식당 시연 비교 영상 강조 표시 (tools/demo/overlay.py, compose.py plan). ROS·ffmpeg 없이 가짜 녹화로 돈다."""

import math
import sys
from pathlib import Path

import pytest

PIL = pytest.importorskip('PIL')
from PIL import Image  # noqa: E402

DEMO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEMO))

import compose  # noqa: E402
import overlay as ov  # noqa: E402

S11 = DEMO / 'cuts' / 's11.yaml'
ROBOTS_S11 = {'amr_01': (-10.5, -5.6), 'amr_02': (11.2, -5.8), 'amr_03': (1.5, 4.0), 'amr_04': (11.2, -3.4),
              'amr_05': (11.2, -2.2)}


def fonts():
    try:
        return ov.Fonts()
    except FileNotFoundError:
        pytest.skip('한글 글꼴 없음')


# ---------------------------------------------------------------- 카메라
def test_top_camera_projection_is_the_simple_top_down_formula():
    top = ov.load_cameras()['top']
    # 15 m 위, 가로 화각 90° → fx 640. 화면 위 = 북쪽(+y), 오른쪽 = 동쪽(+x)
    for x, y, z in ((0, 0, 0), (-10.5, -5.6, 1.0), (1.5, 4.15, 1.0), (8, -3, 0.5)):
        u, v, depth = top.project(x, y, z)
        f = 640 / (15 - z)
        assert (u, v, depth) == pytest.approx((640 + x * f, 360 - y * f, 15 - z), abs=0.05)   # yaml 1.5708 ≈ π/2
    # 11/8 녹화 top.mp4에서 1호 번호판이 보인 자리 (약 150, 612)
    u, v, _ = top.project(-10.5, -5.6, 1.0)
    assert abs(u - 150) < 15 and abs(v - 612) < 15


def test_eye_level_camera_sees_ahead_and_not_behind():
    west = ov.load_cameras()['west']                       # (-10, -7.8, 1.6) 북쪽을 본다
    ahead = west.project(-10.0, 4.15, 0.9)                 # 배식대
    assert ahead and west.inside(ahead[0], ahead[1]) and ahead[2] == pytest.approx(11.95, abs=0.1)
    assert west.project(-10.0, -12.0, 1.0) is None         # 카메라 뒤
    near = west.project(-10.0, -3.0, 1.0)
    assert near[1] > ahead[1]                              # 가까운 것이 화면 아래


def test_top_west_frames_both_candidates_the_buffet_and_the_west_aisle():
    cam = ov.load_cameras()['top_west']
    st = ov.load_stations()
    points = [(*ROBOTS_S11['amr_01'], ov.ROBOT_Z), (*ROBOTS_S11['amr_03'], ov.ROBOT_Z),
              (*st['buffet_1'], ov.STATION_Z), (*st['t01'], 0.8), (-11.9, -5.0, 0.0), (-9.73, 2.99, 0.0)]
    for p in points:
        u, v, _ = cam.project(*p)
        assert cam.inside(u, v, margin=20), p
    # top보다 로봇이 크게 보인다
    top = ov.load_cameras()['top']
    assert cam.fx / (cam.pose[2] - 1) > 1.25 * top.fx / (top.pose[2] - 1)


# ---------------------------------------------------------------- 녹화 기록 → timeline
def fake_records(side='off'):
    """1-1 가짜 녹화: 10 s에 /demo/t0, 끔은 1호(느리게, 중간에 멈춤), 켬은 3호(빠르게)가 buffet_1로 간다."""
    hero = 'amr_01' if side == 'off' else 'amr_03'
    t0, assign, arrive = 10.0, 10.4, (40.0 if side == 'off' else 25.0)
    rows = lambda who: [{'task_id': 's11_buffet1_t01', 'task_type': 0, 'priority': 100, 'pickup': 'buffet_1',  # noqa
                         'dropoff': 't01', 'created': t0, 'assignee': who, 'state': 1}]
    table = [(t0 + 0.1, rows('')), (assign, rows(hero))]
    statuses = [{'t': arrive, 'task_id': 's11_buffet1_t01', 'state': 1, 'phase': 'loading'}]
    start, goal = ROBOTS_S11[hero], (-9.9, 4.15)
    poses = {r: [(t, *xy) for t in (0.0, 60.0)] for r, xy in ROBOTS_S11.items() if r != hero}
    track = []
    for k in range(0, 121):
        t = k * 0.5
        a = min(max((t - assign) / (arrive - assign), 0.0), 1.0)
        if side == 'off' and 20.0 <= t <= 24.0:            # 손님 사이에서 4 s 멈춤
            a = (20.0 - assign) / (arrive - assign)
        elif side == 'off' and 24.0 < t < arrive:
            a = (20.0 - assign + (t - 24.0) * (arrive - 20.0) / (arrive - 24.0)) / (arrive - assign)
        track.append((t, start[0] + (goal[0] - start[0]) * a, start[1] + (goal[1] - start[1]) * a))
    poses[hero] = track
    congestion = [(t, [('west_aisle', [-11.9, -9.73, -5.0, 2.99], 1.0), ('buffet_line', [-11.9, -6.1, 3.0, 4.63], 1.0),
                       ('stall_front', [-6.09, 6.5, 3.0, 4.63], 0.17)]) for t in (1.0, 30.0)]
    videos = {cam: {'topic': f'/demo_cam/{cam}/image', 'start_ns': 2_000_000_000, 'fps': 15.0, 'frames': 900}
              for cam in ('top_west', 'west', 'stalls')}
    return table, statuses, poses, congestion, [(t0, 's11_buffet1_t01')], videos


def fake_run(side):
    return ov.Run.from_timeline(side, ov.build_timeline(*fake_records(side)))


def test_build_timeline_finds_request_assignment_arrival_and_stops():
    tl = ov.build_timeline(*fake_records('off'))
    assert (tl['key_task'], tl['pickup'], tl['assignee'], tl['t0']) == ('s11_buffet1_t01', 'buffet_1', 'amr_01', 10.0)
    assert (tl['assign_t'], tl['arrive_t']) == (10.4, 40.0)
    m = tl['metrics']
    assert (m['assign_s'], m['arrive_s'], m['stops']) == (0.4, 30.0, 1)
    assert m['stop_s'] == pytest.approx(4.5, abs=0.6)
    on = ov.build_timeline(*fake_records('on'))
    assert (on['assignee'], on['metrics']['arrive_s'], on['metrics']['stops']) == ('amr_03', 15.0, 0)


def test_build_timeline_without_t0_needs_the_key_task():
    table, statuses, poses, cong, _, videos = fake_records()
    with pytest.raises(ValueError, match='핵심 작업'):
        ov.build_timeline(table, statuses, poses, cong, [], videos)
    tl = ov.build_timeline(table, statuses, poses, cong, [], videos, key_task='s11_buffet1_t01')
    assert tl['t0'] == 10.0                                        # 요청 시각(created_time)


def test_track_interpolates_and_holds_the_ends():
    tr = ov.Track([(0, 0, 0), (2, 2, 4)])
    assert tr.at(1) == (1, 2) and tr.at(-5) == (0, 0) and tr.at(9) == (2, 4)


# ---------------------------------------------------------------- 컷시트
def test_s11_cut_sheet_uses_known_cameras_layers_and_events():
    sheet = ov.CutSheet.load(S11)
    cams = ov.load_cameras()
    runs = {s: fake_run(s) for s in ov.SIDES}
    ev = ov.events(runs)
    assert (ev['assign_off'], ev['arrive_off'], ev['arrive_on'], ev['arrive']) == pytest.approx((0.4, 30.0, 15.0, 30.0))
    for cut in sheet.cuts:
        if cut.result:
            continue
        assert cut.camera in cams and cut.camera in sheet.camera_names
        start = ov.resolve(cut.start, ev)
        assert start + cut.length * cut.speed <= ev['end']
        assert ov.check_cut(cut, ev) is None, cut.title
    assert sheet.cuts[-1].result
    # 가이드: 컷 전환 바로 이어 붙이고 전체 약 40 s 안팎
    assert 25 <= sum(c.total for c in sheet.cuts) <= 60          # 글자 읽을 시간 (컷 6 s 이상)


def test_resolve_and_check_report_missing_or_reversed_events():
    ev = {'request': 0.0, 'arrive_on': 30.0, 'arrive_off': None, 'end': 90.0}
    assert ov.resolve('arrive_on-8', ev) == 22.0 and ov.resolve('request + 2.5', ev) == 2.5
    with pytest.raises(ValueError, match='arrive_off'):
        ov.resolve('arrive_off-9', ev)
    with pytest.raises(ValueError, match='못 읽는다'):
        ov.resolve('soon', ev)
    cut = ov.Cut(title='오른쪽이 먼저', check='arrive_on < request+10')
    assert '아니다' in ov.check_cut(cut, ev)


def test_plan_drops_cuts_whose_event_is_missing():
    runs = {s: fake_run(s) for s in ov.SIDES}
    runs['off'].arrive_t = None                                       # 끔이 도착하지 못한 녹화
    rows, warnings, _ = compose.plan(ov.CutSheet.load(S11), runs)
    titles = [c.title for c, _, _ in rows]
    assert '왼쪽 {robot_off}도 배식대에 도착합니다' not in titles
    assert any('arrive_off' in w for w in warnings)


def test_text_values_fill_robot_names_times_and_distances():
    runs = {s: fake_run(s) for s in ov.SIDES}
    v = ov.text_values(runs, ov.load_stations(), ['amr_01', 'amr_03'])
    assert (v['robot_off'], v['robot_on'], v['arrive_off'], v['arrive_on']) == ('1호', '3호', '30', '15')
    assert (v['dist_amr_01'], v['dist_amr_03']) == ('9.8', '11.4')
    assert ov.fill('{robot_on}가 {unknown}', v) == '3호가 {unknown}'


# ---------------------------------------------------------------- 그리기
def _gray_frame(cam):
    return Image.new('RGB', (cam.width, cam.height), (90, 90, 90))


def _near(img, xy, color, radius=40, tol=30):
    px = img.load()
    x0, y0 = int(xy[0]), int(xy[1])
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if 0 <= x < img.width and 0 <= y < img.height and all(abs(a - b) <= tol for a, b in zip(px[x, y], color)):
                return True
    return False


def test_assigned_robot_gets_its_side_color_ring_and_others_are_dimmed():
    sheet = ov.CutSheet.load(S11)
    cams = ov.load_cameras()
    painter = ov.Painter(sheet, cams, ov.load_stations(), fonts())
    cam = cams['top_west']
    s = ov.VIDEO_SIZE[0] / cam.width
    for side, hero in (('off', 'amr_01'), ('on', 'amr_03')):
        run = fake_run(side)
        t = run.t0 + 1.0                                               # 배정 0.6 s 뒤
        img = painter.video(_gray_frame(cam), 'top_west', run, t, ['assign', 'dim'])
        assert img.size == ov.VIDEO_SIZE
        u, v, _ = cam.project(*run.pose(hero, t), ov.ROBOT_Z)
        assert _near(img, (u * s, v * s), ov.SIDE_COLOR[side])
        # 동그라미 밖(2호 자리 근처)은 어둡다
        u2, v2, _ = cam.project(*ROBOTS_S11['amr_05'], 0)
        if cam.inside(u2, v2):
            assert sum(img.getpixel((int(u2 * s), int(v2 * s)))) < 3 * 90
    # 배정 전에는 동그라미가 없다
    run = fake_run('off')
    before = painter.video(_gray_frame(cam), 'top_west', run, run.t0 + 0.2, ['assign'])
    u, v, _ = cam.project(*ROBOTS_S11['amr_01'], ov.ROBOT_Z)
    assert not _near(before, (u * s, v * s), ov.SIDE_COLOR['off'])


def test_request_ring_appears_at_the_buffet_from_t0():
    sheet = ov.CutSheet.load(S11)
    cams = ov.load_cameras()
    painter = ov.Painter(sheet, cams, ov.load_stations(), fonts())
    cam, run = cams['top_west'], fake_run('on')
    s = ov.VIDEO_SIZE[0] / cam.width
    u, v, _ = cam.project(*ov.load_stations()['buffet_1'], ov.STATION_Z)
    before = painter.video(_gray_frame(cam), 'top_west', run, run.t0 - 0.5, ['request'])
    after = painter.video(_gray_frame(cam), 'top_west', run, run.t0 + 4.0, ['request'])
    assert not _near(before, (u * s, v * s), ov.REQUEST_COLOR) and _near(after, (u * s, v * s), ov.REQUEST_COLOR)


def test_full_frames_follow_the_guide_canvas(tmp_path):
    sheet = ov.CutSheet.load(S11)
    cams = ov.load_cameras()
    f = fonts()
    painter = ov.Painter(sheet, cams, ov.load_stations(), f)
    runs = {s: fake_run(s) for s in ov.SIDES}
    values = ov.text_values(runs, ov.load_stations(), sheet.candidates)
    cut = sheet.cuts[1]
    panels = {s: painter.video(_gray_frame(cams[cut.camera]), cut.camera, r, r.t0 + 1.0, cut.layers)
              for s, r in runs.items()}
    img = painter.frame(cut, 2, len(sheet.cuts), 0.3, panels, values)
    assert img.size == ov.CANVAS
    assert img.getpixel((10, 10)) == ov.BG
    assert img.getpixel((36 + 5, 243 + 400)) != ov.PANEL_BG               # 원본 영상 자리
    assert img.getpixel((40 + 100, 845)) == ov.BAR_FILL                    # 진행선 채움
    res = ov.result_frame(sheet, runs, f, len(sheet.cuts), len(sheet.cuts), values)
    assert res.size == ov.CANVAS
    img.save(tmp_path / 'cut.png')
    res.save(tmp_path / 'result.png')


def test_s11_scene_distances_match_the_scene_file_header():
    st = ov.load_stations()
    assert math.dist(ROBOTS_S11['amr_01'], st['buffet_1']) == pytest.approx(9.77, abs=0.01)
    assert math.dist(ROBOTS_S11['amr_03'], st['buffet_1']) == pytest.approx(11.4, abs=0.01)
