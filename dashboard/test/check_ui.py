"""관제 대시보드 화면 시험 (「관제 대시보드 개발 명세서」 3장 완료 기준, 5장 문구 규칙).

rosbridge 없이 window.dashTest.feed로 메시지를 넣고, 시간은 Playwright 가짜 시계로 돌린다
(/clock이 없으면 대시보드는 브라우저 시각을 쓴다). ROS·Docker가 필요 없다.

    pip install playwright && playwright install chromium
    python3 dashboard/test/check_ui.py                    # 결과 표, test/out/ui_result.json
    python3 dashboard/test/check_ui.py --shots out/shots  # 시연 장면 캡처도 남긴다
    python3 dashboard/test/check_ui.py --channel chrome   # 설치된 Chrome으로
"""

import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path

from playwright.sync_api import sync_playwright

HERE = Path(__file__).resolve().parent
HTML = HERE.parent / 'index.html'
START = datetime(2026, 11, 8, 12, 0, 0)   # 가짜 시계 시작 (스토리 시각)

# 시험용 메시지를 만드는 함수. 페이지 안의 S(정차 자세)를 그대로 쓴다
HELPERS = r"""
window.H = {
  P: (x, y) => ({ position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }),
  at: id => H.P(S[id][0], S[id][1]),
  robot(id, x, y, mode = 0, task = '', b = 70) {
    dashTest.feed('/fleet/robot_states', { robot_id: id, pose: H.P(x, y), mode, battery_percent: b, current_task_id: task });
  },
  task(id, from, to, o = {}) {
    const created = Date.now() / 1000 - (o.age ?? 0);
    return { task: { task_id: id, task_type: o.type ?? 0, item_type: 1, pickup_station_id: from, pickup_pose: H.at(from),
      dropoff_station_id: to, dropoff_pose: H.at(to), priority: o.pr ?? 100, deadline: { sec: 0, nanosec: 0 },
      created_time: { sec: Math.floor(created), nanosec: Math.round((created % 1) * 1e9) } },
      assignee_id: o.who ?? '', assignment_id: 0, state: o.state ?? 0 };
  },
  // 충전 작업 (출발지 없음, 충전소로)
  charge(id, who, o = {}) {
    const t = H.task(id, 'return_2', 'return_2', { type: 1, who, state: 1, ...o });
    t.task.pickup_station_id = ''; t.task.dropoff_station_id = 'charger_1'; t.task.dropoff_pose = H.P(8, -7.6);
    return t;
  },
  table(recs) { dashTest.feed('/fleet/task_table', { records: recs }); },
  status(id, robot, state, phase = '') { dashTest.feed('/fleet/task_status', { task_id: id, robot_id: robot, state, phase }); },
  safety(id, level, reason = 1) { dashTest.feed(`/${id}/safety/status`, { level, reason }); },
  zones(list) { dashTest.feed('/fleet/congestion', { zones: list.map(([zone_id, area, people_count, value]) => ({ zone_id, area, people_count, value })) }); },
  decision(task, robot, cost_fn, cands) {
    dashTest.feed('/fleet/assignment_decisions', { task_id: task, robot_id: robot, cost_fn, congestion_weight: 3,
      candidates: cands.map(([robot_id, distance, congested_length, cost]) => ({ robot_id, distance, congested_length, cost })) });
  },
  // 배터리가 있는 로봇 (rate: 장면 표에 남겨 둔 분당 줄어드는 %. 화면은 %와 색만 쓴다)
  bat(id, x, y, mode, task, b, rate) {
    H.robot(id, x, y, mode, task, b);
  },
  // 홀 투입 그래프 3분 기록: [[몇 초 전부터, 대수], ...]
  hall(segs) {
    const t = now(); hallHistory.length = 0;
    for (let s = 179; s >= 1; s--) { let v = segs[0][1]; for (const [from, n] of segs) if (s <= from) v = n; hallHistory.push([t - s, v]); }
  },
  // 끝난 작업 수를 채운다 (마무리 장면)
  done(n, kind) { for (let i = 0; i < n; i++) { const id = `${kind}_${i}`; doneIds.add(id); doneKinds[id] = kind; } },
  txt: sel => document.querySelector(sel)?.innerText.trim() ?? null,
  // 마지막 상태를 다시 보내 로봇이 '연결 끊김'이 되지 않게 한다
  alive() { Object.values(robots).forEach(r => r.msg && dashTest.feed('/fleet/robot_states', r.msg)); },
  // 1-1 장면: 1호·3호가 놀고 2·4·5호는 일하는 중
  scene11() {
    H.robot('amr_01', -10.5, -5.6); H.robot('amr_03', 1.2, 4.15);
    const busy = [H.task('b2', 'stall_1', 't10', { age: 40, who: 'amr_02', state: 1 }), H.task('b4', 'stall_2', 't13', { age: 40, who: 'amr_04', state: 1 }),
      H.task('b5', 'stall_4', 'L03', { age: 40, who: 'amr_05', state: 1 })];
    H.robot('amr_02', 4.74, -0.7, 1, 'b2'); H.robot('amr_04', -2.7, -2.65, 1, 'b4'); H.robot('amr_05', 7.5, 0.89, 1, 'b5');
    H.zones([['buffet_line', [-11.9, -6.1, 3.0, 4.63], 4, 1.0], ['west_aisle', [-11.9, -9.73, -5.0, 2.99], 3, 1.0]]);
    H.table(busy); dashTest.tick();
    return busy;
  },
  // 로봇 5대가 모두 일하는 중
  allBusy() {
    const xy = { amr_01: [-8.8, -3.05], amr_02: [-0.6, 1.0], amr_03: [3.4, -2.6], amr_04: [6.9, 1.2], amr_05: [-4.5, 3.6] };
    const from = ['buffet_1', 'stall_3', 'stall_2', 'stall_4', 'stall_1'], to = ['t11', 't08', 't15', 'L01', 't02'];
    const recs = Object.keys(xy).map((id, i) => H.task(`w${i}`, from[i], to[i], { age: 90, who: id, state: 1 }));
    Object.entries(xy).forEach(([id, [x, y]], i) => { H.robot(id, x, y, 1, `w${i}`); H.status(`w${i}`, id, 1, i === 4 ? 'loading' : 'move_to_dropoff'); });
    return recs;
  },
};
"""

# 화면에 쓰지 않는 말 (5장 용어 표, 설계안 v6 문구표)
FORBIDDEN = ['amr_0', '1번 로봇', '배달', '배송', '수거', '할당', '지정', '유휴', '쉬는 중', '큐', '지연', '혼잡도', '혼잡 구역',
             '환산', '비용', 'cost', '7번 좌석', '반납', '매장 3', '우선순위', 'priority', '놀고 있는', '가용', 'ETA',
             '멈춤', '신호 없음', '홀 운영', '작업표', '하는 일', '다 차기', '기다림', '기다리는', '점선', '부족', '→ 2호',
             '붐빔', '붐비는', '가지러', '싣는', '옮기는', '내리는', '하러 감', '먼저', '보냄', '순서대로', '1대뿐',
             '째', '앞이 막힘', '버튼 눌림', '확인하세요', '요청됨', '보내지 못함', '다시 연결', '충전까지', '완충까지',
             '위부터', '대기 작업 없음', '최근 3분', '배정 전']


class Check:
    def __init__(self):
        self.rows = []

    def ok(self, ref, name, cond, detail=''):
        self.rows.append({'ref': ref, 'name': name, 'pass': bool(cond), 'detail': str(detail)})


def open_page(browser, query='ws=off', clock=True, start=None):
    page = browser.new_page(viewport={'width': 1280, 'height': 720}, device_scale_factor=2)
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    if clock:
        page.clock.install(time=start or START)
    page.goto(f'{HTML.as_uri()}?{query}')
    if clock:
        # 시계를 멈춰 두고 run()으로만 넘긴다 (클릭하는 동안 시간이 흐르지 않게). 숫자 대신 시각으로 넘겨야 어긋나지 않는다
        page.clock.pause_at(datetime.fromtimestamp(page.evaluate('Date.now()') / 1000 + 0.05))
    page.add_script_tag(content=HELPERS)
    page.evaluate('dashTest.setConnected(true)')
    page.errors = errors
    return page


def run(page, ms):
    page.clock.run_for(ms)


def ev(page, js):
    return page.evaluate(f'() => {{ {js} }}')


def flat(text):
    return re.sub(r'\s+', ' ', text or '').strip()


def style_problems(text):
    """5장: 쓰지 않는 말, 마침표·느낌표, "-하세요" 말고 해요체."""
    bad = [w for w in FORBIDDEN if w in text]
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.endswith(('.', '!')):
            bad.append(f'끝맺음: {line}')
        if re.search(r'(요|니다)$', line) and not line.endswith('세요'):
            bad.append(f'문체: {line}')
    return bad


def test_header(browser, c):
    page = open_page(browser, 'ws=ws://127.0.0.1:9', clock=False)
    page.evaluate('dashTest.setConnected(false)')
    page.wait_for_timeout(1000)
    c.ok('3-1', 'rosbridge가 없으면 "연결 끊김 · 재연결 중"과 흐린 화면', page.inner_text('#conn') == '연결 끊김 · 재연결 중'
         and page.evaluate("document.body.classList.contains('offline')"), page.inner_text('#conn'))
    page.close()
    page = open_page(browser, 'ws=off&clock=12:07:00')
    ev(page, "dashTest.feed('/clock', { clock: { sec: 42, nanosec: 0 } }); dashTest.tick();")
    c.ok('3-1', '?clock=12:07:00이면 시뮬레이션 42초에 12:07:42, 세기 시작한 시각도 시뮬레이션 시간 ("집계 12:07 ~ 지금")',
         page.inner_text('#clock') == '12:07:42' and page.inner_text('#since') == '집계 12:07 ~ 지금',
         f"{page.inner_text('#clock')} / {page.inner_text('#since')}")
    c.ok('3-1', '화면 이름 "푸드코트 로봇 관제"', page.inner_text('header h1') == '푸드코트 로봇 관제' and page.title() == '푸드코트 로봇 관제', page.inner_text('header h1'))
    c.ok('3-1', '연결되면 "연결됨"', page.inner_text('#conn') == '연결됨', page.inner_text('#conn'))
    page.close()


def test_summary(browser, c):
    page = open_page(browser)
    names = page.evaluate("[...document.querySelectorAll('.tile .k')].map(k => k.textContent.trim())")
    c.ok('3-2', '숫자 칸 이름만 (범위 글은 상단 바 한 곳): 홀 투입 / 최장 대기 / 안전 정지 / 완료',
         names == ['홀 투입', '최장 대기', '안전 정지', '완료'], names)

    five = "['amr_01','amr_02','amr_03','amr_04','amr_05'].forEach((id, i) => H.robot(id, -6 + 3 * i, 0));"
    for _ in range(11):   # 그래프는 10초 넘게 쌓인 뒤에 그린다. 로봇 상태는 계속 온다
        ev(page, five); run(page, 1000)
    hall5 = page.inner_text('.tile:nth-child(1) .v')
    for _ in range(2):
        ev(page, five + "H.robot('amr_02', 6.5, -7.0, 2, '', 40);"); run(page, 1000)
    hall4 = page.inner_text('.tile:nth-child(1) .v')
    last_y = page.evaluate("document.querySelector('.tile:nth-child(1) polyline').getAttribute('points').trim().split(' ').pop().split(',')[1]")
    cap = page.inner_text('.tile:nth-child(1) .spark .cap')
    c.ok('3-2', '충전 중이 되면 홀 투입 5 → 4 / 5대, 그래프("3분")가 내려간다', hall5.startswith('5') and hall4.startswith('4') and float(last_y) == 10 and cap == '3분',
         f'{hall5} → {hall4}, 끝점 y={last_y}, {cap}')

    page3 = open_page(browser, 'ws=off&min_hall=3')
    out3 = five + "H.robot('amr_02', 6.5, -7, 2, '', 40); H.robot('amr_04', 8, -7, 2, '', 40); H.robot('amr_05', 5, -7, 2, '', 40);"
    for _ in range(12):
        ev(page3, out3); run(page3, 1000)
    goal = page3.inner_text('.tile:nth-child(1) .goal')
    hot = page3.evaluate("document.querySelector('.tile:nth-child(1)').classList.contains('hot')")
    red = page3.evaluate("!!document.querySelector('.tile:nth-child(1) polyline[clip-path]') && !!document.querySelector('.tile:nth-child(1) rect')")
    c.ok('3-2', '?min_hall=3: 숫자 옆 "최소 3대", 2대면 숫자 빨강, 3대 아래 띠와 그 안의 빨간 선 (점선 없음)', goal == '최소 3대' and hot and red
         and not page3.query_selector('.tile:nth-child(1) line'), f'{goal} 빨강={hot} 띠·선={red}')
    page3.close()

    page2 = open_page(browser)
    ev(page2, "const busy = H.allBusy(); H.table([...busy, H.task('q1', 'stall_1', 't04', { age: 75 }), H.task('q2', 'stall_2', 't05', { age: 30 })]); dashTest.tick();")
    a = page2.inner_text('.tile:nth-child(2) .v')
    hot = page2.evaluate("document.querySelector('.tile:nth-child(2)').classList.contains('hot')")
    run(page2, 1300)
    b = page2.inner_text('.tile:nth-child(2) .v')
    ev(page2, "const busy = H.allBusy(); H.table([...busy, H.task('q1', 'stall_1', 't04', { age: 76, who: 'amr_01' }), H.task('q2', 'stall_2', 't05', { age: 31 })]); dashTest.tick();")
    after = page2.inner_text('.tile:nth-child(2) .v')
    c.ok('3-2', '최장 대기가 1초마다 오르고, 배정되면 다음 작업 시간으로 (1분 넘으면 빨강)', a == '1:15' and b == '1:16' and after == '0:31' and hot, f'{a} → {b} → {after}, 빨강={hot}')
    page2.close()

    ev(page, "H.safety('amr_03', 2);")
    for _ in range(4):
        run(page, 500)
        ev(page, "H.safety('amr_03', 2);")
    ev(page, "H.safety('amr_03', 0); dashTest.tick();")
    one = page.inner_text('.tile:nth-child(3) .v')
    ev(page, "H.safety('amr_04', 2);")
    run(page, 500)
    ev(page, "H.safety('amr_04', 0);")
    run(page, 500)
    still = page.inner_text('.tile:nth-child(3) .v')
    grey = not page.evaluate("document.querySelector('.tile:nth-child(3)').classList.contains('hot')")
    c.ok('3-2', '안전 상태 정지 2초면 안전 정지 +1, 0.5초면 그대로, 칸은 늘 회색', one.startswith('1') and still.startswith('1') and grey, f'{one} / {still}')

    ev(page, """H.table([H.task('x1', 'stall_1', 't04', { who: 'amr_01', state: 1 }), H.task('collect_x2', 't03', 'return_1', { who: 'amr_02', state: 1, pr: 200 })]);
      H.status('x1', 'amr_01', 2); H.status('x1', 'amr_01', 2); H.status('collect_x2', 'amr_02', 2); H.status('x3', 'amr_03', 2); H.status('chg', 'amr_04', 2); dashTest.tick();""")
    run(page, 300)
    before = flat(page.inner_text('.tile:nth-child(4) .v'))
    ev(page, """H.table([H.task('x1', 'stall_1', 't04', { who: 'amr_01', state: 2 }), H.task('collect_x2', 't03', 'return_1', { who: 'amr_02', state: 2, pr: 200 }),
      H.task('x3', 'stall_2', 't05', { who: 'amr_03', state: 2 }), H.charge('chg', 'amr_04', { state: 2 })]); dashTest.tick();""")
    run(page, 300)
    after = flat(page.inner_text('.tile:nth-child(4) .v'))
    c.ok('3-2', '작업이 끝날 때마다 완료 +1 (같은 작업은 한 번, 서빙·회수를 나눠 셈, 완료 보고가 작업표보다 먼저 와도 셈, 충전은 세지 않음)',
         before.startswith('2') and '서빙 1 · 회수 1' in before and after.startswith('3') and '서빙 2 · 회수 1' in after, f'{before} → {after}')
    page.close()


def test_map(browser, c):
    page = open_page(browser)
    ev(page, "H.robot('amr_03', S.stall_3[0], S.stall_3[1]); dashTest.tick();")
    run(page, 600)
    tf = page.evaluate("document.getElementById('bot-amr_03').style.transform")
    c.ok('3-3', '정차 위치(양식 앞)에 서면 지도에서도 그 자리', tf == 'translate(378px, 107.8px)', tf)

    ev(page, "H.zones([['buffet_line', [-11.9, -6.1, 3.0, 4.63], 4, 1.0]]); dashTest.tick();")
    strong = page.evaluate("[...document.querySelectorAll('#map text')].some(t => t.textContent === '혼잡 4명')")
    hint1 = page.inner_text('#mhint')
    ev(page, "H.zones([['buffet_line', [-11.9, -6.1, 3.0, 4.63], 2, 0.4]]); dashTest.tick();")
    weak = page.evaluate("[...document.querySelectorAll('#map rect')].some(r => r.getAttribute('fill') === 'url(#crowd-weak)')"
                         " && ![...document.querySelectorAll('#map text')].some(t => t.textContent.startsWith('혼잡'))")
    ev(page, "H.zones([['buffet_line', [-11.9, -6.1, 3.0, 4.63], 1, 0.2]]); dashTest.tick();")
    gone = page.evaluate("![...document.querySelectorAll('#map rect')].some(r => /crowd/.test(r.getAttribute('fill') || ''))")
    hint0 = page.inner_text('#mhint')
    c.ok('3-3', '혼잡도 1.0 → "혼잡 4명", 0.4 → 옅은 빗금만, 0.2 → 사라짐. 제목 오른쪽 "혼잡 발생"(빨강) / "혼잡 없음"', strong and weak and gone
         and hint1 == '혼잡 발생' and hint0 == '혼잡 없음', f'{strong} {weak} {gone} / {hint1} / {hint0}')

    page.close()
    page = open_page(browser)
    ev(page, """const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07');
      H.decision('d11', 'amr_03', 'congestion', [['amr_03', 11.4, 3.8, 22.8], ['amr_01', 9.8, 9.2, 37.3]]);
      t.assignee_id = 'amr_03'; H.table([...busy, t]); H.robot('amr_03', 1.2, 4.15, 1, 'd11'); dashTest.tick();""")
    blue = "#bot-amr_03 .pulse[stroke='#6ea8ff']"   # 문제 로봇의 빨간 고리와 구분
    blink = page.evaluate(f"!!document.querySelector('#bot-amr_01 .blink') && !!document.querySelector(\"{blue}\")")
    run(page, 3500)
    blink_off = page.evaluate(f"!document.querySelector('#bot-amr_01 .blink') && !!document.querySelector(\"{blue}\")")
    run(page, 7500)
    pulse_off = page.evaluate(f"!document.querySelector(\"{blue}\")")
    c.ok('3-3', '판단이 오면 후보 고리가 3초 깜빡이고, 고른 로봇 파란 고리가 10초', blink and blink_off and pulse_off, f'{blink} {blink_off} {pulse_off}')

    dashed = page.evaluate("[...document.querySelectorAll('#map polyline')].some(l => l.getAttribute('stroke-dasharray') === '9 7')")   # 판단한 로봇의 굵은 점선
    ev(page, "dashTest.feed('/amr_03/plan', { poses: [[1.2, 4.15], [-3, 4.0], [-9.9, 4.0], [-9.9, 4.15]].map(([x, y]) => ({ pose: H.P(x, y) })) }); dashTest.tick();")
    drawn = page.evaluate("[...document.querySelectorAll('#map polyline')].some(l => l.classList.contains('draw'))")
    c.ok('3-3', '경로 토픽이 없으면 목적지까지 점선, 오면 그려지는 효과로 실선', dashed and drawn, f'{dashed} {drawn}')
    page.close()


def test_decision(browser, c):
    page = open_page(browser)
    ev(page, """const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07');
      H.decision('d11', 'amr_03', 'congestion', [['amr_03', 11.4, 3.8, 22.8], ['amr_01', 9.8, 9.2, 37.3]]);
      t.assignee_id = 'amr_03'; H.table([...busy, t]); H.robot('amr_03', 1.2, 4.15, 1, 'd11'); dashTest.tick();""")
    big, why = page.inner_text('#p-decide .big'), page.inner_text('#p-decide .why')
    th = page.evaluate("[...document.querySelectorAll('#p-decide th')].map(t => t.textContent)")
    line = page.evaluate("[document.querySelector('#p-decide .line .tag').textContent, document.querySelector('#p-decide .line .tl > span:last-child').textContent]")
    c.ok('3-4', '혼잡 반영 배정: "3호 배정", 이름표 서빙 + 경로, 근거 "혼잡 회피", 로봇·거리·혼잡 구간', big == '3호 배정' and why == '혼잡 회피'
         and th == ['로봇', '거리', '혼잡 구간'] and line == ['서빙', '배식대 → 7번 테이블'], f'{big} / {why} / {th} / {line}')
    now_text = page.inner_text('#p-decide h2 small')
    run(page, 12300)   # 다시 그리기(0.25초마다)가 12초를 지난 뒤에 한 번 오게
    later = page.inner_text('#p-decide h2 small')
    c.ok('3-4', '판단 시각은 지난 시간으로: "방금" → "12초 전"', now_text == '방금' and later == '12초 전', f'{now_text} → {later}')
    page.close()

    page = open_page(browser)
    ev(page, """const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07');
      H.decision('d11', 'amr_01', 'euclidean', [['amr_01', 9.8, 0, 9.8], ['amr_03', 11.4, 0, 11.4]]);
      t.assignee_id = 'amr_01'; H.table([...busy, t]); dashTest.tick();""")
    th = page.evaluate("[...document.querySelectorAll('#p-decide th')].map(t => t.textContent)")
    c.ok('3-4', '끔(euclidean): 혼잡 구간 열 없이 거리만, 근거 "최단 거리"', th == ['로봇', '거리'] and page.inner_text('#p-decide .why') == '최단 거리', th)
    page.close()

    page = open_page(browser)
    ev(page, "const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07'); t.assignee_id = 'amr_03'; H.table([...busy, t]); dashTest.tick();")
    no_topic = page.inner_text('#p-decide .big') == '3호 배정' and page.query_selector('#p-decide .why') is None and page.query_selector('#p-decide table') is None
    c.ok('3-4', '판단 토픽이 없어도 배정되면 큰 글씨와 작업 줄', no_topic, page.inner_text('#p-decide'))
    page.close()

    page = open_page(browser)
    ev(page, "const recs = H.allBusy(); H.robot('amr_02', -0.6, 1.0, 0); dashTest.tick(); const t = H.task('c1', 't03', 'return_1', { type: 0, pr: 200 }); t.task.task_id = 'collect_c1'; t.assignee_id = 'amr_02'; H.table([...recs.filter(r => r.assignee_id !== 'amr_02'), t]); dashTest.tick();")
    c.ok('3-4', '대기 로봇이 1대뿐이면 근거 "대기 로봇 1대"', page.inner_text('#p-decide .why') == '대기 로봇 1대', page.inner_text('#p-decide'))
    page.close()

    for query, want, why_want, rank in (('ws=off', '회수 1순위', '회수 우선', '1'), ('ws=off&collect_priority=100', '회수 5순위', '요청 순서', '5')):
        page = open_page(browser, query)
        pr = 100 if 'collect_priority' in query else 200
        ev(page, f"""const busy = H.allBusy(); const ds = [40, 35, 30, 25].map((a, i) => H.task(`d${{i}}`, 'stall_1', ['t09', 't15', 't04', 't02'][i], {{ age: a }}));
          H.table([...busy, ...ds]); dashTest.tick(); const col = H.task('collect_x', 't03', 'return_1', {{ pr: {pr} }}); H.table([...busy, ...ds, col]); dashTest.tick();""")
        got, why = page.inner_text('#p-decide .big'), page.inner_text('#p-decide .why')
        row = page.evaluate("[...document.querySelectorAll('#qbody tr')].find(r => r.dataset.task === 'collect_x')?.querySelector('.rk').textContent")
        c.ok('3-4·3-5', f'로봇이 모두 바쁠 때 회수 호출 ({query}) → "{want} · {why_want}", 작업 순서 {rank}번', got == want and why == why_want and row == rank, f'{got} · {why} / {row}번')
        page.close()

    page = open_page(browser, 'ws=off&aging_rate=1.667')
    ev(page, """const busy = H.allBusy(); const d = H.task('d_c1', 'buffet_1', 'c01', { age: 70 });
      H.table([...busy, d]); dashTest.tick(); const col = H.task('collect_y', 't06', 'return_1', { pr: 200 }); H.table([...busy, d, col]); dashTest.tick();""")
    first = page.evaluate("document.querySelector('#qbody tr').dataset.task")
    fresh = page.evaluate("document.querySelector('#qbody tr').classList.contains('new')")
    c.ok('3-4·3-5', 'aging_rate=1.667: 1분 넘게 기다린 서빙 뒤에 새 회수가 서면 "서빙 1순위", 근거 "장기 대기 · 1:10"', page.inner_text('#p-decide .big') == '서빙 1순위'
         and page.inner_text('#p-decide .why') == '장기 대기 · 1:10' and first == 'd_c1' and fresh, f"{flat(page.inner_text('#p-decide'))} / 1번={first}")
    page.close()


def test_charge(browser, c):
    page = open_page(browser)
    ev(page, """H.robot('amr_01', -6, 0, 0, '', 60); H.robot('amr_02', -3, 0, 0, '', 24); H.robot('amr_03', 0, 0, 0, '', 55);
      H.robot('amr_04', 3, 0, 0, '', 50); H.robot('amr_05', 6.5, -7, 2, '', 40); dashTest.tick();
      H.table([H.charge('charge_2', 'amr_02')]); H.robot('amr_02', -3, 0, 1, 'charge_2', 24); dashTest.tick();""")
    big, why, chip = page.inner_text('#p-decide .big'), page.inner_text('#p-decide .why'), page.inner_text('#rc-amr_02 .st')
    line = page.evaluate("[document.querySelector('#p-decide .line .tag').textContent, document.querySelector('#p-decide .line .tag').classList.contains('chg'), document.querySelector('#p-decide .line .tl > span:last-child').textContent]")
    c.ok('3-4·3-6', '충전 작업 배정 → "2호 배정", 근거 "교대 충전 · 홀 투입 3대", 주황 이름표 충전 + 충전소, 칩 "이동 · 충전소"', big == '2호 배정'
         and why == '교대 충전 · 홀 투입 3대' and chip == '이동' and line == ['충전', True, '충전소'] and page.inner_text('#rc-amr_02 .d') == '충전소',
         f'{big} / {why} / {chip} / {line}')

    # 충전을 마치면 서버가 대기 자리로 보내는 복귀 작업(TYPE_RETURN, #312): 칩으로만 보이고 작업 순서·관제 판단·완료에는 없다
    ev(page, """H.status('charge_2', 'amr_02', 2); const back = H.charge('return_amr_02_0001', 'amr_02', { type: 2 });
      back.task.dropoff_station_id = 'wait_2'; back.task.dropoff_pose = H.P(-10, -6);
      H.table([back]); H.robot('amr_02', 6.5, -7, 1, 'return_amr_02_0001', 100); dashTest.tick();""")
    ret = (page.inner_text('#rc-amr_02 .st'), page.inner_text('#rc-amr_02 .d'), page.query_selector('#row-return_amr_02_0001') is None,
           page.inner_text('#p-decide .big'))
    ev(page, "H.status('return_amr_02_0001', 'amr_02', 2); H.robot('amr_02', -10, -6, 0, '', 100); dashTest.tick();")
    done = page.evaluate("document.querySelectorAll('.tile')[3].querySelector('.v').innerText.replace(/\\s+/g, '')")
    c.ok('3-4·3-6', '충전 뒤 복귀 작업 → 칩 "복귀 · 대기 자리", 작업 순서·관제 판단에 없고 완료에 세지 않는다',
         ret == ('복귀', '대기 자리', True, '2호 배정') and done == '0건', f'{ret} / 완료 {done}')
    page.close()


def test_queue(browser, c):
    page = open_page(browser)
    empty = (page.inner_text('#qhint'), page.inner_text('#qbody'))
    c.ok('3-5', '비었을 때 제목 오른쪽 "대기 0건", 본문은 비움', empty == ('대기 0건', ''), empty)
    ev(page, "const busy = H.allBusy(); H.table([...busy, H.task('q1', 'stall_1', 't04', { age: 10 })]); dashTest.tick();")
    tag = page.evaluate("document.querySelector('#row-q1 .tag')?.textContent")
    ev(page, "const busy = H.allBusy(); H.table([...busy, H.task('q1', 'stall_1', 't04', { age: 10, who: 'amr_02' })]); dashTest.tick();")
    bot = page.evaluate("document.querySelector('#row-q1 .bot')?.textContent")
    run(page, 6000)
    gone = page.query_selector('#row-q1') is None
    c.ok('3-5', '줄에 이름표 "서빙", 배정되면 "2호 배정"이 붙고 5초 뒤 빠진다', tag == '서빙' and bot == '2호 배정' and gone, f'{tag} / {bot} / 빠짐={gone}')

    ev(page, "const busy = H.allBusy(); H.table([...busy, H.task('half', 'stall_2', 't05', { age: 30 }), H.task('old', 'stall_1', 't04', { age: 61 })]); dashTest.tick();")
    run(page, 300)
    page.wait_for_timeout(500)
    late = page.evaluate("document.getElementById('row-old').classList.contains('late')")
    widths = page.evaluate("['row-half', 'row-old'].map(id => document.querySelector(`#${id} .wait .b i`).style.width)")
    c.ok('3-5', '막대는 1분에 꽉 차고(30초면 반), 1분을 넘긴 작업은 빨강', late and widths[1] == '100%' and widths[0].startswith('50'), f'빨강={late} 막대={widths}')

    ev(page, "const busy = H.allBusy(); H.table([...busy, ...[...Array(9)].map((_, i) => H.task(`m${i}`, 'stall_1', 't0' + (i % 9 + 1), { age: 50 - i }))]); dashTest.tick();")
    more = page.evaluate("document.querySelector('#qbody tr.more')?.innerText.trim()")
    rows = page.evaluate("document.querySelectorAll('#qbody tr[data-task]').length")
    fits = page.evaluate("document.querySelector('#qbody table').getBoundingClientRect().bottom <= document.getElementById('p-queue').getBoundingClientRect().bottom")
    hint = page.inner_text('#qhint')
    c.ok('3-5', '칸에 다 안 들어가면 들어가는 만큼 + "외 n건"(합 9건, 칸 밖으로 넘치지 않음), 제목 오른쪽 "대기 9건"',
         more == f'외 {9 - rows}건' and rows >= 4 and fits and hint == '대기 9건', f'{rows}줄 + {more} / 넘침 없음={fits} / {hint}')
    page.close()


def test_robots(browser, c):
    page = open_page(browser)
    ev(page, "const t = H.task('r1', 'stall_1', 't07', { who: 'amr_01', state: 1 }); H.table([t]); H.robot('amr_01', 0, 0, 1, 'r1'); dashTest.tick();")
    chips = []
    for phase in ('move_to_pickup', 'loading', 'move_to_dropoff', 'unloading'):
        ev(page, f"H.status('r1', 'amr_01', 1, '{phase}'); dashTest.tick();")
        chips.append(page.inner_text('#rc-amr_01 .st'))
    ev(page, "H.status('r1', 'amr_01', 2); H.robot('amr_01', 0, 0, 0, ''); dashTest.tick();")
    chips.append(page.inner_text('#rc-amr_01 .st'))
    c.ok('3-6', '단계마다 칩이 바뀐다 (이동 → 픽업 → 운반 → 전달 → 대기)', chips == ['이동', '픽업', '운반', '전달', '대기'], chips)

    for _ in range(4):
        ev(page, "H.safety('amr_01', 2); H.robot('amr_01', 0, 0, 0, '');")
        run(page, 500)
    stop = (page.inner_text('#rc-amr_01 .st'), page.inner_text('#rc-amr_01 .d'))
    ev(page, "H.safety('amr_01', 0); dashTest.tick();")
    back = page.inner_text('#rc-amr_01 .st')
    c.ok('3-6', '안전 상태 정지 2초 → "안전 정지 · 장애물 감지", 풀리면 원래대로', stop == ('안전 정지', '장애물 감지') and back == '대기', f'{stop} → {back}')

    ev(page, "H.robot('amr_02', 1, 1, 0, '', 19); H.robot('amr_03', 6.5, -7, 2, '', 40); dashTest.tick();")
    low = (page.inner_text('#rc-amr_02 .bat .num'), page.evaluate("document.querySelector('#rc-amr_02 .bat .num').classList.contains('hot')"),
           page.evaluate("document.querySelector('#rc-amr_02 .cells').classList.contains('hot')"))
    cells = page.evaluate("document.querySelectorAll('#rc-amr_02 .cells span').length")
    chg = page.evaluate("document.querySelector('#rc-amr_03 .cells').classList.contains('chg')")
    c.ok('3-6', '배터리 5칸: 19%면 숫자와 칸이 빨강("부족" 없음), 충전 중이면 주황', low == ('19%', True, True) and cells == 5 and chg, f'{low} 칸={cells} 주황={chg}')

    for _ in range(12):
        ev(page, "H.robot('amr_02', 1, 1, 0, '', 19); H.robot('amr_03', 6.5, -7, 2, '', 40);")
        run(page, 500)
    lost = page.inner_text('#rc-amr_01 .st')
    rhint = page.inner_text('#rhint')
    c.ok('3-6', '상태가 5초 넘게 안 오면 "연결 끊김", 로봇 제목 오른쪽 "도움 필요 1대"', lost == '연결 끊김' and rhint == '도움 필요 1대', f'{lost} / {rhint}')
    page.close()


def test_call(browser, c):
    page = open_page(browser)
    ev(page, "H.allBusy(); dashTest.tick();")
    hint = page.inner_text('#p-call h2 small')
    off = page.is_disabled('#collect') and page.is_disabled('#serve')
    w = page.evaluate("['collect', 'serve'].map(id => Math.round(document.getElementById(id).getBoundingClientRect().width))")
    shown = page.evaluate("['collect', 'serve'].every(id => document.getElementById(id).offsetHeight > 30)")
    c.ok('3-7', '고르기 전: "지도에서 테이블을 고르세요", 회수·서빙 버튼은 늘 보이고 같은 크기, 누를 수 없음', hint == '지도에서 테이블을 고르세요' and off and shown and w[0] == w[1],
         f'{hint} / 못 누름={off} / 보임={shown} / 폭={w}')
    page.click('[data-table="t03"]')
    sky = page.evaluate("[getComputedStyle(document.getElementById('collect')).backgroundColor, document.querySelector('[data-table=\"t03\"] rect').getAttribute('stroke')]")
    c.ok('3-7', '테이블을 고르면 테이블 테두리와 호출 버튼이 하늘색(직원이 누르는 것)', sky == ['rgb(125, 211, 252)', '#7dd3fc'], sky)
    page.click('#collect')
    sent = page.evaluate('dashTest.sent')
    toast = page.inner_text('#p-call .toast')
    m = sent[0]['msg'] if sent else {}
    c.ok('3-7', '테이블 고르고 "빈 그릇 회수" → 1건 (task_type 3, priority 200, collect_, 3번 → 퇴식구)', len(sent) == 1 and m['task_type'] == 3 and m['priority'] == 200
         and m['task_id'].startswith('collect_') and m['pickup_station_id'] == 't03' and m['dropoff_station_id'] == 'return_1', m.get('task_id'))
    c.ok('5-3', '보낸 뒤 "회수 요청 완료 · 3번 테이블"', toast == '회수 요청 완료 · 3번 테이블', toast)
    ev(page, f"const busy = H.allBusy(); const t = H.task('{m['task_id']}', 't03', 'return_1', {{ pr: 200 }}); H.table([...busy, t]); dashTest.tick();")
    in_table = page.query_selector(f"#row-{m['task_id']}") is not None
    btn = (page.inner_text('#collect'), page.is_disabled('#collect'))
    c.ok('3-7', '작업 순서에 나오고, 끝나기 전에는 "회수 요청 완료"(누를 수 없음)', in_table and btn == ('회수 요청 완료', True), f'{in_table} {btn}')
    page.click('#serve')
    stores = page.evaluate("[...document.querySelectorAll('[data-store]')].map(b => b.textContent)")
    on = page.evaluate("document.getElementById('serve').classList.contains('on')")
    page.click('[data-store="stall_1"]')
    m2 = page.evaluate('dashTest.sent[1].msg')
    closed = page.query_selector('[data-store]') is None
    c.ok('3-7', '"음식 서빙"을 누르면 매장 5개가 펼쳐지고, 고르면 서빙 1건(task_type 0, priority 100, 한식 → 3번)을 보내고 닫힌다',
         stores == ['배식대', '한식', '분식', '양식', '카페'] and on and m2['task_type'] == 0 and m2['priority'] == 100 and m2['pickup_station_id'] == 'stall_1'
         and m2['dropoff_station_id'] == 't03' and closed, f'{stores} 파랑={on} 닫힘={closed}')
    c.ok('5-3', '서빙 보낸 뒤 "서빙 요청 완료 · 한식 → 3번 테이블"', page.inner_text('#p-call .toast') == '서빙 요청 완료 · 한식 → 3번 테이블', page.inner_text('#p-call .toast'))
    page.close()

    page = open_page(browser, 'ws=off&collect_priority=100')
    page.click('[data-table="t03"]'); page.click('#collect')
    c.ok('3-7', '?collect_priority=100이면 회수 priority 100', page.evaluate('dashTest.sent[0].msg.priority') == 100, page.evaluate('dashTest.sent[0].msg.priority'))
    page.evaluate('dashTest.setConnected(false)')
    page.click('[data-table="t05"]'); page.click('#collect')
    c.ok('5-3', '연결이 끊겼으면 "요청 실패 · 연결되면 다시 누르세요"', page.inner_text('#p-call .toast') == '요청 실패 · 연결되면 다시 누르세요', page.inner_text('#p-call .toast'))
    page.close()


def test_popups(browser, c):
    page = open_page(browser)
    ev(page, "const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07'); t.assignee_id = 'amr_03'; H.table([...busy, t]); H.robot('amr_03', 1.2, 4.15, 1, 'd11'); dashTest.tick();")
    shown = lambda: page.evaluate("!document.getElementById('pop').hidden")
    page.click('#rc-amr_01'); a = shown() and page.inner_text('#pop h3').startswith('1호')
    page.click('#rc-amr_02'); b = page.inner_text('#pop h3').startswith('2호')
    page.keyboard.press('Escape'); esc_closed = not shown()
    page.click('#bot-amr_03', force=True); bot = shown()  # 퍼지는 고리 애니메이션 때문에 '멈춘 요소'를 기다리지 않는다
    page.click('#pop [data-close]'); x_closed = not shown()
    page.click('#row-d11'); row = shown() and '배식대 → 7번 테이블' in page.inner_text('#pop h3')
    page.click('header'); out_closed = not shown()
    page.click('#p-decide'); dec = shown() and '배식대 → 7번 테이블' in page.inner_text('#pop h3')
    page.click('header')
    page.click('[data-tile="hall"]'); bat = shown() and page.inner_text('#pop h3').startswith('배터리')
    c.ok('3-8', '로봇 원·카드·작업 순서 줄·관제 판단·홀 투입 칸을 누르면 창, 다른 것을 누르면 바뀌고 ✕·Esc·바깥으로 닫힌다',
         all([a, b, esc_closed, bot, x_closed, row, out_closed, dec, bat]), [a, b, esc_closed, bot, x_closed, row, out_closed, dec, bat])
    page.close()

    page = open_page(browser)
    ev(page, "const t = H.task('s1', 'buffet_2', 't11', { who: 'amr_01', state: 1 }); H.table([t]); H.robot('amr_01', -10.5, 0.6, 1, 's1'); H.status('s1', 'amr_01', 1, 'move_to_pickup'); dashTest.tick();")
    page.click('#rc-amr_01')
    early = None
    for i in range(80):
        ev(page, "H.safety('amr_01', 2); H.robot('amr_01', -10.5, 0.6, 1, 's1');")
        run(page, 500)
        if i == 40:
            early = page.inner_text('#pop .alert')
    late = page.inner_text('#pop .alert')
    kv = flat(page.inner_text('#pop .kv'))
    c.ok('3-8', '안전 정지 40초: 상자는 "장애물 감지 · 0:3n"(칩 이름 반복 없음), 30초 전에는 할 일 없음, 지나면 "로봇 앞 통로를 확보하세요". 작업 줄에 이름표 + 경로',
         early and '확보하세요' not in early and late.startswith('장애물 감지 · 0:3') and late.endswith('로봇 앞 통로를 확보하세요')
         and kv.startswith('작업 서빙 배식대 → 11번 테이블'), f'{early!r} → {late!r} / {kv}')
    page.close()

    page = open_page(browser)
    ev(page, "const t = H.task('p1', 'stall_1', 't07'); H.table([t]); H.robot('amr_02', 0, 0, 0); dashTest.tick();")
    page.click('#row-p1')
    wait_step = page.evaluate("[...document.querySelectorAll('#pop li.now')].map(l => l.innerText.replace(/\\s+/g, ' '))")
    ev(page, "const t = H.task('p1', 'stall_1', 't07', { who: 'amr_02', state: 1 }); H.table([t]); H.robot('amr_02', 0, 0, 1, 'p1'); dashTest.tick();")
    page.click('#p-decide')
    steps = []
    for phase in ('move_to_pickup', 'loading', 'move_to_dropoff'):
        ev(page, f"H.status('p1', 'amr_02', 1, '{phase}'); dashTest.tick();")
        run(page, 2000)
        steps.append(page.evaluate("[...document.querySelectorAll('#pop li.now')].map(l => l.innerText.split('\\n')[0])[0]"))
    done_times = page.evaluate("[...document.querySelectorAll('#pop li.done span.num')].filter(s => s.textContent).length")
    c.ok('3-8', '작업 창: 배정 전 "배정 대기", 단계가 차례로 바뀌고 지난 단계에 시각', wait_step == ['배정 대기'] and steps == ['이동', '픽업', '운반']
         and done_times == 4, f'{wait_step} / {steps} / 시각 {done_times}개')
    page.close()

    # 도움이 필요한 상태마다 "이유 · 지난 시간"과 할 일 (비상 정지·오류·연결 끊김은 바로, 센서 이상은 안전 정지라 30초 뒤)
    page = open_page(browser)
    ev(page, "H.robot('amr_01', 0, 0, 4, '', 70); H.robot('amr_02', 1, 0, 3, '', 70); H.robot('amr_03', 2, 0, 0, '', 70); dashTest.tick();")
    boxes = {}
    for _ in range(64):
        ev(page, "H.robot('amr_01', 0, 0, 4, '', 70); H.robot('amr_02', 1, 0, 3, '', 70); H.safety('amr_03', 2, 4); H.robot('amr_03', 2, 0, 0, '', 70);")
        run(page, 500)
    for rid in ('amr_01', 'amr_02', 'amr_03', 'amr_04'):
        if rid == 'amr_04':
            ev(page, "H.robot('amr_04', 3, 0, 0, '', 70); dashTest.tick();")
            for _ in range(14):
                ev(page, "H.robot('amr_01', 0, 0, 4, '', 70); H.robot('amr_02', 1, 0, 3, '', 70); H.safety('amr_03', 2, 4); H.robot('amr_03', 2, 0, 0, '', 70);")
                run(page, 500)
        page.click(f'#rc-{rid}')
        boxes[rid] = page.inner_text('#pop .alert').replace('\n', ' / ')
        page.keyboard.press('Escape')
    place = page.inner_text('#rc-amr_01 .d')
    want = {'amr_01': ('비상 버튼 · 0:3', '현장에서 해제하세요'), 'amr_02': ('오류 · 0:3', '현장에서 로봇을 점검하세요'),
            'amr_03': ('센서 이상 · 0:3', '로봇 센서를 점검하세요'), 'amr_04': ('연결 끊김 · 0:', '로봇 전원과 네트워크를 점검하세요')}
    good = all(boxes[k].startswith(a) and boxes[k].endswith(b) for k, (a, b) in want.items())
    c.ok('3-8', '도움 필요 상태마다 "이유 · m:ss"와 할 일: 비상 버튼(현장에서 해제), 오류(현장 점검), 센서 이상(센서 점검), 연결 끊김(전원·네트워크 점검)',
         good and place == '비상 버튼', f'{boxes} / 카드 {place}')
    page.close()

    page = open_page(browser)
    ev(page, "H.robot('amr_04', 0, 0, 0, '', 45); H.robot('amr_05', 6.5, -7, 2, '', 51); dashTest.tick();")
    run(page, 60000)
    ev(page, "H.robot('amr_04', 0, 0, 0, '', 45); H.robot('amr_05', 6.5, -7, 2, '', 51); dashTest.tick();")
    page.click('#rc-amr_04'); drain = flat(page.inner_text('#pop .kv')); page.keyboard.press('Escape')
    page.click('#rc-amr_05'); charge = flat(page.inner_text('#pop .kv'))
    c.ok('3-8', '로봇 창 배터리는 칸과 %만 (남은 시간 추정 없음). 대기·충전 중이면 작업 줄 없음',
         drain == '배터리 45%' and charge == '배터리 51%', f'{drain!r} / {charge!r}')
    page.close()


def test_battery(browser, c):
    page = open_page(browser, 'ws=off&min_hall=3')
    ev(page, """H.bat('amr_01', -6, 0, 1, '', 24, .6); H.bat('amr_02', 6.5, -7, 2, '', 46, -2); H.bat('amr_03', 0, 0, 0, '', 25, .6);
      H.bat('amr_04', 3, 0, 1, 'chg4', 18, .6); H.bat('amr_05', 5, -7, 2, '', 38, -2); H.table([H.charge('chg4', 'amr_04', { age: 30 })]); dashTest.tick();""")
    run(page, 300)
    page.click('[data-tile="hall"]')
    run(page, 300)
    title = page.evaluate("document.querySelector('#pop h3').childNodes[0].textContent")
    cols = page.evaluate("[...document.querySelectorAll('#pop .vbar')].map(b => [b.innerText.replace(/\\s+/g, ' ').trim(), "
                         "b.querySelector('.vcells').className.replace('vcells', '').trim(), b.querySelectorAll('.vcells span').length])")
    want = [['1호 24%', '', 5], ['2호 46%', 'chg', 5], ['3호 25%', '', 5], ['4호 18%', 'hot', 5], ['5호 38%', 'chg', 5]]
    c.ok('3-8', '배터리 창: 제목 "배터리"만, 5대 세로 5칸 + 이름 + %만 (남은 시간 없음), 20% 아래 빨강, 충전 중 주황',
         title == '배터리' and cols == want, f'{title} / {cols}')
    page.close()


# 지도 위 글자·표시가 서로 겹치거나 경로 선이 글자를 지나가는지 (화면 좌표로 잰다. 경로가 비워 둔 글자 자리는 빼고)
OVERLAP_JS = r"""(() => {
  const svg = document.getElementById('map'), R = e => e.getBoundingClientRect(), items = [];
  svg.querySelectorAll('g.bot .body').forEach(b => items.push({ n: b.closest('g').id, k: 'bot', r: R(b) }));
  svg.querySelectorAll('rect[fill="#ef4444"]').forEach(r => items.push({ n: r.parentNode.textContent, k: 'chip', r: R(r) }));
  svg.querySelectorAll('text.tnum').forEach(t => items.push({ n: `테이블 ${t.textContent}`, k: 'num', r: R(t) }));
  svg.querySelectorAll('text:not(.tnum)').forEach(t => { if (!t.closest('g.bot') && !t.parentNode.querySelector('rect[fill="#ef4444"]')) items.push({ n: t.textContent, k: 'label', r: R(t) }); });
  const bad = { bot: ['num', 'label', 'chip', 'bot'], chip: ['num', 'label', 'chip'], label: ['num', 'label'] };
  const area = (a, b) => Math.max(0, Math.min(a.right, b.right) - Math.max(a.left, b.left) - 1) * Math.max(0, Math.min(a.bottom, b.bottom) - Math.max(a.top, b.top) - 1);
  const out = [];
  items.forEach((a, i) => items.slice(i + 1).forEach(b => { if (((bad[a.k] || []).includes(b.k) || (bad[b.k] || []).includes(a.k)) && area(a.r, b.r) > 6) out.push(`${a.n} × ${b.n}`); }));
  const sc = svg.getScreenCTM(), holes = [...svg.querySelectorAll('mask rect.hole')].map(h => [+h.getAttribute('x'), +h.getAttribute('y'), +h.getAttribute('width'), +h.getAttribute('height')])
    .map(([x, y, w, h]) => ({ left: x * sc.a + sc.e, top: y * sc.d + sc.f, right: (x + w) * sc.a + sc.e, bottom: (y + h) * sc.d + sc.f }));
  const inside = (x, y, b) => x > b.left + 1 && x < b.right - 1 && y > b.top + 1 && y < b.bottom - 1;
  svg.querySelectorAll('polyline').forEach(pl => {
    const ctm = pl.getScreenCTM(), pts = pl.getAttribute('points').trim().split(/\s+/).map(s => s.split(',').map(Number)).map(([x, y]) => [x * ctm.a + ctm.e, y * ctm.d + ctm.f]);
    for (let k = 0; k + 1 < pts.length; k++) { const [x0, y0] = pts[k], [x1, y1] = pts[k + 1], n = Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 2);
      for (let s = 0; s <= n; s++) { const x = x0 + (x1 - x0) * s / n, y = y0 + (y1 - y0) * s / n;
        if (Math.hypot(x - pts[0][0], y - pts[0][1]) < 18 || holes.some(b => inside(x, y, b))) continue;
        items.forEach(it => { if (it.k !== 'bot' && inside(x, y, it.r) && !out.includes(`경로 × ${it.n}`)) out.push(`경로 × ${it.n}`); }); } }
  });
  return out;
})()"""


def test_overlap(browser, c):
    scenes = {'1-1 켬': ('amr_03', 'congestion', "[['amr_03', 11.4, 3.8, 22.8], ['amr_01', 9.8, 9.2, 37.3]]", [1.2, 4.15]),
              '1-1 끔': ('amr_01', 'euclidean', "[['amr_01', 9.8, 0, 9.8], ['amr_03', 11.4, 0, 11.4]]", [-10.5, -5.3])}
    found = {}
    for name, (r, fn, cands, xy) in scenes.items():
        page = open_page(browser)
        ev(page, f"""const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07'); H.decision('d11', '{r}', '{fn}', {cands});
          t.assignee_id = '{r}'; H.table([...busy, t]); H.robot('{r}', {xy[0]}, {xy[1]}, 1, 'd11'); H.status('d11', '{r}', 1, 'move_to_pickup'); dashTest.tick();""")
        run(page, 600)
        page.wait_for_timeout(700)   # 로봇이 미끄러지는 효과(실제 시간 0.5초)가 끝난 자리에서 잰다
        found[name] = page.evaluate(OVERLAP_JS)
        page.close()
    c.ok('3-3', '지도 글자·표시 겹침 0건 (1-1 켬·끔: 로봇 번호, 테이블 번호, 장소 이름, 혼잡 이름표, 경로)', not any(found.values()), found)


def test_style(browser, c):
    page = open_page(browser, 'ws=off&aging_rate=1.667&min_hall=3')
    texts = []
    ev(page, "const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07'); H.decision('d11', 'amr_03', 'congestion', [['amr_03', 11.4, 3.8, 22.8], ['amr_01', 9.8, 9.2, 37.3]]); t.assignee_id = 'amr_03'; H.table([...busy, t]); H.robot('amr_03', 1.2, 4.15, 1, 'd11'); dashTest.tick();")
    texts.append(page.inner_text('body'))
    page.click('#rc-amr_03'); texts.append(page.inner_text('body'))
    page.click('#row-d11'); texts.append(page.inner_text('body'))
    ev(page, "H.robot('amr_02', 1, 1, 0, '', 19); H.safety('amr_05', 2); dashTest.tick();")
    run(page, 1500)
    page.click('[data-tile="hall"]'); texts.append(page.inner_text('body'))
    page.click('header')
    page.click('[data-table="t03"]'); page.click('#collect'); texts.append(page.inner_text('body'))
    page.click('#serve'); texts.append(page.inner_text('body'))
    problems = sorted({p for t in texts for p in style_problems(t)})
    c.ok('5', '화면 문구에 쓰지 않는 말·마침표·느낌표·해요체가 없다 ("-하세요"만 허용)', not problems, problems)
    page.close()


def shots(browser, out):
    """시연 장면 캡처 (설계안 v6 그림 13장): 평소, 1-1 켬/끔, 2-12 끔/켬, 3-02·3-04 배터리 창 끔/켬, 호출 서빙 펼침, 마무리-02, 로봇 창, 작업 창."""
    out.mkdir(parents=True, exist_ok=True)

    def page_at(when, query='ws=off&min_hall=3', hall=None):
        page = open_page(browser, query, start=when)
        if hall:
            ev(page, f'H.hall({json.dumps(hall)});')
        return page

    def keep_alive(page, seconds):
        for _ in range(seconds):
            ev(page, 'H.alive();')
            run(page, 1000)

    def snap(page, name):
        page.wait_for_timeout(700)   # 줄이 미끄러지는 효과(실제 시간 0.4초)가 끝나게
        page.screenshot(path=str(out / f'{name}.png'))
        page.close()

    normal = """H.zones([['buffet_line', [-11.9, -6.1, 3.0, 4.63], 4, 1.0], ['east_aisle', [4.8, 7.6, -2.0, 2.0], 2, 0.4]]);
      const recs = [H.task('n2', 'stall_1', 't09', { age: 50, who: 'amr_02', state: 1 }), H.task('collect_n3', 't13', 'return_1', { age: 30, who: 'amr_03', state: 1, pr: 200 }),
        H.task('n4', 'stall_2', 't04', { age: 20, who: 'amr_04', state: 1 }), H.task('n6', 'stall_4', 'L01', { age: 25 }), H.task('collect_n7', 't05', 'return_2', { age: 12, pr: 200 })];
      H.bat('amr_01', -10.5, -5.6, 0, '', 72, .4); H.bat('amr_02', 0.2, 0.6, 1, 'n2', 64, .5); H.bat('amr_03', -2.7, -2.0, 1, 'collect_n3', 58, .5);
      H.bat('amr_04', -1.5, 3.9, 1, 'n4', 47, .5); H.bat('amr_05', 6.5, -7.3, 2, '', 45, -2);
      H.status('n2', 'amr_02', 1, 'move_to_dropoff'); H.status('collect_n3', 'amr_03', 1, 'move_to_pickup'); H.status('n4', 'amr_04', 1, 'loading');
      H.table(recs); dashTest.tick();"""
    page = page_at(datetime(2026, 11, 8, 12, 20, 0), hall=[[179, 4], [100, 5], [45, 4]])
    ev(page, normal)
    keep_alive(page, 16)
    snap(page, '01_평소')

    s11 = """const busy = H.scene11(); const t = H.task('d11', 'buffet_1', 't07');
      H.decision('d11', '{r}', '{fn}', {c}); t.assignee_id = '{r}'; H.table([...busy, t]);
      H.robot('{r}', ...({{amr_03: [1.2, 4.15], amr_01: [-10.5, -5.3]}})['{r}'], 1, 'd11'); H.status('d11', '{r}', 1, 'move_to_pickup'); dashTest.tick();"""
    for name, r, fn, cands in (('02_1-1_켬', 'amr_03', 'congestion', "[['amr_03', 11.4, 3.8, 22.8], ['amr_01', 9.8, 9.2, 37.3]]"),
                               ('03_1-1_끔', 'amr_01', 'euclidean', "[['amr_01', 9.8, 0, 9.8], ['amr_03', 11.4, 0, 11.4]]")):
        page = page_at(datetime(2026, 11, 8, 12, 7, 10), hall=[[179, 5]])
        ev(page, s11.format(r=r, fn=fn, c=cands))
        run(page, 600)
        snap(page, name)

    s212 = """const busy = H.allBusy(); const d = H.task('dc1', 'buffet_1', 'c01', { age: 75 });
      const cols = [8, 6, 5, 3, 2].map((a, i) => H.task(`collect_${i}`, ['t03', 't01', 't06', 't08', 't12'][i], 'return_1', { age: a, pr: 200 }));
      H.table([...busy, d]); dashTest.tick(); H.table([...busy, d, ...cols]); dashTest.tick();"""
    for name, query in (('04_2-12_끔', 'ws=off&min_hall=3'), ('05_2-12_켬', 'ws=off&min_hall=3&aging_rate=1.667')):
        page = page_at(datetime(2026, 11, 8, 12, 44, 30), query, hall=[[179, 5]])
        ev(page, s212)
        run(page, 1200)
        snap(page, name)

    # 3-02·3-04 (끔은 다 같이 20%대, 켬은 2·5호를 미리 충전 보냄). [x, y, mode, 작업, 배터리, 분당 줄어드는 %]
    battery = {
        '06_3-02_끔': ([[179, 5]], [[-8.8, -3.05, 1, 'w0', 24, .6], [-0.6, 1.0, 1, 'w1', 23, .6], [3.4, -2.6, 1, 'w2', 25, .6], [6.9, 1.2, 1, 'w3', 22, .6], [-4.5, 3.6, 1, 'w4', 26, .6]]),
        '07_3-02_켬': ([[179, 4], [120, 3]], [[-8.8, -3.05, 1, 'w0', 24, .6], [6.5, -7.3, 2, '', 46, -2], [3.4, -2.6, 1, 'w2', 25, .6], [6.9, 1.2, 1, 'w3', 22, .6], [5.0, -7.3, 2, '', 38, -2]]),
        '08_3-04_끔': ([[179, 5], [8, 4]], [[-8.8, -3.05, 1, 'w0', 21, .6], [-0.6, 1.0, 1, 'w1', 20.5, .6], [3.4, -2.6, 1, 'w2', 22, .6], [7.4, -5.0, 1, 'chg4', 18, .6], [-4.5, 3.6, 1, 'w4', 23, .6]]),
        '09_3-04_켬': ([[179, 4], [120, 3]], [[-8.8, -3.05, 1, 'w0', 21, .6], [6.5, -7.3, 2, '', 52, -2], [3.4, -2.6, 1, 'w2', 22, .6], [6.9, 1.2, 1, 'w3', 20.5, .6], [5.0, -7.3, 2, '', 44, -2]]),
    }
    for name, (hall, rows) in battery.items():
        page = page_at(datetime(2026, 11, 8, 13, 22, 40), hall=hall)
        js = "const recs = H.allBusy();"
        if name == '08_3-04_끔':   # 끔: 4호가 스스로 충전하러 간다 (관제 판단 없음)
            js += "recs.push(H.charge('chg4', 'amr_04', { age: 30 }));"
        js += "H.table(recs);"
        js += ''.join(f"H.bat('amr_0{i + 1}', {x}, {y}, {mode}, '{task}', {b}, {rate});" for i, (x, y, mode, task, b, rate) in enumerate(rows))
        ev(page, js + 'dashTest.tick();')
        keep_alive(page, 12)
        page.click('[data-tile="hall"]')
        run(page, 300)
        snap(page, f'{name}_배터리창')

    page = page_at(datetime(2026, 11, 8, 12, 41, 50), hall=[[179, 4], [60, 5]])
    ev(page, normal)
    keep_alive(page, 4)
    page.click('[data-table="t03"]'); run(page, 200)
    page.click('#serve'); run(page, 200)
    snap(page, '10_호출_서빙펼침')

    page = page_at(datetime(2026, 11, 8, 14, 30, 0), hall=[[179, 3]])
    ev(page, """H.bat('amr_01', 11.2, -4.6, 0, '', 31, .3); H.bat('amr_02', 6.5, -7.3, 2, '', 58, -2); H.bat('amr_03', 11.2, -5.8, 0, '', 44, .3);
      H.bat('amr_04', 11.2, -7.0, 0, '', 37, .3); H.bat('amr_05', 5.0, -7.3, 2, '', 66, -2);
      H.done(20, '서빙'); H.done(11, '회수'); stopTotal = 3; dashTest.setCountFrom(now() - 3 * 3600); dashTest.tick();""")
    keep_alive(page, 12)
    snap(page, '11_마무리-02')

    page = page_at(datetime(2026, 11, 8, 12, 12, 30), hall=[[179, 5]])
    ev(page, """const t = H.task('s1', 'buffet_2', 't11', { who: 'amr_01', state: 1 }); H.table([...H.scene11().slice(1), t]);
      H.bat('amr_01', -10.5, 0.6, 1, 's1', 64, .5); H.status('s1', 'amr_01', 1, 'move_to_pickup'); dashTest.tick();""")
    for _ in range(80):
        ev(page, "H.alive(); H.safety('amr_01', 2); H.robot('amr_01', -10.5, 0.6, 1, 's1', 64);")
        run(page, 500)
    page.click('#rc-amr_01'); run(page, 300)
    snap(page, '12_로봇창_안전정지')

    page = page_at(datetime(2026, 11, 8, 12, 42, 3), hall=[[179, 5]])
    ev(page, """const busy = H.allBusy(); busy[1].assignee_id = ''; busy[1].state = 2; H.robot('amr_02', -1.3, 2.5, 1, 'collect_x');
      const ds = [74, 66, 57, 48].map((a, i) => H.task(`d${i}`, ['stall_1', 'stall_2', 'stall_3', 'stall_4'][i], ['t09', 't15', 't04', 't02'][i], { age: a }));
      const col = H.task('collect_x', 't03', 'return_1', { pr: 200, age: 28, who: 'amr_02', state: 1 }); H.table([...busy.filter((_, i) => i !== 1), ...ds, col]);
      tasks['collect_x'] = { firstSeen: now() - 28, tableSeen: true, steps: { '요청': now() - 28, '배정': now() - 27, '이동': now() - 27 }, phase: 'move_to_pickup', assignee: 'amr_02' };
      H.status('collect_x', 'amr_02', 1, 'move_to_pickup'); dashTest.tick();""")
    run(page, 600)
    page.evaluate("() => openPop('task', 'collect_x', 'decide')")
    run(page, 300)
    snap(page, '13_작업창')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shots', type=Path, help='시연 장면 캡처 폴더')
    ap.add_argument('--channel', default=None, help='브라우저 채널 (예: chrome)')
    ap.add_argument('--out', type=Path, default=HERE / 'out')
    args = ap.parse_args()
    c = Check()
    with sync_playwright() as p:
        browser = p.chromium.launch(channel=args.channel, headless=True)
        for test in (test_header, test_summary, test_map, test_decision, test_charge, test_queue, test_robots, test_call, test_popups, test_battery, test_overlap, test_style):
            try:
                test(browser, c)
            except Exception as e:  # 시험 하나가 깨져도 나머지는 돈다
                c.ok('-', f'{test.__name__} 실행 오류', False, repr(e))
        if args.shots:
            shots(browser, args.shots)
        browser.close()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'ui_result.json').write_text(json.dumps(c.rows, ensure_ascii=False, indent=1), encoding='utf-8')
    for row in c.rows:
        print(f"{'통과' if row['pass'] else '실패'}  [{row['ref']}] {row['name']}" + ('' if row['pass'] else f"  ← {row['detail']}"))
    failed = sum(not r['pass'] for r in c.rows)
    print(f'\n{len(c.rows) - failed}/{len(c.rows)} 통과')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
