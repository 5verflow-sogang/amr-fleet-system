"""관제 대시보드를 실제 스택에 붙여 시험한다 (명세 8-2, 11/7 완료 조건 "호출 버튼을 누르면 배정이 화면에 보인다").

먼저 Fleet 서버, 식당 위치 모의 로봇 5대, dashboard 서비스를 띄운다.

    docker compose up -d fleet
    FAKE_ROBOTS_CONFIG=fake_robots_restaurant.yaml docker compose --profile fake up -d fake-robots
    docker compose --profile dashboard up -d
    python3 dashboard/test/check_stack.py --url http://localhost:8080 [--restart "docker compose --profile dashboard restart dashboard"]

모의 로봇(fake_robot_node)은 시뮬레이터 없이 위치만 계산하는 가짜 로봇이라 배터리(100% 고정)와 멈춤(safety/status 없음)은
여기서 보지 않는다. 그건 check_ui.py가 시험용 메시지로 확인한다.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

HERE = Path(__file__).resolve().parent
# fake_robots_restaurant.yaml 시작 위치 → 지도 px (K=28, X=(x+12)K, Y=(8-y)K)
STARTS = {'amr_01': (-10.5, -5.6), 'amr_02': (11.2, -7.0), 'amr_03': (1.5, 4.15), 'amr_04': (11.2, -5.8), 'amr_05': (11.2, -4.6)}


def rows(page):
    return page.evaluate("[...document.querySelectorAll('#qbody tr[data-task]')].map(r => r.innerText.replace(/\\s+/g, ' ').trim())")


def wait_until(page, js, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if page.evaluate(js):
            return time.time() - t0
        page.wait_for_timeout(100)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://localhost:8080')
    ap.add_argument('--channel', default=None)
    ap.add_argument('--restart', default='', help='rosbridge를 껐다 켜는 명령 (연결 끊김 시험). 비우면 건너뛴다')
    ap.add_argument('--out', type=Path, default=HERE / 'out')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    res = []
    ok = lambda ref, name, cond, detail='': res.append({'ref': ref, 'name': name, 'pass': bool(cond), 'detail': str(detail)})

    with sync_playwright() as p:
        browser = p.chromium.launch(channel=args.channel, headless=True)
        page = browser.new_page(viewport={'width': 1280, 'height': 720}, device_scale_factor=2)
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(args.url)
        t = wait_until(page, "document.getElementById('conn').textContent === '연결됨' && document.querySelectorAll('#map .bot').length === 5", 60)
        ok('3-1·3-3', 'rosbridge에 연결되고 모의 로봇 5대가 지도에 뜬다', t is not None, f'{t:.1f}s' if t else '60초 안에 안 됨')
        page.wait_for_timeout(1500)
        pos = page.evaluate("Object.fromEntries([...document.querySelectorAll('#map .bot')].map(b => [b.id.slice(4), b.style.transform]))")
        want = {k: f'translate({(x + 12) * 28:g}px, {(8 - y) * 28:g}px)' for k, (x, y) in STARTS.items()}
        ok('3-3', '모의 로봇이 식당 시작 위치(정차 위치)에 보인다', pos == want, pos)
        page.screenshot(path=str(args.out / 'stack_0_시작.png'))

        page.click('[data-table="t03"]')
        page.click('#collect')
        sent = page.evaluate('dashTest.sent[0].msg.task_id')
        t = wait_until(page, f"!!recordOf('{sent}')?.assignee_id", 10)
        decided = page.inner_text('#p-decide .big') if t is not None else ''
        ok('완료 조건', '테이블을 고르고 회수 호출 → 배정이 작업표와 관제 판단에 보인다', t is not None and decided.endswith('배정'),
           f'{t:.2f}s, {decided}, {rows(page)}' if t is not None else '10초 안에 배정 안 됨')
        page.wait_for_timeout(300)
        page.screenshot(path=str(args.out / 'stack_1_회수배정.png'))
        robot = page.evaluate(f"recordOf('{sent}').assignee_id")
        chips = set()
        t0 = time.time()
        while time.time() - t0 < 40 and not page.evaluate(f"doneIds.has('{sent}')"):
            chips.add(page.inner_text(f'#rc-{robot} .st'))
            page.wait_for_timeout(200)
        done = page.evaluate(f"doneIds.has('{sent}')")
        page.wait_for_timeout(500)   # 다음 그리기(0.25초)까지
        tile = page.inner_text('.tile:nth-child(4) .v').replace('\n', ' ')
        ok('3-6·3-2', '회수하는 로봇의 칩이 단계대로 바뀌고 끝나면 완료가 오른다 (서빙 0 · 회수 1)', done and {'이동', '픽업', '운반', '전달'} <= chips
           and tile.startswith('1') and '회수 1' in tile and page.inner_text('#collect') == '빈 그릇 회수',
           f"{sorted(chips)}, 완료={done}, 칸={tile}, 버튼={page.inner_text('#collect')}")
        page.click(f'#rc-{robot}')
        page.wait_for_timeout(300)
        page.click('#p-decide')
        page.wait_for_timeout(400)
        steps = page.evaluate("[...document.querySelectorAll('#pop li.done')].map(l => l.innerText.replace(/\\s+/g, ' '))")
        ok('3-8', '작업 창에 요청부터 완료까지 단계별 시각이 남는다', len(steps) == 7, steps)
        page.screenshot(path=str(args.out / 'stack_2_작업창.png'))
        page.keyboard.press('Escape')

        page.click('[data-table="t07"]')
        page.click('#serve')                 # 음식 서빙을 누르면 매장 버튼이 펼쳐진다
        page.click('[data-store="buffet_1"]')
        sent2 = page.evaluate('dashTest.sent[1].msg.task_id')
        t = wait_until(page, f"!!recordOf('{sent2}')?.assignee_id", 10)
        ok('완료 조건', '배식대 → 7번 테이블 서빙 호출 → 배정이 보인다', t is not None, f'{t:.2f}s' if t is not None else '10초 안에 배정 안 됨')
        page.wait_for_timeout(300)
        page.screenshot(path=str(args.out / 'stack_3_서빙배정.png'))

        if args.restart:
            subprocess.Popen(args.restart, shell=True)
            t_off = wait_until(page, "document.getElementById('conn').textContent === '연결 끊김 · 재연결 중'", 20)
            t_on = wait_until(page, "document.getElementById('conn').textContent === '연결됨'", 90)
            ok('3-1', 'rosbridge를 껐다 켜면 "연결 끊김 · 재연결 중" → 다시 "연결됨"', t_off is not None and t_on is not None,
               f'끊김 {t_off}, 다시 연결 {t_on}')
        ok('-', '페이지 오류 없음', not errors, errors[:3])
        browser.close()

    (args.out / 'stack_result.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf-8')
    for r in res:
        print(f"{'통과' if r['pass'] else '실패'}  [{r['ref']}] {r['name']}  ({r['detail']})")
    failed = sum(not r['pass'] for r in res)
    print(f'\n{len(res) - failed}/{len(res)} 통과')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
