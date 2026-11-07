#!/usr/bin/env python3
"""측정 bag에서 뽑은 메시지(dashboard.jsonl)를 대시보드에 시뮬레이션 시각대로 넣으며 화면을 녹화한다. ROS 없이 돈다.

    python3 tools/demo/record_dashboard.py <실행 폴더>/dashboard.jsonl --out s11_on_dashboard.mp4
    python3 tools/demo/record_dashboard.py ... --lead 3 --length 50 --clock 12:20 --scale 2

dashboard.jsonl은 하네스 후처리(tools/demo/bag_export.py)가 실행 폴더에 남긴다. 버킷에서 받아 맥에서 돌리면 된다.
- 대시보드는 ?ws=off로 열고 window.dashTest.feed로 메시지를 넣는다 (rosbridge 없음, 호출 버튼은 안 씀).
- 0초(/demo/t0) - lead 전까지는 한꺼번에 넣어 화면 상태를 만들고, 그 뒤로는 시뮬레이션 1초 = 실제 1초로 넣는다.
  시뮬레이터가 느려도(RTF 0.07) 로봇은 실제 속도로 움직인다.
- 화면은 Chrome 스크린캐스트(JPEG, 기본 2배 해상도)로 받아 30 fps mp4로 만든다. 영상 lead초 지점이 0초다
  (<out>.json에 남긴다. 카메라 영상과 맞출 때 쓴다).
필요: pip install playwright && playwright install chromium (또는 --channel chrome), ffmpeg (없으면 pip의 imageio-ffmpeg)
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import shutil
import subprocess
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
DASHBOARD = os.path.join(HERE, '..', '..', 'dashboard', 'index.html')
W, H = 1280, 720                                   # 대시보드 화면 크기 (dashboard/README.md)


def load(path: str):
    with open(path, encoding='utf-8') as f:
        head = json.loads(f.readline())
        items = [json.loads(line) for line in f if line.strip()]
    items.sort(key=lambda m: m['t'])
    return head, items


def find_ffmpeg() -> str:
    exe = shutil.which('ffmpeg')
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        raise SystemExit('ffmpeg가 없다: brew install ffmpeg 또는 pip install imageio-ffmpeg')


def frame_at(frames: list, t: float, i: int) -> int:
    """frames[i]부터 보며 시각 t 이전의 마지막 장 번호 (없으면 첫 장)."""
    while i + 1 < len(frames) and frames[i + 1][0] <= t:
        i += 1
    return i


def record(page, cdp, items, start: float, end: float, speed: float):
    """start~end(시뮬레이션 s)를 실제 시간으로 넣으며 스크린캐스트 장면을 모은다. [(시뮬레이션 시각, jpeg)]"""
    frames, pending = [], []

    def on_frame(p):
        pending.append(p)

    cdp.on('Page.screencastFrame', on_frame)
    cdp.send('Page.startScreencast', {'format': 'jpeg', 'quality': 92, 'maxWidth': W * 4, 'maxHeight': H * 4})
    i = 0
    while i < len(items) and items[i]['t'] < start:
        i += 1
    w0 = time.time()
    while True:
        sim = start + (time.time() - w0) * speed
        batch = []
        while i < len(items) and items[i]['t'] <= sim:
            batch.append(items[i])
            i += 1
        if batch:
            page.evaluate('ms => ms.forEach(m => dashTest.feed(m.topic, m.msg))', batch)
        for p in pending:
            stamp = p['metadata'].get('timestamp') or time.time()
            frames.append((start + (stamp - w0) * speed, base64.b64decode(p['data'])))
            cdp.send('Page.screencastFrameAck', {'sessionId': p['sessionId']})
        pending.clear()
        if sim >= end:
            break
        page.wait_for_timeout(10)
    cdp.send('Page.stopScreencast')
    return frames


def encode(frames, start: float, end: float, fps: float, out: str, crf: int) -> int:
    """장면들을 fps 간격으로 골라 mp4로 (JPEG를 그대로 ffmpeg에 흘린다)."""
    proc = subprocess.Popen([find_ffmpeg(), '-y', '-loglevel', 'error', '-f', 'image2pipe', '-c:v', 'mjpeg',
                             '-framerate', f'{fps:g}', '-i', '-', '-c:v', 'libx264', '-preset', 'medium',
                             '-crf', str(crf), '-vf', 'scale=out_range=tv', '-pix_fmt', 'yuv420p', out],
                            stdin=subprocess.PIPE)
    n, i = int((end - start) * fps), 0
    for k in range(n):
        i = frame_at(frames, start + k / fps, i)
        proc.stdin.write(frames[i][1])
    proc.stdin.close()
    if proc.wait() != 0:
        raise SystemExit(f'ffmpeg 실패 (종료 코드 {proc.returncode})')
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('jsonl', help='dashboard.jsonl (tools/demo/bag_export.py)')
    ap.add_argument('--out', required=True, help='mp4')
    ap.add_argument('--lead', type=float, default=3.0, help='0초 앞 길이 (s)')
    ap.add_argument('--length', type=float, default=None, help='0초 뒤 길이 (s). 기본은 메시지 끝까지')
    ap.add_argument('--t0', type=float, default=None, help='0초 (시뮬레이션 s). 기본은 jsonl의 /demo/t0')
    ap.add_argument('--clock', default='12:20', help='화면 시계: 시뮬레이션 0초를 이 시각으로 (대시보드 ?clock=)')
    ap.add_argument('--scale', type=float, default=2.0, help='화면 배율 (2면 2560x1440으로 받는다)')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--speed', type=float, default=1.0, help='시뮬레이션 1초를 실제 1/speed초로 (1 = 실제 속도)')
    ap.add_argument('--crf', type=int, default=18)
    ap.add_argument('--dashboard', default=DASHBOARD, help='dashboard/index.html')
    ap.add_argument('--channel', default=None, help='브라우저 채널 (예: chrome = 설치된 Chrome, dashboard/test와 같다)')
    ap.add_argument('--headed', action='store_true', help='브라우저 창을 띄운다 (확인용)')
    ap.add_argument('--query', default='', help='대시보드 주소 옵션 더 붙이기 (예: min_hall=3&charge_why=배터리 20%% 미만)')
    args = ap.parse_args(argv)

    from playwright.sync_api import sync_playwright

    head, items = load(args.jsonl)
    if not items:
        raise SystemExit(f'{args.jsonl}: 메시지가 없다')
    t0 = args.t0 if args.t0 is not None else head.get('t0')
    if t0 is None:
        raise SystemExit(f'{args.jsonl}: 0초(/demo/t0)가 없다. --t0으로 준다 (메시지 {items[0]["t"]}~{items[-1]["t"]} s)')
    start = max(items[0]['t'], t0 - args.lead)
    end = items[-1]['t'] if args.length is None else min(items[-1]['t'], t0 + args.length)
    url = f'file://{os.path.abspath(args.dashboard)}?ws=off&clock={args.clock}'
    if args.query:
        url += '&' + urllib.parse.quote(args.query, safe='=&')
    with sync_playwright() as pw:
        # 스크린캐스트는 창의 실제 화면을 받는다: Playwright 화면 흉내(viewport·배율)가 아니라 창 크기와
        # --force-device-scale-factor로 맞춘다. 창 테두리만큼 키워 안쪽을 정확히 1280x720으로 (11/7 확인: 머리 87 px)
        browser = pw.chromium.launch(channel=args.channel, headless=not args.headed,
                                     args=[f'--window-size={W},{H}', f'--force-device-scale-factor={args.scale:g}'])
        page = browser.new_context(no_viewport=True).new_page()
        page.goto(url)
        cdp = page.context.new_cdp_session(page)
        iw, ih = page.evaluate('[innerWidth, innerHeight]')
        win = cdp.send('Browser.getWindowForTarget')['windowId']
        cdp.send('Browser.setWindowBounds', {'windowId': win, 'bounds': {'width': 2 * W - iw, 'height': 2 * H - ih}})
        page.wait_for_timeout(300)
        if page.evaluate('[innerWidth, innerHeight]') != [W, H]:
            raise SystemExit(f'창 안쪽을 {W}x{H}로 못 맞췄다: {page.evaluate("[innerWidth, innerHeight]")}')
        page.evaluate('() => dashTest.setConnected(true)')
        before = [m for m in items if m['t'] < start]
        for k in range(0, len(before), 500):           # 0초 앞까지는 한꺼번에 넣어 화면 상태를 만든다
            page.evaluate('ms => ms.forEach(m => dashTest.feed(m.topic, m.msg))', before[k:k + 500])
        page.evaluate('() => dashTest.tick()')
        page.wait_for_timeout(1200)                      # 로봇 이동 애니메이션(0.5 s)이 끝나게
        frames = record(page, cdp, items, start, end, args.speed)
        browser.close()
    if not frames:
        raise SystemExit('스크린캐스트 장면을 하나도 못 받았다')
    n = encode(frames, start, end, args.fps, args.out, args.crf)
    meta = {'jsonl': os.path.abspath(args.jsonl), 'scene': head.get('scene', ''), 't0_sim': t0, 'start_sim': start,
            'lead_s': t0 - start, 'fps': args.fps, 'frames': n, 'captured': len(frames), 'clock': args.clock}
    with open(os.path.splitext(args.out)[0] + '.json', 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    print(f'{args.out}: {n}프레임 ({n / args.fps:.1f} s, 0초는 {t0 - start:.1f} s 지점), 받은 장면 {len(frames)}장')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
