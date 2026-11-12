"""실험이 끝난 뒤 녹화 영상과 실패한 반복을 Claude로 짧게 점검해 실험 결과 채널에 한 메시지로 더 보낸다.

    python3 tools/harness/ai_summary.py <결과 폴더> --title runs/demo_s11-on/... --url <Actions 실행 주소>
    python3 tools/harness/ai_summary.py <결과 폴더> --dry-run    # API·Discord 없이 무엇을 볼지만 출력

<결과 폴더>는 실행 폴더(meta.json이 있는 폴더)들의 부모다 (fleet-test.yml ai-summary가 버킷에서 rosbag만 빼고 받는다).
- 영상: 실행 폴더 video/*.mp4마다 프레임 4장을 보여 주고 빈 화면·로봇 안 보임·가림·끼임·안 움직임을 묻는다
- 실패: outcome이 completed가 아닌 반복마다 meta.json error와 로그의 오류 줄(전체 로그는 보내지 않음)로 원인을 한 줄로 묻는다
볼 게 없으면 API를 부르지 않는다. 판정은 AI 추정이라 메시지에 그렇게 적는다 (11/12 시험: 12개 항목 중 11개 맞음).
인증은 SDK가 환경 변수에서 찾는다: CI는 GitHub OIDC 연동(ANTHROPIC_FEDERATION_RULE_ID 등), 로컬은 ANTHROPIC_API_KEY.
API·Discord가 실패해도 0으로 끝난다 (실험 결과·기존 요약 알림에 영향 없음).
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.notify import ENV, send  # noqa: E402

MODEL = 'claude-sonnet-5-5'   # Haiku는 화면 앞 사람의 가림과 로봇 생김새를 놓쳤다 (11/12 시험)
FRAMES = 4
MAX_LOG_LINES = 60
DISCORD_LIMIT = 2000

VIDEO_PROMPT = (
    '식당 서빙 로봇 시뮬레이션 녹화 영상에서 시간 순서로 뽑은 프레임들이다. '
    '로봇은 파란 바닥판 위에 흰 선반 기둥이 선 모양이고, 위에서 보면 흰색·파란색 사각형이다. '
    '갈색 옷을 입은 사람들은 손님·직원이다. JSON 한 줄로만 답하라: '
    '{"blank": 화면이 검거나 비어 있으면 true, "robots_visible": 로봇이 한 대라도 보이면 true, '
    '"occluded": 카메라 바로 앞의 사람이나 벽·기둥이 화면의 4분의 1 이상을 계속 가리면 true, '
    '"stuck": 로봇이 가구·벽 안으로 파고들었거나 넘어져 보이면 true, "moved": 프레임 사이에 로봇이 움직였으면 true, '
    '"note": 한국어 한 문장}')
FAILURE_PROMPT = (
    'ROS 2 로봇 군집 시뮬레이션 실험의 반복 하나가 실패했다. 아래 meta.json 요약과 로그의 오류 줄만 보고 '
    '가장 그럴듯한 원인을 한국어 한 문장(80자 이내)으로 답하라. drive.log 줄은 하네스가 직접 남긴 실패 이유라 가장 먼저 본다 '
    '(예: "시계(/clock)가 오지 않음"은 Gazebo 시계를 못 받은 기동 실패). meta.json error는 실패한 단계, '
    'startup_failure가 true면 기동 단계 실패다. 이것들로도 알 수 없을 때만 "로그로는 원인 불명"이라고 답하라.\n\n')

# 문제로 볼 항목 (값이 이러면 표시한다)
FLAGS = (('blank', True, '빈 화면'), ('robots_visible', False, '로봇 안 보임'), ('occluded', True, '가림'),
         ('stuck', True, '끼임'), ('moved', False, '안 움직임'))
ERROR_RE = re.compile(r'\[(ERROR|FATAL)\]|\w+(Error|Exception):|aborted|timed out|시간 초과|실패')
# 하네스가 끝에 SIGINT로 정리할 때 나는 줄. 원인이 아니라 결과라 빼야 진짜 오류가 남는다 (11/7 /clock 실패 로그의 대부분)
NOISE_RE = re.compile(r'\[(INFO|DEBUG)\]|failed to terminate|exit code -2|KeyboardInterrupt|ExternalShutdownException'
                      r'|context is invalid|Cannot shutdown')


def run_dirs(root: Path) -> list[Path]:
    return sorted(p.parent for p in root.glob('*/meta.json'))


def failed_runs(root: Path) -> list[tuple[Path, dict]]:
    out = []
    for run in run_dirs(root):
        meta = json.loads((run / 'meta.json').read_text(encoding='utf-8'))
        if meta.get('outcome') != 'completed':
            out.append((run, meta))
    return out


def videos(root: Path) -> list[Path]:
    return sorted(root.glob('*/video/*.mp4'))


def error_lines(log_dir: Path, limit: int = MAX_LOG_LINES) -> list[str]:
    """로그 파일들의 오류 줄. drive.log는 끝 5줄 그대로, 나머지는 오류 줄만 파일마다 끝쪽 10줄, 같은 줄은 한 번,
    전체 limit줄 (토큰을 아끼고 마지막 원인을 남긴다)."""
    out = []
    for log in sorted(log_dir.glob('*.log')):
        text = log.read_text(encoding='utf-8', errors='replace').splitlines()
        if log.name == 'drive.log':   # 하네스 주행 단계의 실패 이유가 여기 한두 줄로 남는다 (예: 시계(/clock)가 오지 않음)
            out += [f'drive.log: {line.strip()[:300]}' for line in text[-5:] if line.strip()]
            continue
        hits = []
        for line in text:
            line = line.strip()[:300]
            if ERROR_RE.search(line) and not NOISE_RE.search(line) and line not in hits:
                hits.append(line)
        out += [f'{log.name}: {h}' for h in hits[-10:]]
    return out[-limit:]


def failure_context(run: Path, meta: dict) -> str:
    brief = {k: meta.get(k) for k in ('scenario', 'label', 'repeat', 'outcome', 'error', 'startup_failure',
                                       'ready_s', 'real_time_factor', 'duration_wall_s')}
    lines = error_lines(run / 'logs')
    return (f'meta.json: {json.dumps(brief, ensure_ascii=False)}\n로그 오류 줄 {len(lines)}개:\n'
            + ('\n'.join(lines) or '(없음)'))


def flags(verdict: dict) -> list[str]:
    return [name for key, bad, name in FLAGS if key in verdict and verdict[key] is bad]


def parse_json(text: str) -> dict:
    match = re.search(r'\{.*\}', text, re.S)
    try:
        return json.loads(match.group(0)) if match else {'note': text}
    except json.JSONDecodeError:
        return {'note': text}


def frames(path: Path, n: int = FRAMES, width: int = 768) -> list[bytes]:
    """영상 전체에 고르게 n장 (JPEG). 768 px면 장당 약 450 토큰."""
    import cv2   # CI 하네스 테스트에는 없어도 되게 여기서 부른다
    cap = cv2.VideoCapture(str(path))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out = []
    for k in range(n):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(total * (k + 0.5) / n))
        ok, img = cap.read()
        if ok:
            h, w = img.shape[:2]
            img = cv2.resize(img, (width, int(h * width / w)))
            out.append(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
    return out


def ask(client, model: str, content: list) -> str:
    r = client.messages.create(model=model, max_tokens=2000, messages=[{'role': 'user', 'content': content}])
    # 답 앞에 thinking 블록이 올 수 있어 text 블록만 모은다
    return ''.join(b.text for b in r.content if b.type == 'text').strip()


def build_message(title: str, url: str, model: str, video_results: list[tuple[str, dict]],
                  failure_results: list[tuple[str, str, str]]) -> str:
    head = f'🤖 **AI 점검** (추정, {model}) · `{title}`' + (f' · [실행]({url})' if url else '')
    lines = [head]
    if video_results:
        lines.append('🎬 영상')
        for name, v in video_results:
            bad = flags(v)
            mark = f"⚠️ {'·'.join(bad)}" if bad else '✅'
            lines.append(f"- `{name}` {mark} — {v.get('note', '')}")
    if failure_results:
        lines.append('❗ 실패한 반복')
        for name, outcome, why in failure_results:
            lines.append(f'- `{name}` ({outcome}) — {why}')
    text = '\n'.join(lines)
    return text if len(text) <= DISCORD_LIMIT else text[:DISCORD_LIMIT - 20] + '\n…(잘림)'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('root', type=Path)
    ap.add_argument('--title', default='')
    ap.add_argument('--url', default='')
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)

    vids, fails = videos(a.root), failed_runs(a.root)
    print(f'영상 {len(vids)}개, 실패한 반복 {len(fails)}개')
    if not vids and not fails:
        return 0
    if a.dry_run:
        for v in vids:
            print('영상', v.relative_to(a.root))
        for run, meta in fails:
            print(f'--- 실패 {run.name}\n{failure_context(run, meta)}')
        return 0

    try:
        import anthropic
        client = anthropic.Anthropic()
        video_results = []
        for v in vids:
            shots = frames(v)
            content = [{'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                                     'data': base64.b64encode(s).decode()}} for s in shots]
            name = f'{v.parent.parent.name}/{v.stem}'
            video_results.append((name, parse_json(ask(client, a.model, content + [{'type': 'text', 'text': VIDEO_PROMPT}]))
                                  if shots else {'blank': True, 'note': '프레임을 읽지 못함'}))
        failure_results = [(run.name, meta.get('outcome', '?'),
                            ask(client, a.model, [{'type': 'text', 'text': FAILURE_PROMPT + failure_context(run, meta)}]))
                           for run, meta in fails]
    except Exception as e:   # 인증·네트워크·영상 문제로 실험 알림을 막지 않는다
        print(f'경고: AI 점검 실패 ({type(e).__name__}: {e})', file=sys.stderr)
        return 0

    message = build_message(a.title, a.url, a.model, video_results, failure_results)
    print(message)
    webhook = os.environ.get(ENV, '').strip()
    if webhook:
        try:
            send(webhook, {'content': message})
        except Exception as e:
            print(f'경고: Discord 전송 실패 ({e})', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
