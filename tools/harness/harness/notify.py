"""하네스 요약을 Discord 채널로 보낸다. ROS 없이, 표준 라이브러리만 쓴다.

웹훅 주소는 환경 변수 HARNESS_DISCORD_WEBHOOK에서 읽는다. 없으면 아무것도 하지 않는다.
보내기가 실패해도 실험 결과에는 영향이 없다 (경고만 출력).
실행 폴더에 plots/timeline.png(Fleet 시나리오, fleet_plot)가 있으면 작업 완료율이 가장 낮은 실행의 그림 한 장을
그 조건의 embed에 붙인다. 반복마다 다 보내면 채널이 그림으로 덮이므로 "어디를 열어 볼지"만 알린다.
"""

from __future__ import annotations

import json
import os
import urllib.request
import uuid
from pathlib import Path
from typing import Iterable, Optional

from .summary import _fmt, fleet_metrics, load_meta

ENV = 'HARNESS_DISCORD_WEBHOOK'

COLOR = {'PASS': 0x2ECC71, 'MARGINAL': 0xF1C40F, 'FAIL': 0xE74C3C, 'NONE': 0x95A5A6}
ICON = {'PASS': '✅', 'MARGINAL': '⚠️', 'FAIL': '❌', 'NONE': 'ℹ️'}

# KPI 판정이 있는 지표 외에 같이 보여 줄 지표
HEADLINE = (
    'fleet.latency_ms.mean', 'fleet.latency_ms.p95', 'fleet.loss_rate',
    'fleet.ack_rtt_ms.mean', 'fleet.ack_rate', 'fleet.resends.mean',
    'fleet.task_completion_rate', 'fleet.recovery_s.mean', 'fleet.reclaimed_completed_rate',
    'run.real_time_factor',
)

# Discord 한도: 메시지당 embed 10개, embed당 field 25개
MAX_EMBEDS = 10
MAX_FIELDS = 25

# 실행 폴더 안 그림 (fleet_plot). 첨부 파일 이름이 embed의 attachment://<이름>과 같아야 embed 안에 보인다
PLOT = Path('plots') / 'timeline.png'


def group_status(group: dict) -> str:
    """조건 묶음 하나의 판정. 완료가 없으면 FAIL, KPI FAIL이 있으면 FAIL,
    MARGINAL이나 실패한 반복이 있으면 MARGINAL, KPI가 모두 PASS면 PASS, KPI가 없으면 NONE."""
    if not group['runs']:
        return 'FAIL'
    verdicts = [d['kpi'] for d in group['metrics'].values() if d.get('kpi')]
    if 'FAIL' in verdicts:
        return 'FAIL'
    if 'MARGINAL' in verdicts or group['failed']:
        return 'MARGINAL'
    return 'PASS' if verdicts else 'NONE'


def context_line(run_dirs: Iterable[Path]) -> str:
    """메시지 첫 줄: git, 호스트, 장애 조건, seed 범위 (첫 실행의 meta.json 기준)."""
    metas = [m for m in (load_meta(Path(d)) for d in run_dirs) if m]
    if not metas:
        return '하네스 결과'
    first = metas[0]
    parts = [f"git `{first.get('git_sha', '?')}`"]
    host = first.get('host') or {}
    if host.get('hostname'):
        parts.append(f"{host['hostname']} ({host.get('cores', '?')}코어)")
    fault = first.get('fault')
    if fault:
        parts.append(f"지연 {fault['delay_ms']:g} ms · 손실 {fault['loss_pct']:g}%")
    seeds = sorted(m['seed'] for m in metas if isinstance(m.get('seed'), int))
    if seeds:
        parts.append(f'seed {seeds[0]}~{seeds[-1]}')
    return '하네스 결과 · ' + ' · '.join(parts)


def pick_plot(run_dirs: Iterable[Path]) -> Optional[tuple[Path, str, str]]:
    """그림이 있는 실행 중 작업 완료율이 가장 낮은 것의 (그림 경로, 실행 이름, 설명). 같으면 앞 실행. 그림이 없으면 None.

    완료율이 없는 실행(작업 결과가 없음)은 그림이 있어도 뒤로 미룬다.
    """
    candidates = []
    for i, run_dir in enumerate(Path(d) for d in run_dirs):
        if (run_dir / PLOT).exists():
            rate = fleet_metrics(run_dir).get('fleet.task_completion_rate')
            candidates.append((rate is None, rate or 0.0, i, run_dir))
    if not candidates:
        return None
    missing, rate, _, run_dir = min(candidates)
    reason = '작업 결과 없음' if missing else f'작업 완료율 {rate:.0%}로 가장 낮음'
    return run_dir / PLOT, run_dir.name, f'그림: {run_dir.name} (그림이 있는 {len(candidates)}회 중 {reason})'


def build_payload(summary: dict, context: str = '', plot: Optional[tuple[str, str]] = None) -> dict:
    """summarize() 결과 → Discord 웹훅 JSON.

    plot = (실행 이름, 설명): 그 실행이 속한 조건의 embed에 첨부 그림(PLOT 파일 이름)을 넣고 설명을 덧붙인다.
    """
    embeds = []
    for name, group in list(summary.items())[:MAX_EMBEDS]:
        status = group_status(group)
        fields = []
        for metric, d in group['metrics'].items():
            if not d.get('kpi') and metric not in HEADLINE:
                continue
            ci = '' if d['ci_low'] is None else f" (95% CI {_fmt(d['ci_low'], metric)} ~ {_fmt(d['ci_high'], metric)})"
            label = f"{metric} · {d['kpi']}" if d.get('kpi') else metric
            fields.append({'name': label[:256], 'value': f"{_fmt(d['mean'], metric)}{ci}"[:1024], 'inline': True})
        description = f"완료 {len(group['runs'])}회, 실패 {len(group['failed'])}회"
        if group['failed']:
            description += ' (' + ', '.join(f"{f['run_id']}: {f['outcome']}" for f in group['failed'][:5]) + ')'
        embed = {
            'title': f'{ICON[status]} {name}'[:256],
            'description': description[:4096],
            'color': COLOR[status],
            'fields': fields[:MAX_FIELDS],
        }
        if plot is not None and plot[0] in group['runs']:
            embed['description'] = f"{description}\n{plot[1]}"[:4096]
            embed['image'] = {'url': f'attachment://{PLOT.name}'}
        embeds.append(embed)
    return {'content': context[:2000], 'embeds': embeds}


def send(webhook: str, payload: dict, attachment: Optional[Path] = None, timeout: float = 10.0,
         image: Optional[Path] = None) -> None:
    """웹훅으로 보낸다. attachment(요약 md)·image(PNG)가 있으면 파일로 첨부한다 (multipart)."""
    boundary = uuid.uuid4().hex
    parts = [('payload_json', 'application/json', json.dumps(payload, ensure_ascii=False).encode(), None)]
    files = [(f, 'text/markdown') for f in (attachment,) if f is not None]
    files += [(f, 'image/png') for f in (image,) if f is not None]
    for i, (path, ctype) in enumerate(files):
        parts.append((f'files[{i}]', ctype, path.read_bytes(), path.name))
    body = b''
    for name, ctype, data, filename in parts:
        disposition = f'form-data; name="{name}"' + (f'; filename="{filename}"' if filename else '')
        body += (f'--{boundary}\r\nContent-Disposition: {disposition}\r\n'
                 f'Content-Type: {ctype}\r\n\r\n').encode() + data + b'\r\n'
    body += f'--{boundary}--\r\n'.encode()
    request = urllib.request.Request(webhook, data=body, method='POST', headers={
        'Content-Type': f'multipart/form-data; boundary={boundary}',
        # 기본 User-Agent(Python-urllib)는 Discord가 막는 경우가 있다
        'User-Agent': 'amr-fleet-harness',
    })
    with urllib.request.urlopen(request, timeout=timeout) as response:
        response.read()


def notify(summary: dict, run_dirs: Iterable[Path], summary_path: Optional[Path] = None) -> bool:
    """HARNESS_DISCORD_WEBHOOK이 있으면 요약을 보낸다. 보냈으면 True."""
    webhook = os.environ.get(ENV, '').strip()
    if not webhook or not summary:
        return False
    try:
        run_dirs = list(run_dirs)
        picked = pick_plot(run_dirs)
        send(webhook, build_payload(summary, context_line(run_dirs), picked[1:] if picked else None), summary_path,
             image=picked[0] if picked else None)
    except Exception as e:  # noqa: BLE001 - 알림 실패로 실험을 실패시키지 않는다
        print(f'Discord 알림 실패 (결과에는 영향 없음): {type(e).__name__}: {e}')
        return False
    print('Discord 알림 보냄')
    return True
