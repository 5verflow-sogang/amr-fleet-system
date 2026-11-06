#!/usr/bin/env python3
"""GCP 비용 일일 보고 → 디스코드 (.github/workflows/cost-report.yml이 매일 09:00 KST에 부른다).

결제 내보내기(BigQuery billing_export, 표준 사용량 비용)에서 어제(KST) 비용, 이번 달 누적, 서비스별·실험별 비용을
읽는다. 실험별 비용은 러너 VM 라벨 run(= Actions 실행 번호, #314)을 fleet_ops.runs와 이어 시나리오 이름을 붙인다.
이번 달 누적이 예산의 50·90·100 %를 어제 넘었으면 알림 줄을 맨 위에 붙인다 (예산 메일과 같은 문턱).

    python3 tools/infra/gcp/cost_report.py                 # 디스코드로 보낸다 (COST_DISCORD_WEBHOOK)
    python3 tools/infra/gcp/cost_report.py --dry-run       # 보낼 내용만 출력

환경 변수: COST_DISCORD_WEBHOOK(없으면 출력만), BUDGET_KRW(기본 100000), GCP_PROJECT(기본 amr-fleet).
결제 데이터는 몇 시간 늦게 들어오므로 어제 비용은 덜 잡힐 수 있다. 다음 날 보고의 누적에는 들어간다.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import urllib.request

THRESHOLDS = (0.5, 0.9, 1.0)
KST = dt.timezone(dt.timedelta(hours=9))
# 크레딧(무료 등급 등)을 뺀 실제 비용
NET = 'cost + IFNULL((SELECT SUM(c.amount) FROM UNNEST(credits) c), 0)'


def crossed(before: float, now: float, budget: float, thresholds=THRESHOLDS) -> list[float]:
    """누적 비용이 before → now로 늘 때 새로 넘은 예산 비율들."""
    return [t for t in thresholds if before < t * budget <= now]


def build_payload(day: dt.date, mtd: float, before: float, yday: float, budget: float,
                  services: list[tuple[str, float]], runs: list[dict]) -> dict:
    """디스코드 웹훅 JSON. services: [(서비스, 비용)], runs: [{run_id, scenario, label, cost}]."""
    hits = crossed(before, mtd, budget)
    lines = []
    if hits:
        lines.append(f"🚨 이번 달 GCP 비용이 예산 {budget:,.0f}원의 **{max(hits):.0%}**를 넘었습니다")
    lines.append(f"💰 GCP 비용 {day:%m/%d}(KST): 어제 **{yday:,.0f}원** · 이번 달 누적 **{mtd:,.0f}원** "
                 f"/ 예산 {budget:,.0f}원 ({mtd / budget:.0%})")
    fields = []
    if services:
        fields.append({'name': '어제 서비스별', 'inline': True,
                       'value': '\n'.join(f'{name}: {cost:,.0f}원' for name, cost in services)[:1024]})
    if runs:
        fields.append({'name': f'어제 실험별 (VM 라벨, {len(runs)}건)', 'inline': True,
                       'value': '\n'.join(f"{r['scenario'] or '?'}·{r['label'] or '?'} (#{r['run_id']}): {r['cost']:,.0f}원"
                                          for r in runs[:10])[:1024]})
    payload = {'content': '\n'.join(lines)}
    if fields:
        payload['embeds'] = [{'color': 0xE74C3C if hits else 0x3498DB, 'fields': fields,
                              'footer': {'text': '결제 내보내기는 몇 시간 늦게 들어와 어제 비용이 덜 잡힐 수 있다'}}]
    return payload


def bq(project: str, sql: str) -> list[dict]:
    out = subprocess.run(['bq', f'--project_id={project}', 'query', '--nouse_legacy_sql', '--format=json',
                          '--max_rows=1000', sql], check=True, capture_output=True, text=True).stdout
    return json.loads(out or '[]')


def billing_table(project: str) -> str | None:
    rows = bq(project, "SELECT table_name FROM `billing_export.INFORMATION_SCHEMA.TABLES` "
                       "WHERE table_name LIKE 'gcp_billing_export_v1_%' ORDER BY table_name LIMIT 1")
    return f"{project}.billing_export.{rows[0]['table_name']}" if rows else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='GCP 비용 일일 보고 → 디스코드')
    ap.add_argument('--dry-run', action='store_true', help='보내지 않고 출력만')
    args = ap.parse_args(argv)
    project = os.environ.get('GCP_PROJECT', 'amr-fleet')
    budget = float(os.environ.get('BUDGET_KRW') or 100000)
    webhook = os.environ.get('COST_DISCORD_WEBHOOK', '')

    table = billing_table(project)
    if table is None:
        print('결제 내보내기 테이블이 아직 없다 (켠 뒤 몇 시간 걸린다). 보내지 않는다')
        return 0
    yday = dt.datetime.now(KST).date() - dt.timedelta(days=1)
    first = yday.replace(day=1)
    day = f"DATE(usage_start_time, 'Asia/Seoul')"
    where = f"{day} BETWEEN '{first}' AND '{yday}'"
    total = bq(project, f"""
        SELECT SUM({NET}) AS mtd, SUM(IF({day} < '{yday}', {NET}, 0)) AS before,
               SUM(IF({day} = '{yday}', {NET}, 0)) AS yday
        FROM `{table}` WHERE {where}""")[0]
    services = bq(project, f"""
        SELECT service.description AS name, SUM({NET}) AS cost FROM `{table}`
        WHERE {day} = '{yday}' GROUP BY name HAVING cost >= 1 ORDER BY cost DESC LIMIT 5""")
    runs = bq(project, f"""
        SELECT l.value AS run_id, ANY_VALUE(r.scenario) AS scenario, ANY_VALUE(r.label) AS label, SUM({NET}) AS cost
        FROM `{table}`, UNNEST(labels) l
        LEFT JOIN `{project}.fleet_ops.runs` r ON r.run_id = l.value
        WHERE {day} = '{yday}' AND l.key = 'run' GROUP BY run_id ORDER BY cost DESC""")

    payload = build_payload(
        yday, float(total['mtd'] or 0), float(total['before'] or 0), float(total['yday'] or 0), budget,
        [(s['name'], float(s['cost'])) for s in services],
        [{**r, 'cost': float(r['cost'])} for r in runs])
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if webhook and not args.dry_run:
        req = urllib.request.Request(webhook, data=json.dumps(payload).encode(), method='POST',
                                     headers={'Content-Type': 'application/json', 'User-Agent': 'amr-fleet-cost-report'})
        urllib.request.urlopen(req, timeout=10).close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
