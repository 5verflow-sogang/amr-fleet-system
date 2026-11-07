"""GCP 비용 일일 보고 (tools/infra/gcp/cost_report.py). BigQuery·디스코드 없이 문턱과 메시지만 본다."""

import datetime as dt
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cost_report import build_payload, crossed  # noqa: E402

DAY = dt.date(2026, 11, 6)


def test_crossed_only_new_thresholds():
    assert crossed(40000, 52000, 100000) == [0.5]
    assert crossed(52000, 60000, 100000) == []             # 이미 넘은 문턱은 다시 알리지 않는다
    assert crossed(45000, 95000, 100000) == [0.5, 0.9]      # 하루에 둘을 넘으면 둘 다
    assert crossed(95000, 100000, 100000) == [1.0]


def test_payload_plain_day():
    p = build_payload(DAY, 12000, 10000, 2000, 100000, [('Compute Engine', 1800.4)],
                      [{'run_id': '42172811997', 'scenario': 'demo_smoke', 'label': 'cpu3', 'cost': 120.0}])
    assert p['content'].startswith('💰 GCP 비용 11/06(KST): 어제 **2,000원** · 이번 달 누적 **12,000원**')
    assert '🚨' not in p['content']
    fields = p['embeds'][0]['fields']
    assert fields[0]['value'] == 'Compute Engine: 1,800원'
    assert fields[1]['value'] == 'demo_smoke·cpu3 (#42172811997): 120원'


def test_payload_alert_first_line():
    p = build_payload(DAY, 91000, 85000, 6000, 100000, [], [])
    assert p['content'].splitlines()[0] == '🚨 이번 달 GCP 비용이 예산 100,000원의 **90%**를 넘었습니다'
    assert 'embeds' not in p                                # 내역이 없으면 embed 없이 한 줄만


def test_unknown_run_label_still_listed():
    # 라벨은 있는데 실행 목록에 없는 VM (index.csv에 줄이 안 남은 실패 실행)
    p = build_payload(DAY, 1000, 0, 1000, 100000, [], [{'run_id': '1', 'scenario': None, 'label': None, 'cost': 5.0}])
    assert p['embeds'][0]['fields'][0]['value'] == '?·? (#1): 5원'
