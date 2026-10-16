#!/usr/bin/env python3
"""조건 목록을 GCP 러너 슬롯에 줄 세워 돌린다. 슬롯이 빌 때마다 다음 조건을 gcp_run.sh로 넣는다.

GitHub는 같은 슬롯(concurrency 그룹)에 대기 실행을 1개만 남겨서, 조건을 한꺼번에 넣으면 앞의 것이 취소된다.
이 스크립트는 슬롯마다 하나씩만 넣고 끝나기를 기다린다.

    caffeinate -i python3 tools/harness/gcp_batch.py conditions.txt --slots b1,b2,b3,b4 --ref dev
    python3 tools/harness/gcp_batch.py conditions.txt --slots b1,b2 --ref dev --dry-run   # 넣을 순서만 본다

conditions.txt: 한 줄 = gcp_run.sh 인자 한 벌 (시나리오부터, --label 필수, # 주석)
    fleet_fake -n 10 --set delay_ms=0   --label central_d0
    fleet_fake -n 10 --set delay_ms=200 --label central_d200

기록: <conditions.txt>.log.csv (--log). 줄마다 이어 써서, 꺼졌다 다시 켜면 돌던 실행부터 다시 지켜보고 이어 간다.
실패한 조건은 --retries번(기본 1) 목록 끝에서 다시 넣는다 (Spot 회수·VM 한도로 실패하는 경우).
Mac에서 돌릴 때는 caffeinate로 잠들지 않게 한다. 필요: gh (gh auth login)
"""

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.batch import (append_log, free_slots, parse_conditions, parse_slots, pending,  # noqa: E402
                           read_log, running, with_ref)

GCP_RUN = Path(__file__).resolve().parent / 'gcp_run.sh'
RUN_URL = re.compile(r'https://github\.com/\S+/actions/runs/(\d+)')


def now() -> str:
    return time.strftime('%Y-%m-%dT%H:%M:%S')


def dispatch(args: tuple, slot: str) -> tuple[str, str]:
    """gcp_run.sh로 넣고 (실행 번호, 링크)를 돌려준다. 못 넣었으면 ('', 출력 끝)."""
    proc = subprocess.run(['bash', str(GCP_RUN), *args, '--slot', slot], capture_output=True, text=True)
    out = proc.stdout + proc.stderr
    print(out.rstrip())
    m = RUN_URL.search(out)
    if proc.returncode != 0 or not m:
        return '', out.strip().splitlines()[-1] if out.strip() else f'종료 코드 {proc.returncode}'
    return m.group(1), m.group(0)


def run_state(repo: str, run_id: str) -> tuple[str, str]:
    """(status, conclusion). gh가 잠깐 실패하면 ('', '')로 두고 다음 확인 때 다시 본다."""
    proc = subprocess.run(['gh', 'run', 'view', run_id, '-R', repo, '--json', 'status,conclusion',
                           '-q', '.status + " " + (.conclusion // "")'], capture_output=True, text=True)
    if proc.returncode != 0:
        return '', ''
    status, _, conclusion = proc.stdout.strip().partition(' ')
    return status, conclusion


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='조건 목록을 GCP 러너 슬롯에 줄 세워 돌린다')
    ap.add_argument('conditions', type=Path, help='조건 목록 (한 줄 = gcp_run.sh 인자)')
    ap.add_argument('--slots', required=True, help='쓸 슬롯 (예: b1,b2,b3,b4). a와 b를 섞지 않는다')
    ap.add_argument('--ref', help='돌릴 코드 (줄에 --ref·--pr이 없을 때)')
    ap.add_argument('--pr', help='돌릴 PR 번호 (줄에 --ref·--pr이 없을 때)')
    ap.add_argument('--log', type=Path, help='실행 기록 CSV (기본 <conditions>.log.csv)')
    ap.add_argument('--retries', type=int, default=1, help='실패한 조건을 다시 넣는 횟수')
    ap.add_argument('--poll', type=int, default=60, help='상태 확인 간격 (초)')
    ap.add_argument('--repo', default='5verflow-sogang/amr-fleet-system')
    ap.add_argument('--dry-run', action='store_true', help='넣을 순서만 보여 주고 끝낸다')
    args = ap.parse_args(argv)

    try:
        conditions = parse_conditions(args.conditions.read_text(encoding='utf-8'))
        slots = parse_slots(args.slots)
        for cond in conditions:
            with_ref(cond.args, args.ref, args.pr)  # 돌릴 코드가 없는 줄은 시작 전에 막는다
    except ValueError as e:
        print(f'오류: {e}', file=sys.stderr)
        return 2
    log_path = args.log or args.conditions.with_name(args.conditions.name + '.log.csv')

    if args.dry_run:
        todo = pending(conditions, read_log(log_path), args.retries)
        print(f'남은 조건 {len(todo)}개 / 전체 {len(conditions)}개, 슬롯 {",".join(slots)}, 기록 {log_path}')
        for cond in todo:
            print('  ' + ' '.join(with_ref(cond.args, args.ref, args.pr)))
        return 0

    while True:
        # 돌고 있는 실행이 끝났는지 본다
        for slot, row in running(read_log(log_path)).items():
            if not row['run_id']:
                continue
            status, conclusion = run_state(args.repo, row['run_id'])
            if status == 'completed':
                append_log(log_path, {**row, 'status': 'completed', 'conclusion': conclusion, 'finished': now()})
                print(f'[{now()}] 끝: 슬롯 {slot} {conclusion} — {row["key"]}')

        log = read_log(log_path)
        busy = running(log)
        todo = pending(conditions, log, args.retries)
        if not todo and not busy:
            break

        for slot in free_slots(slots, busy):
            if not todo:
                break
            cond = todo.pop(0)
            attempt = sum(1 for r in log if r['key'] == cond.key and r['status'] == 'dispatched') + 1
            run_id, url = dispatch(with_ref(cond.args, args.ref, args.pr), slot)
            base = {'key': cond.key, 'attempt': attempt, 'slot': slot, 'run_id': run_id, 'url': url,
                    'started': now()}
            append_log(log_path, {**base, 'status': 'dispatched'})
            if not run_id:
                # 넣지 못한 것도 시도 1번으로 센다 (입력 오류가 끝없이 반복되지 않게)
                append_log(log_path, {**base, 'status': 'completed', 'conclusion': 'dispatch_failed',
                                      'finished': now()})
                print(f'[{now()}] 넣기 실패: 슬롯 {slot} — {cond.key}: {url}')
            else:
                print(f'[{now()}] 시작: 슬롯 {slot} — {cond.key} ({url})')
            log = read_log(log_path)
            time.sleep(5)  # gcp_run.sh가 방금 넣은 실행을 '가장 최근 실행'으로 찾으므로 다음 넣기와 간격을 둔다
        time.sleep(args.poll)

    last = {}
    for row in read_log(log_path):
        last[row['key']] = row
    ok = sum(1 for r in last.values() if r['conclusion'] == 'success')
    print(f'끝: 성공 {ok} / 전체 {len(conditions)}. 기록 {log_path}')
    for r in last.values():
        if r['conclusion'] != 'success':
            print(f'  실패 ({r["conclusion"]}, {r["attempt"]}회 시도): {r["key"]} {r["url"]}')
    return 0 if ok == len(conditions) else 1


if __name__ == '__main__':
    sys.exit(main())
