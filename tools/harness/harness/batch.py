"""GCP 실행 줄 세우기(gcp_batch.py)의 판단 부분. ROS·gh 없이 돈다.

조건 목록 파일의 한 줄 = gcp_run.sh 인자 한 벌(시나리오부터). 실행 기록(CSV)을 보고 아직 안 끝난 조건을
빈 슬롯에 넣는다. 기록은 줄마다 이어 써서, 스크립트가 중간에 꺼져도 다시 켜면 이어서 돈다.
"""

from __future__ import annotations

import csv
import shlex
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

SLOTS = ('a', 'b1', 'b2', 'b3', 'b4')
LOG_HEADER = ['key', 'attempt', 'slot', 'run_id', 'url', 'status', 'conclusion', 'started', 'finished']
# 줄에 쓰면 안 되는 인자: 슬롯은 디스패처가 정하고, --watch는 실행이 끝날 때까지 디스패처를 멈춘다
FORBIDDEN = ('--slot', '--watch')


@dataclass(frozen=True)
class Condition:
    key: str          # 줄 원문 (공백 정리). 기록에서 조건을 찾는 이름
    args: tuple       # gcp_run.sh 인자


def parse_conditions(text: str) -> list[Condition]:
    """조건 목록 → Condition 목록. 빈 줄·# 주석은 건너뛴다.

    줄마다 --label이 있어야 한다 (결과 경로·index.csv에서 조건을 가르는 이름). 같은 줄이 두 번 있으면 거부한다.
    """
    out, seen = [], set()
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        args = tuple(shlex.split(line))
        if args[0].startswith('-'):
            raise ValueError(f'{number}행: 첫 칸은 시나리오 이름: {line}')
        bad = [a for a in args if a.split('=', 1)[0] in FORBIDDEN]
        if bad:
            raise ValueError(f'{number}행: {bad[0]}는 쓰지 않는다 (슬롯은 --slots로 정한다)')
        if '--label' not in args:
            raise ValueError(f'{number}행: --label이 없다: {line}')
        key = ' '.join(args)
        if key in seen:
            raise ValueError(f'{number}행: 같은 조건이 두 번 있다: {line}')
        seen.add(key)
        out.append(Condition(key, args))
    return out


def with_ref(args: tuple, ref: Optional[str], pr: Optional[str]) -> tuple:
    """줄에 --ref·--pr이 없으면 공통 값을 붙인다. 둘 다 없으면 gcp_run.sh가 지금 브랜치의 PR을 찾으려 해서 막는다."""
    if '--ref' in args or '--pr' in args:
        return args
    if pr:
        return args + ('--pr', pr)
    if ref:
        return args + ('--ref', ref)
    raise ValueError(f'돌릴 코드가 없다 (--ref·--pr): {" ".join(args)}')


def read_log(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def append_log(path: Path, row: dict) -> None:
    new = not path.exists() or path.stat().st_size == 0
    with path.open('a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, LOG_HEADER)
        if new:
            writer.writeheader()
        writer.writerow({k: row.get(k, '') for k in LOG_HEADER})


def latest(log: Iterable[dict]) -> dict[str, dict]:
    """조건(key)마다 마지막 기록. 기록은 실행 시작(dispatched)과 끝(completed)을 따로 한 줄씩 쓴다."""
    last: dict[str, dict] = {}
    for row in log:
        last[row['key']] = row
    return last


def running(log: Iterable[dict]) -> dict[str, dict]:
    """아직 끝 기록이 없는 실행 (슬롯 → 기록). 디스패처가 다시 켜지면 이것부터 다시 지켜본다."""
    return {row['slot']: row for row in latest(log).values() if row['status'] == 'dispatched'}


def attempts(log: Iterable[dict], key: str) -> int:
    return sum(1 for row in log if row['key'] == key and row['status'] == 'dispatched')


def pending(conditions: list[Condition], log: list[dict], retries: int) -> list[Condition]:
    """아직 넣을 조건. 성공한 것과 돌고 있는 것은 빼고, 실패한 것은 재시도 횟수가 남았으면 다시 넣는다.

    처음 넣는 조건이 먼저, 재시도가 뒤다 (Spot 회수처럼 잠깐의 실패가 바로 반복되지 않게).
    """
    last = latest(log)
    fresh, retry = [], []
    for cond in conditions:
        row = last.get(cond.key)
        if row is None:
            fresh.append(cond)
        elif row['status'] == 'completed' and row['conclusion'] != 'success' \
                and attempts(log, cond.key) <= retries:
            retry.append(cond)
    return fresh + retry


def free_slots(slots: Iterable[str], busy: Iterable[str]) -> list[str]:
    busy = set(busy)
    return [s for s in slots if s not in busy]


def parse_slots(text: str) -> list[str]:
    slots = [s.strip() for s in text.split(',') if s.strip()]
    bad = [s for s in slots if s not in SLOTS]
    if bad or not slots:
        raise ValueError(f'슬롯은 {", ".join(SLOTS)} 중에서 쉼표로: {text}')
    if len(set(slots)) != len(slots):
        raise ValueError(f'같은 슬롯이 두 번 있다: {text}')
    kinds = {s[0] for s in slots}
    if len(kinds) > 1:
        # 지연을 재는 실험은 기계가 다르면 값이 달라진다. 한 목록은 한 종류 슬롯에서만 돌린다
        raise ValueError(f'a와 b 슬롯을 섞지 않는다 (VM 종류가 다르다): {text}')
    return slots
