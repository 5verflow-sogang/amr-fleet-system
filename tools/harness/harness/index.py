"""실행 목록(index.csv). 하네스 실행 한 번(N회 반복)을 한 줄로 남긴다. ROS 없이 돈다.

결과 버킷의 index.csv에 줄을 더해, 어떤 조건·커밋으로 몇 번 돌렸고 결과가 어디 있는지 한눈에 본다.
"""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Iterable, Optional

from .summary import load_meta

# 새 열은 끝에만 더한다. 예전 머리줄로 쓰인 파일은 append가 새 머리줄로 옮긴다
INDEX_HEADER = ['date', 'scenario', 'label', 'delay_ms', 'loss_pct', 'kill_robot', 'kill_robot_at_s',
                'relay_auction', 'repeats', 'completed', 'git_sha', 'path', 'actions_url', 'machine_type',
                'triggered_by']


def _num(value) -> str:
    return '' if value is None else f'{float(value):g}'


def _flag(value) -> str:
    return '' if value is None else str(bool(value)).lower()


def index_row(run_dirs: Iterable[Path], path: str, actions_url: str = '',
              date: Optional[str] = None, machine_type: str = '', triggered_by: str = '') -> list[str]:
    """반복 폴더들(같은 실행) → index.csv 한 줄. 조건은 첫 반복의 meta.json에서 읽는다.

    machine_type: 러너 VM 종류 (예: c2d-standard-8). 지연을 재는 실험은 기계가 다르면 값이 달라져서,
    같은 실험의 조건들이 같은 기계에서 돌았는지 이 열로 확인한다.
    """
    metas = [m for m in (load_meta(Path(d)) for d in sorted(run_dirs)) if m]
    if not metas:
        raise ValueError('meta.json이 있는 반복 폴더가 없다')
    first = metas[0]
    fault = first.get('fault') or {}
    return [
        date or time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        first.get('scenario', ''),
        first.get('label', ''),
        _num(fault.get('delay_ms')),
        _num(fault.get('loss_pct')),
        fault.get('kill_robot', ''),
        _num(fault.get('kill_robot_at_s')),
        _flag(fault.get('relay_auction')),
        str(len(metas)),
        str(sum(m.get('outcome') == 'completed' for m in metas)),
        first.get('git_sha', ''),
        path,
        actions_url,
        machine_type,
        triggered_by,
    ]


def _upgrade_row(header: list[str], row: list[str]) -> list[str]:
    """예전 파일의 한 줄 → INDEX_HEADER 순서.

    머리줄과 길이가 같은 줄은 그 머리줄의 이름으로 맞춘다. 더 긴 줄은 머리줄을 바꾸지 않은 채
    새 열을 더해 쓴 줄이라 INDEX_HEADER 앞부분 순서로 본다 (10/16 버킷 index.csv: 머리줄 10열에
    kill_robot·kill_robot_at_s·relay_auction이 들어간 13열 줄이 섞여 있었다).
    """
    if len(row) == len(header):
        named = dict(zip(header, row))
        return [named.get(k, '') for k in INDEX_HEADER]
    if len(row) <= len(INDEX_HEADER):
        return row + [''] * (len(INDEX_HEADER) - len(row))
    raise ValueError(f'열이 INDEX_HEADER보다 많은 줄: {row}')


def _migrate_header(index_path: Path) -> None:
    """예전 머리줄로 쓰인 파일을 새 머리줄로 옮긴다. 줄마다 열을 맞추고 없는 열은 빈칸으로 둔다."""
    with index_path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    header = rows[0]
    if header == INDEX_HEADER and all(len(r) == len(INDEX_HEADER) for r in rows[1:]):
        return
    unknown = [h for h in header if h not in INDEX_HEADER]
    if unknown:
        raise ValueError(f'{index_path} 머리줄에 모르는 열이 있다: {unknown}')
    upgraded = [_upgrade_row(header, row) for row in rows[1:]]
    with index_path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(INDEX_HEADER)
        writer.writerows(upgraded)


def append(index_path: Path, row: list[str]) -> None:
    """index.csv 끝에 한 줄 더한다. 파일이 없거나 비어 있으면 머리줄부터 쓴다."""
    new = not index_path.exists() or index_path.stat().st_size == 0
    if not new:
        _migrate_header(index_path)
    with index_path.open('a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(INDEX_HEADER)
        writer.writerow(row)
