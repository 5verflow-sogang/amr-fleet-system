"""실행 목록(index.csv). 하네스 실행 한 번(N회 반복)을 한 줄로 남긴다. ROS 없이 돈다.

결과 버킷의 index.csv에 줄을 더해, 어떤 조건·커밋으로 몇 번 돌렸고 결과가 어디 있는지 한눈에 본다.
"""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Iterable, Optional

from .summary import load_meta

INDEX_HEADER = ['date', 'scenario', 'label', 'delay_ms', 'loss_pct', 'kill_robot', 'kill_robot_at_s',
                'relay_auction', 'repeats', 'completed', 'git_sha', 'path', 'actions_url']


def _num(value) -> str:
    return '' if value is None else f'{float(value):g}'


def _flag(value) -> str:
    return '' if value is None else str(bool(value)).lower()


def index_row(run_dirs: Iterable[Path], path: str, actions_url: str = '',
              date: Optional[str] = None) -> list[str]:
    """반복 폴더들(같은 실행) → index.csv 한 줄. 조건은 첫 반복의 meta.json에서 읽는다."""
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
    ]


def append(index_path: Path, row: list[str]) -> None:
    """index.csv 끝에 한 줄 더한다. 파일이 없거나 비어 있으면 머리줄부터 쓴다."""
    new = not index_path.exists() or index_path.stat().st_size == 0
    with index_path.open('a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(INDEX_HEADER)
        writer.writerow(row)
