#!/usr/bin/env python3
"""하네스 실행 한 번을 실행 목록(index.csv)에 한 줄 더한다. ROS 없이 돈다.

    python3 tools/harness/index_run.py <결과 폴더> --index index.csv --path gs://.../runs/<...>/ --url <Actions 링크>

<결과 폴더>는 run.py --out 폴더다 (그 아래 반복 폴더마다 meta.json). fleet-test.yml이 버킷에 올리며 쓴다.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.index import append, index_row  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='실행 목록(index.csv)에 한 줄 더하기')
    ap.add_argument('results', help='run.py --out 폴더')
    ap.add_argument('--index', required=True, help='더할 index.csv (없으면 만든다)')
    ap.add_argument('--path', required=True, help='결과가 올라간 위치 (예: gs://amr-fleet-results/runs/...)')
    ap.add_argument('--url', default='', help='실행 링크 (Actions)')
    ap.add_argument('--machine', default='', help='러너 VM 종류 (예: c2d-standard-8)')
    ap.add_argument('--by', default='', help='실행한 GitHub 계정')
    args = ap.parse_args(argv)
    run_dirs = [p.parent for p in Path(args.results).glob('*/meta.json')]
    row = index_row(run_dirs, args.path, args.url, machine_type=args.machine, triggered_by=args.by)
    append(Path(args.index), row)
    print(','.join(row))
    return 0


if __name__ == '__main__':
    sys.exit(main())
