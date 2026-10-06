#!/usr/bin/env python3
"""실행 폴더들을 다시 요약한다. ROS 없이 돈다.

    python3 tools/harness/summarize.py tools/harness/results/loc_basic_*
    python3 tools/harness/summarize.py tools/harness/results/* --json summary.json

같은 시나리오·라벨(meta.json)끼리 묶어 지표별 평균·95% 신뢰구간(실행 단위 부트스트랩)을 낸다.
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.summary import summarize, to_markdown  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='하네스 실행 폴더 요약')
    ap.add_argument('runs', nargs='+', help='results/<run_id> 폴더들')
    ap.add_argument('--json', help='요약을 JSON으로도 저장')
    args = ap.parse_args(argv)
    summary = summarize(Path(r) for r in args.runs if Path(r).is_dir())
    print(to_markdown(summary))
    if args.json:
        Path(args.json).write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
