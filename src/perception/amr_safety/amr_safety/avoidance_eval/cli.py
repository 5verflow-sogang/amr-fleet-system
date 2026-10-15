"""회피 평가 명령줄 도구. ROS 없이 돈다 (`python3 -m amr_safety.avoidance_eval.cli ...`로도 부른다).

    # 기록 폴더 다시 계산 (기준값을 바꿔 볼 때)
    avoidance_eval_offline <기록 폴더> [--set return_deviation=0.15 ...]

    # 여러 실행 → I17 판정 (30회 모두 충돌 0, 최대 이탈 ≤ 1 m, 복귀 ≤ 5 s)
    avoidance_report tools/harness/results/nav_avoidance_* [--out report.md] [--json report.json]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import fields, replace
from pathlib import Path
from typing import Optional

from .metrics import AvoidanceParams, aggregate, aggregate_markdown, format_summary
from .recorder import load_recorder, params_from_dict

SUMMARY = 'avoidance_summary.json'


def _parse_sets(base: AvoidanceParams, sets: list[str]) -> AvoidanceParams:
    types = {f.name: f.type for f in fields(AvoidanceParams)}
    changes = {}
    for s in sets:
        name, _, value = s.partition('=')
        if name not in types:
            raise SystemExit(f'모르는 기준값: {name} (가능: {", ".join(sorted(types))})')
        default = getattr(base, name)
        if isinstance(default, bool):
            changes[name] = value.lower() in ('1', 'true', 'yes')
        elif isinstance(default, int):
            changes[name] = int(value)
        elif isinstance(default, float):
            changes[name] = float(value)
        else:
            changes[name] = value
    return replace(base, **changes)


def offline_main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description='회피 평가 기록 폴더를 다시 계산한다')
    ap.add_argument('dirs', nargs='+', help='avoidance_config.json이 있는 기록 폴더')
    ap.add_argument('--set', action='append', default=[], metavar='이름=값', help='판정 기준값 덮어쓰기')
    ap.add_argument('--out', help='결과 폴더 (기본: 기록 폴더에 덮어쓴다)')
    args = ap.parse_args(argv)
    for d in args.dirs:
        with open(os.path.join(d, 'avoidance_config.json'), encoding='utf-8') as f:
            base = params_from_dict(json.load(f).get('params', {}))
        rec = load_recorder(d, _parse_sets(base, args.set))
        out = args.out if args.out and len(args.dirs) == 1 else (
            os.path.join(args.out, os.path.basename(os.path.normpath(d))) if args.out else d)
        summary = rec.save(out)
        print(f'{d}: {format_summary(summary)}')
    return 0


def find_summaries(run_dirs: list[str]) -> tuple[list[tuple[str, Optional[dict]]],
                                                 list[tuple[str, str, Optional[dict]]]]:
    """하네스 실행 폴더(또는 기록 폴더)마다 요약을 찾는다.

    meta.json outcome이 completed가 아니면 실패 목록 (실행, outcome, 요약 또는 None)으로 따로 낸다.
    실패 실행도 요약이 있으면 읽는다. 그 안의 충돌은 판정에 들어간다 (aggregate).
    """
    runs, failed = [], []
    for d in sorted(Path(x) for x in run_dirs):
        if not d.is_dir():
            continue
        outcome = 'completed'
        meta_path = d / 'meta.json'
        if meta_path.exists():
            outcome = str(json.loads(meta_path.read_text(encoding='utf-8')).get('outcome'))
        paths = [d / SUMMARY] if (d / SUMMARY).exists() else sorted(d.glob(f'**/{SUMMARY}'))
        found = [(d.name if p.parent == d else f'{d.name}/{p.parent.relative_to(d)}',
                  json.loads(p.read_text(encoding='utf-8'))) for p in paths]
        if outcome != 'completed':
            failed.extend((name, outcome, s) for name, s in found)
            if not found:
                failed.append((d.name, outcome, None))
        elif found:
            runs.extend(found)
        else:
            runs.append((d.name, None))
    return runs, failed


def report_main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description='여러 실행의 회피 평가를 모아 I17 KPI를 판정한다')
    ap.add_argument('dirs', nargs='+', help='하네스 실행 폴더 또는 기록 폴더')
    ap.add_argument('--runs', type=int, default=AvoidanceParams.kpi_runs, help='KPI 실행 횟수 (기본 30)')
    ap.add_argument('--out', help='Markdown 표 저장 경로 (없으면 화면에만)')
    ap.add_argument('--json', help='판정 JSON 저장 경로')
    args = ap.parse_args(argv)
    runs, failed = find_summaries(args.dirs)
    # KPI 기준값은 첫 요약의 params를 쓴다 (실행마다 같은 설정이어야 한다)
    base = next((params_from_dict(s.get('params', {})) for _, s in runs if s), AvoidanceParams())
    result = aggregate(runs, replace(base, kpi_runs=args.runs), failed)
    md = aggregate_markdown(result)
    print(md)
    if args.out:
        Path(args.out).write_text(md, encoding='utf-8')
    if args.json:
        Path(args.json).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0 if result['verdict'] == 'PASS' else 1


def main(argv: Optional[list[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ('offline', 'report'):
        print('사용법: python3 -m amr_safety.avoidance_eval.cli {offline|report} ...', file=sys.stderr)
        return 2
    return (offline_main if argv[0] == 'offline' else report_main)(argv[1:])


if __name__ == '__main__':
    sys.exit(main())
