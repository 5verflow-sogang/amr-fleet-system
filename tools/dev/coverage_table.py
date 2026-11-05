#!/usr/bin/env python3
"""colcon 커버리지 결과(Cobertura XML)를 모듈별 표로 만든다. ROS 없이, 표준 라이브러리만 쓴다.

원 명세 "주요 모듈 커버리지 70% 이상, 모듈별 분리 보고"(docs/test-plan.md 3절)의 보고용이다.

    python3 tools/dev/coverage_table.py --build build --src src > coverage.md

- Python: `colcon test --pytest-with-coverage`가 만든 build/<패키지>/coverage.xml
- C++: gcovr로 만든 build/<패키지>/coverage_cpp.xml (ci.yml Coverage 단계)
- 파일마다 노드 파일과 로직 모듈로 나눈다. rclpy·rclcpp(또는 Nav2·pluginlib 플러그인)를 쓰면 노드 파일이다. 70% 판정은 로직 모듈 합으로 한다
  (노드 연결은 통합 테스트가 확인한다, test-plan.md 3절). 노드 파일도 숨기지 않고 표에 함께 보인다
- 테스트 코드(test/), setup.py, launch 파일은 뺀다
- --fail-under를 주면 로직 모듈 합이 그보다 낮을 때 1로 끝난다 (P4에 CI 판정으로 켠다)
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional

THRESHOLD = 70.0
# ROS 연결 코드: rclpy·rclcpp를 직접 쓰거나, Nav2·pluginlib 헤더를 거쳐 쓰는 플러그인 (amr_nav2_plugins)
NODE_RE = re.compile(r'^\s*(import rclpy|from rclpy\b|#include\s*[<"](rclcpp|nav2_|pluginlib)|PLUGINLIB_EXPORT_CLASS)',
                     re.M)
SKIP_PARTS = {'test', 'tests', 'launch'}
SKIP_NAMES = {'setup.py', 'conftest.py'}


@dataclass
class FileCov:
    path: Path
    covered: int
    valid: int
    node: bool


@dataclass
class PackageCov:
    name: str
    files: list[FileCov] = field(default_factory=list)

    def total(self, node: Optional[bool] = None) -> tuple[int, int]:
        sel = [f for f in self.files if node is None or f.node == node]
        return sum(f.covered for f in sel), sum(f.valid for f in sel)


def pct(covered: int, valid: int) -> Optional[float]:
    return None if valid == 0 else 100.0 * covered / valid


def is_node_file(path: Path) -> bool:
    try:
        return bool(NODE_RE.search(path.read_text(encoding='utf-8', errors='replace')))
    except OSError:
        return False


def skipped(path: Path) -> bool:
    return path.name in SKIP_NAMES or bool(SKIP_PARTS & set(path.parts)) or path.name.endswith('.launch.py')


def read_cobertura(xml_path: Path) -> list[tuple[Path, int, int]]:
    """Cobertura XML → [(파일 절대 경로, 실행된 줄, 실행 가능한 줄)]. 같은 파일이 여러 class로 나오면 합친다."""
    root = ET.parse(xml_path).getroot()
    sources = [Path(s.text.strip()) for s in root.iter('source') if s.text and s.text.strip()]
    lines: dict[Path, dict[int, bool]] = {}
    for cls in root.iter('class'):
        name = Path(cls.get('filename', ''))
        path = name if name.is_absolute() else next(
            (s / name for s in sources if (s / name).exists()), (sources[0] / name) if sources else name)
        hits = lines.setdefault(path.resolve() if path.exists() else path, {})
        for line in cls.iter('line'):
            n = int(line.get('number', 0))
            hits[n] = hits.get(n, False) or int(line.get('hits', 0)) > 0
    return [(p, sum(h.values()), len(h)) for p, h in lines.items()]


def collect(build: Path, src: Path) -> list[PackageCov]:
    packages = []
    for pkg_dir in sorted(p for p in build.iterdir() if p.is_dir()):
        xmls = [x for x in (pkg_dir / 'coverage.xml', pkg_dir / 'coverage_cpp.xml') if x.exists()]
        if not xmls:
            continue
        pkg = PackageCov(pkg_dir.name)
        for xml in xmls:
            for path, covered, valid in read_cobertura(xml):
                try:
                    rel = path.relative_to(src.resolve())
                except ValueError:
                    rel = path
                if valid == 0 or skipped(rel):
                    continue
                pkg.files.append(FileCov(rel, covered, valid, is_node_file(path)))
        if pkg.files:
            packages.append(pkg)
    return packages


def fmt(value: Optional[float]) -> str:
    return '—' if value is None else f'{value:.1f}%'


def to_markdown(packages: Iterable[PackageCov], threshold: float = THRESHOLD) -> tuple[str, Optional[float]]:
    """(표, 로직 모듈 전체 %)."""
    packages = list(packages)
    out = ['## 커버리지 (모듈별)', '',
           '70% 판정은 **로직 모듈**(ROS 연결 코드가 아닌 파일) 합으로 한다. 노드 파일은 통합 테스트가 확인한다 '
           '(docs/test-plan.md 3절).', '',
           '| 패키지 | 전체 | 로직 모듈 | 노드 파일 | 판정 (로직 ≥ 70%) |', '| --- | --- | --- | --- | --- |']
    logic_c = logic_v = 0
    for pkg in packages:
        lc, lv = pkg.total(node=False)
        logic_c, logic_v = logic_c + lc, logic_v + lv
        logic = pct(lc, lv)
        mark = '—' if logic is None else ('✅' if logic >= threshold else '❌')
        out.append(f'| `{pkg.name}` | {fmt(pct(*pkg.total()))} | {fmt(logic)} | '
                   f'{fmt(pct(*pkg.total(node=True)))} | {mark} |')
    overall = pct(logic_c, logic_v)
    out += ['', f'**로직 모듈 전체: {fmt(overall)}** ({logic_c}/{logic_v}줄)', '']
    low = sorted((f for pkg in packages for f in pkg.files if not f.node and pct(f.covered, f.valid) < threshold),
                 key=lambda f: pct(f.covered, f.valid))
    if low:
        out += ['<details><summary>70% 미만 로직 파일</summary>', '', '| 파일 | 커버리지 |', '| --- | --- |']
        out += [f'| `{f.path}` | {fmt(pct(f.covered, f.valid))} ({f.covered}/{f.valid}) |' for f in low]
        out += ['', '</details>', '']
    return '\n'.join(out) + '\n', overall


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='colcon 커버리지 → 모듈별 표 (markdown)')
    ap.add_argument('--build', type=Path, default=Path('build'), help='colcon build 폴더')
    ap.add_argument('--src', type=Path, default=Path('src'), help='패키지 소스 폴더 (파일 경로를 상대로 보일 기준)')
    ap.add_argument('--fail-under', type=float, help='로직 모듈 전체가 이보다 낮으면 1로 끝낸다')
    args = ap.parse_args(argv)
    table, overall = to_markdown(collect(args.build, args.src))
    print(table)
    if args.fail_under is not None and (overall is None or overall < args.fail_under):
        print(f'로직 모듈 커버리지 {fmt(overall)} < {args.fail_under}%', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
