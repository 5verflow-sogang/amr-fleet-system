"""coverage_table: Cobertura XML → 모듈별 표 (ROS 없이)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import coverage_table  # noqa: E402


def cobertura(source: Path, files: dict) -> str:
    """files: {상대 경로: [(줄 번호, hits)]}."""
    classes = ''.join(
        f'<class name="{name}" filename="{name}"><lines>'
        + ''.join(f'<line number="{n}" hits="{h}"/>' for n, h in lines)
        + '</lines></class>'
        for name, lines in files.items())
    return (f'<?xml version="1.0" ?><coverage><sources><source>{source}</source></sources>'
            f'<packages><package name="."><classes>{classes}</classes></package></packages></coverage>')


def make_package(tmp_path, name, files, contents, xml_name='coverage.xml'):
    src = tmp_path / 'src' / name
    for rel, text in contents.items():
        (src / rel).parent.mkdir(parents=True, exist_ok=True)
        (src / rel).write_text(text)
    build = tmp_path / 'build' / name
    build.mkdir(parents=True, exist_ok=True)
    (build / xml_name).write_text(cobertura(src, files))


def test_logic_and_node_files_are_reported_separately(tmp_path):
    make_package(tmp_path, 'pkg_a', {
        'pkg_a/core.py': [(1, 1), (2, 1), (3, 1), (4, 0)],          # 로직 75%
        'pkg_a/thing_node.py': [(1, 0), (2, 0)],                    # 노드 0%
        'test/test_core.py': [(1, 1)],                              # 테스트 코드는 뺀다
        'setup.py': [(1, 1)],
    }, {'pkg_a/core.py': 'x = 1\n', 'pkg_a/thing_node.py': 'import rclpy\n',
        'test/test_core.py': '', 'setup.py': ''})
    make_package(tmp_path, 'pkg_b', {'src/planner.cpp': [(1, 1), (2, 0), (3, 0)]},     # C++ 로직 33%
                 {'src/planner.cpp': '#include <vector>\n'}, xml_name='coverage_cpp.xml')
    packages = coverage_table.collect(tmp_path / 'build', tmp_path / 'src')
    assert [p.name for p in packages] == ['pkg_a', 'pkg_b']
    a = packages[0]
    assert sorted(str(f.path) for f in a.files) == ['pkg_a/pkg_a/core.py', 'pkg_a/pkg_a/thing_node.py']
    assert a.total(node=False) == (3, 4) and a.total(node=True) == (0, 2)
    table, overall = coverage_table.to_markdown(packages)
    assert overall == 100.0 * 4 / 7
    assert '| `pkg_a` | 50.0% | 75.0% | 0.0% | ✅ |' in table
    assert '| `pkg_b` | 33.3% | 33.3% | — | ❌ |' in table
    assert 'pkg_b/src/planner.cpp' in table          # 70% 미만 로직 파일 목록


def test_rclcpp_include_is_node_file(tmp_path):
    path = tmp_path / 'plugin.cpp'
    path.write_text('#include "rclcpp/rclcpp.hpp"\n')
    assert coverage_table.is_node_file(path)
    # Nav2 플러그인은 rclcpp를 헤더 너머로 쓴다: 플러그인 등록으로 알아본다
    path.write_text('#include "amr_nav2_plugins/dwa.hpp"\nPLUGINLIB_EXPORT_CLASS(a::B, nav2_core::Controller)\n')
    assert coverage_table.is_node_file(path)
    path.write_text('#include <vector>\n#include "amr_planning_core/dwa.hpp"\n')
    assert not coverage_table.is_node_file(path)


def test_fail_under(tmp_path, capsys):
    make_package(tmp_path, 'pkg_a', {'pkg_a/core.py': [(1, 1), (2, 0)]}, {'pkg_a/core.py': ''})
    args = ['--build', str(tmp_path / 'build'), '--src', str(tmp_path / 'src')]
    assert coverage_table.main(args) == 0
    assert coverage_table.main(args + ['--fail-under', '70']) == 1
    assert coverage_table.main(args + ['--fail-under', '50']) == 0
