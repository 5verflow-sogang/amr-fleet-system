"""CPU 사용량 측정 (harness/cpu.py, summary.cpu_metrics). 가짜 /proc·cgroup 폴더로 본다. ROS 없이 돈다."""

import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.cpu import (CSV_HEADER, Sampler, classify, parse_cgroup_usage_us, parse_cpu_max,  # noqa: E402
                         parse_pid_ticks, parse_proc_stat, proc_label, role)
from harness.summary import cpu_metrics, kpi_verdict  # noqa: E402


def test_parsers():
    assert parse_cgroup_usage_us('usage_usec 2500000\nuser_usec 2000000\n') == 2500000
    assert parse_cpu_max('max 100000\n', 16) == 16.0
    assert parse_cpu_max('800000 100000\n', 16) == 8.0
    # user nice system idle iowait irq softirq steal
    assert parse_proc_stat('cpu  100 0 50 800 50 0 0 0 0 0\ncpu0 ...') == (150, 1000)
    # 이름에 공백·괄호가 있어도 utime(14)·stime(15)을 찾는다
    stat = '42 (ign gazebo (x)) S 1 2 3 4 5 6 7 8 9 10 300 200 0 0'
    assert parse_pid_ticks(stat) == 500


@pytest.mark.parametrize('cmdline, group', [
    ('ruby /usr/bin/ign gazebo -s warehouse.sdf', 'sim'),
    ('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge /amr_01/scan', 'sim'),
    ('/opt/ros/humble/lib/nav2_controller/controller_server --ros-args', 'nav2'),
    ('/opt/ros/humble/lib/robot_localization/ekf_node', 'localization'),
    ('/ros2_ws/install/amr_bt/lib/amr_bt/amr_bt_node', 'fleet'),
    ('python3 /ros2_ws/install/amr_safety/lib/amr_safety/safety_node', 'perception'),
    ('/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', 'other'),
])
def test_classify(cmdline, group):
    assert classify(cmdline) == group


def _write_fs(root: Path, usage_us: int, host: tuple, procs: dict, cpu_max='max 100000'):
    (root / 'sys/fs/cgroup').mkdir(parents=True, exist_ok=True)
    (root / 'sys/fs/cgroup/cpu.stat').write_text(f'usage_usec {usage_us}\n')
    (root / 'sys/fs/cgroup/cpu.max').write_text(cpu_max + '\n')
    (root / 'proc').mkdir(exist_ok=True)
    busy, idle = host
    (root / 'proc/stat').write_text(f'cpu  {busy} 0 0 {idle} 0 0 0 0 0 0\n')
    for pid, (cmd, ticks) in procs.items():
        d = root / 'proc' / str(pid)
        d.mkdir(exist_ok=True)
        (d / 'cmdline').write_bytes(cmd.replace(' ', '\0').encode())
        (d / 'stat').write_text(f'{pid} (x) S 1 2 3 4 5 6 7 8 9 10 {ticks} 0 0 0')


def test_sampler_cores_and_groups(tmp_path):
    sampler = Sampler(root=tmp_path, cpu_count=16)
    sampler.clk_tck = 100
    procs = {10: ('ign gazebo -s', 0), 20: ('controller_server', 0)}
    _write_fs(tmp_path, 0, (0, 0), procs)
    assert sampler.sample(100.0) is None  # 첫 호출은 기준만

    # 2초 동안 컨테이너가 CPU 6초(= 3코어)를 씀. gazebo 2코어(400 tick), Nav2 0.5코어(100 tick)
    procs = {10: ('ign gazebo -s', 400), 20: ('controller_server', 100), 30: ('new_node', 999)}
    _write_fs(tmp_path, 6_000_000, (800, 1200), procs)
    row = sampler.sample(102.0)
    assert row['cores'] == pytest.approx(3.0)
    assert row['allowed_cores'] == 16.0
    assert row['cpu_pct'] == pytest.approx(100 * 3 / 16)
    assert row['host_cpu_pct'] == pytest.approx(40.0)
    assert row['cores_sim'] == pytest.approx(2.0)
    assert row['cores_nav2'] == pytest.approx(0.5)
    assert row['cores_other'] == 0  # 이번 구간에 새로 생긴 프로세스는 다음 구간부터
    assert set(row) == set(CSV_HEADER)


@pytest.mark.parametrize('cmdline, label', [
    ('/opt/ros/humble/lib/nav2_planner/planner_server --ros-args -r __node:=planner_server -r __ns:=/amr_01 '
     '--params-file /tmp/x.yaml', 'amr_01/planner_server'),
    ('/usr/bin/python3 /ws/lib/amr_fleet_manager/fleet_manager --ros-args -r __node:=fleet_manager', 'fleet_manager'),
    ('/usr/bin/python3 /repo/tools/harness/cpu_monitor.py --out /r/cpu.csv', 'cpu_monitor.py'),
    ('ruby /usr/bin/ign gazebo -s -r world.sdf', 'ruby'),
    ('', '?'),
])
def test_proc_label(cmdline, label):
    assert proc_label(cmdline) == label


def test_sampler_top_processes(tmp_path):
    sampler = Sampler(root=tmp_path, cpu_count=16)
    sampler.clk_tck = 100
    a = 'planner_server --ros-args -r __node:=planner_server -r __ns:=/amr_01'
    b = 'amcl --ros-args -r __node:=amcl -r __ns:=/amr_01'
    _write_fs(tmp_path, 0, (0, 0), {1: (a, 0), 2: (b, 0)})
    sampler.sample(0.0)
    _write_fs(tmp_path, 1_000_000, (0, 0), {1: (a, 300), 2: (b, 100)})  # 2초에 3 s, 1 s
    sampler.sample(2.0)
    top = sampler.top_processes(2.0)
    assert [(n, g) for n, g, _, _ in top] == [('amr_01/planner_server', 'nav2'), ('amr_01/amcl', 'localization')]
    assert top[0][2] == pytest.approx(3.0) and top[0][3] == pytest.approx(1.5)


def test_sampler_cpu_quota_is_reference_machine(tmp_path):
    # docker --cpus=8: 같은 3코어 사용도 8코어 기준이면 37.5 %
    sampler = Sampler(root=tmp_path, cpu_count=16)
    _write_fs(tmp_path, 0, (0, 0), {}, cpu_max='800000 100000')
    sampler.sample(0.0)
    _write_fs(tmp_path, 3_000_000, (1, 1), {}, cpu_max='800000 100000')
    row = sampler.sample(1.0)
    assert (row['allowed_cores'], row['cpu_pct']) == (8.0, pytest.approx(37.5))


def test_cpu_metrics_and_kpi(tmp_path):
    with (tmp_path / 'cpu.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for cores, sim in [(4.0, 2.0), (6.0, 3.0), (5.0, 2.5)]:
            w.writerow([0, cores, 8.0, 100 * cores / 8, 30.0, sim, 1.0, 0.5, 0.2, 0.1, 0.0])
    m = cpu_metrics(tmp_path)
    assert m['sys.cpu_cores.mean'] == pytest.approx(5.0)
    assert m['sys.cpu_cores.max'] == 6.0
    assert m['sys.cpu_pct.mean'] == pytest.approx(62.5)
    assert m['sys.cpu_cores_no_sim.mean'] == pytest.approx(2.5)
    assert m['sys.cpu_cores.sim.mean'] == pytest.approx(2.5)
    assert m['sys.allowed_cores'] == 8.0
    assert kpi_verdict('sys.cpu_pct.mean', 62.5, 70.0) == 'PASS'
    assert kpi_verdict('sys.cpu_pct.mean', 79.0, 85.0) == 'MARGINAL'
    assert kpi_verdict('sys.cpu_pct.mean', 81.0, 85.0) == 'FAIL'


def test_no_cpu_csv_gives_no_metrics(tmp_path):
    assert cpu_metrics(tmp_path) == {}


@pytest.mark.parametrize('cmdline, expected', [
    ('ruby /usr/bin/ign gazebo -s warehouse.sdf', 'sim'),
    ('/ws/install/amr_fleet_manager/lib/amr_fleet_manager/fleet_manager --ros-args -r __node:=fleet_manager', 'server'),
    ('/usr/bin/python3 /opt/ros/humble/bin/ros2 launch amr_fleet_manager fleet_manager.launch.py', 'other'),
    ('/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o run -e /fleet/.*', 'tool'),
    ('/ws/install/amr_fault_injector/lib/amr_fault_injector/fault_injector --ros-args', 'tool'),
    ('controller_server --ros-args -r __node:=controller_server -r __ns:=/amr_03', 'amr_03'),
    ('zone_client --ros-args -r __node:=zone_client -r __ns:=/amr_12', 'amr_12'),
    ('robot_state_publisher --ros-args', 'other'),
])
def test_role(cmdline, expected):
    assert role(cmdline) == expected


def test_sampler_roles(tmp_path):
    # 서버 0.5코어, amr_01 1.5코어(두 노드 합), amr_02 1코어, 도구 0.5코어 → 로봇 최대 1.5, 평균 1.25
    sampler = Sampler(root=tmp_path, cpu_count=16)
    sampler.clk_tck = 100
    procs = {1: 'fleet_manager --ros-args -r __node:=fleet_manager',
             2: 'planner_server --ros-args -r __node:=planner_server -r __ns:=/amr_01',
             3: 'amcl --ros-args -r __node:=amcl -r __ns:=/amr_01',
             4: 'amcl --ros-args -r __node:=amcl -r __ns:=/amr_02',
             5: 'python3 fault_injector --ros-args'}
    _write_fs(tmp_path, 0, (0, 0), {pid: (cmd, 0) for pid, cmd in procs.items()})
    sampler.sample(0.0)
    used = {1: 100, 2: 200, 3: 100, 4: 200, 5: 100}  # 2초 동안의 tick (100 tick = 1 s)
    _write_fs(tmp_path, 1_000_000, (0, 0), {pid: (cmd, used[pid]) for pid, cmd in procs.items()})
    row = sampler.sample(2.0)
    assert row['cores_server'] == pytest.approx(0.5)
    assert row['cores_tool'] == pytest.approx(0.5)
    assert row['cores_robot_max'] == pytest.approx(1.5)
    assert row['cores_robot_mean'] == pytest.approx(1.25)


def test_cpu_metrics_role_percentages(tmp_path):
    # 16코어 VM, 서버 0.2코어, 가장 바쁜 로봇 1.6코어 → VM 대비 1.25 %·10 %, 4코어 기준 5 %·40 %
    rows = [{'time': 0, 'cores': 8.0, 'allowed_cores': 16.0, 'cpu_pct': 50.0, 'host_cpu_pct': '', 'cores_nav2': 4.0,
             'cores_server': 0.2, 'cores_tool': 0.5, 'cores_robot_max': 1.6, 'cores_robot_mean': 1.2}] * 3
    with (tmp_path / 'cpu.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER)
        w.writeheader()
        w.writerows(rows)
    m = cpu_metrics(tmp_path)
    assert m['sys.cpu_pct.nav2.mean'] == pytest.approx(25.0)
    assert m['sys.cpu_pct.server.mean'] == pytest.approx(1.25)
    assert m['sys.cpu_pct.robot_max.mean'] == pytest.approx(10.0)
    assert m['sys.cpu_ref_pct.server.mean'] == pytest.approx(5.0)
    assert m['sys.cpu_ref_pct.robot_max.mean'] == pytest.approx(40.0)
    assert 'sys.cpu_ref_pct.tool.mean' not in m
