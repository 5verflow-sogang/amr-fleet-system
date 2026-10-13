"""CPU 사용량 측정 (cpu_monitor.py). ROS 없이 돈다.

원 명세 "5대 로봇 동시 운용 시 CPU 사용률이 80% 이하" (10장 성능 요구사항)를 재려고 둔다.
사용률(%)은 실행한 컴퓨터의 코어 수에 따라 달라지므로, 기기와 덜 상관있는 "사용 코어 수"를 함께 남긴다.

- 사용 코어 수: 컨테이너 cgroup이 쓴 CPU 시간 ÷ 경과 시간. 컨테이너 안의 모든 프로세스(시뮬레이터 포함) 합
- 사용률: 사용 코어 수 ÷ 컨테이너가 쓸 수 있는 코어 수(cgroup cpu.max, 없으면 CPU 수). docker --cpus로 기준 기기를 흉내 낼 수 있다
- 호스트 사용률: /proc/stat (컨테이너 안에서도 VM 전체 값). 참고용
- 묶음별 코어 수: /proc/<pid>/stat의 utime+stime을 명령줄로 묶는다 (시뮬레이터를 뺀 값을 보고하려고)
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

CGROUP = Path('/sys/fs/cgroup')

# 명령줄에 이 문자열이 들어가면 그 묶음이다. 위에서부터 처음 맞는 묶음 (sim을 먼저 본다)
GROUPS: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    # 시뮬레이터와 그 브리지. 실제 로봇에는 없는 부분이라 따로 센다 (robot_state_publisher는 실제 로봇에도 있어 other)
    ('sim', ('ign gazebo', 'gz sim', 'gazebo', 'parameter_bridge', 'ros_gz', 'ros_ign')),
    ('nav2', ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator', 'nav2_')),
    ('localization', ('amcl', 'ekf_node', 'robot_localization', 'map_server', 'amr_odometry', 'imu_filter',
                      'scan_filter', 'wheel_odometry')),
    ('fleet', ('fleet_manager', 'fault_injector', 'amr_bt', 'task_generator', 'zone_client', 'task_auction')),
    ('perception', ('tracking_node', 'safety_node', 'detection_node', 'docking_server')),
)
GROUP_NAMES = tuple(name for name, _ in GROUPS) + ('other',)
CSV_HEADER = ['time', 'cores', 'allowed_cores', 'cpu_pct', 'host_cpu_pct'] + [f'cores_{g}' for g in GROUP_NAMES]


def classify(cmdline: str) -> str:
    """프로세스 명령줄 → 묶음 이름."""
    for name, needles in GROUPS:
        if any(n in cmdline for n in needles):
            return name
    return 'other'


def parse_cgroup_usage_us(cpu_stat: str) -> Optional[int]:
    """cgroup v2 cpu.stat의 usage_usec (마이크로초)."""
    for line in cpu_stat.splitlines():
        key, _, value = line.partition(' ')
        if key == 'usage_usec':
            return int(value)
    return None


def parse_cpu_max(cpu_max: str, cpu_count: int) -> float:
    """cgroup v2 cpu.max ("<quota> <period>" 또는 "max <period>") → 쓸 수 있는 코어 수."""
    quota, _, period = cpu_max.strip().partition(' ')
    if quota == 'max' or not period:
        return float(cpu_count)
    return int(quota) / int(period)


def parse_proc_stat(text: str) -> Tuple[int, int]:
    """/proc/stat 첫 줄(cpu 합계) → (바쁜 tick, 전체 tick). idle·iowait를 쉬는 시간으로 본다."""
    fields = [int(v) for v in text.splitlines()[0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    # guest·guest_nice는 user·nice에 이미 들어 있어 뺀다
    total = sum(fields[:8]) if len(fields) >= 8 else sum(fields)
    return total - idle, total


def parse_pid_ticks(stat: str) -> int:
    """/proc/<pid>/stat → utime + stime (tick). 프로세스 이름에 공백·괄호가 있을 수 있어 마지막 ')' 뒤를 나눈다."""
    rest = stat[stat.rindex(')') + 2:].split()
    # rest[0]이 state(3번째 필드). utime·stime은 14·15번째 필드 → rest[11]·rest[12]
    return int(rest[11]) + int(rest[12])


class Sampler:
    """이전 측정과의 차이로 구간마다 사용량을 낸다. read_* 함수를 바꿔 끼워 시험한다."""

    def __init__(self, root: Path = Path('/'), cpu_count: Optional[int] = None):
        self.root = root
        self.cpu_count = cpu_count or os.cpu_count() or 1
        self.clk_tck = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100
        self.prev_time: Optional[float] = None
        self.prev_cgroup: Optional[int] = None
        self.prev_host: Optional[Tuple[int, int]] = None
        self.prev_pids: Dict[int, int] = {}

    # --- 읽기 (테스트에서 root를 가짜 폴더로) ---
    def _read(self, rel: str) -> Optional[str]:
        try:
            return (self.root / rel).read_text()
        except OSError:
            return None

    def read_cgroup_usage_us(self) -> Optional[int]:
        text = self._read('sys/fs/cgroup/cpu.stat')
        return parse_cgroup_usage_us(text) if text else None

    def allowed_cores(self) -> float:
        text = self._read('sys/fs/cgroup/cpu.max')
        return parse_cpu_max(text, self.cpu_count) if text else float(self.cpu_count)

    def read_host(self) -> Optional[Tuple[int, int]]:
        text = self._read('proc/stat')
        return parse_proc_stat(text) if text else None

    def read_pids(self) -> Iterable[Tuple[int, str, int]]:
        """(pid, 명령줄, tick). 읽는 사이 끝난 프로세스는 건너뛴다."""
        proc = self.root / 'proc'
        for entry in proc.iterdir() if proc.is_dir() else ():
            if not entry.name.isdigit():
                continue
            try:
                cmdline = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
                ticks = parse_pid_ticks((entry / 'stat').read_text())
            except (OSError, ValueError, IndexError):
                continue
            yield int(entry.name), cmdline, ticks

    # --- 한 번 측정 ---
    def sample(self, now: float) -> Optional[dict]:
        """첫 호출은 기준만 잡고 None. 이후 호출마다 지난 호출 이후 구간의 사용량."""
        cgroup = self.read_cgroup_usage_us()
        host = self.read_host()
        pids = {pid: (cmd, ticks) for pid, cmd, ticks in self.read_pids()}
        row = None
        if self.prev_time is not None and now > self.prev_time:
            dt = now - self.prev_time
            groups = {g: 0.0 for g in GROUP_NAMES}
            for pid, (cmd, ticks) in pids.items():
                # 이번 구간에 새로 생긴 프로세스는 다음 구간부터 센다
                if pid in self.prev_pids:
                    groups[classify(cmd)] += max(ticks - self.prev_pids[pid], 0) / self.clk_tck / dt
            if cgroup is not None and self.prev_cgroup is not None:
                cores = (cgroup - self.prev_cgroup) / 1e6 / dt
            else:
                # cgroup이 없으면(컨테이너 밖) 프로세스 합으로 대신한다
                cores = sum(groups.values())
            allowed = self.allowed_cores()
            host_pct = None
            if host is not None and self.prev_host is not None and host[1] > self.prev_host[1]:
                host_pct = 100.0 * (host[0] - self.prev_host[0]) / (host[1] - self.prev_host[1])
            row = {'time': now, 'cores': cores, 'allowed_cores': allowed, 'cpu_pct': 100.0 * cores / allowed,
                   'host_cpu_pct': host_pct, **{f'cores_{g}': v for g, v in groups.items()}}
        self.prev_time, self.prev_cgroup, self.prev_host = now, cgroup, host
        self.prev_pids = {pid: ticks for pid, (_, ticks) in pids.items()}
        return row
