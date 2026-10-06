"""시나리오 1회 실행: 프로세스 기동 → 준비 확인 → 주행(또는 대기) → 정리 → 후처리 → meta.json.

ROS 환경(워크스페이스 source 완료) 안에서 돈다. rclpy는 주행 프로세스(drive.py)만 쓰고 여기서는 쓰지 않는다.
반복마다 시뮬레이터부터 전부 새로 띄워 실행끼리 상태가 섞이지 않게 한다.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

HARNESS_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = HARNESS_DIR.parents[1]


class RunError(Exception):
    """실행 실패. outcome은 meta.json에 남긴다 (error, timeout).

    startup=True는 주행 전 기동 단계 실패다. run.py가 같은 반복을 다시 시도한다.
    """

    def __init__(self, outcome: str, message: str, startup: bool = False):
        super().__init__(message)
        self.outcome = outcome
        self.startup = startup


@dataclass
class Proc:
    name: str
    cmd: str
    popen: subprocess.Popen
    log: Path
    stop_first: bool = False
    wait_files: list[str] = field(default_factory=list)


def render(value, params: dict):
    """문자열·목록·사전 안의 {이름}을 params로 채운다."""
    if isinstance(value, str):
        return value.format_map(params)
    if isinstance(value, list):
        return [render(v, params) for v in value]
    if isinstance(value, dict):
        return {k: render(v, params) for k, v in value.items()}
    return value


def start(name: str, cmd: str, log_dir: Path, **kw) -> Proc:
    """새 프로세스 그룹으로 띄운다. 정리할 때 그룹 전체에 신호를 보내 ros2 run·launch 자식까지 닿게 한다."""
    log = log_dir / f'{name}.log'
    f = log.open('w', encoding='utf-8')
    popen = subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True)
    return Proc(name, cmd, popen, log, **kw)


def stop(proc: Proc, grace_s: float = 10.0) -> None:
    """SIGINT(launch·rclpy가 정상 종료) → SIGTERM → SIGKILL 순서로 그룹을 정리한다."""
    for sig, wait in ((signal.SIGINT, grace_s), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        if proc.popen.poll() is not None:
            break
        try:
            os.killpg(proc.popen.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.popen.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    # 그룹 리더가 끝나도 남은 자식(예: gz server)이 있으면 마저 정리한다
    try:
        os.killpg(proc.popen.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def wait_topic(topic: str, timeout_s: float) -> float:
    """토픽이 보일 때까지 기다린다. 걸린 시간(s)을 돌려준다."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        out = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=30).stdout
        if topic in out.split():
            return time.monotonic() - t0
        time.sleep(1.0)
    raise RunError('error', f'토픽이 나오지 않음: {topic} ({timeout_s:.0f}s)', startup=True)


def wait_cmd(cmd: str, timeout_s: float) -> float:
    """명령이 성공(종료 코드 0)할 때까지 다시 실행한다. 예: lifecycle 노드가 active인지 확인."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, timeout=30).returncode == 0:
            return time.monotonic() - t0
        time.sleep(1.0)
    raise RunError('error', f'준비 확인 실패: {cmd} ({timeout_s:.0f}s)', startup=True)


def wait_files(paths: list[str], timeout_s: float) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if all(Path(p).exists() for p in paths):
            return
        time.sleep(0.5)
    missing = [p for p in paths if not Path(p).exists()]
    raise RunError('error', f'결과 파일이 없음: {missing}')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


class ConfigOverrides:
    """설치된 패키지 설정 파일을 실행 동안만 바꿔 끼운다 (예: ekf.yaml 후보값 비교).

    spec: '<패키지>/<share 아래 경로>=<로컬 파일>'. 끝나면 원래 파일로 되돌린다.
    """

    def __init__(self, specs: list[str]):
        self.items = []
        for spec in specs:
            target, _, source = spec.partition('=')
            pkg, _, rel = target.partition('/')
            prefix = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True).stdout.strip()
            if not prefix or not source:
                raise ValueError(f'--config 형식: <패키지>/<경로>=<파일> (지금 {spec!r})')
            self.items.append((target, Path(prefix) / 'share' / pkg / rel, Path(source).resolve()))

    def __enter__(self):
        for _, dest, source in self.items:
            backup = dest.with_name(dest.name + '.harness_orig')
            if not backup.exists():
                shutil.copy2(dest, backup)
            shutil.copy2(source, dest)
        return self

    def __exit__(self, *exc):
        for _, dest, _ in self.items:
            backup = dest.with_name(dest.name + '.harness_orig')
            if backup.exists():
                shutil.move(backup, dest)

    def meta(self) -> dict:
        return {target: {'file': str(source), 'sha256': sha256(source)} for target, _, source in self.items}


def host_info() -> dict:
    cpu = platform.processor()
    try:
        for line in Path('/proc/cpuinfo').read_text().splitlines():
            if line.startswith('model name'):
                cpu = line.split(':', 1)[1].strip()
                break
        mem_kb = next(int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                      if line.startswith('MemTotal'))
    except (OSError, StopIteration):
        mem_kb = 0
    return {'cpu': cpu, 'cores': os.cpu_count(), 'mem_gb': round(mem_kb / 1024 / 1024, 1),
            'gpu': shutil.which('nvidia-smi') is not None, 'hostname': platform.node()}


def git_sha() -> str:
    try:
        sha = subprocess.run(['git', '-C', str(REPO_DIR), 'rev-parse', '--short', 'HEAD'],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(REPO_DIR), 'status', '--porcelain', '--untracked-files=no'],
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return sha + ('-dirty' if dirty else '') if sha else 'unknown'
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown'


def fault_meta(params: dict) -> Optional[dict]:
    """연구 1 장애 조건 (README meta.json `fault`). 시나리오가 delay_ms를 쓰지 않으면 None."""
    if 'delay_ms' not in params:
        return None
    return {'delay_ms': float(params['delay_ms']), 'loss_pct': float(params.get('loss_pct', 0)),
            'direction': 'server_to_robot', 'kill_at_s': float(params.get('kill_at_s', -1)),
            'restart_after_s': float(params.get('restart_after_s', -1))}


def run_once(scenario: dict, params: dict, run_dir: Path, overrides: Optional[ConfigOverrides] = None) -> dict:
    """1회 실행하고 meta를 돌려준다. 실패해도 meta.json은 남긴다."""
    log_dir = run_dir / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    params = {**params, 'run_dir': str(run_dir)}
    meta = {
        'run_id': run_dir.name, 'scenario': scenario['name'], 'label': params.get('label', ''),
        'repeat': params['repeat'], 'seed': params['seed'], 'git_sha': git_sha(),
        'params': {k: v for k, v in params.items() if k not in ('run_dir',)},
        'num_robots': scenario.get('num_robots', 1), 'use_sim_time': scenario.get('use_sim_time', True),
        'fault': fault_meta(params), 'host': host_info(),
        'versions': {'ros_distro': os.environ.get('ROS_DISTRO', ''), 'image': os.environ.get('HARNESS_IMAGE', ''),
                     'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp (기본)'),
                     'python': sys.version.split()[0]},
        'config_overrides': overrides.meta() if overrides else {},
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }
    procs: list[Proc] = []
    t_start = time.monotonic()
    try:
        for p in render(scenario['processes'], params):
            procs.append(start(p['name'], p['cmd'], log_dir, stop_first=p.get('stop_first', False),
                               wait_files=p.get('wait_files', [])))
            if 'ready_topic' in p:
                wait_topic(p['ready_topic'], float(p.get('ready_timeout_s', 180)))
            for cmd in p.get('ready_cmds', []):
                wait_cmd(cmd, float(p.get('ready_timeout_s', 60)))
            time.sleep(float(p.get('settle_s', 0)))
            if procs[-1].popen.poll() is not None:
                raise RunError('error', f"{p['name']}이(가) 바로 끝남 (로그: {procs[-1].log})", startup=True)

        driver = render(scenario['driver'], params)
        cmd = [sys.executable, str(HARNESS_DIR / 'harness' / 'drive.py'), json.dumps(driver)]
        timeout_s = float(scenario.get('timeout_s', 600))
        with (log_dir / 'drive.log').open('w', encoding='utf-8') as f:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=f, text=True, timeout=timeout_s)
        if res.returncode != 0:
            raise RunError('error', f'주행 실패 (로그: {log_dir / "drive.log"})')
        drive = json.loads(res.stdout.strip().splitlines()[-1])
        meta['real_time_factor'] = drive.get('rtf')
        meta['drive'] = drive

        # 기록 노드(stop_first)를 먼저 끝내 결과 파일을 쓰게 한 뒤 나머지를 거꾸로 정리한다
        for p in [p for p in procs if p.stop_first]:
            stop(p)
            wait_files(p.wait_files, 30.0)
        meta['outcome'] = 'completed'
    except subprocess.TimeoutExpired:
        meta['outcome'] = 'timeout'
    except RunError as e:
        meta['outcome'] = e.outcome
        meta['error'] = str(e)
        meta['startup_failure'] = e.startup
    finally:
        for p in reversed(procs):
            stop(p)
        for cmd in scenario.get('cleanup', []):
            subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True)

    if meta['outcome'] == 'completed':
        for step in render(scenario.get('postprocess', []), params):
            with (log_dir / f"{step['name']}.log").open('w', encoding='utf-8') as f:
                res = subprocess.run(step['cmd'], shell=True, executable='/bin/bash', stdout=f,
                                     stderr=subprocess.STDOUT)
            if res.returncode != 0:
                meta['outcome'] = 'error'
                meta['error'] = f"후처리 실패: {step['name']}"
                break
    meta['duration_wall_s'] = round(time.monotonic() - t_start, 1)
    # 후처리(fleet_metrics)가 쓴 키(observation_cutoff)를 지우지 않게 합친다
    path = run_dir / 'meta.json'
    existing = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    path.write_text(json.dumps({**existing, **meta}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return meta
