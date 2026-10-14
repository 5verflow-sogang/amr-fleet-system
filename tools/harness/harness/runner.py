"""시나리오 1회 실행: 프로세스 기동 → 준비 확인 → 주행(또는 대기) → 정리 → 후처리 → meta.json.

ROS 환경(워크스페이스 source 완료) 안에서 돈다. rclpy는 주행 프로세스(drive.py)만 쓰고 여기서는 쓰지 않는다.
반복마다 시뮬레이터부터 전부 새로 띄워 실행끼리 상태가 섞이지 않게 한다.
DDS도 반복마다 떼어 놓는다: 시도마다 ROS_DOMAIN_ID를 바꾸고, 끝나면 ros2 daemon과 Fast DDS 공유 메모리 찌꺼기를 지운다.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import re
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


# Fast DDS 포트는 7400 + 250 × 도메인이라 리눅스 임시 포트(32768~)와 겹치지 않는 0~101만 안전하다.
# 0은 손으로 띄운 노드의 기본값이라 피하고 1~100을 돈다
DOMAIN_MIN, DOMAIN_COUNT = 1, 100


def dds_domain(attempt_index: int) -> int:
    """시도 번호(재시도 포함 0부터)마다 다른 ROS_DOMAIN_ID. 100회마다 돌아온다."""
    return DOMAIN_MIN + attempt_index % DOMAIN_COUNT


def dds_reset() -> None:
    """지난 시도의 DDS 흔적을 지운다: 현재 도메인의 ros2 daemon(토픽 목록 캐시)과 죽은 프로세스의 공유 메모리 파일.

    `fastdds shm clean`은 아직 쓰는 파일은 남기므로 같은 컨테이너에 다른 노드가 떠 있어도 안전하다.
    """
    for cmd in (['ros2', 'daemon', 'stop'], ['fastdds', 'shm', 'clean']):
        try:
            subprocess.run(cmd, capture_output=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            pass


# 정리 뒤에도 남으면 다음 실행과 섞이는 프로세스 (시뮬레이터·브리지·ROS 노드).
# ros2 daemon은 뺀다: dds_reset이 멈추라고 한 뒤 잠깐 남아 있고, 도메인별로 따로 떠서 다음 실행과 섞이지 않는다.
# 좀비(<defunct>)도 뺀다: 이미 끝나 아무것도 안 하고, 거둘 부모가 없어 남은 것뿐이다 (docker_run.sh는 --init으로 거둔다)
LEFTOVER_RE = re.compile(r'\b(ign|gz|gazebo|ruby|parameter_bridge|ros2)\b|/opt/ros/|/install/')
LEFTOVER_IGNORE_RE = re.compile(r'ros2cli\.daemon|<defunct>')


def leftover_processes() -> list[str]:
    """정리가 끝난 뒤에도 살아 있는 ROS·Gazebo 프로세스의 명령줄. 하네스 자신과 조상은 뺀다."""
    try:
        out = subprocess.run(['ps', '-eo', 'pid=,ppid=,args='], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    rows = {}
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[0].isdigit():
            rows[int(parts[0])] = (int(parts[1]), parts[2])
    mine, pid = set(), os.getpid()
    while pid in rows and pid not in mine:
        mine.add(pid)
        pid = rows[pid][0]
    return [args[:200] for pid, (ppid, args) in sorted(rows.items())
            if pid not in mine and not args.startswith('ps -eo') and LEFTOVER_RE.search(args)
            and not LEFTOVER_IGNORE_RE.search(args)]


CLI_TIMEOUT_S = 30.0


def run_cli(cmd, timeout_s: float = CLI_TIMEOUT_S) -> tuple[Optional[int], str]:
    """준비 확인용 명령을 자기 프로세스 그룹으로 돌린다. (종료 코드, stdout). 시간 초과면 (None, '').

    shell=True로 띄운 명령이 멈추면 subprocess.run의 timeout은 셸만 죽이고 그 아래 ros2 CLI는 남아 재시도마다 쌓인다
    (#148 하연 리뷰). 그래서 새 세션으로 띄우고 시간 초과면 그룹째 죽인다.
    """
    shell = isinstance(cmd, str)
    proc = subprocess.Popen(cmd, shell=shell, executable='/bin/bash' if shell else None, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, start_new_session=True)
    try:
        out, _ = proc.communicate(timeout=timeout_s)
        return proc.returncode, out
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.communicate()
        return None, ''


def _warn_cli_timeout(what: str, warned: list) -> None:
    # CLI가 DDS 발견을 못 끝내 멈춘 것이다. 실행 전체 시간 초과로 번지지 않게 "아직 준비 안 됨"으로 보고 다시 확인한다.
    # 같은 대기에서 한 번만 알린다
    if not warned:
        print(f'  경고: 준비 확인 명령이 {CLI_TIMEOUT_S:.0f}s 응답 없음, 다시 확인한다: {what}', flush=True)
        warned.append(True)


def wait_topic(topic: str, timeout_s: float, proc: Optional[Proc] = None) -> float:
    """토픽이 보일 때까지 기다린다. 걸린 시간(s)을 돌려준다. 기다리는 동안 proc이 끝나면 바로 실패한다."""
    t0 = time.monotonic()
    warned = []
    while time.monotonic() - t0 < timeout_s:
        if proc is not None and proc.popen.poll() is not None:
            raise RunError('error', f'{proc.name}이(가) 끝남 (로그: {proc.log})', startup=True)
        code, out = run_cli(['ros2', 'topic', 'list'])
        if code is None:
            _warn_cli_timeout('ros2 topic list', warned)
        if topic in out.split():
            return time.monotonic() - t0
        time.sleep(1.0)
    raise RunError('error', f'토픽이 나오지 않음: {topic} ({timeout_s:.0f}s)', startup=True)


def wait_cmd(cmd: str, timeout_s: float) -> float:
    """명령이 성공(종료 코드 0)할 때까지 다시 실행한다. 예: lifecycle 노드가 active인지 확인."""
    t0 = time.monotonic()
    warned = []
    while time.monotonic() - t0 < timeout_s:
        code, _ = run_cli(cmd)
        if code == 0:
            return time.monotonic() - t0
        if code is None:
            _warn_cli_timeout(cmd, warned)
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


def patch_yaml(source: Path, sets: dict, dest: Path) -> Path:
    """source YAML에서 sets의 값만 바꿔 dest에 쓴다. 키는 점으로 이은 경로 (예: controller_server.ros__parameters.DWA.max_v).

    시나리오 config_patch가 쓴다: 설정 파일을 통째로 복사해 두면 원본이 바뀔 때 어긋나므로 바꿀 값만 적는다.
    """
    import yaml
    data = yaml.safe_load(source.read_text(encoding='utf-8'))
    for key, value in sets.items():
        node, parts = data, key.split('.')
        for part in parts[:-1]:
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f'{source.name}에 {key}가 없다 ({part})')
            node = node[part]
        if parts[-1] not in node:
            raise KeyError(f'{source.name}에 {key}가 없다')
        node[parts[-1]] = value
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return dest


def installed_path(target: str) -> Path:
    """'<패키지>/<share 아래 경로>' → 설치된 파일 경로."""
    pkg, _, rel = target.partition('/')
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True).stdout.strip()
    if not prefix:
        raise ValueError(f'패키지를 찾지 못함: {pkg}')
    return Path(prefix) / 'share' / pkg / rel


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
            'restart_after_s': float(params.get('restart_after_s', -1)),
            # 로봇 장애 (fake_robots.launch.py kill_robot). 없으면 none, -1
            'kill_robot': str(params.get('kill_robot', 'none')),
            'kill_robot_at_s': float(params.get('kill_robot_at_s', -1))}


def run_once(scenario: dict, params: dict, run_dir: Path, overrides: Optional[ConfigOverrides] = None,
             domain_id: Optional[int] = None, reset_dds: bool = True) -> dict:
    """1회 실행하고 meta를 돌려준다. 실패해도 meta.json은 남긴다.

    domain_id가 있으면 이 실행의 모든 자식 프로세스(시뮬레이터·노드·ros2 CLI·drive.py)가 그 도메인을 쓴다.
    reset_dds면 끝난 뒤 DDS 흔적을 지운다(dds_reset).
    """
    if domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        # Gazebo(ign-transport)는 DDS와 따로 찾으므로 파티션도 나눈다
        os.environ['IGN_PARTITION'] = os.environ['GZ_PARTITION'] = f'harness_{domain_id}'
    log_dir = run_dir / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    params = {**params, 'run_dir': str(run_dir)}
    meta = {
        'run_id': run_dir.name, 'scenario': scenario['name'], 'label': params.get('label', ''),
        'repeat': params['repeat'], 'seed': params['seed'], 'git_sha': git_sha(),
        'params': {k: v for k, v in params.items() if k not in ('run_dir',)},
        # --set num_robots로 대수를 바꾸는 시나리오(fleet_fake)는 params 값이 실제 대수다
        'num_robots': int(params.get('num_robots', scenario.get('num_robots', 1))),
        'use_sim_time': scenario.get('use_sim_time', True),
        'fault': fault_meta(params), 'host': host_info(),
        'versions': {'ros_distro': os.environ.get('ROS_DISTRO', ''), 'image': os.environ.get('HARNESS_IMAGE', ''),
                     'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp (기본)'),
                     'python': sys.version.split()[0]},
        'dds': {'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID') or 0), 'reset': reset_dds},
        'config_overrides': overrides.meta() if overrides else {},
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }
    procs: list[Proc] = []
    t_start = time.monotonic()
    # 실행 시작부터 각 프로세스가 준비(ready_topic·ready_cmds 통과)될 때까지 걸린 시간. 뒤 반복에서 늘어나면 격리 문제다
    meta['ready_s'] = {}
    try:
        for p in render(scenario['processes'], params):
            procs.append(start(p['name'], p['cmd'], log_dir, stop_first=p.get('stop_first', False),
                               wait_files=p.get('wait_files', [])))
            if 'ready_topic' in p:
                wait_topic(p['ready_topic'], float(p.get('ready_timeout_s', 180)), procs[-1])
            for cmd in p.get('ready_cmds', []):
                wait_cmd(cmd, float(p.get('ready_timeout_s', 60)))
            meta['ready_s'][p['name']] = round(time.monotonic() - t_start, 1)
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
        if reset_dds:
            dds_reset()
        meta['leftover_processes'] = leftover_processes()
        if meta['leftover_processes']:
            print(f"  경고: 정리 뒤에도 남은 프로세스 {len(meta['leftover_processes'])}개 (meta.json leftover_processes)",
                  flush=True)

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
