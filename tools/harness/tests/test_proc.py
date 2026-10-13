"""command 주행 모드의 프로세스 그룹 정리 (ROS 없이, proc.run_group). 시간 초과·정상 종료 뒤 손자가 안 남는지 본다."""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.proc import TIMEOUT, run_group  # noqa: E402

pytestmark = pytest.mark.skipif(os.name != 'posix', reason='프로세스 그룹(killpg)은 POSIX만')


def alive(marker: str) -> bool:
    return subprocess.run(['pgrep', '-f', marker], capture_output=True).returncode == 0


def test_timeout_kills_whole_group():
    # 명령이 손자(백그라운드 sleep)를 띄우고 오래 걸린다 - 측정 스크립트가 멈춘 상황
    marker = 'sleep 307.25'
    t0 = time.monotonic()
    code = run_group(f'{marker} & {marker}; wait', timeout_s=1.0)
    assert code == TIMEOUT
    assert time.monotonic() - t0 < 10.0
    assert not alive(marker), '시간 초과 뒤에도 자식·손자가 남음'


def test_normal_exit_cleans_background_grandchild():
    # 명령은 바로 끝나지만 백그라운드로 남긴 프로세스가 있다
    marker = 'sleep 308.75'
    code = run_group(f'{marker} & exit 3', timeout_s=10.0)
    assert code == 3
    assert not alive(marker)


def test_poll_called_while_waiting():
    calls = []
    assert run_group('sleep 0.5', timeout_s=10.0, poll=lambda: (calls.append(1), time.sleep(0.05))) == 0
    assert len(calls) >= 3
