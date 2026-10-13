"""자식 명령을 자기 프로세스 그룹으로 띄우고, 끝·시간 초과·예외 때 그룹째 정리한다 (ROS 불필요).

drive.py의 command 모드가 쓴다. 명령이 띄운 손자 프로세스(예: nav_measure.py)까지 같은 그룹이라
killpg 한 번으로 남김없이 끝낼 수 있다. runner는 시간 초과 때 drive.py를 SIGKILL로 죽여 정리할 기회가
없으므로, drive.py가 runner보다 먼저 끝나도록 timeout_s를 runner의 timeout_s보다 짧게 준다.
"""

import os
import signal
import subprocess
import sys
import time
from typing import Callable, Optional

TIMEOUT = -1  # run_group이 시간 초과로 끝냈을 때의 반환값


def kill_group(proc: subprocess.Popen, grace_s: float = 3.0) -> None:
    """그룹에 SIGTERM, grace_s 안에 안 끝나면 SIGKILL. 이미 끝난 그룹이면 아무것도 안 한다.

    start_new_session으로 띄웠으므로 그룹 번호는 리더 pid와 같다. 리더가 먼저 끝나 회수된 뒤에도
    (os.getpgid는 실패한다) 남은 손자에게 신호가 가도록 pid를 그대로 그룹 번호로 쓴다.
    """
    pgid = proc.pid
    for sig, wait_s in ((signal.SIGTERM, grace_s), (signal.SIGKILL, 1.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            if proc.poll() is not None and not _group_alive(pgid):
                return
            time.sleep(0.05)


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def run_group(cmd: str, timeout_s: Optional[float] = None, poll: Optional[Callable[[], None]] = None,
              stdout=None, stderr=None) -> int:
    """셸 명령을 새 세션(프로세스 그룹)으로 돌린다. 종료 코드, 시간 초과면 TIMEOUT.

    poll: 기다리는 동안 반복해서 부를 함수 (drive.py는 rclpy.spin_once로 시계를 돌린다). 없으면 0.1초씩 잔다.
    어떤 식으로 끝나든(정상·시간 초과·예외·SIGTERM) 그룹에 남은 프로세스를 정리한다.
    """
    proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', start_new_session=True,
                            stdout=stdout if stdout is not None else sys.stderr,
                            stderr=stderr if stderr is not None else sys.stderr)
    previous = signal.getsignal(signal.SIGTERM)

    def on_term(signum, frame):
        kill_group(proc)
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, on_term)
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    try:
        while proc.poll() is None:
            if deadline is not None and time.monotonic() >= deadline:
                return TIMEOUT
            if poll is not None:
                poll()
            else:
                time.sleep(0.1)
        return proc.returncode
    finally:
        # 명령 자신은 끝났어도 백그라운드로 남긴 손자가 있을 수 있다
        kill_group(proc)
        signal.signal(signal.SIGTERM, previous)
