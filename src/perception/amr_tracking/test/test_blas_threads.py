"""numpy(OpenBLAS) 계산 스레드를 1개로 두는지 (amr_tracking/__init__.py).

노드 모듈을 새 프로세스에서 불러오고 큰 행렬 곱을 한 번 한 뒤, 환경 변수 값과 프로세스 스레드 수를 본다.
OpenBLAS는 보조 스레드를 불러올 때가 아니라 처음 큰 계산을 할 때 만들기 때문에 곱을 먼저 한다. 값이 numpy보다
늦게 정해지면 CPU 수만큼 보조 스레드가 생겨, 값을 처음부터 1로 준 프로세스보다 스레드가 많다.
"""

import os
import subprocess
import sys

PROBE = ('import os, amr_tracking.tracking_node, numpy; '
         'a = numpy.ones((400, 400)); a @ a; '
         "print(os.environ['OPENBLAS_NUM_THREADS'], len(os.listdir('/proc/self/task')))")


def _probe(**env_override):
    """(OPENBLAS_NUM_THREADS 값, 스레드 수). 이 프로세스에 걸린 값은 지우고 env_override만 준다."""
    env = {k: v for k, v in os.environ.items() if k != 'OPENBLAS_NUM_THREADS'}
    env.update(env_override)
    out = subprocess.run([sys.executable, '-c', PROBE], env=env, capture_output=True, text=True,
                         check=True)
    value, threads = out.stdout.split()
    return value, int(threads)


def test_single_thread_by_default():
    value, threads = _probe()
    assert value == '1'
    assert threads == _probe(OPENBLAS_NUM_THREADS='1')[1]


def test_keeps_explicit_value():
    assert _probe(OPENBLAS_NUM_THREADS='2')[0] == '2'
