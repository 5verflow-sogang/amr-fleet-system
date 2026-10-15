#!/usr/bin/env python3
"""CPU 사용량을 주기적으로 cpu.csv에 남긴다. 하네스 시나리오의 프로세스로 띄운다 (SIGINT로 끝난다). ROS 없이 돈다.

    python3 tools/harness/cpu_monitor.py --out <실행 폴더>/cpu.csv --interval 1.0

열: time, cores(컨테이너가 쓴 코어 수), allowed_cores, cpu_pct(쓸 수 있는 코어 대비 %), host_cpu_pct(VM 전체, 참고),
cores_<묶음>(sim·nav2·localization·fleet·perception·other). 계산은 harness/cpu.py.
끝날 때 같은 폴더에 cpu_procs.csv(프로세스별 평균 코어 수, 큰 순서 40개)를 남긴다.
"""

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.cpu import CSV_HEADER, Sampler  # noqa: E402


def _fmt(v):
    return '' if v is None else (f'{v:.6f}' if isinstance(v, float) else str(v))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='CPU 사용량 기록')
    ap.add_argument('--out', required=True, help='cpu.csv 경로')
    ap.add_argument('--interval', type=float, default=1.0, help='측정 주기 (s)')
    args = ap.parse_args(argv)

    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sampler = Sampler()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        f.flush()
        started = time.time()
        sampler.sample(started)
        while not stopping:
            time.sleep(args.interval)
            row = sampler.sample(time.time())
            if row:
                writer.writerow([_fmt(row[k]) for k in CSV_HEADER])
                # 하네스가 강제로 끝내도 그때까지의 행은 남게 한다
                f.flush()
    # 프로세스별 평균 코어 수 (어느 노드가 CPU를 먹는지)
    with out.with_name('cpu_procs.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['process', 'group', 'cpu_s', 'mean_cores'])
        for name, group, sec, cores in sampler.top_processes(time.time() - started):
            writer.writerow([name, group, f'{sec:.1f}', f'{cores:.3f}'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
