#!/usr/bin/env python3
"""GCP 하네스 결과를 버킷에서 받고, 파일마다 버킷의 md5·크기와 대조한다.

    python3 fetch_runs.py <gcp_batch 기록 CSV> --out <폴더> [--with-rosbag] [--only tracking_kpi_c10 ...]

- 기록(<conditions>.log.csv)에서 조건마다 마지막으로 성공한 실행만 고른다 (재시도 전 실패 실행은 받지 않는다)
- gs://amr-fleet-results/runs/<시나리오>-<라벨>/<날짜>_<run_id>/ 를 <out>/<시나리오>-<라벨>/<날짜>_<run_id>/ 로 받는다
- 녹화(rosbag/)는 기본으로 뺀다. 판정에 쓰지 않고, gcloud가 큰 파일 끝에서 죽거나(TypeError) 크기는 같은데
  내용이 깨진 파일을 남긴 것이 녹화에서만 있었다 (10/30 짧은 시험)
- 받은 뒤 모든 파일을 대조한다. 없거나 다르면 `gcloud storage cat`으로 하나씩 다시 받아 다시 대조한다
- 다시 돌리면 이미 맞는 파일은 그대로 둔다
"""
import argparse
import base64
import csv
import hashlib
import os
import subprocess
import sys
from pathlib import Path

BUCKET = 'gs://amr-fleet-results/runs'
TMP_SUFFIX = '_.gstmp'   # gcloud가 받다가 죽으면 남기는 임시 파일


def final_runs(log_path):
    """조건 → (시나리오, 라벨, run_id). 마지막으로 성공한 실행.
    성공이 없는 조건은 아직 도는 것(running)과 끝내 실패한 것(missing)으로 나눠 돌려준다."""
    last_ok, seen, done, started = {}, [], set(), {}
    with open(log_path, newline='') as f:
        for row in csv.DictReader(f):
            if row['key'] not in seen:
                seen.append(row['key'])
            started.setdefault(row['key'], set()).add(row['run_id'])
            if row['status'] == 'completed':
                done.add(row['run_id'])
                if row['conclusion'] == 'success':
                    last_ok[row['key']] = row['run_id']
    runs, missing, running = [], [], []
    for key in seen:
        if key not in last_ok:
            (running if started[key] - done else missing).append(key)
            continue
        args = key.split()
        label = args[args.index('--label') + 1] if '--label' in args else ''
        runs.append((args[0], label, last_ok[key]))
    return runs, missing, running


def gcloud(args, **kw):
    return subprocess.run(['gcloud', 'storage'] + args, capture_output=True, text=True, **kw)


def find_folder(scenario, label, run_id):
    parent = f'{BUCKET}/{scenario}-{label}/'
    r = gcloud(['ls', parent])
    hits = [line.strip() for line in r.stdout.splitlines() if line.strip().endswith(f'_{run_id}/')]
    if len(hits) != 1:
        raise RuntimeError(f'{parent}에서 run {run_id} 폴더를 {len(hits)}개 찾음: {r.stderr.strip()[-200:]}')
    return hits[0]


def list_objects(folder):
    """폴더 아래 객체 → [(상대 경로, md5 base64, 크기)]."""
    r = gcloud(['objects', 'list', folder + '**', '--format=value(name,md5_hash,size)'])
    if r.returncode != 0:
        raise RuntimeError(f'{folder} 목록 실패: {r.stderr.strip()[-200:]}')
    prefix = folder[len('gs://'):].split('/', 1)[1]   # 버킷 이름 뒤 경로
    out = []
    for line in r.stdout.splitlines():
        name, md5, size = line.split('\t')
        out.append((name[len(prefix):], md5, int(size)))
    return out


def md5_b64(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return base64.b64encode(h.digest()).decode()


def matches(path, md5, size):
    return path.is_file() and path.stat().st_size == size and md5_b64(path) == md5


def refetch(url, path, md5, size):
    """한 파일을 흘려 받아(cat) 맞으면 제자리에 둔다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    with open(part, 'wb') as f:
        r = subprocess.run(['gcloud', 'storage', 'cat', url], stdout=f, stderr=subprocess.PIPE)
    if r.returncode == 0 and matches(part, md5, size):
        os.replace(part, path)
        return True
    part.unlink(missing_ok=True)
    return False


def fetch_run(folder, dest, with_rosbag):
    objects = [o for o in list_objects(folder) if with_rosbag or 'rosbag' not in o[0].split('/')[:-1]]
    todo = [o for o in objects if not matches(dest / o[0], o[1], o[2])]
    if todo:
        # 한 번에 받는다 (파일마다 gcloud를 띄우면 느리다). 죽어도 아래 대조가 잡는다
        dest.mkdir(parents=True, exist_ok=True)
        rsync = ['rsync', '--recursive', folder, str(dest)]
        if not with_rosbag:
            rsync += ['--exclude', r'(.*/)?rosbag/.*']
        gcloud(rsync)
    stats = {'files': len(objects), 'ok': 0, 'refetched': 0, 'failed': []}
    for rel, md5, size in objects:
        path = dest / rel
        if matches(path, md5, size):
            stats['ok'] += 1
        elif refetch(folder + rel, path, md5, size):
            stats['refetched'] += 1
        else:
            stats['failed'].append(rel)
    for tmp in dest.rglob('*' + TMP_SUFFIX):
        tmp.unlink()   # 죽은 gcloud가 남긴 임시 파일 (진짜 파일은 위에서 대조를 마쳤다)
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(description='GCP 하네스 결과 내려받기 + md5 대조')
    ap.add_argument('log', type=Path, help='gcp_batch.py 기록 CSV (<conditions>.log.csv)')
    ap.add_argument('--out', type=Path, required=True, help='받을 폴더')
    ap.add_argument('--with-rosbag', action='store_true', help='녹화(rosbag/)도 받는다')
    ap.add_argument('--only', nargs='+', help='이 시나리오만 (예: tracking_kpi_c10)')
    a = ap.parse_args(argv)

    runs, missing, running = final_runs(a.log)
    if a.only:
        runs = [r for r in runs if r[0] in a.only]
    bad = 0
    for scenario, label, run_id in runs:
        try:
            folder = find_folder(scenario, label, run_id)
            dest = a.out / f'{scenario}-{label}' / folder.rstrip('/').rsplit('/', 1)[1]
            s = fetch_run(folder, dest, a.with_rosbag)
        except RuntimeError as e:
            print(f'실패  {scenario}-{label} run {run_id}: {e}')
            bad += 1
            continue
        mark = '맞음' if not s['failed'] else '실패'
        print(f"{mark}  {scenario}-{label} run {run_id}: 파일 {s['files']}개, 대조 맞음 {s['ok']}, "
              f"다시 받아 맞음 {s['refetched']}, 실패 {len(s['failed'])} {s['failed'][:3] if s['failed'] else ''}")
        bad += bool(s['failed'])
    for key in running:
        print(f'진행 중  {key}')
    for key in missing:
        print(f'성공 없음  {key}')
    print(f'끝: 실행 {len(runs)}개 중 문제 {bad}개, 진행 중 {len(running)}개, 성공 없는 조건 {len(missing)}개. '
          f'폴더 {a.out}')
    return 1 if bad or missing else 0


if __name__ == '__main__':
    sys.exit(main())
