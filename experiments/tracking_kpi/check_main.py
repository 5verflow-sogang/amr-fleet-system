#!/usr/bin/env python3
"""본 측정 결과가 '돌아갔는지'만 본다. KPI 값(P1·P2·D1, 오차, 가짜 비율)은 출력하지 않는다.

    python3 -I check_main.py results/tracking_kpi_c10-main [results/...]

실행 폴더(*_00 ~ *_09)마다 한 줄. 마지막에 장면 전체 확인(실행 수, 시드 1~10, 같은 커밋).
- 로그 오류는 종료 신호(SIGINT) 전에 나온 것만 문제로 센다. 종료 단계 오류는 짧은 시험에서 모두 무해했다
"""
import json
import re
import sys
from pathlib import Path

ERROR = re.compile(r'Traceback|\bERROR\b|\[ERROR\]|Exception|terminate called|died')
# 평가 노드는 ros2 run이라 launch의 종료 신호 줄이 없다. 결과를 저장한 뒤("저장:")부터는 종료 단계로 본다
SHUTDOWN = re.compile(r"sending signal 'SIGINT'|user interrupted with ctrl-c|signal_handler\(SIGINT"
                      r"|tracking_eval\]: 저장: ")
FORWARDED = 'Passing message from ROS geometry_msgs/msg/Twist'
TRACK_HZ = 10   # 추적 주기 (LiDAR)


def early_errors(log):
    """종료 신호 전 오류 줄."""
    bad = []
    for line in log.read_text(errors='replace').splitlines():
        if SHUTDOWN.search(line):
            break
        if ERROR.search(line):
            bad.append(line.strip())
    return bad


def check_run(run):
    problems = []
    meta = json.loads((run / 'meta.json').read_text())
    if meta.get('outcome') != 'completed':
        problems.append(f"outcome {meta.get('outcome')}")
    for log in sorted((run / 'logs').glob('*.log')):
        bad = early_errors(log)
        if bad:
            problems.append(f'{log.name} 종료 전 오류 {len(bad)}줄: {bad[0][:100]}')
    # 11/1 B 시드 10: 장애물 속도 명령이 Gazebo로 한 번도 안 넘어가 장애물이 서 있었다 (브리지가 처음 넘길 때 남기는 줄)
    obstacles_log = run / 'logs' / 'obstacles.log'
    if obstacles_log.is_file() and FORWARDED not in obstacles_log.read_text(errors='replace'):
        problems.append('장애물 속도 명령이 Gazebo로 안 넘어감 (장애물이 안 움직였을 수 있음)')
    path = run / 'tracking' / 'summary.json'
    if not path.is_file():
        return meta, problems + ['tracking/summary.json 없음'], {}
    s = json.loads(path.read_text())
    inp, o = s['input'], s['overall']
    if s['frames'] <= 0 or inp['tf_fail'] or inp['frame_world']:
        problems.append(f"프레임 {s['frames']}, TF 실패 {inp['tf_fail']}, world 프레임 {inp['frame_world']}")
    # 로봇 GT가 오기 전 프레임(no_robot_gt)은 설계대로 건너뛴다. 그 밖에 처리 안 된 메시지만 문제로 본다
    if inp['msgs'] - s['frames'] - inp['no_robot_gt'] > 0:
        problems.append(f"트랙 메시지 {inp['msgs']}개 중 처리 {s['frames']}개 (로봇 GT 없던 {inp['no_robot_gt']}개 말고도 빠짐)")
    if inp['no_robot_gt'] > 10:
        problems.append(f"로봇 GT 없던 프레임 {inp['no_robot_gt']}개")
    expected = s['duration_s'] * TRACK_HZ
    if expected > 0 and s['frames'] < 0.95 * expected:
        problems.append(f"프레임 {s['frames']}개 (기록 {s['duration_s']:.0f} s × {TRACK_HZ} Hz의 95% 미만)")
    empty_gt = [g for g, n in inp['obstacle_gt_samples'].items() if n <= 0]
    if empty_gt:
        problems.append(f'GT 없음 {empty_gt}')
    if o['pred_1']['n'] <= 0 or o['vel']['n'] <= 0:
        problems.append(f"P1 표본 {o['pred_1']['n']}, P2 표본 {o['vel']['n']}")
    if s.get('false_dynamic_safety') is None:
        problems.append('가짜 감속·정지 블록 없음')
    info = {'frames': s['frames'], 'no_robot_gt': inp['no_robot_gt'], 'pairs': o['frames_matched']}
    ap = s.get('approach')
    if ap is not None:
        info['d1'] = '잼' if ap.get('ttc_s') is not None else f"못 잼 ({ap.get('note')})"
    return meta, problems, info


def main(argv):
    total_bad = 0
    for scene in map(Path, argv):
        runs = sorted(scene.glob('*/*_0[0-9]'))
        print(f'## {scene.name}: 실행 {len(runs)}개')
        seeds, shas, rtfs = [], set(), []
        for run in runs:
            meta, problems, info = check_run(run)
            seeds.append(meta.get('seed'))
            shas.add(str(meta.get('git_sha', ''))[:7])
            if isinstance(meta.get('real_time_factor'), (int, float)):
                rtfs.append(meta['real_time_factor'])
            mark = 'ok  ' if not problems else '문제'
            extra = ', '.join(f'{k} {v}' for k, v in info.items())
            print(f"  {mark} {run.name[-2:]} 시드 {meta.get('seed')} RTF {meta.get('real_time_factor')} | {extra}")
            for p in problems:
                print(f'       - {p}')
            total_bad += bool(problems)
        whole = []
        if len(runs) != 10:
            whole.append(f'실행 {len(runs)}개 (10개여야 함)')
        if sorted(seeds) != list(range(1, 11)):
            whole.append(f'시드 {sorted(seeds)}')
        if len(shas) != 1:
            whole.append(f'커밋이 여럿 {sorted(shas)}')
        rtf = f'{min(rtfs):.3f}~{max(rtfs):.3f}' if rtfs else '없음'
        print(f"  장면 전체: {'문제 ' + '; '.join(whole) if whole else 'ok'} (커밋 {', '.join(sorted(shas))}, RTF {rtf})")
        total_bad += bool(whole)
    print('모두 통과' if not total_bad else f'문제 {total_bad}건')
    return 1 if total_bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
