#!/usr/bin/env python3
"""손님 일과 설정(guest_routines_*.yaml) → 동적 장애물 yaml (pattern routine). 경로는 정답 지도에서 뽑는다.

    C=src/simulation/amr_gazebo/config; M=src/localization/amr_mapping/maps/restaurant_gt.yaml
    python3 src/simulation/amr_gazebo/scripts/gen_guest_routines.py $C/guest_routines_restaurant.yaml \\
        --map $M --out $C/dynamic_obstacles_restaurant_routine.yaml          # 식당 기본 손님 10명
    python3 src/simulation/amr_gazebo/scripts/gen_guest_routines.py $C/guest_routines_restaurant_peak.yaml \\
        --map $M --out $C/dynamic_obstacles_restaurant_routine_peak.yaml     # 기본 10명 + 몰림 10명 (시연 1-1·1-2)

ROS 없이 돈다 (저장소 루트에서). 지도·자리를 바꾸면 둘 다 다시 돌려 결과 파일을 같이 커밋한다 (몰림 설정이 기본 설정을
include한다).
"""

import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_gazebo.guest_routes import GridMap, build_obstacles, load_spec, path_length  # noqa: E402

HEADER = """\
# 생성 파일: {spec} 에서 scripts/gen_guest_routines.py로 만들었다 (지도 {map}). 손으로 고치지 말고 설정을 고쳐 다시 만든다.
#
# 손님 {n}명, 모두 pattern routine (amr_gazebo/obstacle_motion.py): steps를 차례로 - path를 따라 걸어 마지막 점에서 wait_s 동안
# 머물고, 마지막 단계 뒤에는 loop_from 단계부터 되풀이한다. 걷는 동안은 로봇·다른 손님 앞에서 서고, 머무는 동안 로봇이 오면
# evade_dirs 쪽으로 비켰다가 돌아온다. 손님 파일(dynamic_obstacles_file)로 띄워 모두 혼잡도에 센다.
"""


class _Flow(list):
    pass


def _flow(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('spec')
    ap.add_argument('--map', required=True, help='map_server 형식 지도 yaml')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    spec = load_spec(args.spec)
    grid = GridMap.load(args.map)
    obstacles = build_obstacles(spec, grid)

    yaml.add_representer(_Flow, _flow)
    for ob in obstacles:
        for k in ('speed_range_mps', 'yield_to_robots', 'yield_robot_size'):
            if k in ob:
                ob[k] = _Flow(ob[k])
        for st in ob['steps']:
            st['path'] = _Flow(_Flow(p) for p in st['path'])
            st['wait_s'] = _Flow(st['wait_s'])
            st['evade_dirs'] = _Flow(_Flow(d) for d in st['evade_dirs'])
        start = (ob['spawn']['x'], ob['spawn']['y'])
        here = start
        legs = []
        for st in ob['steps']:
            legs.append(f'{st["name"]} {path_length(here, st["path"]):.1f} m')
            here = st['path'][-1]
        print(f'{ob["id"]}: ' + ' → '.join(legs))

    body = yaml.dump({'obstacles': obstacles}, allow_unicode=True, sort_keys=False, width=200)
    head = HEADER.format(spec=Path(args.spec).name, map=Path(args.map).name, n=len(obstacles))
    Path(args.out).write_text(head + '\n' + body, encoding='utf-8', newline='\n')
    print(f'{args.out}: 손님 {len(obstacles)}명')


if __name__ == '__main__':
    main()
