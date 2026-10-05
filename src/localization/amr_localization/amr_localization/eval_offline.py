"""저장된 TUM 궤적으로 위치 오차를 다시 계산한다 (ROS 불필요).

    ros2 run amr_localization eval_offline <결과 폴더> --robot amr_01

eval_node가 강제 종료돼 궤적(원자료)만 남았거나, 구간 분류 기준을 바꿔 다시 계산할 때 쓴다.
<결과 폴더>/trajectories/<robot>_gt.tum·_est.tum을 읽어 localization.csv·localization_summary.json을 쓴다.
"""

import argparse
import sys

from .eval import EvalRecorder, format_summary


def main(argv=None):
    parser = argparse.ArgumentParser(description='TUM 궤적으로 위치 오차 재계산')
    parser.add_argument('out_dir', help='eval_node 결과 폴더 (trajectories/ 포함)')
    parser.add_argument('--robot', default='amr_01')
    parser.add_argument('--max-gap', type=float, default=0.05, help='정답 보간 허용 간격 [s]')
    parser.add_argument('--moving-v', type=float, default=0.01, help='구간 분류 기준 [m/s]')
    parser.add_argument('--moving-w', type=float, default=0.05, help='구간 분류 기준 [rad/s]')
    args = parser.parse_args(argv)
    rec = EvalRecorder.from_tum(
        args.out_dir, args.robot, max_gap=args.max_gap, moving_v=args.moving_v, moving_w=args.moving_w)
    print(format_summary(rec.save(args.out_dir)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
