"""속도별 급정지 결과(rl_eval_node mode:=brake_sweep)를 요약하고, 모델(2D)보다 더 간 거리를 원인별로 나눈다 (#144 리뷰 1).

2D(dynamics.step)는 명령 순간부터 a = 2τ/(r·(M + m))로 감속한다. Gazebo와 다른 점 두 가지를 따로 잰다.

- 바퀴 관성: 토크가 바퀴 회전도 멈춰야 해서 감속이 a·(M + m)/(M + m + 2·I/r²)로 약해진다
  (바퀴 원통 1.5 kg·r 0.1 m → I/r² = 0.75 kg씩). 몫 = v²/2a' − v²/2a, v²에 비례
- 나머지 = 차 − 관성 몫. 명령 지연이면 v에 비례한다 → 나머지 / v = 실효 지연
  (brake_metrics dead_time_s는 감속이 걸리기 시작하는 구간까지 섞여 이보다 조금 크다)

    python3 brake_sweep.py brake.csv
"""

import csv
import statistics as st
import sys

# amr_description properties.xacro, research2 dynamics.RobotParams
ROBOT_MASS = 28.9
WHEEL_MASS = 1.5
WHEEL_INERTIA_MASS = 2 * WHEEL_MASS / 2   # 바퀴 2개, 원통 I = m·r²/2 → I/r² = m/2


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    print('| 질량 kg | 명령 속도 m/s | 횟수 | 제동 거리 Gazebo m | 모델 v²/2a m | 차 m (평균 [최소~최대]) '
          '| 감속 Gazebo / 바퀴 관성 넣은 모델 / 모델 m/s² | 관성 몫 m | 나머지 m | 실효 지연 s (나머지 / v) |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for payload in sorted({num(r['payload_kg']) for r in rows}):
        sel = [r for r in rows if num(r['payload_kg']) == payload]
        a = num(sel[0]['decel_model'])
        mass = ROBOT_MASS + payload
        a_inertia = a * mass / (mass + WHEEL_INERTIA_MASS)
        for v in sorted({num(r['v_target']) for r in sel}):
            g = [r for r in sel if num(r['v_target']) == v]
            vc = st.mean(num(r['v_cmd']) for r in g)
            ex = [num(r['excess']) for r in g]
            dec = [num(r['decel_fit']) for r in g if num(r['decel_fit']) is not None]
            inertia_part = vc * vc / (2 * a_inertia) - vc * vc / (2 * a)
            rest = st.mean(ex) - inertia_part
            print(f'| {payload:g} | {vc:.3f} | {len(g)} | '
                  f'{st.mean(num(r["stop_dist"]) for r in g):.3f} | {st.mean(num(r["model_dist"]) for r in g):.3f} | '
                  f'{st.mean(ex):+.3f} [{min(ex):+.3f}~{max(ex):+.3f}] | '
                  f'{st.mean(dec) if dec else float("nan"):.2f} / {a_inertia:.2f} / {a:.2f} | '
                  f'{inertia_part:+.3f} | {rest:+.3f} | {rest / vc:.3f} |')


if __name__ == '__main__':
    main(sys.argv[1])
