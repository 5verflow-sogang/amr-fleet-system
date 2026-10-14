"""같은 정책·시나리오·질량으로 2D 학습 환경을 돌려 Gazebo 결과와 짝지을 CSV를 만든다.

정책은 Gazebo와 같은 함수(amr_gazebo.rl_policies, obs dict → v, ω)다. 2D 환경 상태로 같은 obs dict를 만들고
행동을 2D 행동 공간([−1, 1]²)으로 바꿔 넣는다. 지표는 2D evaluation.run_episode 그대로다.

    python3 run_2d.py --payloads 0 10 25 --episodes 10 --policy brake --csv /tmp/rl/2d.csv

ROS Humble(Python 3.10)에서도 돌도록 payload_nav 패키지 __init__(tomllib)을 거치지 않고 모듈만 읽는다. gymnasium 필요.
"""

import argparse
import csv
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
R2_DIR = os.environ.get('AMR_RESEARCH2_DIR') or os.path.join(HERE, '..', 'research2')


def load(mod):
    name = f'payload_nav_gz.{mod}'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, os.path.join(R2_DIR, 'payload_nav', f'{mod}.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def main():
    from amr_gazebo.rl_eval import eval_seed, load_research2
    from amr_gazebo.rl_policies import load_policy

    ap = argparse.ArgumentParser()
    ap.add_argument('--payloads', nargs='+', type=float, default=[0.0, 10.0, 25.0])
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--eval-list', type=int, default=0)
    ap.add_argument('--scenarios', nargs='+', default=['S1', 'S2'])
    ap.add_argument('--policy', default='brake')
    ap.add_argument('--csv', required=True)
    a = ap.parse_args()

    load_research2(R2_DIR)            # payload_nav_gz.dynamics·scenarios
    env_mod, evaluation = load('env'), load('evaluation')
    policy = load_policy(a.policy)
    env = env_mod.PayloadNavEnv()
    p = env.cfg.robot

    class Driver:
        """evaluation.run_episode가 부르는 model.predict 자리: env 상태로 obs dict를 만들어 정책을 부른다."""

        def predict(self, _obs, deterministic=True):
            s, sc = env.state, env.scenario
            seen = [{'x': o.x, 'y': o.y, 'radius': o.radius,
                     'vx': o.vx if env.t > o.hold_until else 0.0, 'vy': o.vy if env.t > o.hold_until else 0.0}
                    for o in env.obstacles]
            obs = {'t': env.t, 'x': s.x, 'y': s.y, 'theta': s.theta, 'v': s.v, 'w': s.w,
                   'goal_x': sc.goal_distance, 'goal_y': 0.0, 'half_width': env.half_width,
                   'initial_speed': sc.initial_speed, 'payload_kg': env.payload,
                   'remaining_s': (env.cfg.max_steps - env.steps) * env.cfg.dt, 'dt': env.cfg.dt, 'obstacles': seen}
            v, w = policy(obs)
            return [2.0 * min(max(v, 0.0), p.v_max) / p.v_max - 1.0, max(-1.0, min(1.0, w / p.w_max))], None

    class Identity:
        def normalize_obs(self, o):
            return o

    os.makedirs(os.path.dirname(a.csv) or '.', exist_ok=True)
    rows = []
    for payload in a.payloads:
        for k in range(a.episodes):
            for name in a.scenarios:
                seed = eval_seed(a.eval_list, k)
                m = evaluation.run_episode(Driver(), Identity(), env, seed,
                                           options={'popout': name == 'S1', 'payload_kg': payload})
                m.pop('mass_known', None)
                rows.append({'scenario': name, 'scenario_seed': seed, 'eval_list': a.eval_list, 'episode': k,
                             'policy': a.policy, **m})
    with open(a.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'{len(rows)}개 → {a.csv}')


if __name__ == '__main__':
    main()
