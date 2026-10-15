#!/usr/bin/env python3
"""회귀 테스트 하네스 진입점. 시나리오를 N회 반복 실행하고 요약한다.

ROS 환경(워크스페이스를 source한 셸 또는 컨테이너) 안에서 실행한다.

    python3 tools/harness/run.py loc_basic -n 10
    python3 tools/harness/run.py loc_basic -n 10 --label q01 \\
        --config amr_localization/config/ekf.yaml=/tmp/ekf_q01.yaml
    python3 tools/harness/run.py fleet_fake -n 5 --set delay_ms=200 --set loss_pct=5 --label d200_l5

결과: tools/harness/results/<run_id>/ (README "실행 결과 배치"), 끝나면 summary.md를 출력한다.
"""

import argparse
import re
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness.runner import ConfigOverrides, dds_domain, dds_reset, installed_path, patch_yaml, run_once  # noqa: E402
from harness.notify import notify  # noqa: E402
from harness.summary import summarize, to_markdown  # noqa: E402

HERE = Path(__file__).resolve().parent


def load_scenario(name: str) -> dict:
    path = Path(name)
    if not path.exists():
        path = HERE / 'scenarios' / f'{name}.yaml'
    scenario = yaml.safe_load(path.read_text(encoding='utf-8'))
    if 'extends' in scenario:
        # 다른 시나리오를 그대로 쓰고 이름·params만 바꾼다 (예: loc_kpi_*는 loc_basic + 지도·원점)
        base = load_scenario(scenario.pop('extends'))
        params = {**(base.get('params') or {}), **(scenario.get('params') or {})}
        scenario = {**base, **scenario, 'params': params}
    for key in ('name', 'processes', 'driver'):
        if key not in scenario:
            raise ValueError(f'{path}: {key}가 없다')
    return scenario


def parse_set(items):
    out = {}
    for item in items or []:
        key, sep, value = item.partition('=')
        if not sep:
            raise ValueError(f'--set은 이름=값 형식 (지금 {item!r})')
        out[key] = value
    return out


def render_patch(sets: dict, params: dict) -> dict:
    """config_patch 값 안의 {이름}을 params로 채운다. 문자열이면 YAML 값으로 읽어 숫자·불도 제 타입이 되게 한다."""
    out = {}
    for key, value in sets.items():
        if isinstance(value, str) and '{' in value:
            value = yaml.safe_load(value.format_map(params))
        out[key] = value
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='시나리오 N회 반복 실행 + 실행 단위 신뢰구간')
    ap.add_argument('scenario', help='scenarios/<이름>.yaml의 이름 또는 파일 경로')
    ap.add_argument('-n', '--repeats', type=int, default=10)
    ap.add_argument('--seed-base', type=int, default=None,
                    help='반복 i의 seed = seed_base + i (기본 1 = 본 실험. 튜닝·예비 실험은 1001)')
    ap.add_argument('--label', default='', help='조건 이름. 요약에서 같은 라벨끼리 묶는다')
    ap.add_argument('--set', action='append', help='시나리오 params 덮어쓰기 (이름=값, 여러 번)')
    ap.add_argument('--config', action='append', default=[],
                    help='설치된 설정 파일을 실행 동안 바꿔 끼움: <패키지>/<share 경로>=<파일>')
    ap.add_argument('--out', default=str(HERE / 'results'))
    ap.add_argument('--max-attempts', type=int, default=3,
                    help='기동 단계에서 실패하면 같은 반복을 다시 시도하는 최대 횟수. 실패한 시도 폴더는 _failN으로 남긴다')
    ap.add_argument('--no-dds-isolation', action='store_true',
                    help='시도마다 ROS_DOMAIN_ID를 바꾸고 DDS 흔적을 지우는 것을 끈다 (지금 셸의 도메인을 그대로 씀)')
    ap.add_argument('--repeat-offset', type=int, default=0,
                    help='반복 번호·seed를 이만큼 밀어 시작한다. docker_run.sh 새 컨테이너 모드가 반복 하나씩 부를 때 쓴다')
    ap.add_argument('--stamp', default=None,
                    help='결과 폴더 이름의 시각. 같은 값으로 나눠 부르면 요약이 그동안의 반복을 모두 묶는다')
    args = ap.parse_args(argv)

    scenario = load_scenario(args.scenario)
    params = {**(scenario.get('params') or {}), **parse_set(args.set), 'label': args.label}
    seed_base = args.seed_base if args.seed_base is not None else int(scenario.get('seed_base', 1))
    if seed_base < 1:
        # 0은 패키지마다 뜻이 다르다(매번 무작위 / 고정값). 하네스는 0을 넘기지 않는다 (README "seed 규칙")
        ap.error('--seed-base는 1 이상이어야 한다 (튜닝 실험은 1001부터)')
    stamp = args.stamp or time.strftime('%Y%m%d-%H%M%S')
    out = Path(args.out)
    tag = f"{scenario['name']}{'-' + args.label if args.label else ''}"
    isolate = not args.no_dds_isolation
    if isolate:
        dds_reset()  # 이전 작업이 남긴 흔적부터 지우고 시작한다
    attempts = args.repeat_offset
    # 시나리오 config_patch: 설치된 설정 파일에서 몇 값만 바꾼 사본을 실행 동안 끼운다 (--config와 같은 방식, meta에 해시)
    configs = list(args.config)
    # 값에 {이름}을 쓰면 시나리오 params로 채운 뒤 YAML 값으로 읽는다 (예: count: "{task_count}" → 120)
    for target, sets in (scenario.get('config_patch') or {}).items():
        dest = out / f'_config_{tag}_{stamp}' / Path(target).name
        patch_yaml(installed_path(target), render_patch(sets, params), dest)
        configs.append(f'{target}={dest}')
    with ConfigOverrides(configs) as overrides:
        for i in range(args.repeat_offset, args.repeat_offset + args.repeats):
            run_dir = out / f'{tag}_{stamp}_{i:02d}'
            run_params = {**params, 'repeat': i, 'seed': seed_base + i}
            for attempt in range(1, args.max_attempts + 1):
                # 재시도도 새 도메인에서 한다. 앞 시도의 늦은 메시지·발견 정보가 섞이지 않게
                domain = dds_domain(attempts) if isolate else None
                attempts += 1
                meta = run_once(scenario, run_params, run_dir, overrides, domain_id=domain, reset_dds=isolate)
                if not meta.get('startup_failure') or attempt == args.max_attempts:
                    break
                # 기동 실패(예: lifecycle 전환 응답 유실)는 시도 폴더를 남기고 같은 seed로 다시 한다
                print(f"  기동 실패, 다시 시도 ({attempt}/{args.max_attempts}): {meta.get('error')}", flush=True)
                run_dir.rename(run_dir.with_name(f'{run_dir.name}_fail{attempt}'))
            print(f"[{i + 1}/{args.repeat_offset + args.repeats}] {run_dir.name}: {meta['outcome']} "
                  f"(RTF {meta.get('real_time_factor')}, {meta['duration_wall_s']}s)", flush=True)

    # 같은 stamp로 나눠 부른 반복(새 컨테이너 모드)까지 묶는다. _failN 시도 폴더는 뺀다
    pattern = re.compile(rf'{re.escape(tag)}_{re.escape(stamp)}_\d+')
    run_dirs = sorted(d for d in out.glob(f'{tag}_{stamp}_*') if d.is_dir() and pattern.fullmatch(d.name))
    summary = summarize(run_dirs)
    summary_md = to_markdown(summary)
    path = out / f'summary_{tag}_{stamp}.md'
    path.write_text(summary_md, encoding='utf-8')
    print(summary_md)
    print(f'요약: {path}')
    # HARNESS_DISCORD_WEBHOOK이 있을 때만 보낸다 (README "Discord 알림")
    notify(summary, run_dirs, path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
