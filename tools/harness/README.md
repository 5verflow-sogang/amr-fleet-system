# harness

회귀 테스트 하네스 (담당: 민지). 명세서 5-2절.

- 같은 시나리오를 N회 반복해 KPI의 평균·신뢰구간을 산출한다.
- 장애 조건(지연·손실)별 결과와 ros2_tracing 데이터를 수집한다.
- `.github/workflows/fleet-test.yml`이 GCP 러너에서 `docker_run.sh`로 이 하네스를 돌린다 (입력: 시나리오, 반복 횟수, 지연·손실, 로봇 강제종료, 라벨, 시나리오 값 `params`, 새 컨테이너 모드). 로컬 명령의 `docker_run.sh`를 `gcp_run.sh`로 바꾸면 같은 인자로 GCP에서 돈다 ([GCP README](../infra/gcp/README.md) "3. 실행").
- 로그 포맷은 명세서 1-4절 "성능 지표 측정 절차" 표를 따른다.

## 실행 (하네스 v1)

시나리오를 N회 반복하고, 실행마다 `results/<run_id>/`에 결과와 `meta.json`을 남긴 뒤 실행 단위 부트스트랩으로 요약한다. 반복마다 시뮬레이터부터 전부 새로 띄운다.

```bash
# ROS 환경(워크스페이스 source) 안에서
python3 tools/harness/run.py loc_basic -n 10
python3 tools/harness/run.py fleet_fake -n 10 --set delay_ms=200 --set loss_pct=5 --label d200_l5

# 호스트에서 컨테이너로 (필요한 패키지만 빌드하고 run.py를 부른다)
tools/harness/docker_run.sh loc_basic -n 10
HARNESS_FRESH=1 tools/harness/docker_run.sh loc_basic -n 10   # 반복마다 새 컨테이너

# GCP 러너로 (같은 인자, 지금 브랜치의 PR 코드. RTF 1.0, 결과는 버킷)
tools/harness/gcp_run.sh loc_basic -n 10
tools/harness/gcp_run.sh fleet_fake -n 10 --set delay_ms=200 --ref dev --slot b1   # 슬롯 b1 (c2d-standard-8)

# 여러 조건을 슬롯에 줄 세워 (GitHub는 같은 슬롯에 대기 실행을 1개만 남긴다. GCP README "슬롯")
caffeinate -i python3 tools/harness/gcp_batch.py conditions.txt --slots b1,b2,b3,b4 --ref dev

# 실행 목록(index.csv)에 한 줄 더하기 (fleet-test.yml이 버킷에 올리며 쓴다, ROS 불필요)
python3 tools/harness/index_run.py tools/harness/results/<out 폴더> --index index.csv --path <결과 위치>

# 이미 있는 결과 폴더를 다시 요약 (ROS 불필요)
python3 tools/harness/summarize.py tools/harness/results/loc_basic_*
```

| 옵션 | 설명 |
| --- | --- |
| `-n` | 반복 횟수. 반복 i의 `seed` = `seed_base` + i (시나리오 기본 1. 0은 여러 노드에서 "매번 무작위"라 쓰지 않는다) |
| `--label` | 조건 이름. 요약에서 같은 시나리오·라벨끼리 묶는다 |
| `--set 이름=값` | 시나리오 `params` 덮어쓰기 (지연·손실, 지도 경로 등) |
| `--config <패키지>/<share 경로>=<파일>` | 설치된 설정 파일을 실행 동안만 바꿔 끼운다 (튜닝 후보 비교). 파일 해시를 `meta.json`의 `config_overrides`에 남기고 끝나면 되돌린다 |
| `--max-attempts` | 기동 단계 실패(lifecycle 응답 유실 등) 때 같은 반복을 다시 시도하는 최대 횟수 (기본 3). 실패한 시도는 `_failN` 폴더로 남는다 |
| `HARNESS_PACKAGES` (환경 변수) | `docker_run.sh`가 빌드할 패키지(`--packages-up-to`). 기본은 `loc_basic`·`fleet_fake`에 필요한 것 |
| `--no-dds-isolation` | 아래 DDS 격리를 끈다. 지금 셸의 `ROS_DOMAIN_ID`를 그대로 쓴다 |
| `--repeat-offset`, `--stamp` | 반복을 나눠 부를 때 반복 번호·seed 시작과 결과 폴더 시각을 맞춘다. 같은 `--stamp`면 요약이 그동안의 반복을 모두 묶는다 (`HARNESS_FRESH`가 쓴다) |

**반복 사이 격리**: 한 컨테이너에서 반복하면 앞 실행의 DDS 흔적(ros2 daemon의 노드·토픽 캐시, 공유 메모리 파일, 늦게 도착한 발견 메시지)이 다음 실행에 섞일 수 있다. 그래서 하네스는 기본으로

1. 시도(재시도 포함)마다 `ROS_DOMAIN_ID`를 1~100 사이에서 바꾼다. 시뮬레이터·노드·`ros2` CLI·주행 프로세스가 모두 같은 값을 물려받고, Gazebo 통신도 `IGN_PARTITION`으로 나눈다. 0은 손으로 띄운 노드의 기본값이라 쓰지 않고, 101보다 크면 Fast DDS 포트가 리눅스 임시 포트와 겹친다.
2. 시도가 끝나면 `ros2 daemon stop`과 `fastdds shm clean`(죽은 프로세스의 공유 메모리 파일만 지움)을 한다.
3. 쓴 값은 `meta.json`의 `dds`(`ros_domain_id`, `reset`)에 남는다.

격리가 되는지 보는 기록도 `meta.json`에 남긴다. 같은 조건을 5~10번 연달아 돌려 뒤 반복으로 갈수록 나빠지는지 본다.

- `ready_s`: 실행 시작부터 프로세스마다 준비(`ready_topic`·`ready_cmds` 통과)될 때까지 걸린 시간(초). 뒤 반복에서 늘거나 기동 실패(`_failN`)가 늘면 격리 문제다.
- `leftover_processes`: 정리(프로세스 그룹 종료·`cleanup`) 뒤에도 살아 있는 ROS·Gazebo 프로세스. 비어 있어야 한다. 남은 시뮬레이터가 `/clock`을 계속 내면 다음 실행의 시계가 튀고, 같은 이름의 발행자가 겹친다.

그래도 뒤 반복으로 갈수록 결과가 달라지는지 의심되면 `HARNESS_FRESH=1`로 반복마다 컨테이너를 새로 띄워 비교한다. 빌드 결과는 이번 실행 동안만 쓰는 docker 볼륨에 두어 두 번째 컨테이너부터는 빌드가 거의 없고, 끝나면 볼륨을 지운다. Discord 요약은 마지막 반복에서 한 번만 보낸다.

**시나리오** (`scenarios/*.yaml`)

| 이름 | 내용 | 주요 결과 |
| --- | --- | --- |
| `loc_basic` | 로봇 1대, 지도 v0. 정지 → 직진 3 m → 한 바퀴 → 반 바퀴 → 복귀 직진 → 정지. 약 1분 | `eval_ekf/`·`eval_amcl/`의 `localization.csv`·궤적 |
| `kidnap` | 로봇 1대, 지도 v1. 직진 3 m 후 정지하면 Gazebo `set_pose`로 순간 이동, 복구 노드(`kidnap_recovery`)가 감지·복구. `--set mode=far`면 다른 통로로 옮긴다. 약 2분 | `kidnap/kidnap.json`(판정), `kidnap_samples.csv`(진행·옆·방향 오차), `kidnap_states.csv`, `events.csv`. 요약 `kidnap.success` 평균 = 성공률 |
| `fleet_fake` | 서버 + 장애주입기 + 가짜 로봇 5대 + 작업 생성기(60건) + rosbag. 약 11분. `kill_robot`으로 로봇 장애 | `fleet_latency.csv`, `response_time.csv`(Ack), `task_outcomes.csv`, `recovery.csv`(로봇 장애 시), `tasks.json`, `assignments.csv`, `fault_events.csv`, `robot_states.csv`, `plots/` |
| `nav_compare` | 로봇 1대, 지도 v1, Nav2 데모 + 목표 1곳. `--set controller=dwa\|teb --set scenario=clear\|obstacle`. 약 1분. `HARNESS_PACKAGES="amr_bringup amr_nav2_plugins"` 필요 | `nav_result.json`(성공·도착 시간·최소 장애물 거리·경로 길이·CTE, GT), `nav_series.csv` ([experiments/teb_compare](../../experiments/teb_compare/README.md)) |
| `nav_avoidance` (**초안, 미실행**) | 로봇 1대 DWA 데모 + 1.0 m/s로 경로를 가로지르는 작업자 1명 + 추적 노드. 약 1분. test-plan I17 | `avoidance/`의 `avoidance.csv`·`avoidance_summary.json`·GT 원자료 ([amr_safety README](../../src/perception/amr_safety/README.md#회피-평가-i17)) |

`fleet_fake`는 Fleet 계층(지연·손실·Ack)만 잰다. 가짜 로봇은 `odom`을 내지 않아 응답 시간 KPI(200 ms)는 비어 있으므로, 그 Before 데이터는 실제 로봇 스택 시나리오가 따로 필요하다. 서버 강제종료 `kill_at_s`는 서버 launch 시작 기준이라 작업 시작보다 약 10초 빠르다.

시나리오 파일은 `processes`(띄울 명령, `ready_topic`으로 준비 확인, `stop_first`·`wait_files`로 기록 노드를 먼저 끝냄), `driver`(`segments`: `cmd_vel` 구간 / `wait`: 시간만 기다림 / `command`: 셸 명령이 끝날 때까지, 0이 아닌 종료 코드면 주행 실패), `postprocess`(예: `fleet_metrics`), `cleanup`으로 이루어진다. 명령 안의 `{run_dir}`, `{seed}`, `params` 이름은 실행마다 채워진다.

- 프로세스에 `enabled: "{이름}"`을 두면 채운 값이 `false`·`0`·`no`·`off`·빈칸일 때 그 프로세스를 띄우지 않는다.
- `param_sets`는 param 하나의 값에 딸린 다른 params를 채운다. `--set`으로 직접 준 이름은 덮어쓰지 않고, 표에 없는 값이면 실행 전에 멈춘다. 값 안의 `{이름}`은 다른 params로 채운다. 표에 `_blocked: 이유`가 있으면 그 값은 실행 전에 멈춘다(선행 PR 머지 전. 로컬 스모크만 `--allow-blocked`).
  - `fleet_fake`의 `fleet_mode`(연구 1 비교군): `central`(기본, 서버만) / `decentral`(서버 없음, 로봇마다 경매 노드, 작업 생성기는 경매 노드 N개가 붙을 때까지 기다림) / `hybrid`(서버 + 경매 노드, 경매 노드에 `fleet_mode:=hybrid`). 분산·하이브리드면 장애 주입기가 경매 토픽도 중계한다(`relay_auction`). `meta.json` `fault`에 `fleet_mode`·`relay_auction`이 남는다. 분산·하이브리드는 #213·#215 머지 전까지 `_blocked`다.
  - 예: `python3 tools/harness/run.py fleet_fake -n 10 --set fleet_mode=decentral --set delay_ms=200 --label dec_d200`

**그림과 녹화** (`loc_basic`, `nav_kpi`, `fleet_fake`)

- 실행이 끝나면 `postprocess`가 `plot_run.py`로 `plots/*.png`를 남긴다. 위치추정: `loc_traj.png`(지도 위 GT·EKF·AMCL 궤적), `loc_error.png`(시간에 따른 오차, 구간 색, KPI 점선). Nav KPI: `nav_map.png`(지도 위 계획 경로·GT 궤적·성공/실패 목표), `nav_time.png`(예측·실제 주행시간). 그림이 실패해도 실행 결과는 그대로다(늘 0으로 끝남). 지난 결과도 `python3 tools/harness/plot_run.py <실행 폴더> --map <지도 yaml>`로 다시 그린다 (ROS 불필요)
- `fleet_fake`는 `fleet_metrics` 다음에 `fleet_plot`(`amr_fleet_manager`)이 `plots/timeline.png`(로봇별 작업 구간·상태 끊김·강제종료·회수된 작업의 재할당)를 남긴다. 실패 정책은 같다(늘 0으로 끝남). 지난 결과는 `PYTHONPATH=src/fleet/amr_fleet_manager python3 -m amr_fleet_manager.fleet_plot <실행 폴더>` (ROS 불필요). 가짜 로봇은 스테이션 사이를 직선으로 움직여 지도 위 궤적은 그리지 않는다
- `rosbag/`에 가벼운 토픽만 녹화한다: `/map`, `/tf`, `/tf_static`, `amcl_pose`, GT (`nav_kpi`는 `plan`·`cmd_vel` 추가). 스캔·카메라는 뺀다. Foxglove·Lichtblick에 `rosbag_0.db3`를 끌어다 놓으면 열린다. 이미지에 mcap 저장 플러그인이 없어 sqlite3로 남긴다
- 크기: `loc_basic` 1회 약 9 MB(0.025 m 지도 4 MB + GT). `nav_kpi`는 50쌍이면 수백 MB라 기본으로 끄고, 실패한 쌍을 볼 때 `--set record=true`. `--set record=false`로 끈다
- 추천 흐름: N회 돌린 뒤 `plots/`로 이상한 실행을 고르고, 그 실행의 bag을 열어 자세히 본다

**seed 규칙** (하연 제안, 10/6)

1. 하네스는 seed를 항상 직접 넘기고 1부터 시작한다. N회 반복이면 1, 2, …, N. 0은 넘기지 않는다(`--seed-base`가 1 미만이면 거부).
2. 실행 하나에 seed 하나. 그 실행의 모든 무작위 요소(작업 일정, 통신 손실, 엔코더 노이즈, 동적 장애물)에 같은 값을 넘긴다. 패키지마다 내부 난수열을 따로 만들어 서로 얽히지 않는다.
3. 0은 손으로 돌릴 때의 기본값으로 남긴다. 패키지마다 0의 뜻이 다르다(`amr_odometry`·`amr_gazebo`는 매번 무작위, Fleet 쪽은 고정값). 하네스 결과에는 0이 나오지 않는다.
4. 튜닝·예비 실험은 `--seed-base 1001`, 본 실험(보고·KPI 판정)은 1~N. 튜닝에 쓴 seed로 성능을 보고하면 결과가 좋게 치우친다(명세서 4장 연구 2와 같은 원칙).
5. `meta.json`의 `seed`에 실제로 넘긴 값을 남긴다.

seed로 묶이지 않는 무작위성도 있다. 같은 seed로 다시 돌려도 위치 오차 RMSE가 최대 2 cm 달랐다(10/6, AMCL 파티클 필터·Gazebo 센서 노이즈·메시지 도착 순서 등). 그래서 같은 seed가 "같은 조건"은 보장하지만 "같은 결과"는 보장하지 않고, 판정은 N회 반복의 신뢰구간으로 한다.

**회피 지표** (`nav_avoidance`, test-plan I17): `avoid.<로봇>.collisions`·`min_clearance`·`max_deviation`·`max_return_s`·`pass`(`avoidance_summary.json`). 원 명세 KPI는 "30회 모두 충돌 0건·이탈 1 m·복귀 5초"라 평균·CI가 아니라 최악값으로 판정해야 하므로 요약의 KPI 열은 비워 두고, 판정은 `ros2 run amr_safety avoidance_report <실행 폴더들>`로 한다.

**작업·로봇 장애 지표** (`fleet_fake`): `fleet.tasks_requested`·`tasks_completed`·`task_completion_rate`(`task_outcomes.csv`), 완료한 작업의 `fleet.task_time_s.mean`(요청~완료)·`distance_m.total`·`distance_m.mean`, `fleet.throughput_per_min`(완료 작업 수 / 첫 요청~마지막 완료 분), `fleet.duplicate_tasks`(같은 작업을 둘 이상의 로봇이 움직이며 한 수, `runners` 열. 하이브리드 중복 실행), 하이브리드 실행이면 `fleet.fallbacks`(분산 전환 횟수)·`fleet.decentral_s`(분산 모드 시간 합, `fleet_modes.csv`), 로봇 장애 실행이면 `fleet.reclaimed_tasks`·`recovery_s.mean`·`recovery_s.max`·`reclaimed_completed_rate`(`recovery.csv`). 실행 끝 무렵에 요청된 작업은 실행 안에 못 끝나므로 완료율은 같은 seed(같은 작업 일정)의 장애 있음·없음끼리 비교한다.

**요약** (`summary_<시나리오>-<라벨>_<시각>.md`): 지표마다 실행별 값의 평균·표준편차·범위와 95% 신뢰구간. KPI가 있는 지표는 CI 상한까지 기준 안이면 PASS, 평균만 기준 안이면 MARGINAL, 아니면 FAIL이다.

`nav_kpi`는 `navkpi.*`로 요약한다: 경로 계획 성공률(≥ 98 %, 클수록 좋아 CI **하한**으로 판정), 재계획 P95(≤ 500 ms), 주행시간 예측 오차 평균(≤ 15 %), CTE 직선·곡선(≤ 5·10 cm). CTE는 `controller=pp`(path_tracker) 실행만 KPI로 판정하고 DWA로 잰 값은 `navkpi.ref_cte_*`(참고)로 남는다. 실행 1회면 CI가 없어 기준 안이어도 MARGINAL로 나온다.

주의:

- 위치 오차는 로봇이 **처음 움직인 뒤부터** 센다(`eval_node`의 `skip_initial_stationary`, 기본 켬). 초기 위치를 정답으로 주므로 그 전 구간은 오차가 0이라 정지 RMSE를 낮춘다(10/6 실측: 대기 60초가 5.5 → 2.4 cm로 낮춤).
- 같은 설정·seed에서도 AMCL 파티클 무작위성 때문에 실행마다 RMSE가 최대 2 cm 다르다(10/6). 튜닝값 비교와 KPI 판정은 단일 실행이 아니라 이 요약의 CI로 한다.

**Discord 알림**: 환경 변수 `HARNESS_DISCORD_WEBHOOK`(Discord 채널 웹후크 URL)이 있으면 `run.py`가 끝날 때 요약을 그 채널로 보낸다. 없으면 보내지 않는다.

```bash
export HARNESS_DISCORD_WEBHOOK='https://discord.com/api/webhooks/...'   # 셸 설정에만 두고 커밋하지 않는다
tools/harness/docker_run.sh fleet_fake -n 10 --label d100      # 컨테이너에도 그대로 넘어간다
python3 tools/harness/summarize.py tools/harness/results/fleet_fake-d100_* --notify   # 지난 결과 다시 보내기
```

- 조건 묶음마다 embed 하나: ✅ KPI 모두 PASS, ⚠️ MARGINAL 또는 실패한 반복 있음, ❌ KPI FAIL 또는 완료 0회, ℹ️ KPI 없음. KPI 지표와 Fleet 주요 지표(지연·손실률·Ack)의 평균·95% CI를 싣고 `summary.md`를 첨부한다.
- 첫 줄에 git SHA, 호스트, 장애 조건, seed 범위를 적는다.
- 실행 폴더에 `plots/timeline.png`(Fleet 시나리오, `fleet_plot`)가 있으면 **작업 완료율이 가장 낮은 실행의 그림 한 장**을 그 조건의 embed 안에 붙이고, 어느 실행인지 적는다. 반복마다 다 보내면 채널이 그림으로 덮이므로 "어디를 열어 볼지"만 알린다. 나머지 그림은 각 실행 폴더에 있다. 그림을 붙인 전송이 실패하면(413 크기 초과 등) 그림 없이 한 번 더 보낸다.
- 보내기가 실패해도 실험 결과에는 영향이 없다 (경고만 출력).
- CI 실패 알림(`DISCORD_WEBHOOK_URL`, [workflows/README](../../.github/workflows/README.md))과 다른 채널을 쓰려고 이름을 따로 둔다.

## 로그 포맷

> 상태: 형식은 합의됐다([합의 기록](#합의-기록)). 실측 후 확정할 숫자 기준값만 "초안"으로 표시한다. 컬럼 이름과 순서는 명세서 1-4절 표를 그대로 따르고, 이 문서는 단위·파일 배치·부가 컬럼·원자료 보관만 정한다.

### 기본 원칙: 원자료를 남기고 지표는 나중에 계산한다

위치추정·내비게이션 벤치마크들은 계산된 지표보다 **가공하지 않은 원자료**(궤적, 토픽 기록)를 표준 형식으로 남기고, 지표는 오프라인 도구로 계산한다(TUM RGB-D 벤치마크와 evo, Arena-Rosnav). 이렇게 하면 다음이 가능하다.

- 구간 분류 기준이나 지표 정의가 바뀌어도 다시 실행하지 않고 재계산한다.
- 표준 도구(evo)로 ATE·RPE를 뽑아 논문에 쓰고, 직접 만든 계산기의 결과를 교차 검증한다.

그래서 실행마다 **원자료**와 명세서 형식의 **지표 CSV**를 함께 저장한다. 동료 평가용 KPI는 지표 CSV로 제출한다.

### 공통 규칙

- 형식: UTF-8 CSV, 첫 줄은 헤더. 지표 하나당 파일 하나.
- 시각: 초 단위 실수 (예: `123.456789`).
- 시간 기준: 기본은 **시뮬레이션 시간**이다(GT와 추정값을 같은 시계로 맞춰야 하므로). 단, **응답 시간과 연구 1의 지연은 실제 시간**으로 잰다(하연 합의, #5).
- 단위: 거리는 m, 각도는 rad, 지연은 ms. 컬럼 이름에 단위가 없으면 이 규칙을 따른다.
- 명세서 컬럼은 이름·순서를 바꾸지 않는다. 부가 컬럼은 **명세서 컬럼 뒤에만** 추가한다. 앞쪽 열만 잘라내면 명세서 형식이 되도록 하기 위해서다.
- 다중 로봇 시나리오는 부가 컬럼 `robot`(예: `amr_01`)을 붙인다.

### 원자료

| 파일 | 형식 | 내용 | 담당 |
| --- | --- | --- | --- |
| `trajectories/<robot>_gt.tum` | TUM 궤적 형식 | 시뮬레이터 Ground Truth 궤적 | 민지 (GT 토픽은 지수가 `amr_gazebo`에서 제공) |
| `trajectories/<robot>_est.tum` | TUM 궤적 형식 | EKF 추정 궤적 (`odometry/filtered`) | 민지 |
| `rosbag/` | rosbag2 (mcap) | 시나리오에서 정한 토픽 기록 (선택). 단 Fleet 시나리오는 `/fleet/*`, `/<robot>/fleet/*` 녹화가 필수 (`fleet_latency.csv`·Ack 컬럼의 원자료) | 시나리오 작성자, Fleet 토픽은 하연 |
| `trace/` | LTTng CTF | ros2_tracing 원본 (UST `ros2:*` 이벤트). `hop_latency.csv`의 원자료 | 하연 (수집 절차는 [runbook](../../docs/runbook.md) "트레이싱") |
| `fault_events.csv` | CSV `time, event` | 서버 강제종료·재시작 시각 (실제 시간, UNIX 초). 서버 장애 시나리오에서만 | 하연 (`fleet_manager.launch.py`의 `fault_events_file`) |
| `robot_states.csv` | CSV `time, robot, x, y, mode, task_id` | `/fleet/robot_states` 1건마다 1행 (map 좌표, bag 수신 시각). 그림의 원자료 | 하연 (`fleet_metrics`) |
| `plots/` | PNG | 보기용 그림. 판정에 쓰지 않는다. 위치추정·Nav KPI는 `plot_run.py`, Fleet은 `timeline.png` (위 "그림과 녹화") | 민지 (`plot_run.py`), 하연 (`fleet_plot`, [amr_fleet_manager README](../../src/fleet/amr_fleet_manager/README.md) "작업 타임라인 그림") |

- TUM 궤적 형식: 한 줄에 `timestamp x y z qx qy qz qw` (공백 구분, 초·m·쿼터니언). evo가 그대로 읽는다.
- **좌표계 정렬**: GT(Gazebo world)와 추정값(`map`)은 서로 다른 좌표계다. 두 좌표계 사이 변환을 시나리오 설정에 고정값으로 두고, 정렬 방식을 `meta.json`에 기록한다. 평가 도구가 임의로 최적 정렬(Umeyama 등)을 하면 오차가 실제보다 작게 나올 수 있으므로 KPI 산출에는 고정 변환만 쓴다.
- rosbag은 용량이 커서 기본으로 켜지 않는다. 켜는 경우에도 커밋하지 않는다(CONTRIBUTING 6절). Fleet 시나리오의 `/fleet/*` 녹화는 예외로 항상 켠다. 센서·지도 같은 큰 토픽이 아니라 용량이 작다.

### 지표 CSV

| 파일 | 명세서 컬럼 | 부가 컬럼 | 샘플 수 | 담당 |
| --- | --- | --- | --- | --- |
| `localization.csv` | `timestamp, gt_x, gt_y, est_x, est_y, error` | `segment` (`stationary`·`straight`·`rotation`), `yaw_error`, `robot` | 시나리오당 100개 시점 이상 | 민지 |
| `cte.csv` | `timestamp, planned_x, planned_y, actual_x, actual_y, cte` | `segment` (`straight`·`curve`), `robot` | 구간별 분리 | 이진 |
| `response_time.csv` | `cmd_time, response_time, latency_ms` | `robot`, `task_id`, `latency_sim_ms`, `assignment_id`, `ack_time`, `ack_rtt_ms`, `resends`, `recv_time`, `latency_recv_ms`, `moving_at_cmd` | 50회 이상 | 하연 |
| `zone_waits.csv` | `robot, zone, request_id, request_time, grant_time, wait_s, outcome, release_time` | — | 구역 진입 요청마다 1행 (구역 기록이 있을 때만) | 하연 (`fleet_metrics`) |
| `stalls.csv` | `robot, task_id, phase, start_time, end_time, duration_s` | — | 이동 단계에서 60 s 넘게 멈춘 구간마다 1행 (odom 기록이 있을 때만) | 하연 (`fleet_metrics`) |
| `cpu.csv` | `time, cores, allowed_cores, cpu_pct, host_cpu_pct` | `cores_<묶음>` (sim·nav2·localization·fleet·perception·other) | 1초마다 1행 | 하연 (`cpu_monitor.py`) |
| `fleet_latency.csv` | — (연구 1, 아래) | — | 메시지 1건 × 로봇 1대마다 1행 | 하연 정의, 하네스 기록기 (민지) |
| `hop_latency.csv` | — (연구 1, 아래) | — | 구독 수신 1건마다 1행 | 하연 (트레이스 변환) |
| `tasks.csv` | 명세서 작업 로그 필드 (아래) | — | 작업마다 1행 | 하네스 기록기 (민지) |
| `assignments.csv` | — (팀 정의, 아래) | — | 할당 호출마다 | 하연 (`amr_fleet_manager`) |
| `task_outcomes.csv` | — (팀 정의) | `task_id, request_time, first_assign_time, robots, final_state, end_time, task_time_s, distance_m, error_code, runners` | 작업마다 1행 | 하연 (`fleet_metrics`, bag에서 만든 Fleet 쪽 결과. 로봇 BT 작업 로그 `tasks.csv`와 별개) |
| `recovery.csv` | — (연구 1 로봇 장애) | `robot, kill_time, task_id, reassign_time, reassigned_robot, recovery_s, completed` | 로봇 강제종료 × 하던 작업마다 1행 | 하연 (`fleet_metrics`) |
| `coverage/` | `colcon test --coverage` 텍스트·HTML | 모듈별 하위 폴더 | — | 전원 |

- `localization.csv`
  - `error`는 `sqrt((gt_x-est_x)^2 + (gt_y-est_y)^2)`이다. 명세서 KPI(정지 3cm·직선 5cm·회전 8cm)는 `segment`별로 산출한다.
  - `yaw_error`는 방향 오차(rad, -π~π로 정규화)다. 명세서 KPI는 위치만 보지만 회전 구간 분석과 RPE 대조에 쓴다.
  - `segment` 분류 기준(초안, `amr_localization.eval`): 정답 궤적의 각속도가 0.05 rad/s 이상이면 `rotation`, 아니고 선속도가 0.01 m/s 이상이면 `straight`, 둘 다 아니면 `stationary`. 곡선 주행은 `rotation`으로 센다.
  - KPI 판정은 구간별 **RMSE**가 기준 이하이고 샘플이 100개 이상일 때 통과로 본다. Max는 함께 보고한다.
  - 같은 실행의 원자료로 evo ATE를 계산해 `error`의 RMSE와 일치하는지 확인한다.
- `response_time.csv` (#18 리뷰에서 도출)
  - `cmd_time`: 서버가 `/fleet/task_assignments`에 발행한 명령을 하네스 기록기가 받은 시각. 장애 주입기를 거치기 **전** 토픽을 기록하므로 주입된 지연이 응답 시간에 포함된다.
  - `response_time`: 해당 로봇의 `/<robot>/odom` 속도가 임계값(선속도 0.01 m/s 또는 각속도 0.05 rad/s, 초안)을 처음 넘은 메시지를 받은 시각.
  - `RobotState.velocity`(1~5Hz)는 쓰지 않는다. 최대 1초 오차가 생겨 200ms KPI를 잴 수 없다. `odom`은 50Hz라 오차가 20ms 이내다.
  - 두 시각 모두 **실제 시간**이다. 메시지의 `header.stamp`는 시뮬레이션 시간이라 쓰지 않고, 기록기의 수신 시각을 쓴다(rosbag2는 기본으로 실제 시간을 기록한다).
  - `latency_sim_ms`는 같은 구간을 시뮬레이션 시간으로 잰 참고값이다. `real_time_factor`와 함께 보면 시뮬레이터 속도 영향을 가를 수 있다.
  - Ack 왕복시간 (하연 제안, #33). 응답 시간과 같은 기록기가 같은 실제 시간으로 잰다.
    - `cmd_time`은 그 `assignment_id`의 **첫 발송**을 기록기가 받은 시각이다. 재전송은 `cmd_time`을 바꾸지 않는다.
    - `ack_time`: 같은 `robot`·`assignment_id`의 `TaskAssignmentAck`를 `/fleet/task_assignment_acks`에서 처음 받은 시각. `result`(수락·거부)와 상관없이 첫 Ack.
    - `ack_rtt_ms` = (`ack_time` − `cmd_time`) × 1000. 첫 발송 기준이라 재전송에 걸린 시간이 들어간다.
    - `resends`: 기록기가 같은 `assignment_id`를 받은 횟수 − 1.
    - Ack를 못 받으면 `ack_time`, `ack_rtt_ms`를 비운다. 단 실행 끝 무렵에 보낸 명령은 Ack가 오는 중에 실행이 끝날 수 있으므로, 아래 "관측 마감" 규칙으로 빼고 센다.
    - 로봇 → 서버 방향은 장애 주입을 하지 않으므로(D8, [api.md 4-4절](../../docs/api.md#4-4-쟁점-정리)) `ack_rtt_ms`에는 주입 지연이 한 번만 들어간다. 왕복 지연이 아니라 "서버 → 로봇 지연 + 되돌아오는 1홉"으로 해석한다. 역방향 주입을 추가하면 `meta.json`의 `fault.direction`으로 구분한다.
- `tasks.csv`: `/fleet/task_status`의 마지막 이벤트(COMPLETED·FAILED)를 작업 1건당 1행으로 옮긴다. 명세서가 요구하는 작업 로그 필드와 Fleet 지표(평균 작업 시간·처리량·총 이동 거리)를 여기서 계산한다.
  - 컬럼: `task_id, robot, state, start_time, end_time, duration, distance_traveled, error_code`
  - 로봇 가동률은 `/fleet/robot_states`의 `mode`가 BUSY인 시간 비율로 따로 계산한다.
- `assignments.csv`: 할당 알고리즘을 호출할 때마다 결과를 남긴다. `AssignmentResult`는 토픽으로 나가지 않으므로 `amr_fleet_manager`가 직접 기록한다(launch 인자 `assignments_csv`에 실행별 경로를 준다).
  - 컬럼: `timestamp, call_id, algorithm, cost_fn, task_id, robot_id, cost`
  - 할당 못 한 작업은 `robot_id`를 비운다. 호출별 `total_cost`는 같은 `call_id`의 `cost` 합이다.
  - 알고리즘 3종 비교(연구 1)의 할당 최적성 지표로 쓴다.

### 연구 1 지연

명세서 3-2절의 "두 레이어 P95/P99"를 두 파일로 잰다. 레이어 1은 로봇 내부 노드 간, 레이어 2는 Fleet 서버 ↔ 로봇이다.

| 파일 | 무엇을 재나 | 원자료 | 레이어 |
| --- | --- | --- | --- |
| `fleet_latency.csv` | 서버 발행 → 로봇 쪽 도착 (장애 주입 포함, 끝에서 끝) | 하네스 기록기 수신 시각 | 2 |
| `hop_latency.csv` | 토픽 한 구간씩의 DDS 전달·콜백 시간 (병목 위치) | `trace/` | 1, 2 |

- `fleet_latency.csv`: 메시지 1건 × 로봇 1대마다 1행. 장애주입기 중계 대상 4개 토픽(`task_assignments`, `heartbeat`, `zone_grants`, `task_table`)
  - 컬럼: `topic, robot, key, send_time, recv_time, latency_ms, delivered`
  - `key`: 원본과 중계본을 짝짓는 번호. `task_assignments`는 `assignment_id`, `zone_grants`는 `request_id`, `heartbeat`·`task_table`은 `epoch:seq`. 장애주입기는 이 필드를 바꾸지 않는다([amr_fault_injector](../../src/fleet/amr_fault_injector/README.md))
  - `send_time`: 기록기가 원본 `/fleet/<topic>`을 받은 시각. `recv_time`: 기록기가 중계본 `/<robot>/fleet/<topic>`을 받은 시각. 둘 다 실제 시간
  - **기록 방법**: rclpy 기록 노드를 쓰지 않고 rosbag2로 `/fleet/*`, `/<robot>/fleet/*`, `/fleet/task_assignment_acks`를 녹화한 뒤 CSV를 오프라인으로 만든다. 변환은 `ros2 run amr_fleet_manager fleet_metrics <bag> --out <실행 폴더> --delay-ms <지연>`이 한다([amr_fleet_manager README](../../src/fleet/amr_fleet_manager/README.md) "Fleet 지표 변환"). 수신 시각은 bag에 저장된 수신 시각을 쓴다. Python 노드는 부하가 걸리면 수신 시각을 수 ms 늦게 적는데, 재려는 값이 바로 그 ms 단위 지연이라 측정이 틀어진다(민지, #33). `response_time.csv`의 Ack 컬럼도 같은 bag에서 만든다
  - `latency_ms` = (`recv_time` − `send_time`) × 1000. 주입 지연 + 중계 1홉 + DDS. 주입 0ms 조건이 중계 오버헤드 기준선이다
  - `delivered`: 중계본이 오면 `1`, 안 오면 `0`(`recv_time`, `latency_ms`는 비움). 관측 시작 전·관측 마감 뒤에 보낸 메시지는 비운다(판정 안 함). 손실률은 `0`/`1`인 행만으로 센다
  - **관측 시작**: rosbag은 토픽을 찾는 대로 구독해서 녹화 초반에는 원본만 녹화되고 로봇별 중계본은 아직 구독 전일 수 있다. 주기 토픽(heartbeat, task_table)의 모든 로봇 중계본이 한 번씩 녹화된 시각 − `delay_ms`보다 먼저 보낸 메시지는 판정하지 않는다. 실행 하나에 한 시각을 쓴다(링크마다 자르면 링크마다 초반 손실이 빠져 손실률이 낮게 나온다). 10/10 GCP 첫 실행에서 손실 0% 주입에 0.8%로 나오던 것이 0%가 됐다(녹화 시작 후 1.2초, 45행 제외)
  - 같은 `key`가 여러 번 오면(재전송) 첫 도착만 쓴다
  - **관측 마감**: 실행 종료 시각에서 `delay_ms` + 여유 시간(`grace_s`, 기본 1초)을 뺀 시각 이후에 보낸 메시지는 지연·손실 계산에서 뺀다. 예를 들어 지연 0.5초 조건에서 실행이 10초에 끝나면, 9.8초에 보낸 메시지는 10.3초에 도착할 예정이라 아직 오는 중이다. 이걸 손실로 세면 지연이 클수록 손실이 많은 것처럼 나와 연구 1의 결론이 왜곡된다(민지, #33). Ack도 같은 규칙을 쓰되, 여유 시간에 서버 재전송 간격 × 최대 재전송 횟수를 더한다(`amr_fleet_manager`의 `resend.interval_s` × `resend.max_resends`, 기본 1초 × 5 = 5초). 쓴 마감 시각은 `meta.json`의 `observation_cutoff`에 남긴다
- `hop_latency.csv`: 구독 노드가 메시지를 꺼낸 1건(`ros2:rmw_take`, `taken=1`)마다 1행. `trace/`에서 오프라인으로 변환한다
  - 컬럼: `layer, node, topic, source_time, take_time, dds_ms, callback_ms`
  - `layer`: 토픽 이름이 `/fleet/*` 또는 `/<robot>/fleet/*`면 `2`, 나머지는 `1`
  - `node`, `topic`: 구독 핸들을 `rcl_node_init`·`rcl_subscription_init` 이벤트로 노드 이름·토픽에 대응시킨다
  - `source_time`: `rmw_take`의 `source_timestamp` (보낸 쪽이 DDS에 쓴 시각). `take_time`: `rmw_take` 이벤트 시각을 트레이스 시계 오프셋으로 실제 시간으로 바꾼 값
  - `dds_ms` = (`take_time` − `source_time`) × 1000. DDS 전달 + 실행기가 꺼내기까지 대기
  - `callback_ms`: 이어지는 `callback_start` ~ `callback_end`. rclpy 노드처럼 콜백 이벤트가 없으면 비운다
  - Humble의 `rmw_publish` 이벤트에는 시각 필드가 없어 발행 이벤트와 수신 이벤트를 1:1로 짝짓지 않는다. 대신 수신 쪽의 `source_timestamp`를 쓴다 (2주차 트레이스에서 필드 확인, #32)
  - 보낸 쪽과 받은 쪽이 같은 컴퓨터일 때만 맞다. 실물 네트워크 고도화처럼 컴퓨터가 나뉘면 시계 동기화(chrony 등)를 먼저 한다
- 요약 통계
  - 실행마다 그룹별로 P50·P95·P99·최대·N을 낸다. 그룹은 `fleet_latency`는 (`topic`, `robot`), `hop_latency`는 (`layer`, `topic`, `node`)
  - 반복 간에는 위 "요약 통계"와 같이 평균과 부트스트랩 95% 신뢰구간을 낸다
  - P99는 표본이 100개 미만이면 사실상 최대값이다. 실행 1회의 그룹 표본이 100개 미만이면 반복 실행을 합쳐(pool) 계산하고 그렇게 했다고 표시한다
  - 신뢰구간은 **실행 단위로** 다시 뽑는다. 부트스트랩 한 번에 실행 N개를 복원 추출하고, 뽑힌 실행의 메시지를 합쳐 분위수를 구한다. 메시지 단위로 섞어 뽑지 않는다. 같은 실행 안의 메시지는 같은 조건·같은 순간의 부하를 공유해 서로 닮아 있으므로, 메시지 1,000개를 독립 표본으로 보면 신뢰구간이 실제보다 좁게 나온다. 독립인 정보량은 실행 수에 가깝다(민지, #33)
  - 손실된 메시지(`delivered=0`)는 지연 분위수에서 빼고 손실률로 따로 보고한다

### 실행 결과 배치

```
tools/harness/results/<run_id>/
├── meta.json
├── trajectories/
│   ├── amr_01_gt.tum
│   └── amr_01_est.tum
├── localization.csv
├── cte.csv
├── response_time.csv
├── tasks.csv
├── tasks.json
├── assignments.csv
├── fleet_latency.csv
├── fault_events.csv
├── hop_latency.csv
├── trace/
└── coverage/
```

- `run_id`는 `<시나리오>_<YYYYMMDD-HHMMSS>_<반복 번호>` 형식을 쓴다.
- `tools/harness/results/`는 `.gitignore`에 등록돼 있어 커밋되지 않는다. 원자료는 공유 드라이브에 두고 경로만 기록한다(CONTRIBUTING 6절).

`meta.json`에는 결과를 나중에 추적·재현하는 데 필요한 정보를 적는다.

| 필드 | 예 | 이유 |
| --- | --- | --- |
| `git_sha` | `9ef33ea` | 어느 코드로 나온 결과인지 |
| `scenario`, `repeat`, `seed` | `loc_square_10m`, `3`, `42` | 반복·재현. Fleet 시나리오의 `seed`는 작업 생성기 seed이고, 넣은 작업 일정은 `tasks.json`(JSON Task 파일, `task_generator`의 `schedule_out`)에 남는다 |
| `fault` | `{"delay_ms": 100, "loss_pct": 0, "direction": "server_to_robot", "relay_auction": false, "kill_at_s": 60, "restart_after_s": 10, "kill_robot": "amr_03", "kill_robot_at_s": 120}` | 연구 1 조건별 비교. 서버 강제종료가 없으면 `kill_at_s`는 -1. 로봇 장애가 없으면 `kill_robot`은 `none`, `kill_robot_at_s`는 -1 (`fault_events.csv`에 `robot_kill:<로봇>`). `direction`은 지연·손실을 건 방향(`server_to_robot`, 역방향 추가 시 `both`). `relay_auction`은 분산 경매 토픽(로봇 → 로봇)에도 걸었는지 (분산·하이브리드 비교군은 `true`) |
| `num_robots` | `5` | 다중 로봇 조건 |
| `assignment_algorithm`, `cost_fn` | `hungarian`, `euclidean` | 할당 알고리즘 비교. 비용 함수가 다르면 `total_cost`끼리 비교할 수 없다 |
| `controller` | `pure_pursuit` · `stanley` | 경로 추종 제어기 비교 (이진 요청, #5) |
| `outcome` | `completed` · `timeout` · `collision` · `error` | 실패한 실행을 통계에서 구분 |
| `frame_alignment` | `fixed` | 좌표계 정렬 방식 |
| `host` | CPU 모델·코어 수, 메모리, GPU 유무 | CPU 80%·응답 시간 KPI는 실행 컴퓨터에 따라 달라짐 |
| `versions` | ROS 배포판, 시뮬레이터, Docker 이미지 태그 | 환경 차이 추적 |
| `use_sim_time`, `real_time_factor` | `true`, `0.87` | 시뮬레이터가 실제보다 느리게 돌았는지 |
| `rmw_implementation` | `rmw_cyclonedds_cpp` | 연구 1 After 측정의 DDS 벤더 비교 (하연, #33) |
| `qos_profile` | `default` | 연구 1 After 측정의 QoS 튜닝 전후 구분. 값은 [api.md 6절](../../docs/api.md#6-qos-기본값) 프로파일 이름 (하연, #33) |
| `observation_cutoff` | `{"end": 612.3, "grace_s": 1.0, "cutoff": 610.8, "start": 12.4}` | 연구 1 지연·손실·Ack에서 뺀 실행 끝 구간(하연·민지, #33)과 녹화 초반 구간(`start`) |

### 요약 통계

- 지표마다 평균·최대·N과 반복 간 **95% 신뢰구간**을 낸다. 명세서가 응답 시간에 평균과 최대를 함께 요구한다.
- 신뢰구간은 분포 가정이 필요 없는 **부트스트랩** 방식을 기본으로 한다. 반복 수가 적은 로봇 실험에서 쓰는 방식이다.
- 반복 횟수 N은 감으로 정하지 않고, 구분하고 싶은 차이 크기에서 **검정력 분석**으로 정한다(하네스 설계 과제, 명세서 8-1절).
- `outcome`이 `completed`가 아닌 실행은 지표 통계에서 빼고 실패율로 따로 보고한다.

### 합의 기록

| 항목 | 결정 | 출처 |
| --- | --- | --- |
| 응답 시간·연구 1 지연의 시간 기준 | 실제 시간 | 하연, #5 |
| 실제 시간 측정 방식 | 하네스 기록기 수신 시각 (노드 `header.stamp`는 시뮬레이션 시간 유지) | 하연, #27 |
| `response_time.csv` 부가 컬럼, `meta.json` 필드 | 설계와 맞으니 우선 진행, 바뀌면 수정 | 하연, #5 |
| `cte.csv` 부가 컬럼 | `segment`, `robot` 확정. `meta.json`에 `controller` 추가 | 이진, #5 |
| 위치 오차 KPI 판정 | 구간별 **RMSE**로 판정, Max는 함께 보고 | 하연, #28 |
| `response_time.csv` Ack 컬럼 | `assignment_id`, `ack_time`, `ack_rtt_ms`, `resends`. 첫 발송 기준이라 재전송 시간 포함 | 하연·민지, #33 |
| 응답 시간 두 기준 (10/13) | `latency_ms`는 서버 발행 → 첫 움직임(원 명세 측정 표 "작업 명령 발행", 통신 지연 포함), `latency_recv_ms`는 로봇이 받은 중계본 도착 → 첫 움직임(성능 요구사항 "명령 수신 ~ 로봇 반응"). 둘 다 보고하고 판정 기준은 측정 뒤 회의에서 | 하연 |
| 응답 시간 보조 지표 (10/15, #171 리뷰) | 판정 기준은 10/19 회의에서 정하고, 그 전에 볼 값만 더한다. `fleet.response_sim_ms`(시뮬레이션 시간, RTF < 1이면 실제 시간 값이 부푼다), `fleet.response_coverage`(움직임을 찾은 명령 ÷ 명령. 못 찾은 명령은 평균에서 빠진다), `fleet.response_moving_at_cmd`(명령 순간 이미 움직이던 경우 수. 지연이 0에 가깝고 `latency_recv_ms`는 음수도 나온다. 빼지 않고 표시만). 실행을 모두 합쳐 움직임을 찾은 명령이 50건 미만이면 응답 시간 판정을 비우고 "보류 (n=…<50)"로 적는다 | 하연·민지 |
| 교착 판정 (10/13) | 구역 허가를 30 s(서버 `release_timeout_s`) 넘게 못 받은 요청 0건(`fleet.zone_long_waits`)과 이동 단계 60 s 넘는 정체 0건(`fleet.stalls`). 정체는 구역 밖 교착도 잡는다. 수치는 측정 뒤 조정 가능 | 하연 |
| CPU 판정 (10/13) | `sys.cpu_pct.mean` ≤ 80. 컨테이너가 쓸 수 있는 코어 기준이라 기준 기기는 `docker --cpus`로 맞춘다. 기기와 덜 상관있는 `sys.cpu_cores`(사용 코어 수)와 시뮬레이터를 뺀 `sys.cpu_cores_no_sim`을 같이 보고. 작업이 끝난 뒤 쉬는 구간도 평균에 들어가므로 `sys.cpu_pct.p95`도 본다. 작업 구간만 자를지와 기준 기기는 10/19 회의. 10/28: 역할별 코어 수 `sys.cpu_cores.server`(Fleet 서버), `.robot_max`(구간마다 가장 많이 쓴 로봇 1대), `.robot_mean`, `.tool`(장애 주입기·작업 생성기·녹화 등 실험 도구)을 추가. 시뮬레이터·실험 도구를 빼고 서버와 로봇 1대를 따로 판정하려는 것. 비율은 두 가지: `sys.cpu_pct.<묶음·역할>.mean`(실행한 컴퓨터 대비), `sys.cpu_ref_pct.{server,robot_max}`(각각 4코어 컴퓨터로 본 사용률, `summary.REF_CORES`, 팀 합의 전 가정) | 하연 |
| 판정하지 않는 지표 | `fleet_deadlock`은 교착을 일부러 만드는 시나리오라 `fleet.zone_long_waits`·`fleet.stalls`를 판정하지 않는다(값만 보고). 이 시나리오의 기준("유도된 교착이 풀린 시간" 등)은 회의에서 | 하연·민지 |
| 연구 1 지연 파일 | `fleet_latency.csv`(끝에서 끝, 손실 포함), `hop_latency.csv`(구간별, `trace/`에서 변환) | 하연·민지, #33 |
| Fleet 지연 기록 방법 | rclpy 기록 노드 대신 rosbag2로 녹화 후 오프라인 변환 (Python 수신 시각 지연 방지) | 하연·민지, #33 |
| 손실 판정 | 관측 마감(종료 − 주입 지연 − 여유) 이후 보낸 메시지는 지연·손실에서 제외 | 하연·민지, #33 |
| 지연 신뢰구간 | 실행 단위 부트스트랩 (메시지 단위로 섞지 않음) | 하연·민지, #33 |
| `meta.json` 추가 필드 | `rmw_implementation`, `qos_profile`, `observation_cutoff` | 하연·민지, #33 |
| 장애 주입 방향 (D8) | 서버 → 로봇만. 로봇 → 서버(`zone_requests`, `task_assignment_acks`)는 그대로 둔다. 양방향이 필요하면 연구 1 After 실험 때 역방향 링크를 추가하고 `fault.direction`으로 구분한다. 보완(10/7): 분산·하이브리드 비교군은 경매 토픽(로봇 → 로봇)에도 같은 지연·손실을 건다(장애 주입기 `relay_auction:=true`, `fault.relay_auction`) | 하연·민지, D8 |

남은 것 (실측 후 확정)

- 응답 시간 판정 임계값(0.01 m/s·0.05 rad/s): 하네스 구현 때 정지 상태 `odom` 노이즈를 보고 확정한다.
- 위치 오차 `segment` 분류 기준(0.05 rad/s·0.01 m/s, 곡선은 `rotation`): 우리 로봇 EKF 측정 때 정답 속도 분포를 보고 확정한다.

### 참고

- Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012 — TUM 궤적 형식, ATE·RPE
- Kümmerle et al., "On Measuring the Accuracy of SLAM Algorithms", Autonomous Robots 2009 — 상대 변위 기반 평가
- Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018 — 궤적 정렬 방식과 지표의 장단점
- M. Grupp, evo — https://github.com/MichaelGrupp/evo
- Arena-Rosnav / arena-evaluation — 토픽별 CSV 기록 후 오프라인 지표 계산, 에피소드 결과 분류
- NVIDIA ros2_benchmark — 결과와 함께 실행 환경·버전·입력 해시 기록
- Colas et al., "How Many Random Seeds? Statistical Power Analysis in Deep RL Experiments", 2018 — 반복 횟수 결정, 부트스트랩 신뢰구간
