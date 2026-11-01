# 추적 정확도 KPI 본 측정

원 명세 평가 항목 "칼만 필터 기반 객체 추적이 구현되고, 예측 정확도가 적절한가?"를 재는 본 측정의 조건·받기·검사·표·분석이다. 판정 기준과 결과는 [perception.md 1-7](../../docs/algorithms/perception.md)에 있다. 측정 도구(평가 노드 `tracking_eval`, 하네스 시나리오 `tracking_kpi_*`, 요약 `track.*`)는 #251이다.

## 순서

결과는 `raw/`(git 제외)에 받는다. 저장소 루트에서 돌린다.

```bash
# 1. GCP에 장면 7개 × 10회를 넣는다 (Mac이면 nohup caffeinate -i …로 세션과 떼어 띄운다)
python3 tools/harness/gcp_batch.py experiments/tracking_kpi/conditions/tracking_main.txt --slots a --pr 251

# 2. 받고 파일마다 버킷 md5와 대조한다 (녹화는 뺀다. 필요하면 --with-rosbag). B 다시 측정 기록도 받는다
for log in tracking_main tracking_main_b_rerun; do
  python3 -I experiments/tracking_kpi/fetch_runs.py experiments/tracking_kpi/conditions/$log.txt.log.csv \
      --out experiments/tracking_kpi/raw/results
done

# 3. 실행이 제대로 돌았는지 검사한다 (perception.md 1-7 "무효 실행" 기준, KPI 값은 출력하지 않는다)
#    첫 B 묶음(tracking_kpi_b-main)의 시드 10이 걸려 "문제 1건"이 나오는 것이 맞다
python3 -I experiments/tracking_kpi/check_main.py experiments/tracking_kpi/raw/results/tracking_kpi_*-main*

# 4. 판정에 쓰는 실행만 묶어 하네스 요약을 다시 낸다 (B는 main_rerun, 나머지는 main)
R=experiments/tracking_kpi/raw/results
python3 tools/harness/summarize.py $R/tracking_kpi_{a,c05,c10,c15,d,e}-main/*/*_0[0-9] \
    $R/tracking_kpi_b-main_rerun/*/*_0[0-9] --json experiments/tracking_kpi/raw/summary.json

# 5. 결과 표, 결과 뒤 분석, 녹화에서 트랙 반경
python3 -I experiments/tracking_kpi/make_tables.py $R experiments/tracking_kpi/raw/summary.json
uv run --no-project --with numpy python -I experiments/tracking_kpi/analyze.py $R
uv run --no-project --with rosbags --with numpy python -I experiments/tracking_kpi/measure_radius.py \
    <장면 C 실행의 rosbag 폴더> src/interfaces/amr_msgs/msg
```

## 파일

| 파일 | 내용 |
| --- | --- |
| `conditions/tracking_main.txt` | 본 측정 조건 (장면 7개 × 10회, 라벨 `main`). 짧은 C10을 맨 앞에 둬서 10회 연속 실행을 먼저 확인했다 |
| `conditions/tracking_main_b_rerun.txt` | B 다시 측정 (라벨 `main_rerun`, perception.md 1-7 "무효 실행") |
| `conditions/tracking_smoke.txt` | 짧은 시험 (장면마다 1회, 라벨 `smoke`) |
| `conditions/tracking_dev.txt` | 추적기 개발용 녹화 (라벨 `dev`, 판정에 안 씀). A·B·D·E는 장애물 시드 1001~1003, C는 개발용 장면 `tracking_kpi_cdev` |
| `conditions/*.log.csv` | `gcp_batch.py` 실행 기록. 조건마다 GitHub Actions 실행 번호와 결과 |
| `fetch_runs.py` | 기록에서 조건마다 마지막으로 성공한 실행만 골라 버킷에서 받고, 파일마다 md5·크기를 대조한다. 틀리면 `gcloud storage cat`으로 하나씩 다시 받는다 |
| `check_main.py` | 실행별 완료·종료 전 오류·장애물 속도 명령 전달·프레임 수·평가 입력·결과 칸, 장면별 10회·시드 1~10·같은 커밋 |
| `make_tables.py` | perception.md 1-7 결과 표 다섯 개 (마크다운) |
| `analyze.py` | 결과 뒤 분석: P2 원인 분해, 물체별 ID 전환·NEES, 처음 잡는 거리, 가짜 동적 트랙, 판정 안 된 구간 |
| `measure_radius.py` | 녹화에서 작업자를 잡은 트랙의 반경과 LiDAR 빔 간격 (`frames.csv`에는 짝 트랙의 반경이 없다) |
| `replay_bench.py` | 재생 벤치. 녹화된 스캔·TF로 추적기를 ROS·Gazebo 없이 다시 돌리고 평가한다 (아래 "재생 벤치") |

## 재생 벤치 (추적기 수정 비교)

녹화(rosbag)의 스캔·TF로 추적 노드와 같은 계산(군집화 → 빈 공간 증거 → 추적)을 코어 함수로 다시 하고, `tracking_eval`과 같은 평가를 낸다. Gazebo도 GCP도 없이 실행 하나가 몇 초다. 출력은 GCP 실행 폴더와 같은 모양이라 4~5번(`summarize.py`·`make_tables.py`·`analyze.py`)을 그대로 쓴다.

```bash
# 녹화까지 받는다
python3 -I experiments/tracking_kpi/fetch_runs.py <기록 CSV> --out experiments/tracking_kpi/raw/dev --with-rosbag
# 지금 설정 / 설정을 바꿔서 (tracking.yaml 이름=값)
uv run --no-project --with rosbags --with numpy --with pyyaml python -I experiments/tracking_kpi/replay_bench.py \
    experiments/tracking_kpi/raw/dev/*/*/*_0[0-9] --out experiments/tracking_kpi/raw/bench/base --check-node
uv run --no-project --with rosbags --with numpy --with pyyaml python -I experiments/tracking_kpi/replay_bench.py \
    experiments/tracking_kpi/raw/dev/*/*/*_0[0-9] --out experiments/tracking_kpi/raw/bench/sa2 --set accel_noise_std=2.0
```

- 설정은 저장소의 `src/perception/amr_tracking/config/tracking.yaml`을 읽는다. 바꾼 값은 결과의 `meta.json` `bench`에 남는다
- 평가 시간 범위는 실행 폴더에 GCP 평가 결과(`tracking/frames.csv`)가 있으면 그 범위, 없으면 첫 프레임부터 `duration_s`
- 확인 (11/1, 본 측정 C10·E·B 1회씩): 설정 그대로면 녹화 속 노드 출력과 C10 504/504, E 902/905, B 2996/3105프레임 같다(녹화 시작 5 s 뒤부터, 트랙 번호가 아니라 위치로 비교). 다른 프레임은 하네스가 추적 노드를 녹화보다 약 2 s 먼저 띄워 생긴 정지 물체 트랙의 차이다
- **수정 효과는 개발용 녹화(장애물 시드 1001~, `--set seed_offset=1000`)로 본다.** 본 측정 녹화(시드 1~10)로 값을 맞추면 판정에 쓴 장면으로 튜닝하는 셈이라, 본 측정 녹화는 `--check-node`로 벤치가 노드·GCP 평가와 같은지 확인할 때만 쓴다(GT와 짝지어지는 값이 GCP와 다르면 실패 코드)
- 장면 C는 작업자 속력이 고정이라 시드가 장면을 바꾸지 않는다. 개발용 C는 따로 만든 `tracking_kpi_cdev`(1.25 m/s, 12 m 출발, 60 s, 실행마다 약 7번 반전)를 쓴다. 지금 추적기는 1.25 m/s 반전에서 트랙을 놓쳐 헛트랙이 드러나고, 12 m에서 출발해 처음 잡는 거리가 늘었는지 본다
- 폐루프는 못 본다. 로봇이 트랙에 반응하는 실험(I17)은 시뮬레이션으로 잰다
- 노드에서만 하는 일(메시지로 낼 때의 반경 규칙, 카메라 연관, 지도 대조, TTC)은 벤치에 없다. **추적기 수정은 코어(`amr_perception_core`)에 넣어야 벤치로 비교된다.** `tracking.yaml`에 벤치가 모르는 값이 생기면 경고한다
- GCP 평가와 비교하면 GT와 짝지어지는 값(RMSE·짝 수·판정 지연·D1)은 같다(본 측정 C10·E·B 1회씩 확인). 프레임당 값(가짜 동적·트랙 수)은 녹화 전 프레임만큼 분모가 달라 조금 다르다. 수정 전후는 벤치끼리 비교한다

## 원자료

- 버킷 `gs://amr-fleet-results/runs/tracking_kpi_<장면>-<라벨>/`. 본 측정은 라벨 `main`(B는 `main_rerun`), 짧은 시험은 `smoke`
- `tracking_kpi_b-main`에는 무효 실행(시드 10)이 든 첫 B 묶음이 그대로 있다. 판정에는 쓰지 않는다
- 본 측정 70회는 모두 #251의 커밋 4ffa929로 돌았다

## 알아 둘 것

- `gcloud storage cp`는 큰 녹화 파일 끝에서 죽거나(TypeError), 크기는 같은데 내용이 깨진 파일을 남긴 적이 있다. 받은 파일은 `fetch_runs.py`처럼 md5로 꼭 대조한다
- GCP 러너는 시드를 바꿀 수 없다(`gcp_run.sh`·`fleet-test.yml`에 시드 입력이 없다). 짧은 시험도 본 측정과 같은 시드 1이라, 시험 숫자로 기준·코드를 바꾸지 않았다
- 받는 데 보통 장면당 30초 안팎이지만, 16분 걸린 적도 있다(결과는 정상)
