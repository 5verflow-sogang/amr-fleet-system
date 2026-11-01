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
| `conditions/*.log.csv` | `gcp_batch.py` 실행 기록. 조건마다 GitHub Actions 실행 번호와 결과 |
| `fetch_runs.py` | 기록에서 조건마다 마지막으로 성공한 실행만 골라 버킷에서 받고, 파일마다 md5·크기를 대조한다. 틀리면 `gcloud storage cat`으로 하나씩 다시 받는다 |
| `check_main.py` | 실행별 완료·종료 전 오류·장애물 속도 명령 전달·프레임 수·평가 입력·결과 칸, 장면별 10회·시드 1~10·같은 커밋 |
| `make_tables.py` | perception.md 1-7 결과 표 다섯 개 (마크다운) |
| `analyze.py` | 결과 뒤 분석: P2 원인 분해, 물체별 ID 전환·NEES, 처음 잡는 거리, 가짜 동적 트랙, 판정 안 된 구간 |
| `measure_radius.py` | 녹화에서 작업자를 잡은 트랙의 반경과 LiDAR 빔 간격 (`frames.csv`에는 짝 트랙의 반경이 없다) |

## 원자료

- 버킷 `gs://amr-fleet-results/runs/tracking_kpi_<장면>-<라벨>/`. 본 측정은 라벨 `main`(B는 `main_rerun`), 짧은 시험은 `smoke`
- `tracking_kpi_b-main`에는 무효 실행(시드 10)이 든 첫 B 묶음이 그대로 있다. 판정에는 쓰지 않는다
- 본 측정 70회는 모두 #251의 커밋 4ffa929로 돌았다

## 알아 둘 것

- `gcloud storage cp`는 큰 녹화 파일 끝에서 죽거나(TypeError), 크기는 같은데 내용이 깨진 파일을 남긴 적이 있다. 받은 파일은 `fetch_runs.py`처럼 md5로 꼭 대조한다
- GCP 러너는 시드를 바꿀 수 없다(`gcp_run.sh`·`fleet-test.yml`에 시드 입력이 없다). 짧은 시험도 본 측정과 같은 시드 1이라, 시험 숫자로 기준·코드를 바꾸지 않았다
- 받는 데 보통 장면당 30초 안팎이지만, 16분 걸린 적도 있다(결과는 정상)
