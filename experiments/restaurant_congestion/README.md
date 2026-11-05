# 식당 시연 1-1: 혼잡도 반영 할당 켬/끔 (민지, 7주차 작업 5)

손님이 한쪽(서쪽 통로·배식대 앞)에 몰렸을 때, 할당 비용에 혼잡도를 넣으면(`cost_fn:=congestion`, #297) 덜 막히는 로봇이
일을 받는지, 그래서 대기·완료·정지 시간이 줄어드는지 잰다. 효과가 작으면 작다고 그대로 적는다.

> 상태: **측정 중**. 식당 5대 실행(#308)과 하네스 시나리오 `fleet_restaurant_congestion`으로 돌린다.

## 조건

| 항목 | 값 |
| --- | --- |
| 월드 | `amr_gazebo` `restaurant.sdf`, 서빙 로봇 몸체(`body:=serving`), 식당 정답 지도 `restaurant_gt`, map = world([0, 0, 0]) |
| 손님 | 피크 몰림 `dynamic_obstacles_restaurant_peak.yaml` (13명, 서쪽 통로·배식대 앞에 6명 몰림). 직원·서 있는 손님은 혼잡도에 세지 않는다 (11/5) |
| 혼잡도 | `congestion.launch.py site:=restaurant guests:=peak` (#305). 평상시 대비 피크에서 `west_aisle` 0.02 → 1.00, `buffet_line` 0.06 → 0.87 (#305 확인) |
| 작업 | `config/scenarios/restaurant_serving.yaml`: 24건, 평균 20 s 간격(5대 처리량의 약 0.7배), 배식대 픽업 2/3·매장 카운터 1/3 → 테이블. 1-1은 놀고 있는 로봇이 2대 이상이어야 할당이 달라질 수 있어 처리량보다 느리게 넣는다 |
| 할당 | `nearest`(기본). `load_balance`는 할당 수를 먼저 봐서 혼잡도가 거의 안 먹힌다 (#297) |
| 켬 / 끔 | `cost_fn:=congestion congestion_weight:=3.0` / `cost_fn:=euclidean`. 다른 조건은 같다 |
| Nav2 혼잡도 필터 | **양쪽 다 끈다.** 1-1(할당)만 보려는 것이다. 필터(1-2, 이진)까지 켜면 주행 효과가 섞인다 |
| 반복 | 조건마다 seed 1~5 (seed가 손님 움직임·작업 일정을 정한다. 켬·끔이 같은 seed를 쓴다) |

## 지표 (`congestion_metrics.py`)

| 지표 | 정의 | 기대 |
| --- | --- | --- |
| 가까운 로봇 아닌 로봇 비율 | 할당 순간 놀고 있던 로봇이 2대 이상인 작업 중, 픽업까지 직선거리가 가장 가까운 로봇이 아닌 로봇을 보낸 비율 | 끔 0, 켬 > 0. **1-1 장면이 실제로 몇 번 일어났는지**다 |
| 할당 대기 | 요청 → 첫 할당 | 비슷 (처리량보다 느리게 넣어서) |
| 완료 시간 평균·p95 | 요청 → 완료 | 켬이 짧거나 같음 |
| 작업 중 정지 합 | 작업 중인데 0.05 m/s 아래인 시간, 로봇 전체 | 켬이 짧음 |
| 혼잡 구역 체류 합 | 작업 중 `west_aisle`·`buffet_line` 안에 있던 시간, 로봇 전체 | 켬이 짧음. 단 배식대 픽업은 누가 가도 `buffet_line`에 들어가서 차이가 작을 수 있다 |

시간은 모두 시뮬레이션 시간이다(meta.json `fleet_clock.rtf`, #285).

## 실행

하네스 시나리오 `tools/harness/scenarios/fleet_restaurant_congestion.yaml`(#308의 `system.launch.py` 식당 인자 + 혼잡도 노드 +
작업 생성기 + rosbag). GCP 러너(RTF 1.0 근처)로 돌린다. 노트북 Docker는 5대 + 손님 13명에서 RTF가 낮아 작업 일정(실제 시간,
#285 머지 전)이 상대적으로 몰린다.

```bash
tools/harness/gcp_run.sh fleet_restaurant_congestion -n 5 --set assign=off --label off
tools/harness/gcp_run.sh fleet_restaurant_congestion -n 5 --set assign=on  --label on
# 결과 폴더를 raw/off/, raw/on/ 아래로 옮긴 뒤
python3 experiments/restaurant_congestion/analyze.py raw/ --out experiments/restaurant_congestion
```

`analyze.py`는 실행별 `summary.csv`와 조건별 `results.md`(평균 ± 표준편차)를 쓴다. 시간은 `meta.json`의
`fleet_clock.rtf`(#285 뒤) 또는 `real_time_factor`를 곱해 시뮬레이션 시간으로 바꾼다.

## 스택에 필요한 것 (하연 식당 실행과 맞출 것)

`system.launch.py`에 지금 없는 것:

1. 월드 선택 (`restaurant.sdf`)과 식당 로봇 스폰 위치 파일 (5대)
2. 동적 장애물 설정 파일 선택 (`dynamic_obstacles_restaurant_peak.yaml`)
3. Fleet 서버 `cost_fn`·`congestion_weight` 전달 (fleet_manager.launch.py에는 있다)
4. 작업 생성기 `stations_file:=stations_restaurant.yaml`, `scenario_file:=restaurant_serving.yaml`, `map_origin_in_world:=[0, 0, 0]`
5. 혼잡도 노드: 하네스 시나리오에서 `congestion.launch.py site:=restaurant guests:=peak`를 따로 띄워도 된다

## 테스트

```bash
python3 -m pytest experiments/restaurant_congestion
```
