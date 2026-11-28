# 식당 시연 1-1: 혼잡도 반영 할당 켬/끔 (민지, 7주차 작업 5)

손님이 한쪽(서쪽 통로·배식대 앞)에 몰렸을 때, 할당 비용에 혼잡도를 넣으면(`cost_fn:=congestion`, #297) 덜 막히는 로봇이
일을 받는지, 그래서 대기·완료·정지 시간이 줄어드는지 잰다. 효과가 작으면 작다고 그대로 적는다.

> 상태: **보류 (11/6)**. 식당 5대 스택에서 로봇이 홀을 지나지 못해 켬/끔을 비교할 수 없었다. 아래 "11/6 첫 측정" 참고.

## 11/6 첫 측정 (비교 불가)

GCP에서 끔 2회·켬 1회가 끝까지 돌았다(b 슬롯 c2d-8, 시험 1회는 a 슬롯 n2d-16). 다른 회차는 기동 단계에서 5대 Nav2가
360 s 안에 다 켜지지 않아 3번 시도 끝에 실패했다(시도 기준 8번).

| 항목 | 값 |
| --- | --- |
| RTF | 0.21~0.22. a 슬롯(16코어)도 0.222라 코어 수 문제가 아니다 |
| 완료 / 작업 | 회차마다 0~3 / 24 |
| 멈춘 곳 (작업 중 0.02 m/s 아래 시간) | `hall_east` 58 %, `buffet_line` 26 %, `hall_west` 12 %, 손님이 몰린 `west_aisle` 1 % |
| Nav2 로그 (끔 seed 4) | DWA "유효한 궤적을 못 찾아 정지" 약 900회, 제어 주기 놓침 약 1200회, `Failed to make progress` → navigate_to_pose 3회 재시도 실패 |

- 혼잡 구역보다 테이블 사이 홀(`hall_east` x 0~5, y −3~1)에서 더 오래 멈췄다. 할당 비용이 아니라 식당 주행이 먼저 막힌다.
- 멈춘 로봇이 배식대 정차 위치를 잡고 있으면 서버가 같은 위치(1 m 안) 작업을 잡아 둔다(`station_clearance_m`). 작업 2/3가
  배식대 픽업이라 큐 전체가 멈추고 놀고 있는 로봇이 있어도 할당이 없다.
- 11/7 다시 시험(#325 손님 양보·#320 lifecycle 뒤, 끔 seed 1, b 슬롯): RTF 0.214, 완료 1 / 24, 배정 14건 중 12건이
  `code=201`(navigate_to_pose 실패)로 끝났다. 그대로다.
- 스택이 홀을 지나게 되면(주행 쪽, 위치추정 밀림 #327) 같은 시나리오로 다시 돈다.

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
작업 생성기 + rosbag). GCP b 슬롯(c2d-standard-8)에서 RTF가 약 0.25라, 작업 일정과 관측 길이를 시뮬레이션 시간으로 둔다
(작업 생성기 `use_sim_time`). 실행 하나가 실제 약 50분이다.

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
