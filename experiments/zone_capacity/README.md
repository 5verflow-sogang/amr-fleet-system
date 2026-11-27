# zone_capacity: 혼잡도 구역 정원 측정 (민지, 11/28)

혼잡도는 `min(1, 구역 안 사람 수 ÷ 정원)`이다. 정원을 통로 길이로 어림한 가정값(약 2.3 m당 1명)에서 **측정값**으로 바꾸려고,
손님 수에 따라 로봇이 통로를 지나는 시간이 얼마나 늘어나는지 잰다.

## 정의

- **통과 시간**: 로봇 1대가 통로 남쪽 끝에서 북쪽 끝까지(또는 반대로) 가는 시간. Nav2 목표 수락 → 도착 (시뮬레이션 s)
- **정원 C**: 통과 시간 중앙값이 손님 0명일 때의 **2배**가 되는 손님 수
- **BPR 식**: `T(n) = T0 × (1 + (n / C)^β)`. 교통공학에서 도로 혼잡을 나타내는 BPR(Bureau of Public Roads) 함수에서
  α = 1로 둔 꼴이라, n = C일 때 T = 2 × T0가 되어 위 정의와 같다. β는 손님이 늘수록 얼마나 급하게 막히는지다
- 측정한 인원 사이의 값은 BPR 식으로 메우고, 직접 읽기(중앙값이 2 × T0를 넘는 두 인원 사이 직선)도 함께 적는다

## 조건

| 항목 | 값 |
| --- | --- |
| 시나리오 | `tools/harness/scenarios/zone_pass.yaml` |
| 월드·로봇 | 식당 `restaurant.sdf`, 서빙 몸체 1대, Nav2 식당 프로필(1초 재계획), 추적·안전 켬 (시연과 같다) |
| 위치 | 정답 위치 (통과 시간에 위치추정 밀림이 섞이지 않게) |
| 통로 | `west_aisle`(x −10.5, 폭 약 1.6 m), `aisle_3`(x −1.175, 1.37 m), `aisle_5`(x 4.72, 1.0 m). 모양이 다른 세 종류. aisle_5는 손님 0명에서도 10번 중 5번 멈춰(아래 한계) 정원을 재지 않았다 |
| 손님 | 0, 1, 2, 4명. 그 통로 띠 안에서만 무작위로 걷는다 (`pattern: random`, 로봇 양보 1.2 m는 시연과 같다) |
| 통과 | 실행마다 10번 (왕복 5번). 한 번 제한 120 s, 넘으면 120 s로 넣는다 (검열값) |
| 반복 | 통로 × 손님 수마다 seed 1·2·3 세 실행 = 통과 30번 |

설정 파일(`waypoints/`, `robots/`, `guests/`)은 `make_configs.py`가 만든다. 통로 가운데 줄은 정답 지도에서 장애물까지
여유가 가장 큰 x로 잡았다.

## 실행

```bash
# 컨테이너는 한 번에 하나만. 두 개를 같이 띄우면 둘 다 ROS 도메인 1·Gazebo 파티션 harness_1을 써서
# 서로의 로봇·손님·시계가 섞인다 (11/28 seed 2·3 첫 시도가 이것으로 전부 무효)
for seed in 1 2 3; do for n in 0 1 2 4; do for z in west_aisle aisle_3; do
  tools/harness/docker_run.sh zone_pass -n 1 --seed-base $seed --set zone=$z --set guests_n=$n --label ${z}_n${n}_s$seed
done; done; done
python3 experiments/zone_capacity/analyze.py tools/harness/results --zones west_aisle aisle_3 --out experiments/zone_capacity
python3 experiments/zone_capacity/plots.py experiments/zone_capacity/results.json --out experiments/zone_capacity/plots
python3 -m pytest experiments/zone_capacity -q
```

`analyze.py`는 `results.md`(표), `results.json`(통과별 원값), `pass_time.png`(통로별 손님 수 - 통과 시간과 BPR 곡선)를 쓴다.
`plots.py`는 보고서용 그래프(A 통과 시간, B 0명 대비 배수, C 통과 한 번씩, D 60 s 안 비율, E 가정값 대비 측정값)를
컬러(`_color`)와 흑백(`_mono`)으로, B·E를 합친 `poster_mono.png`를 쓴다.

## 결과를 쓰는 곳

- **정원**: 구역 종류별 C를 `amr_fleet_manager/config/congestion_zones_restaurant.yaml`에 넣는다. 잰 세 통로와 모양이 같은
  구역에 같은 값을 쓴다 (서쪽 통로 → `west_aisle`, 폭 1.37 m 세로 통로 → `aisle_1`~`aisle_4`, 폭 1.0 m → `aisle_5`)
- **weight**: 혼잡도 v인 구역 1 m를 (1 + weight × v) m로 친다. BPR에서 n = C이면 2배이므로 β = 1이면 weight = 1이다.
  지금 3.0은 "정원만큼 차면 4배"라는 가정이다. β가 1보다 크면 혼잡도를 `(n / C)^β`로 바꾸는 것도 검토한다 (`bpr.congestion_weight`)

## 한계

- 통로 하나에 로봇 1대만 잰다. 여러 로봇과 손님이 섞인 효과는 1-1·1-2 반복 측정에서 본다
- 손님은 띠 안에서 무작위로 걷고 로봇에 양보한다. 줄을 서거나 멈춰 서 있는 손님은 따로 재지 않았다
- `aisle_5`(폭 1.0 m)는 손님이 없어도 로봇이 통로 안에서 멈춘다 (통과 10번 중 5번 실패). 혼잡이 아니라 주행 문제라
  정원은 기하 어림값 2를 그대로 둔다
- 통로 × 손님 수마다 통과 30번이라 중앙값의 흔들림이 있다. 결론이 경계에 걸리면 seed를 늘린다
