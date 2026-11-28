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
교착 장면 영상: `record=true`로 한 실행을 다시 돌리고(rosbag), 컨테이너에서 CSV로 바꾼 뒤 그린다.
측정 결과 폴더와 섞이지 않게 영상용 실행은 `tools/harness/results/` 밖으로 옮긴다.

```bash
tools/harness/docker_run.sh zone_pass -n 1 --set zone=west_aisle --set guests_n=1 --set record=true --label video_west_n1
# 컨테이너 안 (amr_msgs 빌드 후): python3 experiments/zone_capacity/bag_to_csv.py <실행 폴더>
python3 experiments/zone_capacity/animate.py <실행 폴더> --pair 3 --out deadlock.mp4   # --pair 없으면 첫 시간 초과
```

`plots.py`는 보고서용 그래프(A 통과 시간, B 0명 대비 배수, C 통과 한 번씩, D 60 s 안 비율, E 가정값 대비 측정값)를
컬러(`_color`)와 흑백(`_mono`)으로, B·E를 합친 `poster_mono.png`를 쓴다.

## 결과 (11/28, seed 1·2·3, 조건마다 통과 30번)

**결론: 정원 값으로 쓰지 않는다.** 숫자로는 "2배가 되는 손님 수"가 0.4~0.6명이지만, 손님이 있으면 통과의 30~50%가
120 s 안에 못 지나간다(교착). 이 실험이 잰 것은 통로 혼잡보다 **좁은 통로에서 마주친 손님과 로봇이 서로 기다리다 멈추는 교착**이다.
정원은 통로 길이 어림값(통로 3명, `aisle_5` 2명)을 그대로 두고, 교착을 고친 뒤 다시 잰다.

| 통로 | 손님 0명 | 1명 | 2명 | 4명 |
| --- | --- | --- | --- | --- |
| `west_aisle` (1.6 m) 중앙값 | 24.7 s | 82.5 s (×3.3) | 98.5 s (×4.0) | 78.4 s (×3.2) |
| 　120 s 안에 못 지나감 | 0% | 30% | 30% | 20% |
| 　지나간 통과만 | 24.7 s | 53.5 s (×2.2) | 91.7 s (×3.7) | 74.5 s (×3.0) |
| `aisle_3` (1.37 m) 중앙값 | 43.2 s | 118.4 s (×2.7) | 116.3 s (×2.7) | 120.0 s (×2.8) |
| 　120 s 안에 못 지나감 | 0% | 50% | 50% | 53% |
| 　지나간 통과만 | 43.2 s | 50.4 s (×1.2) | 84.3 s (×2.0) | 76.1 s (×1.8) |

- 손님 0명은 30번 모두 지나간다. 손님이 1명만 있어도 교착이 30~50%로 뛰고, 그 뒤로는 손님 수와 거의 상관없다
- 교착 장면 (record:=true 재실행, `animate.py`): 로봇 앞 약 1 m에 마주 오던 손님이 서고(1.2 m 규칙), 손님이 통로 반을
  막은 채 "움직이는 장애물"로 잡혀 로봇은 안전 노드 감속 상태로 0.01 m/s로 기어간다. 손님은 5초마다 돌아서지만 무작위로
  고른 새 목표가 다시 로봇 쪽이라 0.05초 만에 또 선다 (손님이 있는 18실행에서 돌아섬 2,046번 중 60%가 0.5초 안에 다시 섬,
  `obstacle_motion._keep_away`가 돌아설 때 로봇 위치를 안 봄). 한 통과 120 s 중 안전 노드는 감속 1,043번·정지 65번
- 실행 하나당 안전 정지 경고("진행 경로에 장애물")가 손님 0명 0~49번 → 1명 26~261번 → 2~4명 80~351번으로 는다.
  손님이 로봇 경로에 들어오면 로봇이 서고, 손님은 로봇에게서 1.2 m 떨어지려고 비켜 서면서 서로 기다린다
- 교착을 빼고 지나간 통과만 보면 세로 통로는 손님 2명에서 2배(정원 약 2명), 서쪽 통로는 1명에서 2배다. 다만 손님 0명
  통과가 빠른(24.7 s) 서쪽 통로가 배수로는 더 크게 나와, 더 넓은 통로의 정원이 더 작아지는 모순이 있다
- 손님이 2분 내내 통로 안을 서성이는 조건이라 실제(몇 초 만에 지나감)보다 나쁜 쪽이다

**다시 잴 때**: 교착(손님 돌아서기 버그, 로봇이 선 손님을 비켜 가지 못함) 수정이 들어간 커밋에서, 손님이 통로를 지나가는 패턴으로 바꿔 잰다.
표·원값은 `results.md`·`results.json`, 그래프는 `plots/` (F = 교착 비율, G = 지나간 통과만의 배수).

## 결과를 쓰는 곳 (다시 재서 교착이 줄었을 때)

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
