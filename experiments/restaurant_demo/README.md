# restaurant_demo: 식당 시연 녹화 지표

클라우드에서 녹화한 시연 rosbag으로 영상 자막·발표에 넣을 숫자를 낸다. Gazebo 없이 bag만 읽는다.

## 입력

하네스·버킷 경로 그대로: `runs/demo_<장면>-<on|off>/<실행 id>/` (#319 `demo_s11.yaml` 머리말, 실행 id는 하네스 `<시나리오>-<라벨>_<YYYYMMDD-HHMMSS>_<nn>`).

- 장면 이름: `demo_s11` → 1-1, `demo_s12` → 1-2, `demo_s2…` → 2 (장면 2의 1막·2막은 따로 녹화하고 이름은 아직 미정이라 `s2`로 시작하면 모두 2). 숫자 이름(`demo_1-2-on`)도 받는다
- 실행 id마다 한 줄씩 낸다. 표(`results.md`)에는 장면 폴더마다 가장 최근 실행만 쓰고(`--all`이면 전부), `results.json`에는 전부 남긴다
- bag: 실행 폴더의 측정용 `rosbag/`을 먼저 쓴다. mp4로 바꾸지 못해 `camera_bag/`이 남아 있어도 고르지 않는다

| 장면 | 쓰는 토픽 | 지표 |
| --- | --- | --- |
| `s12` → 1-2 (붐비는 통로 돌아가기) | `/fleet/task_table`, `/fleet/task_status`, `/fleet/robot_states` | 요청 → 배식대 2번 자리(`buffet_2`) 도착 시간, 그 사이 멈춘 횟수 |
| `s2…` → 2 (회수 호출 + aging) | `/fleet/task_table` | 1막 회수 대기 시간(평균·최대·미할당), 2막 기아 시간 |
| `s3` → 3 (충전 스케줄링) | `/fleet/robot_states`, `/fleet/task_table`, `/demo/t0` | 홀 운영 대수 최저, 동시 충전 최대, 대기 배달 최대, 배달 대기(평균·최대·미할당), 0초 배터리, 충전 작업 시각. 실행 폴더에 시간별 `scene3_timeline.csv` |

녹화할 때 이 토픽이 bag에 들어가야 한다. 시각은 메시지 안 시각(use_sim_time이면 시뮬레이션 s)이다.

## 정의

- 요청 시각: `Task.created_time`. 0이면 작업표에 처음 나온 시각
- 도착: 그 자리를 pickup으로 가진 작업은 `loading`, dropoff면 `unloading` 단계를 처음 보고한 `TaskStatus` 시각. 단계 보고가 없고 dropoff면 COMPLETED 시각
- 도착 시간은 docking → loading 단계 기준이라 자리 맞추는 시간도 들어간다 (끔/켬 같은 기준)
- 멈춘 횟수: 그 로봇 `RobotState` 자세 차분 속도가 0.05 m/s 밑으로 1 s 이상 머문 구간 수(`--stop-speed`, `--stop-min-s`). 출발 전 대기와 도착 직전 정차는 빼고 따로 낸다. 정지 비율 = 멈춘 시간 합 / (출발 ~ 도착) `RobotState.velocity`는 amr_bt가 채우지 않아 자세로 잰다(2 Hz)
- 도착하지 못한 실행: 도착 시간은 `-`, 멈춘 횟수·시간·정지 비율은 녹화 끝까지로 낸다 (끝까지 멈춰 있던 구간도 멈춤 하나)
- 회수 대기: 회수 호출(`TYPE_COLLECT`)마다 첫 할당(작업표 `assignee_id`가 처음 찬 시각) − 요청
- 홀 운영 대수 (장면 3): 0초(`/demo/t0`, 로봇 하나가 31 % 밑이 된 순간)부터 녹화 끝까지 0.5 s마다 센다. `hall` = 충전 작업(`TYPE_CHARGE`)을 맡지 않은 로봇 수(충전소로 가는 중도 뺀다, 서버 `min_in_service`와 같은 뜻), `hall_mode` = `MODE_CHARGING`이 아닌 로봇 수(시나리오 문서의 계산). 배달 대기는 0초 뒤 요청된 배달의 할당 − 요청. `--plot 그림.png`이면 켬/끔 `hall`을 0초에 맞춰 한 그래프에
- 기아 시간: 일반 작업이 기다리는데 일반 작업 할당이 하나도 없던 가장 긴 구간. #306 `experiments/fleet_collect_aging/collect_aging.py`의 `starvation`과 같은 식이다(그쪽은 하네스 csv, 여기는 `/fleet/task_table`). `--act2 START END`(첫 요청 기준 s)를 주면 그 구간과 겹치는 것만 본다

## 실행

```bash
source /ros2_ws/install/setup.bash          # rosbag2_py, amr_msgs
python3 experiments/restaurant_demo/analyze.py runs/ --out experiments/restaurant_demo/results.md
python3 -m pytest experiments/restaurant_demo -q   # 가짜 데이터 시험 (가짜 rosbag 끝까지 2개 포함)
```

`results.md`(표)와 `results.json`(실행별 원값, 멈춘 구간·할당 순서 포함)을 쓴다. 원자료(`runs/`)는 커밋하지 않는다.

실제 Gazebo 장면 3의 실행 조건·동선·녹화 성공 기준은 [scene3_gazebo.md](scene3_gazebo.md)를 참고한다.
