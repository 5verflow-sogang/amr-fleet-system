# restaurant_demo: 식당 시연 녹화 지표

클라우드에서 녹화한 시연 rosbag으로 영상 자막·발표에 넣을 숫자를 낸다. Gazebo 없이 bag만 읽는다.

## 입력

녹화 실행 폴더 규칙(클라우드 시나리오 녹화): `runs/demo_<장면>-<on|off>/`. 그 아래 rosbag(`metadata.yaml`이 있는 폴더)을 하나 찾는다.

| 장면 | 쓰는 토픽 | 지표 |
| --- | --- | --- |
| `1-2` (붐비는 통로 돌아가기) | `/fleet/task_table`, `/fleet/task_status`, `/fleet/robot_states` | 요청 → 배식대 2번 자리(`buffet_2`) 도착 시간, 그 사이 멈춘 횟수 |
| `2`, `2-…` (회수 호출 + aging) | `/fleet/task_table` | 1막 회수 대기 시간(평균·최대·미할당), 2막 기아 시간 |

녹화할 때 이 토픽이 bag에 들어가야 한다. 시각은 메시지 안 시각(use_sim_time이면 시뮬레이션 s)이다.

## 정의

- 요청 시각: `Task.created_time`. 0이면 작업표에 처음 나온 시각
- 도착: 그 자리를 pickup으로 가진 작업은 `loading`, dropoff면 `unloading` 단계를 처음 보고한 `TaskStatus` 시각. 단계 보고가 없고 dropoff면 COMPLETED 시각
- 멈춘 횟수: 그 로봇 `RobotState` 자세 차분 속도가 0.05 m/s 밑으로 1 s 이상 머문 구간 수(`--stop-speed`, `--stop-min-s`). 출발 전 대기와 도착 직전 정차는 빼고 따로 낸다. `RobotState.velocity`는 amr_bt가 채우지 않아 자세로 잰다(2 Hz)
- 회수 대기: 회수 호출(`TYPE_COLLECT`)마다 첫 할당(작업표 `assignee_id`가 처음 찬 시각) − 요청
- 기아 시간: 일반 작업이 기다리는데 일반 작업 할당이 하나도 없던 가장 긴 구간. #306 `experiments/fleet_collect_aging/collect_aging.py`의 `starvation`과 같은 식이다(그쪽은 하네스 csv, 여기는 `/fleet/task_table`). `--act2 START END`(첫 요청 기준 s)를 주면 그 구간과 겹치는 것만 본다

## 실행

```bash
source /ros2_ws/install/setup.bash          # rosbag2_py, amr_msgs
python3 experiments/restaurant_demo/analyze.py runs/ --out experiments/restaurant_demo/results.md
python3 -m pytest experiments/restaurant_demo -q   # 가짜 데이터 22개 (가짜 rosbag 끝까지 1개 포함)
```

`results.md`(표)와 `results.json`(실행별 원값, 멈춘 구간·할당 순서 포함)을 쓴다. 원자료(`runs/`)는 커밋하지 않는다.
