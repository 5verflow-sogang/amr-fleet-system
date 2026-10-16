# 배송 중 Kidnapped와 BT (4주차 작업 5, 10/16)

배송 도중 로봇을 순간 이동시켜, 위치 복구 노드(`amr_localization` `kidnap_recovery`, #79·#178)와 BT(`amr_bt` MoveTo 멈춤·재개, #141·#166)가 Gazebo에서 실제로 맞물려 도는지 본다. 지금까지는 가짜 상태 토픽으로 gtest·launch 테스트만 했다.

```bash
# 시스템 (로봇 1대) + 복구 노드. system.launch.py에는 아직 복구 노드 켜기 인자가 없어 따로 띄운다
ros2 launch amr_bringup system.launch.py num_robots:=1 nav2:=true docking:=true enable_camera:=true
ros2 run amr_localization kidnap_recovery --ros-args -r __ns:=/amr_01 \
  --params-file $(ros2 pkg prefix amr_localization)/share/amr_localization/config/kidnap_recovery.yaml \
  -p use_sim_time:=true -p base_frame:=base_footprint -p cmd_vel_topic:=cmd_vel
# 같은 통로 3회 / 다른 통로 (복구 실패 뒤 GT로 위치를 다시 잡아 줌)
python3 experiments/bt_kidnap/run_kidnap.py --mode near --seeds 1 2 3 --pairs rack_a6:rack_a5 rack_a5:rack_a6 --csv near.csv
python3 experiments/bt_kidnap/run_kidnap.py --mode far --restore --seeds 1 2 3 --pairs rack_a5:rack_a6 rack_a6:rack_a5 --csv far.csv
```

- 배송(rack_a5 ↔ rack_a6, A–B 통로)의 드롭오프 이동을 GT로 2 m 간 뒤 `set_pose`로 옮긴다. 옮길 자리는 `kidnap_trial`과 같은 `pick_target(mode, seed)`: near = 같은 통로 1–2.5 m·방향 ±0.2–0.6 rad, far = B–C 통로 아무 데나
- 복구 노드의 회전 명령은 `cmd_vel`로 바로 낸다 (이 launch에는 구역·안전 노드와 twist_mux가 없다. BT가 멈춰 있는 동안 Nav2는 명령을 내지 않는다)
- 시각은 스크립트 기준 실제 시간(순간 이동 = 0 s)이다. BT·복구 노드 상한은 시뮬레이션 시간으로 센다

## 결과

### 같은 통로 (near) 3/3: 멈춤 → 복구 → 재개 → COMPLETED

| 회 | 경로 | 옮긴 거리 | 위치 잃음 | Nav2 목표 취소 (BT 멈춤) | tracking 복귀 | 같은 목표 다시 보냄 (재개) | 결과 | 끝 위치 오차 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | rack_a6 → rack_a5 | 2.46 m | 2.0 s | 2.2 s | 40.4 s | 41.1 s | COMPLETED | 0.59 cm |
| 2 | rack_a5 → rack_a6 | 2.77 m | 2.6 s | 2.7 s | 42.0 s | 42.8 s | COMPLETED | 0.29 cm |
| 3 | rack_a6 → rack_a5 | 3.51 m | 2.7 s | 2.9 s | 42.9 s | 43.6 s | COMPLETED | 0.34 cm |

- 복구 노드가 tracking을 벗어나고 0.1–0.2 s 안에 BT가 Nav2 목표를 취소했다. tracking으로 돌아오고 0.7–0.8 s 뒤(`resume_hold_s` 0.3 s + tick) 같은 목표를 다시 보냈다
- 3회 모두 복구는 시도 2번째(정지 → 회전 → 정지 확인)에서 됐다 (약 40 s). 그동안 BT는 `settling`·`rotating` 멈춤으로 기다렸다 (상한 `kidnap_pause_timeout_s` 120 s)
- 재개 뒤 드롭오프 도킹까지 끝내 끝 위치 오차 0.3–0.6 cm

### 다른 통로 (far) 3/3: 복구 실패 → 바로 FAILED → MODE_ERROR → 위치 회복 뒤 서버가 다시 할당

| 회 | 경로 | 옮긴 거리 | 위치 잃음 / BT 멈춤 | 복구 노드 failed | BT 결과 | 그 뒤 | 위치 회복 → IDLE |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | rack_a5 → rack_a6 | 25.6 m | 2.1 / 2.3 s | 129.0 s | 바로 FAILED 203 | MODE_ERROR | 손으로 (아래 표) |
| 2 | rack_a6 → rack_a5 | 9.8 m | 2.3 / 2.5 s | 127.6 s | 바로 FAILED 203 | MODE_ERROR | 손으로 |
| 3 | rack_a5 → rack_a6 | 21.9 m | 2.1 / 2.2 s | 127.8 s | 바로 FAILED 203 | MODE_ERROR | 스크립트 initialpose 뒤 0.7 s tracking, 1.4 s IDLE |

- 3회 모두 복구 노드가 시도 3번 끝에 `failed`를 냈고, BT는 0.1 s 안에 FAILED(`LOCALIZATION_LOST` 203)로 끝냈다. 120 s 상한까지 기다리지 않는다 (#166)
- 실제 시간으로 약 128 s지만 시뮬레이션 시간으로는 lost → failed 86 s다 (#178 예상 약 84 s). BT 상한(120 s, 시뮬레이션 시간) 안이다
- 다른 통로는 스캔만으로 못 찾는 경우라 복구 실패가 예상된 결과다 (0/10, #148). 이 실험은 그때 BT·서버가 맞게 움직이는지를 본다

1회를 끝까지 따라간 것:

| 시각 (순간 이동 = 0 s) | 일 |
| --- | --- |
| 0 s | rack_a5 → rack_a6 드롭오프 이동 중 B–C 통로 (−14.6, −4.8)로 옮김 (25.6 m) |
| 2.1 / 2.3 s | 복구 노드 tracking → settling / BT가 Nav2 목표 취소하고 멈춤 |
| 2.3–129 s | 복구 시도 3번 (정지 → 회전 → 정지 확인), 모두 못 찾음. BT는 멈춘 채 기다림 |
| 129.0 s | 복구 노드 `failed` (시뮬레이션 시간으로는 lost → failed 86 s, #178 예상 약 84 s와 같음) |
| 129.1 s | BT가 기다리지 않고 바로 **FAILED, `LOCALIZATION_LOST`(203)** "move to dropoff: localization lost (kidnap_recovery failed)" |
| 129.1 s | Fleet 서버가 실패 보고를 받고 작업을 대기열로 되돌림 ("다시 할당 1/1") |
| 129.7 s | RobotState **MODE_ERROR** ("할당에서 빠짐: localization lost"). 서버는 MODE_IDLE에만 할당해 이 로봇에 다시 주지 않음 |
| (수동) | GT로 initialpose를 줘 위치를 다시 잡음 → 복구 노드 `failed` → `tracking` |
| +0.7 s | BT가 MODE_ERROR 해제 ("118.0 s 동안 할당에서 빠짐") → MODE_IDLE |
| +0.002 s | 서버가 같은 작업을 다시 할당 → 수락 → 픽업·드롭오프 도킹까지 **COMPLETED** (끝 위치 오차 0.38 cm) |

- #166에서 정한 대로 됐다: 복구 노드가 포기(`failed`)하면 120 s 상한을 기다리지 않고 바로 실패하고, 위치를 잃은 로봇은 할당에서 빠졌다가 위치가 돌아오면 다시 받는다
- 1·2회는 스크립트의 위치 회복이 듣지 않아 손으로 다시 보냈다 (아래 "돌리며 찾은 것"). 그래서 1회 MODE_ERROR 118 s에는 기다린 시간이 섞여 있다. 2회도 회복 뒤 서버가 같은 작업을 다시 할당했다

## 돌리며 찾은 것

- BT는 TaskStatus를 단계가 바뀔 때만 낸다. 그래서 이동 중에는 `distance_traveled`가 늘지 않아, 순간 이동 시점은 GT 경로 길이로 잡았다
- 위치 회복 initialpose 두 가지 (1·2회):
  - 이 스크립트 노드는 실제 시계라 지금 시각을 넣으면 sim 시계보다 미래라 AMCL이 받지 못했다. 시각 0(최신 TF)으로 보낸다
  - 정렬 원점 대신 (0, 1.5)로 바꾸면 GT와 6 cm 어긋나는데, 이것만으로 AMCL이 0.7 m 벗어나 복구 노드가 tracking으로 못 돌아왔다. TF world → map에서 원점을 읽고 3번 보낸다 (3회는 이걸로 0.7 s 만에 tracking)
- 같은 컨테이너에서 전날 실행이 남긴 노드(지도 서버 등 같은 이름)가 살아 있으면 새 실행의 `map_server`가 configure에서 멈춘다. 시작 전에 이전 ROS 프로세스를 다 정리해야 한다

## 한계

- 로봇 1대, 동적 장애물 없음, 구역·안전 노드 끔. 복구 회전 명령이 안전 노드를 거치는 경로(twist_mux)는 이 실행에서 보지 않았다
- near 3회·far 3회라 성공률이 아니라 동작 확인이다 (Kidnapped 성공률은 민지 하네스 `kidnap` 시나리오가 잰다)
- 다른 통로는 복구 노드가 스캔만으로는 못 찾는다 (0/10). ArUco 위치 노드(`amr_docking` `aruco_localizer`)를 복구 노드에 붙이면(민지) 다시 잰다
