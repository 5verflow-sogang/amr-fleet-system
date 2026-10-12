# DWA vs TEB 로컬 컨트롤러 비교 ((10) 작업)

우리 DWA(`amr_nav2_plugins::DwaController`, #103)와 TEB(`teb_local_planner`, #110 underlay)를 같은 Gazebo 데모에서
하네스 시나리오 `nav_compare`로 비교한다. 지표 정의·조건은 [docs/algorithms/navigation.md](../../docs/algorithms/navigation.md) 7절.

## 실행

호스트(Windows Git Bash)에서. 반복마다 새 컨테이너(`HARNESS_FRESH=1`), 한 번에 한 조합씩.

```bash
export MSYS_NO_PATHCONV=1 HARNESS_FRESH=1 HARNESS_PACKAGES="amr_bringup amr_nav2_plugins"
tools/harness/docker_run.sh nav_compare -n 3 --set controller=dwa --set scenario=clear    --label dwa_clear
tools/harness/docker_run.sh nav_compare -n 3 --set controller=dwa --set scenario=obstacle --label dwa_obstacle
tools/harness/docker_run.sh nav_compare -n 3 --set controller=teb --set scenario=clear    --label teb_clear
tools/harness/docker_run.sh nav_compare -n 3 --set controller=teb --set scenario=obstacle --label teb_obstacle
python3 experiments/teb_compare/analyze.py tools/harness/results/nav_compare-*_<시각>_*   # 아래 표
```

| 파일 | 내용 |
| --- | --- |
| `tools/harness/scenarios/nav_compare.yaml` | 데모 launch(`nav2_dwa_demo.launch.py send_goal:=false controller:=…`) + (obstacle면) 장애물 + 주행 명령 |
| `nav_measure.py` | 주행 명령. 목표 1곳을 보내고 GT 지표를 `nav_result.json`, 시계열을 `nav_series.csv`로 남긴다 |
| `obstacle.yaml` | 경로를 가로지르는 동적 장애물 1개(`dynamic_worker`, 0.5 m/s) |
| `analyze.py` | 하네스 결과 폴더들 → 아래 표 (ROS 불필요) |

하네스에 더한 것(범용): `drive.py`의 `command` 모드(명령이 끝날 때까지 시계를 돌림), `summary.py`의 `nav.*` 지표,
`docker_run.sh`의 `HARNESS_PACKAGES`(빌드 패키지 덮어쓰기)와 이미지 underlay(`/opt/teb_ws`) source.

## 조건

- 월드 v0 + 지도 `warehouse_v1`, 로봇 1대, 위치추정 없음(map→odom 항등, DiffDrive odom). 컨트롤러끼리 비교라 둘 다 같은 조건이면 된다
- 목표: map (0, 0) → (3.0, 1.5). 전역 계획 NavFn(A* 모드), costmap·풋프린트(0.6 × 0.4 m)·속도 한계(2.0 m/s, 1.5 rad/s, 1.0 m/s²)·goal checker 같음
- `clear`: 장애물 없음. `obstacle`: `dynamic_worker`(충돌 실린더 반지름 0.25 m, 명세 0.3 m에 가장 가까운 기존 모델)가 경로 중점(map x 1.5)을 남북으로 3 m 왕복(0.5 m/s, 12초 주기)
- 교차 시점 맞춤: 장애물이 북쪽 끝에 와 남쪽으로 출발할 때 목표를 보낸다. 그냥 보내면 Nav2 기동 시간에 따라 실행마다 만나는 시점이 달라 (예비 실행에서 TEB가 장애물을 거의 안 만났다) 비교가 안 된다
- 장애물 인식: TEB는 실제 LiDAR → costmap(`costmap_converter::CostmapToDynamicObstacles`)으로 본다. DWA는 LiDAR 군집화 노드가 아직 없어 `nav_measure.py`가 장애물 GT를 `perception/obstacles`로 그대로 보낸다 - DWA 쪽 인식이 지연·잡음 없는 GT라 **DWA에 유리한 비대칭**이다
- 지표는 전부 GT(`ground_truth/odom`): 도착 시간(목표 수락 → SUCCEEDED, 시뮬레이션 시간), 최소 장애물 거리(사각 풋프린트와 장애물 원 표면 사이), 경로 길이(GT 증분 합), CTE(그 시각의 전역 경로 `/amr_01/plan`에 투영)
- **성공 = Nav2 SUCCEEDED이고 충돌 없음.** 풋프린트와 장애물이 1 cm 미만으로 가까워진 적이 있으면 충돌로 본다 - Gazebo에서 닿은 몸체는 겹치지 않고 밀려나서 접촉 중에도 0.1~1 mm로 나온다(진단 실행에서 0.2 mm로 15초 붙어 있었다)

## 결과 (10/12, 조합마다 3회)

| 컨트롤러 | 시나리오 | 성공/전체 | Nav2 도착 | 충돌 | 도착 시간 [s] (성공만) | Nav2 도착 시간 [s] (충돌 포함) | 최소 장애물 거리 [cm] | 경로 길이 [m] | CTE 평균 [cm] | CTE 최대 [cm] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| DWA | clear | 3/3 | 3/3 | 0/3 | 9.5 (9.2~9.8) | 9.5 (9.2~9.8) | — | 3.14 (3.14~3.14) | 1.3 (1.2~1.4) | 3.9 (3.7~4.2) |
| DWA | obstacle | 0/3 | 3/3 | 3/3 | — | 39.5 (33.0~43.7) | 0.0 (0.0~0.0) | 4.44 (3.67~5.18) | 30.8 (22.2~39.3) | 138.2 (96.3~169.4) |
| TEB | clear | 3/3 | 3/3 | 0/3 | 6.8 (5.6~8.8) | 6.8 (5.6~8.8) | — | 3.36 (3.32~3.42) | 2.7 (2.3~3.5) | 8.1 (5.6~13.0) |
| TEB | obstacle | 0/3 | 3/3 | 3/3 | — | 7.2 (7.2~7.3) | 0.0 (0.0~0.0) | 4.01 (3.97~4.05) | 7.1 (5.3~8.4) | 28.3 (27.4~29.0) |

값은 평균(최소~최대). 도착 시간은 성공(Nav2 도착이고 충돌 없음)한 실행만, Nav2 도착 시간은 Nav2가 목표에 닿은 실행 모두(충돌 포함), 최소 장애물 거리는 obstacle만 (하네스 `nav.arrival_s`·`nav.reached_s`와 같은 정의). 하네스 단계 실패(기동 실패·timeout)는 없었다.

- **장애물 없음**: 둘 다 3/3. TEB가 30% 빠르다(6.8 vs 9.5 s) - 속도 한계는 같고, TEB는 시간 최적화라 가속·순항을 더 쓰고 DWA는 heading 가중치(0.8)가 커서 보수적이다. 대신 TEB는 경로를 더 크게 돌아(3.36 vs 3.14 m) CTE가 크다(최대 8.1 vs 3.9 cm). 둘 다 KPI(직선 5·곡선 10 cm) 근처지만 이 경로는 직선·곡선 구간을 나누지 않았다
- **장애물 있음**: 둘 다 3회 모두 **장애물에 닿았다**(성공 0/3). 목표에는 6회 모두 닿았다
  - TEB는 앞질러 지나가려다 닿는다(Nav2 도착 7.2 s, 경로 4.0 m). 장애물을 LiDAR→costmap으로 늦게 보고, 장애물 원이 아니라 costmap 점으로 본다
  - DWA는 쓸고 지나가는 선 바로 앞에서 기다리다 되돌아온 장애물에 옆면을 맞는다(Nav2 도착 39.5 s, 경로 4.4 m, CTE 최대 1.4 m의 큰 우회). GT 직결 인식으로도 못 피했다
  - 이 장애물은 로봇을 피하지 않고 같은 선을 12초마다 계속 쓸고 지나간다. 두 컨트롤러 모두 "등속 예측"으로는 왕복하는 장애물의 반환을 예측하지 못한다 - 컨트롤러 비교보다 시나리오가 가혹한 쪽에 가깝다. 아래 "다음" 참고
- 첫 장애물 배치(09:12·09:26)는 측정 스크립트가 충돌한 실행의 도착 시간을 남기지 않아(성공일 때만 기록) 같은 스크립트로 다시 돌렸다(09:35·09:40, 위 표). 첫 배치도 충돌 3/3·3/3로 결론은 같다(DWA는 그때 Nav2 도착 1/3)

원자료: `tools/harness/results/nav_compare-*_20261012-09{0933,1229,2321,2626,3530,4011}_*` (로컬, git 제외). 진단 실행(시계열): `nav_compare-diag_*_20261012-093*`.

## 다음

- 장애물 시나리오를 "한 번만 가로지르기"(왕복 없음)와 "로봇 앞에서 멈췄다 지나가기"로 나눠 다시 잰다 - 지금 왕복 시나리오는 두 컨트롤러 모두 0/3이라 차이를 못 가린다
- DWA 쪽 인식을 LiDAR 군집화(`amr_tracking`)로 바꿔 TEB와 같은 실제 센서 경로로 맞춘다 (지금은 DWA에 유리)
- 위치추정(EKF+AMCL)·지도 정렬(#98)을 켠 조건과 좁은 통로·곡선 목표 쌍으로 넓힌다 (7-4절)
