# cte_baseline

경로 추종 CTE 측정 (담당: 이진). Gazebo 월드, 로봇 1대, Nav2 없이 고정 경로를 퍼블리셔로 주고
`amr_control` `path_tracker_node`로 주행한다. 결과와 해석은 [docs/algorithms/navigation.md](../../docs/algorithms/navigation.md) 5-8·5-9절.

## 구성

| 파일 | 내용 |
| --- | --- |
| `path_def.py` | 고정 경로 (출발 자세 = 원점): 직선 3 m → 왼쪽 반원 R = 1 m → 돌아오는 직선 3 m, 점 간격 5 cm. 투영·CTE 계산 |
| `path_publisher.py` | `/amr_01/plan`을 시작 직후 3번 내고, `/amr_01/ground_truth/odom`을 이 출발 자세 기준으로 옮겨 `gt.csv`에 기록. 경로 끝에서 2초 서 있으면 끝 |
| `analyze.py` | `cte.csv`(추종기, odom 기준)와 `gt.csv`(GT 재투영)를 직선·곡선, 경로 순서별로 요약 → `cte_summary.json`. `--correction DX DY DYAW`로 #98 지도 정렬 보정 적용본도 같이 냄 (ROS 불필요) |
| `run.sh` | 위치추정 없이(DiffDrive 이상적 odom): Gazebo(`sim.launch.py`) → 추종기 → 퍼블리셔 → 분석 |
| `run_localized.sh` | 위치추정 켜고(EKF+AMCL, `amr_bringup system.launch.py`): 추종기 odom을 `odometry/local`(ekf_local) 또는 `odometry/filtered`(ekf_global, AMCL 보정)로 선택 |
| `robots_single.yaml` | `run_localized.sh`가 쓰는 1대짜리 `robots_file` (스폰 world (0, 1.5, 0) = `path_def.py` 가정과 같음) |

측정 전용 연결(둘 다): `cmd_vel_nav` → `cmd_vel` (`amr_zone_client`·`amr_safety` 없이 바로).
`run.sh`는 추종기 `odom` ← `diff_drive/odom`(이상적 DiffDrive odom, EKF 없음).

## 실행

```bash
# 호스트에서 (experiments를 컨테이너에 마운트. Windows Git Bash는 Windows 경로로 줘야 마운트가 된다)
docker compose run --rm -v "C:\path\to\repo\experiments:/ros2_ws/experiments" dev bash -c \
  "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-up-to amr_control amr_gazebo amr_description \
   && source install/setup.bash && bash experiments/cte_baseline/run.sh experiments/cte_baseline/raw/pp_run1 pure_pursuit"

# 다른 제어기·파라미터
bash experiments/cte_baseline/run.sh <결과 폴더> stanley -p velocity_pid.enabled:=true

# 위치추정 켜고(EKF+AMCL). colcon build --packages-up-to amr_bringup amr_control로 바꿔야 한다
bash experiments/cte_baseline/run_localized.sh <결과 폴더> local      # ekf_local, <ns>/odom 기준
bash experiments/cte_baseline/run_localized.sh <결과 폴더> filtered   # ekf_global, map 기준(AMCL 보정)
```

결과 폴더: `cte.csv`, `gt.csv`, `cte_summary.json`, `sim.log`, `tracker.log`. `raw/`는 git에서 무시된다.

## 기록

| 날짜 | 실행 | 조건 | 결과 |
| --- | --- | --- | --- |
| 10/7 | `raw/pp_run1` | Pure Pursuit 기본값, PID 끔, 0.5 m/s, 위치추정 없음(DiffDrive 이상적 odom), 실제 시간 주행 21 s | odom 기준 최대 2.4 cm, GT 기준 직선 2에서 최대 31 cm (odom 누적 오차, EKF·AMCL 없어서). navigation.md 5-8절 |
| 10/10 | `raw/localized_local_run1` | 위와 같되 위치추정 켬(EKF+AMCL, 지도 v1), 추종기 odom = `odometry/local`, 실제 시간 주행 22 s | GT 기준 직선 최대 3.11 cm·곡선 최대 2.89 cm. KPI(직선 5 cm·곡선 10 cm) 충족. navigation.md 5-9절 |
| 10/10 | `raw/localized_filtered_run1` | 위와 같되 추종기 odom = `odometry/filtered`(AMCL 보정, map 기준) | GT 기준(#98 지도 정렬 미보정) 직선 최대 **8.60 cm로 KPI 미충족** — #98 보정 적용하면 4.58 cm로 충족. navigation.md 5-9절 |

원자료는 로컬에만 있다(공유 드라이브 업로드 전).
