# cte_baseline

경로 추종 CTE 1회 측정 (담당: 이진). Gazebo 월드 v0, 로봇 1대, Nav2 없이 고정 경로를 퍼블리셔로 주고
`amr_control` `path_tracker_node`로 주행한다. 결과와 해석은 [docs/algorithms/navigation.md](../../docs/algorithms/navigation.md) 5-8절.

## 구성

| 파일 | 내용 |
| --- | --- |
| `path_def.py` | 고정 경로 (odom 프레임): 직선 3 m → 왼쪽 반원 R = 1 m → 돌아오는 직선 3 m, 점 간격 5 cm. 투영·CTE 계산 |
| `path_publisher.py` | `/amr_01/plan`을 시작 직후 3번 내고, `/amr_01/ground_truth/odom`을 odom 프레임으로 옮겨 `gt.csv`에 기록. 경로 끝에서 2초 서 있으면 끝 |
| `analyze.py` | `cte.csv`(추종기, odom 기준)와 `gt.csv`(GT 재투영)를 직선·곡선, 경로 순서별로 요약 → `cte_summary.json` (ROS 불필요) |
| `run.sh` | Gazebo(`sim.launch.py`, GUI·카메라 끔) → 추종기 → 퍼블리셔 → 분석 |

측정 전용 연결: 추종기 `odom` ← `diff_drive/odom`(이상적 DiffDrive odom, EKF 없음), `cmd_vel_nav` → `cmd_vel`
(`amr_zone_client`·`amr_safety` 없이 바로).

## 실행

```bash
# 호스트에서 (experiments를 컨테이너에 마운트)
docker compose run --rm -v "$PWD/experiments:/ros2_ws/experiments" dev bash -c \
  "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select amr_control amr_gazebo amr_description \
   && source install/setup.bash && bash experiments/cte_baseline/run.sh experiments/cte_baseline/raw/pp_run1 pure_pursuit"

# 다른 제어기·파라미터
bash experiments/cte_baseline/run.sh <결과 폴더> stanley -p velocity_pid.enabled:=true
```

결과 폴더: `cte.csv`, `gt.csv`, `cte_summary.json`, `sim.log`, `tracker.log`. `raw/`는 git에서 무시된다.

## 기록

| 날짜 | 실행 | 조건 | 결과 |
| --- | --- | --- | --- |
| 10/7 | `raw/pp_run1` | Pure Pursuit 기본값, PID 끔, 0.5 m/s, 실제 시간 주행 21 s (Gazebo 기동 약 2.5분) | odom 기준 최대 2.4 cm, GT 기준 직선 2에서 최대 31 cm (odom 누적 오차). navigation.md 5-8절 |

원자료는 로컬에만 있다(공유 드라이브 업로드 전).
