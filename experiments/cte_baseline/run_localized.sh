#!/usr/bin/env bash
# CTE 측정 (위치추정 켬): Gazebo 월드 + 로봇 1대 + EKF·AMCL(localization.launch) + path_tracker_node +
# 고정 경로 (Nav2 없음). run.sh(DiffDrive odom, EKF·AMCL 없음)의 위치추정 버전 - 결과 폴더를 분리해서 쓴다.
#   bash experiments/cte_baseline/run_localized.sh <결과 폴더> [local|filtered]
#
# local:    추종기 odom = odometry/local (ekf_local, <ns>/odom 기준) - 경로도 <ns>/odom 프레임
# filtered: 추종기 odom = odometry/filtered (ekf_global, map 기준, AMCL 보정) - 경로도 map 프레임
#   (path_tracker_node는 경로 프레임이 odom 메시지 프레임과 다르면 TF로 돌아가 버리므로, 두 프레임을
#   일치시켜야 실제로 그 추정을 쓴다 - amr_control/src/path_tracker_node.cpp robotPose() 참고)
set -u
OUT=${1:?결과 폴더}
SOURCE=${2:-local}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

case "$SOURCE" in
  local) ODOM_TOPIC=odometry/local; PLAN_FRAME=amr_01/odom ;;
  filtered) ODOM_TOPIC=odometry/filtered; PLAN_FRAME=map ;;
  *) echo "SOURCE는 local 또는 filtered (받음: $SOURCE)" >&2; exit 1 ;;
esac

cleanup() {
  kill "$TRACKER_PID" "$SIM_PID" 2>/dev/null
  sleep 2
  pkill -9 -f "[i]gn gazebo" || true
  pkill -9 -f "[p]arameter_bridge" || true
  pkill -9 -f "[p]ath_tracker_node" || true
  pkill -9 -f "[a]mcl" || true
  pkill -9 -f "[e]kf_node" || true
  pkill -9 -f "[m]ap_server" || true
  pkill -9 -f "[w]heel_odometry_node" || true
  pkill -9 -f "[s]can_filter_node" || true
  pkill -9 -f "[i]mu_filter_node" || true
}
TRACKER_PID=
SIM_PID=
trap cleanup EXIT

# 월드 + 로봇 1대(스폰 world (0, 1.5, 0) = path_def.py가 가정하는 출발 자세) + 오도메트리·센서 필터 +
# EKF·AMCL(지도 v1, #96). BT·Fleet·동적 장애물·카메라는 끈다
ros2 launch amr_bringup system.launch.py num_robots:=1 "robots_file:=$HERE/robots_single.yaml" \
  gui:=false dynamic_obstacles:=false enable_camera:=false bt:=false fleet:=false \
  localization:=true use_imu:=true seed:=1 > "$OUT/sim.log" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 180); do
  timeout 3 ros2 topic echo --once /amr_01/ground_truth/odom > /dev/null 2>&1 && break
  sleep 1
done
# AMCL·EKF가 뜨는 데 더 걸린다(지도 서버 + 라이프사이클 매니저 + amcl_pose 수렴).
# odometry/filtered(ekf_global, amcl_pose 입력)가 나와야 둘 다 준비된 것으로 본다
for _ in $(seq 1 180); do
  timeout 3 ros2 topic echo --once /amr_01/odometry/filtered > /dev/null 2>&1 && break
  sleep 1
done
# IMU 필터가 시작 후 2초간 가만히 있어야 바이어스를 잡는다(sensor_filters.launch.py) - 넉넉히 더 기다린다
sleep 5

# amr_zone_client·amr_safety 없이 cmd_vel_nav를 Gazebo cmd_vel에 바로 잇는다 (측정 전용, run.sh와 동일)
ros2 run amr_control path_tracker_node --ros-args -r __ns:=/amr_01 -r __node:=path_tracker \
  --params-file "$(ros2 pkg prefix amr_control)/share/amr_control/config/path_tracker.yaml" \
  -r "odom:=$ODOM_TOPIC" -r cmd_vel_nav:=cmd_vel -p use_sim_time:=true -p controller:=pure_pursuit \
  -p cte_csv_path:="$OUT/cte.csv" -p robot_id:=amr_01 > "$OUT/tracker.log" 2>&1 &
TRACKER_PID=$!
sleep 3

START=$(date +%s)
(cd "$HERE" && python3 path_publisher.py --out "$OUT" --frame "$PLAN_FRAME" --timeout 1200)
RC=$?
echo "주행 실제 시간 $(( $(date +%s) - START )) s, rc=$RC"
kill -INT "$TRACKER_PID" 2>/dev/null
sleep 1
# GT 비교는 #98 지도 정렬 보정값(map_origin_in_world [-0.0571, 1.5167, 0.00013], v1 0.05 m, amr_mapping README)
# 적용·미적용 둘 다 cte_summary.json에 남긴다
(cd "$HERE" && python3 analyze.py "$OUT" --correction -0.0571 1.5167 0.00013)
