#!/usr/bin/env bash
# CTE 1회 측정: Gazebo 월드 v0 + 로봇 1대 + path_tracker_node + 고정 경로 (Nav2 없음).
# dev 컨테이너 안에서 워크스페이스를 빌드·source한 뒤 실행한다.
#   bash experiments/cte_baseline/run.sh <결과 폴더> [pure_pursuit|stanley] [추가 -p 이름:=값 ...]
set -u
OUT=${1:?결과 폴더}
CONTROLLER=${2:-pure_pursuit}
shift
[ $# -gt 0 ] && shift
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

cleanup() {
  kill "$TRACKER_PID" "$SIM_PID" 2>/dev/null
  sleep 2
  pkill -9 -f "[i]gn gazebo" || true
  pkill -9 -f "[p]arameter_bridge" || true
  pkill -9 -f "[p]ath_tracker_node" || true
}
TRACKER_PID=
SIM_PID=
trap cleanup EXIT

# EKF 없이 DiffDrive odom TF를 쓴다. 카메라·GUI·동적 장애물 끔
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false enable_camera:=false \
  publish_odom_tf:=true dynamic_obstacles:=false > "$OUT/sim.log" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 120); do
  timeout 3 ros2 topic echo --once /amr_01/ground_truth/odom > /dev/null 2>&1 && break
  sleep 1
done

# 추종기는 이상적 DiffDrive odom(diff_drive/odom)을 odom으로 받는다 (amr_odometry를 띄우지 않으므로).
# amr_zone_client·amr_safety 없이 재므로 cmd_vel_nav를 Gazebo cmd_vel에 바로 잇는다 (측정 전용)
ros2 run amr_control path_tracker_node --ros-args -r __ns:=/amr_01 -r __node:=path_tracker \
  --params-file "$(ros2 pkg prefix amr_control)/share/amr_control/config/path_tracker.yaml" \
  -r odom:=diff_drive/odom -r cmd_vel_nav:=cmd_vel -p use_sim_time:=true -p controller:="$CONTROLLER" \
  -p cte_csv_path:="$OUT/cte.csv" -p robot_id:=amr_01 "$@" > "$OUT/tracker.log" 2>&1 &
TRACKER_PID=$!
sleep 3

START=$(date +%s)
(cd "$HERE" && python3 path_publisher.py --out "$OUT" --timeout 1200)
RC=$?
echo "주행 실제 시간 $(( $(date +%s) - START )) s, rc=$RC"
kill -INT "$TRACKER_PID" 2>/dev/null
sleep 1
(cd "$HERE" && python3 analyze.py "$OUT")
