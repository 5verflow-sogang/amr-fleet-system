#!/usr/bin/env bash
# D9 드리프트 실험: 10 m 사각형을 반시계·시계로 seed마다 돌고 휠 오도메트리 복귀 오차를 기록한다.
# 작업 컨테이너 안에서 실행한다 (레포가 /ros2_ws/src에 마운트된 compose dev·gui 컨테이너).
#   bash experiments/odom_drift/run_drift.sh [seed 개수, 기본 10] [결과 csv, 기본 /tmp/odom_drift/results.csv]
#
# 출발 자세 world (−5, 9). 반시계는 동쪽을 보고 출발해 (−5..5, 9..19), 시계는 북쪽을 보고 출발해 같은 사각형을 돈다.
# 메인 통로(벽·랙·기둥과 1 m 넘게 떨어짐), 동적 장애물 없음, 카메라 끔.
N=${1:-10}
CSV=${2:-/tmp/odom_drift/results.csv}
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
cd /ros2_ws || exit 1
colcon build --packages-select amr_description amr_gazebo amr_odometry > /tmp/odom_drift_build.log 2>&1 \
  || { echo "빌드 실패"; tail -20 /tmp/odom_drift_build.log; exit 1; }
source install/setup.bash
mkdir -p "$(dirname "$CSV")"; rm -f "$CSV"

kill_odom() { pkill -f "[w]heel_odometry_node"; sleep 2; }
kill_all() {
  kill_odom
  for p in "[i]gn gazebo" "[p]arameter_bridge" "[r]obot_state_publisher" "[r]os_gz_sim" "amr_gazebo [s]im.launch"; do pkill -9 -f "$p"; done
  sleep 10
}
kill_all

ok=no
for a in 1 2 3; do
  setsid nohup ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false enable_camera:=false x:=-5.0 y:=9.0 \
    > /tmp/odom_drift_sim.log 2>&1 < /dev/null &
  for _ in $(seq 60); do grep -q "OK creation" /tmp/odom_drift_sim.log && { ok=yes; break; }; sleep 1; done
  [ $ok = yes ] && break
  echo "  (스폰 멈춤, 재시도)"; kill_all
done
[ $ok = yes ] || { echo "스폰 실패"; exit 1; }
sleep 5

run_one() {  # run_one <dir> <face deg> <noise true|false> <seed>
  kill_odom
  setsid nohup ros2 launch amr_odometry odometry.launch.py encoder_noise:=$3 encoder_seed:=$4 \
    > /tmp/odom_drift_odo.log 2>&1 < /dev/null &
  for _ in $(seq 30); do ros2 topic list 2>/dev/null | grep -qx /amr_01/odom && break; sleep 1; done
  sleep 2
  scale=$(grep -o "radius scale L=[0-9.]* R=[0-9.]*" /tmp/odom_drift_odo.log | head -1 | sed 's/radius scale //;s/ /,/')
  python3 "$HERE/drift_square.py" --dir "$1" --face "$2" --label "noise=$3;seed=$4;scale=$scale" --csv "$CSV"
}

for spec in "ccw 0" "cw 90"; do
  set -- $spec
  run_one "$1" "$2" false 1
  for s in $(seq 1 "$N"); do run_one "$1" "$2" true "$s"; done
done
kill_all
echo "끝: $CSV"
