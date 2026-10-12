#!/usr/bin/env bash
# 도킹 KPI 실험: 스테이션 근처에 로봇을 흩어 놓고 docking/dock을 불러 끝난 자세를 Gazebo GT로 잰다.
# 작업 컨테이너 안에서 실행한다 (레포가 /ros2_ws/src에 마운트된 compose dev·gui 컨테이너).
#   bash experiments/docking_kpi/run_docking.sh [스테이션마다 횟수, 기본 6] [결과 csv, 기본 /tmp/docking_kpi/results.csv]
#
# 로봇 1대(카메라 켬), 마커판 28곳, 동적 장애물 없음. 도킹 서버는 cmd_vel로 바로 낸다(Nav2·구역·안전 체인 없이).
# 시작 자세는 dock_trials.py 참고 (Nav2 도착 오차 흉내: 남은 거리 −15~+25 cm, 옆 ±15 cm, 방향 ±10°).
N=${1:-6}
CSV=${2:-/tmp/docking_kpi/results.csv}
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
cd /ros2_ws || exit 1
colcon build --packages-select amr_msgs amr_description amr_gazebo amr_docking > /tmp/docking_kpi_build.log 2>&1 \
  || { echo "빌드 실패"; tail -20 /tmp/docking_kpi_build.log; exit 1; }
source install/setup.bash
mkdir -p "$(dirname "$CSV")"; rm -f "$CSV"

kill_all() {
  for p in "lib/amr_docking/[d]ocking_server" "[i]gn gazebo" "[p]arameter_bridge" "[r]obot_state_publisher" \
           "[r]os_gz_sim" "amr_gazebo [s]im.launch"; do pkill -9 -f "$p"; done
  sleep 10
}
kill_all

ok=no
for a in 1 2 3; do
  setsid nohup ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false enable_camera:=true x:=0.0 y:=2.2 yaw:=1.5708 \
    > /tmp/docking_kpi_sim.log 2>&1 < /dev/null &
  for _ in $(seq 60); do grep -q "OK creation" /tmp/docking_kpi_sim.log && { ok=yes; break; }; sleep 1; done
  [ $ok = yes ] && break
  echo "  (스폰 멈춤, 재시도)"; kill_all
done
[ $ok = yes ] || { echo "스폰 실패"; exit 1; }
sleep 5
ros2 launch amr_docking aruco_markers.launch.py > /tmp/docking_kpi_markers.log 2>&1
setsid nohup ros2 launch amr_docking docking.launch.py namespace:=amr_01 cmd_vel_topic:=cmd_vel \
  > /tmp/docking_kpi_server.log 2>&1 < /dev/null &
sleep 5

python3 -u "$HERE/dock_trials.py" --stations rack_a4 rack_c4 charger_c2 dock_2 --per-station "$N" --csv "$CSV"
python3 "$HERE/analyze.py" "$CSV"
kill_all
