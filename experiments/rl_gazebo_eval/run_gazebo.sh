#!/usr/bin/env bash
# 연구 2 RL Gazebo 평가: 질량마다 rl_eval.launch.py를 띄워 같은 평가 목록을 돌리고, 같은 정책으로 2D도 돌려 비교한다.
# 작업 컨테이너 안에서 실행한다 (레포가 /ros2_ws/src에 마운트된 compose dev·gui 컨테이너). 2D 비교에 gymnasium 필요.
#   bash experiments/rl_gazebo_eval/run_gazebo.sh [에피소드 수, 기본 10] [정책, 기본 brake] [결과 폴더, 기본 /tmp/rl_gazebo_eval]
N=${1:-10}
POLICY=${2:-brake}
OUT=${3:-/tmp/rl_gazebo_eval}
HERE=$(cd "$(dirname "$0")" && pwd)
export AMR_RESEARCH2_DIR="$HERE/../research2"
source /opt/ros/humble/setup.bash
cd /ros2_ws || exit 1
colcon build --packages-select amr_description amr_gazebo > /tmp/rl_gazebo_build.log 2>&1 \
  || { echo "빌드 실패"; tail -20 /tmp/rl_gazebo_build.log; exit 1; }
source install/setup.bash
mkdir -p "$OUT"; rm -f "$OUT/gazebo.csv"

kill_all() {
  for p in "[i]gn gazebo" "[p]arameter_bridge" "[r]obot_state_publisher" "[r]l_eval_node" "amr_gazebo [r]l_eval.launch"; do
    pkill -9 -f "$p"; done
  sleep 5
}
for m in 0.0 10.0 25.0; do
  kill_all
  setsid nohup ros2 launch amr_gazebo rl_eval.launch.py payload_mass:=$m episodes:=$N policy:=$POLICY \
    csv:="$OUT/gazebo.csv" > "$OUT/launch_$m.log" 2>&1 < /dev/null &
  for _ in $(seq 360); do grep -qE "끝:|Traceback" "$OUT/launch_$m.log" && break; sleep 5; done
  grep -q Traceback "$OUT/launch_$m.log" && { grep -A8 Traceback "$OUT/launch_$m.log"; exit 1; }
  echo "질량 $m kg 끝"
done
kill_all
python3 "$HERE/run_2d.py" --payloads 0 10 25 --episodes "$N" --policy "$POLICY" --csv "$OUT/2d.csv"
python3 "$HERE/compare.py" "$OUT/gazebo.csv" "$OUT/2d.csv"
