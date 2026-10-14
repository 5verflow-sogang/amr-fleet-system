#!/usr/bin/env bash
# 속도별 급정지 (#144 리뷰 1): 질량마다 rl_eval.launch.py mode:=brake_sweep를 띄워 제동 거리·지연을 재고 brake_sweep.py로 맞춘다.
# 작업 컨테이너 안에서 실행한다 (run_gazebo.sh와 같음).
#   bash experiments/rl_gazebo_eval/run_brake_sweep.sh [반복, 기본 3] [속도, 기본 0.5,1.0,1.5,2.0] [결과 폴더, 기본 /tmp/rl_gazebo_eval]
REPEATS=${1:-3}
SPEEDS=${2:-0.5,1.0,1.5,2.0}
OUT=${3:-/tmp/rl_gazebo_eval}
HERE=$(cd "$(dirname "$0")" && pwd)
export AMR_RESEARCH2_DIR="$HERE/../research2"
source /opt/ros/humble/setup.bash
cd /ros2_ws || exit 1
colcon build --packages-select amr_description amr_gazebo > /tmp/rl_gazebo_build.log 2>&1 \
  || { echo "빌드 실패"; tail -20 /tmp/rl_gazebo_build.log; exit 1; }
source install/setup.bash
mkdir -p "$OUT"; rm -f "$OUT/brake.csv"

kill_all() {
  for p in "[i]gn gazebo" "[p]arameter_bridge" "[r]obot_state_publisher" "[r]l_eval_node" "amr_gazebo [r]l_eval.launch"; do
    pkill -9 -f "$p"; done
  sleep 5
}
for m in 0.0 10.0 25.0; do
  kill_all
  setsid nohup ros2 launch amr_gazebo rl_eval.launch.py mode:=brake_sweep payload_mass:=$m sweep_speeds:=$SPEEDS \
    sweep_repeats:=$REPEATS csv:="$OUT/brake.csv" > "$OUT/brake_$m.log" 2>&1 < /dev/null &
  for _ in $(seq 360); do grep -qE "끝:|Traceback" "$OUT/brake_$m.log" && break; sleep 5; done
  grep -q Traceback "$OUT/brake_$m.log" && { grep -A8 Traceback "$OUT/brake_$m.log"; exit 1; }
  echo "질량 $m kg 끝"
done
kill_all
python3 "$HERE/brake_sweep.py" "$OUT/brake.csv"
