#!/usr/bin/env bash
# 속도 PID 계단 응답 1회: Gazebo 월드 v0 + 로봇 1대(payload_mass) + path_tracker_node(velocity_pid) +
# 직선 경로. dev 컨테이너 안에서 워크스페이스를 빌드·source한 뒤 실행한다.
#   bash experiments/velocity_pid_tuning/run_one.sh <결과 폴더> <적재질량kg> <라벨> <enabled true|false> <kp> <ki> <kd> [target_mps]
set -u
OUT=${1:?결과 폴더}
MASS=${2:?적재질량 kg}
LABEL=${3:?라벨}
ENABLED=${4:?velocity_pid.enabled}
KP=${5:?kp}
KI=${6:?ki}
KD=${7:?kd}
TARGET=${8:-0.5}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

cleanup() {
  kill "$TRACKER_PID" "$SIM_PID" 2>/dev/null
  sleep 2
  pkill -9 -f "[i]gn gazebo" || true
  pkill -9 -f "[p]arameter_bridge" || true
  pkill -9 -f "[p]ath_tracker_node" || true
  # sim.launch.py가 띄우는 자식 노드들(clock_throttle_node.py, joint_state_throttle)은
  # SIM_PID(ros2 launch)에 SIGTERM을 보내도 안 끝나고 남는 걸 실측으로 확인했다(leftover
  # 프로세스가 다음 트라이얼의 /clock과 뒤섞여 "jump back in time"을 일으켰다) - 이름으로
  # 직접 정리한다
  pkill -9 -f "[c]lock_throttle_node" || true
  pkill -9 -f "[j]oint_state_throttle" || true
  pkill -9 -f "[r]obot_state_publisher" || true
  pkill -9 -f "[r]uby.*gz" || true
}
TRACKER_PID=
SIM_PID=
trap cleanup EXIT

echo "=== mass=${MASS}kg label=${LABEL} enabled=${ENABLED} kp=${KP} ki=${KI} kd=${KD} ===" >&2

# 트라이얼 사이에 이전 시뮬레이션의 DDS 흔적(ros2 daemon 토픽 캐시, 죽은 프로세스의 공유 메모리)이
# 남아 있으면 /clock이 거꾸로 튀는 등 혼선이 생긴다(tools/harness/harness/runner.py dds_reset와
# 같은 이유) - 매 트라이얼 시작 전에 지운다. shm clean은 쓰는 중인 파일은 안 건드려 다른
# 컨테이너(pp)에 안전하다.
ros2 daemon stop >/dev/null 2>&1 || true
fastdds shm clean >/dev/null 2>&1 || true

# EKF 없이 DiffDrive odom TF를 쓴다. 카메라·GUI·동적 장애물 끔 (cte_baseline/run.sh와 같은 패턴)
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false enable_camera:=false \
  publish_odom_tf:=true dynamic_obstacles:=false "payload_mass:=${MASS}" \
  > "$OUT/sim_${LABEL}_${MASS}kg.log" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 120); do
  timeout 3 ros2 topic echo --once /amr_01/ground_truth/odom > /dev/null 2>&1 && break
  sleep 1
done

# max_linear_accel을 크게 줘서(사실상 한 틱 안에 목표 속도) 추종기 자체의 가속 램프가 PID 계단
# 응답을 가리지 않게 한다. max_linear_jerk는 이 실험에서는 끈다(0) - 저크 제한은 별도
# 기능(navigation.md 5-3절)이라 PID 응답과 섞이면 분석이 흐려진다. zone_client·safety 없이
# cmd_vel_nav를 Gazebo cmd_vel에 바로 잇는다(측정 전용, cte_baseline/run.sh와 같다)
ros2 run amr_control path_tracker_node --ros-args -r __ns:=/amr_01 -r __node:=path_tracker \
  --params-file "$(ros2 pkg prefix amr_control)/share/amr_control/config/path_tracker.yaml" \
  -r odom:=diff_drive/odom -r cmd_vel_nav:=cmd_vel -p use_sim_time:=true \
  -p desired_speed:="${TARGET}" -p max_linear_accel:=50.0 -p max_linear_jerk:=0.0 \
  -p "velocity_pid.enabled:=${ENABLED}" -p "velocity_pid.kp:=${KP}" \
  -p "velocity_pid.ki:=${KI}" -p "velocity_pid.kd:=${KD}" -p robot_id:=amr_01 \
  > "$OUT/tracker_${LABEL}_${MASS}kg.log" 2>&1 &
TRACKER_PID=$!
sleep 3

(cd "$HERE" && python3 step_response.py --out "$OUT" --mass "$MASS" --label "$LABEL" \
  --enabled "$ENABLED" --kp "$KP" --ki "$KI" --kd "$KD" --target "$TARGET" --duration 6.0)
RC=$?
kill -INT "$TRACKER_PID" 2>/dev/null
sleep 1
exit $RC
