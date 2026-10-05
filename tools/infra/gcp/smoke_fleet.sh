#!/bin/bash
# 러너 스모크 시나리오 (컨테이너 안에서 실행). 하네스(tools/harness)가 나오기 전 러너 확인용이다.
# Fleet 서버 + 장애 주입기 + 가짜 로봇 5대 + 작업 생성기를 정해진 시간 돌리고 /fleet 토픽을 rosbag으로 남긴다.
# Gazebo는 띄우지 않는다.
#
#   bash smoke_fleet.sh <결과 폴더> [시간(s)=120] [delay_ms=100]
set -eo pipefail

out="$1"
duration="${2:-120}"
delay_ms="${3:-100}"
mkdir -p "$out"

cd /ros2_ws
colcon build --packages-up-to amr_fleet_manager amr_fault_injector --event-handlers console_cohesion- \
  > "$out/colcon_build.log" 2>&1
source install/setup.bash

pids=()
run() {
  "$@" > "$out/$(basename "$1")_$((${#pids[@]})).log" 2>&1 &
  pids+=($!)
}

run ros2 launch amr_fleet_manager fleet_manager.launch.py assignments_csv:="$out/assignments.csv"
run ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:="$delay_ms"
run ros2 launch amr_fleet_manager fake_robots.launch.py count:=5 relayed:=true
sleep 5
# 서버 원본, 로봇별 중계본, 로봇 → 서버 (하네스 README: Fleet 시나리오는 /fleet/*, /<robot>/fleet/* 녹화 필수)
run ros2 bag record -o "$out/rosbag" -e '/fleet/.*|/amr_0[1-5]/fleet/.*'
sleep 2
run ros2 launch amr_fleet_manager task_generator.launch.py seed:=0 schedule_out:="$out/tasks.json"

sleep "$duration"

# 녹화를 먼저 닫고 나머지를 끈다 (SIGINT로 끝내야 bag이 정리된다)
kill -INT "${pids[3]}" 2>/dev/null || true
sleep 3
for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
wait || true

ros2 bag info "$out/rosbag" | tee "$out/rosbag_info.txt"
echo "smoke done: duration=${duration}s delay_ms=${delay_ms}" | tee "$out/summary.txt"
# 할당이 한 번도 안 일어났으면 실패로 본다 (헤더 한 줄만 있으면 실패)
test "$(wc -l < "$out/assignments.csv")" -gt 1
