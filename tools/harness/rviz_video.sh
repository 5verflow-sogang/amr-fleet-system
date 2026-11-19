#!/usr/bin/env bash
# 로봇 시점 RViz 영상: 하네스 녹화의 view_bag(시나리오 params view_robot)을 RViz(amr_bringup rviz/robot_view.rviz)에 다시 틀어
# 화면을 mp4로 남긴다. 클라우드 하네스 이미지(base)에는 RViz·가상 화면이 없어 녹화 뒤 로컬 gui 이미지(Xvfb·RViz2)에서 돈다.
#
#   tools/harness/rviz_video.sh <run_dir>/view_bag [로봇=amr_01] [출력 mp4=<run_dir>/video/rviz_<로봇>.mp4]
#   (먼저 docker compose --profile gui build gui 로 amr-fleet-system:gui를 만든다)
# 영상 0초 = /demo/t0 (RVIZ_VIDEO_PRE=초 를 주면 그만큼 앞에서 시작해 그 초가 t0). 시뮬레이션 1 s = 영상 1 s.
# bag 수신 시각은 실제 시간(RTF만큼 느리고 실행 중에 바뀐다)이라 view_bag_simtime.py로 bag 안의 /clock 기준 시각으로 바꿔
# 1배속(--clock)으로 튼다. 다른 장면(1-3 등)도 로봇 이름만 주면 된다 - 설정 안의 amr_01/을 그 이름으로 바꾼다
# (amr_bringup.robot_view와 같은 규칙). 화면 1280x720, 15 fps.
set -euo pipefail
[ $# -ge 1 ] || { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
BAG=$(cd "$1" && pwd)
ROBOT=${2:-amr_01}
OUT=${3:-$(dirname "$BAG")/video/rviz_${ROBOT}.mp4}
mkdir -p "$(dirname "$OUT")"
OUT_DIR=$(cd "$(dirname "$OUT")" && pwd)
REPO=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=${RVIZ_VIDEO_IMAGE:-amr-fleet-system:gui}

MSYS_NO_PATHCONV=1 docker run --rm -e ROBOT="$ROBOT" -e NAME="$(basename "$OUT")" -e PRE="${RVIZ_VIDEO_PRE:-0}" \
  -e TIME_PANEL="${RVIZ_VIDEO_TIME_PANEL:-0}" \
  -e ROS_DOMAIN_ID=77 -v "$REPO/src/bringup/amr_bringup/rviz:/rviz:ro" -v "$REPO/tools/harness:/harness:ro" \
  -v "$BAG:/bag:ro" -v "$OUT_DIR:/out" "$IMAGE" bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  command -v ffmpeg > /dev/null || (apt-get update -qq && apt-get install -y -qq --no-install-recommends ffmpeg > /dev/null)
  START=$(python3 /harness/view_bag_simtime.py /bag /tmp/sim --start t0 --pre "$PRE" | tail -1)
  echo "시작 시뮬레이션 ${START} s"
  Xvfb :99 -screen 0 1280x720x24 -nocursor > /dev/null 2>&1 &
  export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
  sed "s#amr_01/#${ROBOT}/#g" /rviz/robot_view.rviz > /tmp/view.rviz
  # RVIZ_VIDEO_TIME_PANEL=1: 아래에 Time 패널(ROS 시각)을 띄운다 - 영상 0초가 t0인지 확인할 때
  if [ "$TIME_PANEL" = 1 ]; then
    sed -i "s#^Panels: \[\]#Panels:\n  - Class: rviz_common/Time\n    Name: Time#" /tmp/view.rviz
  fi
  ros2 bag play /tmp/sim --clock 100 --start-paused > /tmp/play.log 2>&1 &
  PLAY=$!
  sleep 3
  rviz2 -d /tmp/view.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
  sleep 12
  # 재생 시계(/clock)가 시작 시각을 넘는 순간의 실제 시각을 남긴다 - 영상 자르기 기준 (서비스 호출 시각은 0.4 s쯤 어긋났다)
  python3 -c "
import time, rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node(\"clock_mark\"); done = []
def cb(m):
    if not done and m.clock.sec + m.clock.nanosec * 1e-9 > ${START} + 0.02:
        done.append(time.time()); open(\"/tmp/t_go\", \"w\").write(repr(done[0]))
n.create_subscription(Clock, \"/clock\", cb, qos_profile_sensor_data)
while not done: rclpy.spin_once(n, timeout_sec=0.01)
" &
  MARK=$!
  sleep 3
  # 프레임 시각 = 실제 시각 (첫 프레임 시각을 파일에서 읽는다 - ffmpeg가 뜨는 데 걸리는 시간만큼 어긋나지 않게)
  ffmpeg -loglevel error -y -use_wallclock_as_timestamps 1 -f x11grab -draw_mouse 0 -video_size 1280x720 -framerate 15 -i :99 \
    -copyts -c:v libx264 -preset veryfast -pix_fmt yuv420p /tmp/raw.mkv &
  FF=$!
  sleep 1
  ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > /dev/null
  # 재생 /clock은 best effort라 위 구독도 sensor_data QoS. 10 s 안에 못 받으면 막히지 않게 멈춘다
  for i in $(seq 1 100); do [ -f /tmp/t_go ] && break; sleep 0.1; done
  kill "$MARK" 2> /dev/null || true
  [ -f /tmp/t_go ] || { echo "재생 /clock을 못 받았다"; kill "$PLAY" "$FF" 2> /dev/null; exit 1; }
  T_GO=$(cat /tmp/t_go)
  wait "$PLAY" || true
  sleep 1
  kill -INT "$FF"; wait "$FF" || true
  # 녹화 시작부터 재생 시계가 시작 시각을 넘은 순간까지 잘라 영상 0초 = 시작 시각(t0 - PRE)
  T_FF=$(ffprobe -v error -show_entries format=start_time -of csv=p=0 /tmp/raw.mkv)
  CUT=$(python3 -c "print(round($T_GO - $T_FF, 3))")
  ffmpeg -loglevel error -y -ss "$CUT" -i /tmp/raw.mkv -r 15 -c:v libx264 -preset veryfast -pix_fmt yuv420p \
    "/out/${NAME}"
  echo "앞 ${CUT} s 자름, 영상 ${PRE} s = /demo/t0"
  ls -la "/out/${NAME}"'
echo "$OUT"
