#!/usr/bin/env bash
# 로봇 시점 RViz 영상: 하네스 녹화의 view_bag(시나리오 params view_robot)을 RViz(amr_bringup rviz/robot_view.rviz)에 다시 틀어
# 화면을 mp4로 남긴다. 클라우드 하네스 이미지(base)에는 RViz·가상 화면이 없고, 녹화 중 화면을 찍으면 RTF(약 0.1)만큼
# 느려진 실제 시간으로 찍혀서 bag을 시뮬레이션 시계(--clock)로 다시 트는 쪽을 쓴다. gui 이미지(Xvfb·RViz2)에서 돈다.
#
#   tools/harness/rviz_video.sh <run_dir>/view_bag [로봇=amr_01] [출력 mp4=<run_dir>/video/rviz_<로봇>.mp4]
#   (먼저 docker compose --profile gui build gui 로 amr-fleet-system:gui를 만든다)
# 다른 장면(1-3 등)도 로봇 이름만 주면 된다 - 설정 안의 amr_01/을 그 이름으로 바꾼다 (amr_bringup.robot_view와 같은 규칙).
# 화면 1280x720, 15 fps. 앞뒤 자르기는 /demo/t0(bag 안)을 보고 따로 한다.
set -euo pipefail
[ $# -ge 1 ] || { sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
BAG=$(cd "$1" && pwd)
ROBOT=${2:-amr_01}
OUT=${3:-$(dirname "$BAG")/video/rviz_${ROBOT}.mp4}
mkdir -p "$(dirname "$OUT")"
OUT_DIR=$(cd "$(dirname "$OUT")" && pwd)
REPO=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=${RVIZ_VIDEO_IMAGE:-amr-fleet-system:gui}

MSYS_NO_PATHCONV=1 docker run --rm -e ROBOT="$ROBOT" -e NAME="$(basename "$OUT")" -e ROS_DOMAIN_ID=77 \
  -v "$REPO/src/bringup/amr_bringup/rviz:/rviz:ro" -v "$BAG:/bag:ro" -v "$OUT_DIR:/out" "$IMAGE" bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  command -v ffmpeg > /dev/null || (apt-get update -qq && apt-get install -y -qq --no-install-recommends ffmpeg > /dev/null)
  Xvfb :99 -screen 0 1280x720x24 > /dev/null 2>&1 &
  export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
  sed "s#amr_01/#${ROBOT}/#g" /rviz/robot_view.rviz > /tmp/view.rviz
  sleep 2
  rviz2 -d /tmp/view.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
  sleep 10
  ffmpeg -loglevel error -y -f x11grab -video_size 1280x720 -framerate 15 -i :99 \
    -c:v libx264 -pix_fmt yuv420p "/out/${NAME}" &
  FF=$!
  ros2 bag play /bag --clock 100
  kill -INT "$FF"; wait "$FF" || true
  ls -la "/out/${NAME}"'
echo "$OUT"
