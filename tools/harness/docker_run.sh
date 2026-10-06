#!/bin/bash
# 호스트에서 하네스를 컨테이너로 돌린다. 레포를 /repo에 올리고, 필요한 패키지만 빌드한 뒤 run.py를 부른다.
# 결과는 호스트의 tools/harness/results/에 남는다.
#
#   tools/harness/docker_run.sh loc_basic -n 10
#   HARNESS_IMAGE=amr-fleet-system:latest tools/harness/docker_run.sh fleet_fake -n 5 --set delay_ms=200
#
# HARNESS_IMAGE: 이미지 (기본 amr-fleet-system:latest)
# HARNESS_PREP:  빌드 전에 컨테이너에서 실행할 명령 (예: 이미지에 없는 apt 패키지 설치)
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${HARNESS_IMAGE:-amr-fleet-system:latest}"
PACKAGES="amr_gazebo amr_odometry amr_localization amr_fleet_manager amr_fault_injector"
exec docker run --rm -e HARNESS_IMAGE="$IMAGE" -e HARNESS_PREP="${HARNESS_PREP:-}" \
  -v "$REPO:/repo" -w /repo "$IMAGE" bash -c "
    set -e
    source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash
    if [ -n \"\$HARNESS_PREP\" ]; then bash -c \"\$HARNESS_PREP\"; fi
    colcon build --base-paths src --build-base /tmp/harness_ws/build --install-base /tmp/harness_ws/install \
      --packages-up-to $PACKAGES > /tmp/harness_build.log 2>&1 || { tail -30 /tmp/harness_build.log; exit 1; }
    source /tmp/harness_ws/install/setup.bash
    python3 tools/harness/run.py $(printf '%q ' "$@")"
