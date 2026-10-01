#!/usr/bin/env bash
# Humble 바이너리의 tracetools는 LTTng 없이 빌드돼 tracepoint가 비어 있다 ("Tracing disabled").
# 설치된 바이너리와 같은 버전의 tracetools를 LTTng와 함께 소스 빌드해 오버레이한다.
# rcl·rclcpp는 바이너리 그대로 두고, 이들이 호출하는 libtracetools만 바뀐다.
# 필요: lttng-tools, liblttng-ust-dev, pkg-config, git
# Dockerfile과 .github/workflows/tracing-check.yml이 같이 쓴다.
set -euo pipefail

ws=${1:-/opt/tracing_ws}

set +u
source /opt/ros/humble/setup.bash
set -u

version=$(dpkg-query -W -f='${Version}' ros-humble-tracetools | cut -d- -f1)
echo "tracetools ${version} 소스 빌드 → ${ws}"

mkdir -p "${ws}/src"
git clone --depth 1 -b "${version}" https://github.com/ros2/ros2_tracing.git "${ws}/src/ros2_tracing"
cd "${ws}"
colcon build --packages-select tracetools \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
rm -rf build log src
