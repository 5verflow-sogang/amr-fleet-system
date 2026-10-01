# 공통 ROS2 Humble 개발·실행 이미지
# 의존성은 각 패키지 package.xml에 적고 rosdep으로 설치하는 것이 원칙이다.
# rosdep으로 해결되지 않는 것만 이 파일에 추가한다.
FROM ros:humble AS base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      git \
    && rm -rf /var/lib/apt/lists/*

# ros2_tracing (연구 1 측정 도구). 사용자 공간(UST) 트레이싱만 쓴다.
# 커널 트레이싱은 lttng-modules가 호스트 커널에 있어야 해서 컨테이너에서는 쓰지 않는다.
# 같은 목록과 스크립트를 .github/workflows/tracing-check.yml에서 검증한다.
RUN apt-get update && apt-get install -y --no-install-recommends \
      lttng-tools \
      liblttng-ust-dev \
      pkg-config \
      python3-lttng \
      python3-babeltrace \
      babeltrace \
      ros-humble-ros2trace \
      ros-humble-tracetools-launch \
    && rm -rf /var/lib/apt/lists/*

# Humble 바이너리 tracetools는 LTTng 없이 빌드돼 있어 같은 버전을 LTTng와 함께 빌드해 오버레이한다.
COPY docker/scripts/build_tracetools.sh /tmp/build_tracetools.sh
RUN bash /tmp/build_tracetools.sh /opt/tracing_ws && rm /tmp/build_tracetools.sh

# TODO: rosdep 외 의존성 (예: ultralytics)

WORKDIR /ros2_ws
COPY src ./src

RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/tracing_ws/install/setup.bash && colcon build

# tracing_ws가 /opt/ros/humble 위에 얹혀 있으므로 그것만 source해도 된다
RUN echo "source /opt/tracing_ws/install/setup.bash" >> ~/.bashrc \
    && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

# GUI 이미지 (compose profile: gui)
# 컨테이너 안의 가상 화면(Xvfb)에서 CPU 렌더링으로 RViz2 등을 띄우고, noVNC로 브라우저에 보여준다.
# macOS·Windows의 Docker Desktop은 컨테이너 GUI를 호스트 화면에 직접 띄우기 어렵기 때문이다.
FROM base AS gui

RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb \
      x11vnc \
      novnc \
      fluxbox \
      xterm \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1

# 1주차 임시 검증 환경 (compose profile: sandbox)
# 지수 로봇이 나오기 전까지 TurtleBot4 + Nav2로 위치 추정·경로 계획을 먼저 검증한다.
# TurtleBot3는 Gazebo Classic을 써서 Apple 칩(arm64)에 설치되지 않으므로, Fortress를 쓰는 TurtleBot4를 쓴다.
# TODO(2주차): 지수 로봇으로 교체되면 이 단계와 compose sandbox 서비스를 제거한다.
FROM base AS sandbox

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-turtlebot4-ignition-bringup \
      ros-humble-turtlebot4-navigation \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

# 카메라·LiDAR 센서가 화면(X display)이 있어야 렌더링되므로 가상 화면(Xvfb)을 쓴다. GPU 없이 CPU로 그린다.
ENV LIBGL_ALWAYS_SOFTWARE=1
