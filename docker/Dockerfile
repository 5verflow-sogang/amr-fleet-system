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

# TODO: rosdep 외 의존성 (예: ultralytics, ros2_tracing/LTTng)

WORKDIR /ros2_ws
COPY src ./src

RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
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
