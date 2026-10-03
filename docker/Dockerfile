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

# TODO: rosdep 외 의존성 (예: ros2_tracing/LTTng)

# YOLO 추론 (amr_detection)
# ROS Humble의 cv_bridge는 NumPy 1.x로 컴파일되어 있어 numpy<2를 유지한다. opencv-python 4.12부터 numpy 2를 요구한다.
# ultralytics가 설치하는 torch가 setuptools를 84.x로 올리는데, 이는 colcon-core(<80 요구)와 맞지 않아
# colcon build가 실패하므로 ROS Humble 권장 버전으로 되돌린다.
RUN pip3 install --no-cache-dir ultralytics==8.4.164 "numpy<2" "opencv-python<4.12" \
    && pip3 install --no-cache-dir setuptools==58.2.0
ADD https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt /opt/models/yolov8n.pt

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
