cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 동적 장애물 이동 패턴 (amr_gazebo/obstacle_motion.py)과 노드, 화물 상자 배치 (amr_gazebo/cargo.py),
# 연구 2 RL 평가 환경 (amr_gazebo/rl_eval.py, rl_policies.py, scripts/rl_eval_node.py),
# 시뮬레이션 시계 발행 속도 줄이기 (amr_gazebo/clock_throttle.py, scripts/clock_throttle_node.py)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/dynamic_obstacles_node.py scripts/rl_eval_node.py scripts/clock_throttle_node.py
  DESTINATION lib/${PROJECT_NAME})

# 바퀴 joint_states 발행 속도 줄이기 (src/joint_state_throttle_node.cpp, 규칙 include/amr_gazebo/throttle.hpp)
add_executable(joint_state_throttle src/joint_state_throttle_node.cpp)
target_include_directories(joint_state_throttle PRIVATE include)
target_compile_features(joint_state_throttle PRIVATE cxx_std_17)
ament_target_dependencies(joint_state_throttle rclcpp sensor_msgs)
install(TARGETS joint_state_throttle DESTINATION lib/${PROJECT_NAME})

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_obstacle_motion test/test_obstacle_motion.py)
  ament_add_pytest_test(test_cargo test/test_cargo.py)
  ament_add_pytest_test(test_rl_eval test/test_rl_eval.py)
  ament_add_pytest_test(test_clock_throttle test/test_clock_throttle.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_throttle test/test_throttle.cpp)
  target_include_directories(test_throttle PRIVATE include)
  target_compile_features(test_throttle PRIVATE cxx_std_17)
endif()

ament_package()
