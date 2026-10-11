cmake_minimum_required(VERSION 3.8)
project(amr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS 비의존 코어 (경로 기하, Pure Pursuit, Stanley, 추종기, 선속도 PID, cte.csv). ROS 의존성을 붙이지 않는다
add_library(amr_control_core
  src/core/path_geometry.cpp
  src/core/pure_pursuit.cpp
  src/core/stanley.cpp
  src/core/path_tracker.cpp
  src/core/cte_log.cpp
  src/core/pid.cpp
)
target_include_directories(amr_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node amr_control_core)
ament_target_dependencies(path_tracker_node geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

install(TARGETS amr_control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS path_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp)
  target_link_libraries(test_path_geometry amr_control_core)
  ament_add_gtest(test_path_tracker test/test_path_tracker.cpp)
  target_link_libraries(test_path_tracker amr_control_core)
  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid amr_control_core)
  ament_add_gtest(test_cte_log test/test_cte_log.cpp)
  target_link_libraries(test_cte_log amr_control_core)
endif()

ament_package()
