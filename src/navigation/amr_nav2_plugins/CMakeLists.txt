cmake_minimum_required(VERSION 3.8)
project(amr_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# tracetools가 두 경로에 있다(/opt/ros/humble/lib, docker/scripts/build_tracetools.sh가
# LTTng를 넣어 다시 빌드한 /opt/tracing_ws/install) - 이 패키지가 그 둘을 모두 요구하는
# 의존성(nav2_core 등은 원본, amr_msgs는 새로 빌드되며 오버레이 경로로 풀린다)을 같이
# 링크해서 CMake가 build-tree RPATH 순서를 못 정한다("cycle in the constraint graph").
# install RPATH(고정된 가장 짧은 경로 집합)를 build에도 그대로 쓰게 하면 그 순서 계산을
# 안 거친다.
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(amr_planning_core REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  amr_msgs
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

add_library(waypoint SHARED
  src/waypoint.cpp
)
target_include_directories(waypoint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(waypoint geometry_msgs nav_msgs)

add_library(dwa_controller SHARED
  src/dwa_controller.cpp
)
target_include_directories(dwa_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dwa_controller
  waypoint
  amr_planning_core::amr_planning_core
)
ament_target_dependencies(dwa_controller ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS waypoint dwa_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(waypoint dwa_controller)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies} amr_planning_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_waypoint test/test_waypoint.cpp)
  target_link_libraries(test_waypoint waypoint)

  ament_add_gtest(test_obstacles test/test_obstacles.cpp)
  target_include_directories(test_obstacles PRIVATE include)
  target_link_libraries(test_obstacles amr_planning_core::amr_planning_core)
  ament_target_dependencies(test_obstacles amr_msgs)

  ament_add_gtest(test_avoid_mode test/test_avoid_mode.cpp)
  target_include_directories(test_avoid_mode PRIVATE include)

  ament_add_gtest(test_plugin_loadable test/test_plugin_loadable.cpp)
  target_link_libraries(test_plugin_loadable dwa_controller)
  ament_target_dependencies(test_plugin_loadable pluginlib nav2_core)
endif()

ament_package()
