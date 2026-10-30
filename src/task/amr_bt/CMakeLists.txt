cmake_minimum_required(VERSION 3.8)
project(amr_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# BT 노드 (ROS 노드 없이 테스트할 수 있게 라이브러리로 분리)
add_library(amr_bt_nodes src/bt_nodes.cpp src/check_battery.cpp)
target_include_directories(amr_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_bt_nodes amr_msgs behaviortree_cpp_v3 geometry_msgs)

add_executable(amr_bt_node src/amr_bt_node.cpp)
target_link_libraries(amr_bt_node amr_bt_nodes)
ament_target_dependencies(amr_bt_node
  ament_index_cpp amr_msgs behaviortree_cpp_v3 nav2_msgs rclcpp rclcpp_action std_msgs tf2 tf2_ros)

install(TARGETS amr_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS amr_bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_assignment_filter test/test_assignment_filter.cpp)
  target_include_directories(test_assignment_filter PRIVATE include)
  ament_add_gtest(test_task_cycle test/test_task_cycle.cpp)
  target_link_libraries(test_task_cycle amr_bt_nodes)
  target_compile_definitions(test_task_cycle PRIVATE
    TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task_cycle.xml")
  ament_add_gtest(test_check_battery test/test_check_battery.cpp)
  target_link_libraries(test_check_battery amr_bt_nodes)
  target_compile_definitions(test_check_battery PRIVATE
    TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task_cycle.xml")

  # 가짜 Nav2·Dock 액션 서버로 실제 노드 한 사이클 (launch_testing)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bt_actions.launch.py TIMEOUT 120)
endif()

ament_package()
