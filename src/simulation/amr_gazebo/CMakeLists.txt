cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 동적 장애물 이동 패턴 (amr_gazebo/obstacle_motion.py)과 노드
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/dynamic_obstacles_node.py
  DESTINATION lib/${PROJECT_NAME})

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_obstacle_motion test/test_obstacle_motion.py)
endif()

ament_package()
