cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

ament_package()
