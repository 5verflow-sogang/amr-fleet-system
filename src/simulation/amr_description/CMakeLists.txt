cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

ament_package()
