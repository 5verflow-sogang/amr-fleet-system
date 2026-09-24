cmake_minimum_required(VERSION 3.8)
project(amr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# TODO: 구현 시 의존성 추가 (예: find_package(rclcpp REQUIRED))

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  # TODO: find_package(ament_cmake_gtest REQUIRED) 후 test/ 의 테스트 등록
endif()

ament_package()
