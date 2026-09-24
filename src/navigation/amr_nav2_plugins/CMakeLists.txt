cmake_minimum_required(VERSION 3.8)
project(amr_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# TODO: 구현 시 의존성 추가 (예: find_package(rclcpp REQUIRED))

# TODO: pluginlib_export_plugin_description_file(nav2_core plugins.xml)

if(BUILD_TESTING)
  # TODO: find_package(ament_cmake_gtest REQUIRED) 후 test/ 의 테스트 등록
endif()

ament_package()
