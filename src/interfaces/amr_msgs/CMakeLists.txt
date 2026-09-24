cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

find_package(ament_cmake REQUIRED)

# 인터페이스 변경은 .github/CONTRIBUTING.md 의 "인터페이스 변경 절차"를 따른다.
# TODO: 첫 메시지 추가 시 아래 주석 해제 (package.xml 도 함께)
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Example.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )
# ament_export_dependencies(rosidl_default_runtime)

ament_package()
