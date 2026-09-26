cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 인터페이스 변경은 .github/CONTRIBUTING.md 의 "인터페이스 변경 절차"를 따른다.
rosidl_generate_interfaces(${PROJECT_NAME}
  # perception
  "msg/DetectedObject3D.msg"
  "msg/DetectedObject3DArray.msg"
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "msg/SafetyStatus.msg"
  # fleet
  "msg/Task.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskStatus.msg"
  "msg/RobotState.msg"
  "msg/FleetHeartbeat.msg"
  "msg/TaskBid.msg"
  "msg/TrafficCommand.msg"
  # task
  "action/Dock.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
