cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# extrinsics.yaml 읽기 (amr_description/extrinsics.py). 센서 위치가 필요한 다른 패키지 Python 코드가 쓴다
ament_python_install_package(${PROJECT_NAME})

# OPTIONAL: 비어 있거나 없는 폴더는 건너뛴다
install(DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN ".gitkeep" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_extrinsics test/test_extrinsics.py)
endif()

ament_package()
