cmake_minimum_required(VERSION 3.8)
project(arm_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# ... other dependencies

# ...

# python 스크립트를 실행 파일로 설치합니다.
install(
  PROGRAMS src/aruco_detector_node.py  # 경로를 src/ 로 변경
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
