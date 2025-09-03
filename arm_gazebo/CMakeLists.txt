cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 모델, 월드 등의 리소스만 설치하도록 남겨둡니다.
install(
  DIRECTORY launch models worlds params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()