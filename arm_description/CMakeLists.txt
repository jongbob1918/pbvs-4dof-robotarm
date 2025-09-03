# CMakeLists.txt (arm_description)

cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

# --- only install existing resource dirs ---
set(RESOURCE_DIRS urdf meshes rviz launch config)
foreach(dir IN LISTS RESOURCE_DIRS)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    # 하위 구조 보존을 위해 뒤에 슬래시(/) 포함
    install(DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()