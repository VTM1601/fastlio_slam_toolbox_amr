cmake_minimum_required(VERSION 3.8)
project(fastlio_slam_toolbox_amr NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ros2 run fastlio_slam_toolbox_amr save_map <path/name>
install(PROGRAMS scripts/save_map
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No C/C++ sources; launch files are not public API, so skip docstring and copyright checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
