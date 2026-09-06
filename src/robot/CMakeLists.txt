cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  planner
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/wro_planner.py
  launch/wro_planner_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
