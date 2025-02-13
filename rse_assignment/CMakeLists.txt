cmake_minimum_required(VERSION 3.8)
project(rse_assignment)

# Ensure warnings are enabled if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# If you have tests, enable them
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/httplib
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Add executable for node
add_executable(script_a src/script_a.cpp)
add_executable(script_b src/script_b.cpp)

# Link necessary dependencies to xecutable
ament_target_dependencies(script_a
  rclcpp
  geometry_msgs
)
ament_target_dependencies(script_b rclcpp geometry_msgs)
install(TARGETS
  script_a script_b
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/script_c.py
  scripts/seaborn.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
