cmake_minimum_required(VERSION 3.5)
project(mpcc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Executable
add_executable(mpcc_node
  src/mpcc_node.cpp
  src/controller.cpp
  src/kinematic_model.cpp
  src/reference.cpp
  src/util.cpp
)

ament_target_dependencies(mpcc_node
  rclcpp
  ackermann_msgs
  nav_msgs
  geometry_msgs
  Eigen3
)

# Install targets
install(TARGETS mpcc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
