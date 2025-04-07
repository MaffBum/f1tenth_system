cmake_minimum_required(VERSION 3.5)
project(amcl_timing)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(amcl_timing_node src/amcl_timing_node.cpp)

# Link dependencies
ament_target_dependencies(amcl_timing_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install the node
install(TARGETS
  amcl_timing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Linting / Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
