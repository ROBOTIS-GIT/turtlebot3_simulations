################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_fake_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(EXEC_NAME "turtlebot3_fake_node")

add_executable(${EXEC_NAME} src/turtlebot3_fake_node.cpp)
target_link_libraries(${EXEC_NAME}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_msgs_TARGETS}
  ${turtlebot3_msgs_TARGETS}
)

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_msgs
  turtlebot3_msgs
)
ament_package()
