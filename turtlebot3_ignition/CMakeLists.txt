################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_ignition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY launch urdf config rviz gui worlds
  DESTINATION share/${PROJECT_NAME}
)

add_executable(cmd_republisher src/republisher_cmd.cpp)
ament_target_dependencies(cmd_republisher rclcpp geometry_msgs)

install(TARGETS
  cmd_republisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
