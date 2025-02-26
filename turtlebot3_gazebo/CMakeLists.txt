################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  # ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  # ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

set(EXEC_NAME "turtlebot3_drive")

add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

# add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp)
# target_link_libraries(traffic_light_plugin ${GAZEBO_LIBRARIES})

# add_library(traffic_bar_plugin SHARED src/traffic_bar_plugin.cpp)
# target_link_libraries(traffic_bar_plugin ${GAZEBO_LIBRARIES})

# add_library(obstacle1 SHARED src/obstacle1.cpp)
# target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

# add_library(obstacle2 SHARED src/obstacle2.cpp)
# target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

# add_library(obstacles SHARED src/obstacles.cpp)
# target_link_libraries(obstacles ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(ros_gz_bridge)
ament_export_dependencies(ros_gz_sim)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
