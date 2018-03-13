################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(turtlebot3_gazebo_plugin)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED)

find_package(gazebo REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES turtlebot3_gazebo_plugin
 )

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(turtlebot3 SHARED src/turtlebot3.cc)
target_link_libraries(turtlebot3 ${GAZEBO_LIBRARIES})

add_executable(lidar_listener src/lidar_listener.cc)
target_link_libraries(lidar_listener ${GAZEBO_LIBRARIES} pthread)

add_executable(image_listener src/image_listener.cc)
target_link_libraries(image_listener ${GAZEBO_LIBRARIES} pthread)

################################################################################
# Install
################################################################################
install(DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
