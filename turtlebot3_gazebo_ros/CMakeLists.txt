################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_gazebo_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED
)

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
  INCLUDE_DIRS include
  LIBRARIES turtlebot3_gazebo
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf gazebo_ros
 )

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_drive
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
