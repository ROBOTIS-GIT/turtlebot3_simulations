################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


################################################################################
# Packages
################################################################################
find_package(gazebo REQUIRED)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(obstacle_1 SHARED obstacle_1.cc)
target_link_libraries(obstacle_1 ${GAZEBO_LIBRARIES})

add_library(obstacle_2 SHARED obstacle_2.cc)
target_link_libraries(obstacle_2 ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################


