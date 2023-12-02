# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_turtlebot3_gazebo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED turtlebot3_gazebo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(turtlebot3_gazebo_FOUND FALSE)
  elseif(NOT turtlebot3_gazebo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(turtlebot3_gazebo_FOUND FALSE)
  endif()
  return()
endif()
set(_turtlebot3_gazebo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT turtlebot3_gazebo_FIND_QUIETLY)
  message(STATUS "Found turtlebot3_gazebo: 2.2.6 (${turtlebot3_gazebo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'turtlebot3_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${turtlebot3_gazebo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(turtlebot3_gazebo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${turtlebot3_gazebo_DIR}/${_extra}")
endforeach()
