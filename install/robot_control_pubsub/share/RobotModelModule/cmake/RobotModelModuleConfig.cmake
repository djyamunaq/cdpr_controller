# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_RobotModelModule_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED RobotModelModule_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(RobotModelModule_FOUND FALSE)
  elseif(NOT RobotModelModule_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(RobotModelModule_FOUND FALSE)
  endif()
  return()
endif()
set(_RobotModelModule_CONFIG_INCLUDED TRUE)

# output package information
if(NOT RobotModelModule_FIND_QUIETLY)
  message(STATUS "Found RobotModelModule:  (${RobotModelModule_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'RobotModelModule' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${RobotModelModule_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(RobotModelModule_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${RobotModelModule_DIR}/${_extra}")
endforeach()
