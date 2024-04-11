# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aloam_velodyne_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aloam_velodyne_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aloam_velodyne_FOUND FALSE)
  elseif(NOT aloam_velodyne_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aloam_velodyne_FOUND FALSE)
  endif()
  return()
endif()
set(_aloam_velodyne_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aloam_velodyne_FIND_QUIETLY)
  message(STATUS "Found aloam_velodyne: 0.1.0 (${aloam_velodyne_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aloam_velodyne' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${aloam_velodyne_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aloam_velodyne_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${aloam_velodyne_DIR}/${_extra}")
endforeach()
