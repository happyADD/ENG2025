# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_RGBD_Process_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED RGBD_Process_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(RGBD_Process_FOUND FALSE)
  elseif(NOT RGBD_Process_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(RGBD_Process_FOUND FALSE)
  endif()
  return()
endif()
set(_RGBD_Process_CONFIG_INCLUDED TRUE)

# output package information
if(NOT RGBD_Process_FIND_QUIETLY)
  message(STATUS "Found RGBD_Process: 0.0.0 (${RGBD_Process_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'RGBD_Process' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${RGBD_Process_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(RGBD_Process_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${RGBD_Process_DIR}/${_extra}")
endforeach()
