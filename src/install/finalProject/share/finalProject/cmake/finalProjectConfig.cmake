# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_finalProject_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED finalProject_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(finalProject_FOUND FALSE)
  elseif(NOT finalProject_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(finalProject_FOUND FALSE)
  endif()
  return()
endif()
set(_finalProject_CONFIG_INCLUDED TRUE)

# output package information
if(NOT finalProject_FIND_QUIETLY)
  message(STATUS "Found finalProject: 0.0.0 (${finalProject_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'finalProject' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT finalProject_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(finalProject_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${finalProject_DIR}/${_extra}")
endforeach()
