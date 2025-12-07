# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_finder_program_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED finder_program_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(finder_program_FOUND FALSE)
  elseif(NOT finder_program_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(finder_program_FOUND FALSE)
  endif()
  return()
endif()
set(_finder_program_CONFIG_INCLUDED TRUE)

# output package information
if(NOT finder_program_FIND_QUIETLY)
  message(STATUS "Found finder_program: 0.0.0 (${finder_program_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'finder_program' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT finder_program_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(finder_program_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${finder_program_DIR}/${_extra}")
endforeach()
